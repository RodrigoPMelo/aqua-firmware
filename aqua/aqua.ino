#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Stepper.h>
#include <WiFi.h>
#include "nvs_flash.h"
#include "nvs.h"
#include <Arduino.h>
#include <DS1302.h>
#include "mqtt_client.h"

// Access Point credentials
const char *ap_ssid = "Aqua Access Point";

// Wifi credentials
String ssid = "";
String password = "";

// Device Id
String uuid = "";

// MQTT config
const char *mqtt_server = "mqtt://test.mosquitto.org";
const int mqtt_port = 1883;
char publish_topic[128], subscribe_topic[128];
esp_mqtt_client_handle_t mqtt_client;

// Socket server
WiFiServer server(80);

// Add NVS handle
nvs_handle_t nvs;

// Turbidity Sensor
const int turbidityPin = 34;  // Define the pin for the turbidity sensor

// Pinos do modulo de relogio
const int CLK = 25;
const int DAT = 26;
const int RST = 27;

// DS18B20 Sensor
const int oneWireBus = 13;  // GPIO pin where the DS18B20 is connected
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Stepper Motor
const int stepsPerRevolution = 500;  // change this to fit the number of steps per revolution for your motor
const int motorPin1 = 19;            // IN1 on the ULN2003 driver
const int motorPin2 = 18;            // IN2 on the ULN2003 driver
const int motorPin3 = 17;            // IN3 on the ULN2003 driver
const int motorPin4 = 16;            // IN4 on the ULN2003 driver
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adjust the I2C address if needed

// Define o relogio
DS1302 rtc(RST, DAT, CLK);
int currentHour, currentMinute;

//Parametros horarios de alimentacao
int alimHour1, alimMinute1, feeded1;
int alimHour2, alimMinute2, feeded2;
int alimHour3, alimMinute3, feeded3;


// Initialize variables for period tracking
unsigned long lastMillis = 0;
const size_t valuesLength = 2;

// Initialize variable to start sending data
bool canSendData = false;


// Initialize variables that store sensors data
float turbidityValue = 0;
float temperatureC = 0;

void setup() {
  Serial.begin(115200);
  // Initialize LCD
  lcd.init();       // Inicializa o LCD 16x2
  lcd.backlight();  // Liga a luz de fundo do LCD
  lcd.print("Initializing...");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Open NVS handle
  ret = nvs_open("storage", NVS_READWRITE, &nvs);
  if (ret != ESP_OK) {
    Serial.println("Error opening NVS handle!");
  } else {
    Serial.println("NVS initialized successfully");
    // Load credentials
    loadCredentials(ssid, password, uuid);
    loadAlimentationTime();
    if (connectToWifi()) {
      canSendData = true;  // Enable sending data over MQTT
    } else {
      WiFi.softAP(ap_ssid);
      server.begin();
      Serial.println("Access Point started.");
    }
  }

  // Config the RTC clock
  rtc.halt(false);
  rtc.writeProtect(false);

  // Initialize sensors
  pinMode(turbidityPin, INPUT);
  sensors.begin();

  // Initialize stepper motor
  myStepper.setSpeed(50);  // set the speed to 50 RPM

  lcd.clear();
}

int ciclo = 0;
int alterna = 0;
bool flipflop = false;

void loop() {
  ciclo++;
  if (ciclo > 999) {
    alterna++;
    if (alterna > 5) {
      alterna = 0;
      flipflop = !flipflop;
    }
    ciclo = 0;
    // Obtem a hora atual a partir do RTC
    Time t = rtc.time();
    currentHour = t.hr;
    currentMinute = t.min;
    if (flipflop) {

      //Verifica se e o horário da primeira alimentacao
      if (currentHour == alimHour1 && currentMinute == alimMinute1 && feeded1 == 0) {
        feed(500);
        feeded1 = 1;  //altera status da comida1
      }

      //Verifica se e o horário da segunda alimentacao
      if (currentHour == alimHour2 && currentMinute == alimMinute2 && feeded2 == 0) {
        feed(500);
        feeded2 = 1;  //altera status da comida2
      }

      if (currentHour == alimHour3 && currentMinute == alimMinute3 && feeded3 == 0) {
        feed(500);
        feeded3 = 1;  //altera status da comida2
      }

      // Resets the counters on midnight
      if (currentHour == 0 && currentMinute == 0) {
        feeded1 = 0;
        feeded2 = 0;
        feeded3 = 0;
      }

      // alimentarPeixe(stepsPerRevolution);
    } else {
      // Run every x seconds
      if (canSendData) {

        if (millis() - lastMillis >= 1 * 15000UL) {
          lastMillis = millis();  // get ready for the next iteration
          // Read turbidity sensor
          turbidityValue = analogRead(turbidityPin);
          // Read temperature sensor
          sensors.requestTemperatures();
          temperatureC = sensors.getTempCByIndex(0);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Turbidez: ");
          lcd.print(turbidityValue, 0);
          lcd.setCursor(0, 1);
          lcd.print("Temp: ");
          lcd.print(temperatureC);
          lcd.print(" C");
          sendSensorsData(turbidityValue, temperatureC);
        }
      } else {
        handleSocketClient();
      }
    }
  }
}  // end loop

// Feed the fish
void feed(int steps) {
  delay(1000);
  // Activate the step motor
  myStepper.step(steps);
  // Desliga qualquer bobina do motor que possa ter ficado acionada
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(500);
  myStepper.step(-steps);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(500);
}


void handleSocketClient() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New Client.");
    while (client.connected()) {
      if (client.available()) {
        String received = client.readStringUntil('\n');
        Serial.print("Received: ");
        processClientMessage(received);
        if (ssid != "" && password != "" && uuid != "") {
          char message[256];  // Ensure the buffer is large enough to hold the concatenated string
          snprintf(message, sizeof(message), "ssid: %s\r\npassword: %s\r\nuuid: %s\r\n", ssid, password, uuid.c_str());

          saveCredentials(ssid.c_str(), password.c_str(), uuid.c_str());
          client.println("Valid message received and saved: " + received);
        } else {
          client.println("Invalid message received:" + received);
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
    if (connectToWifi()) {
      canSendData = true;  // habilitates the sending of data over mqtt
    }
  }
}
void processClientMessage(String message) {
  Serial.println("Received message: " + message);

  int firstSeparator = message.indexOf(',');
  int secondSeparator = message.indexOf(',', firstSeparator + 1);

  uuid = message.substring(0, firstSeparator);
  ssid = message.substring(firstSeparator + 1, secondSeparator);
  password = message.substring(secondSeparator + 1);

  Serial.printf("UUID: %s, SSID: %s, Password: %s\n", uuid.c_str(), ssid.c_str(), password.c_str());
}
// save to credentials to nvs flash
void saveCredentials(const char *ssid, const char *password, const char *uuid) {
  esp_err_t err;

  // Save SSID
  err = nvs_set_str(nvs, "ssid", ssid);
  if (err != ESP_OK) {
    Serial.printf("Failed to write SSID to NVS: %s\n", esp_err_to_name(err));
    return;
  }

  // Save password
  err = nvs_set_str(nvs, "password", password);
  if (err != ESP_OK) {
    Serial.printf("Failed to write password to NVS: %s\n", esp_err_to_name(err));
    return;
  }

  // Save UUID
  err = nvs_set_str(nvs, "uuid", uuid);
  if (err != ESP_OK) {
    Serial.printf("Failed to write UUID to NVS: %s\n", esp_err_to_name(err));
    return;
  }

  // Commit written value.
  err = nvs_commit(nvs);
  if (err != ESP_OK) {
    Serial.printf("Failed to commit changes to NVS: %s\n", esp_err_to_name(err));
  } else {
    Serial.println("Credentials saved successfully");
  }
}
// Read the nvs flash and get the wifi credentials
void loadAlimentationTime() {
  char value[64];
  size_t length;

  // Load SSID
  length = sizeof(value);
  esp_err_t err = nvs_get_str(nvs, "1H", value, &length);
  if (err == ESP_OK) {
    processAlimentationTime(String(value), &alimHour1, &alimMinute1);
    Serial.println("H1 loaded: " + String(value));
  } else {
    Serial.printf("Failed to load alimentation time 1 from NVS: %s\n", esp_err_to_name(err));
  }

  // Load password
  length = sizeof(value);
  err = nvs_get_str(nvs, "2H", value, &length);
  if (err == ESP_OK) {
    processAlimentationTime(String(value), &alimHour2, &alimMinute2);
    Serial.println("H2 loaded: " + String(value));
  } else {
    Serial.printf("Failed to load alimentation time 2 from NVS: %s\n", esp_err_to_name(err));
  }

  // Load UUID
  length = sizeof(value);
  err = nvs_get_str(nvs, "3H", value, &length);
  if (err == ESP_OK) {
    processAlimentationTime(String(value), &alimHour3, &alimMinute3);
    Serial.println("H3 loaded: " + String(value));
  } else {
    Serial.printf("Failed to load alimentation time 3 from NVS: %s\n", esp_err_to_name(err));
  }
}

void loadCredentials(String &ssid, String &password, String &uuid) {
  char value[64];
  size_t length;

  // Load SSID
  length = sizeof(value);
  esp_err_t err = nvs_get_str(nvs, "ssid", value, &length);
  if (err == ESP_OK) {
    ssid = String(value);
    Serial.println("SSID loaded: " + ssid);
  } else {
    Serial.printf("Failed to load SSID from NVS: %s\n", esp_err_to_name(err));
  }

  // Load password
  length = sizeof(value);
  err = nvs_get_str(nvs, "password", value, &length);
  if (err == ESP_OK) {
    password = String(value);
    Serial.println("Password loaded.");
  } else {
    Serial.printf("Failed to load password from NVS: %s\n", esp_err_to_name(err));
  }

  // Load UUID
  length = sizeof(value);
  err = nvs_get_str(nvs, "uuid", value, &length);
  if (err == ESP_OK) {
    uuid = String(value);
    Serial.println("UUID loaded: " + uuid);
  } else {
    Serial.printf("Failed to load UUID from NVS: %s\n", esp_err_to_name(err));
  }
}

bool connectToWifi() {
  if (ssid.equals("")) {
    return false;
  }
  ssid.trim();
  password.trim();

  // Attempt to connect to the received Wi-Fi credentials
  WiFi.begin(ssid.c_str(), password.c_str());

  int timeout = 10;  // 10 seconds timeout
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(1000);
    Serial.print(".");
    timeout--;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi!");
    // stops the AP
    if (WiFi.softAPdisconnect(true)) {
      Serial.print("AP closed");
    }
    // closes the web server
    server.close();
    mqtt_app_start();
    delay(5000);
    return true;
  } else {
    Serial.println("\nFailed to connect to Wi-Fi.");
    return false;
  }
}

void cleanup() {
  nvs_close(nvs);
}

void sendSensorsData(float turbidity, float temperatureC) {

  snprintf(publish_topic, sizeof(publish_topic), "aqua/devices/%s/sensors", uuid.c_str());

  char sensorData[64];
  snprintf(sensorData, sizeof(sensorData), "turbidez: %f, temperature: %f", turbidity, temperatureC);

  int msg_id = esp_mqtt_client_publish(mqtt_client, publish_topic, sensorData, 0, 2, 0);
}
void processSubscribeData(String data) {
  Serial.printf("Received data: %s\n", data);
  if (data.startsWith("1H")) {
    processAlimentationTime(data, &alimHour1, &alimMinute1);
  } else if (data.startsWith("2H")) {
    processAlimentationTime(data, &alimHour2, &alimMinute2);
  } else if (data.startsWith("3H")) {
    processAlimentationTime(data, &alimHour3, &alimMinute3);
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

  switch (event_id) {
    case MQTT_EVENT_CONNECTED:
      Serial.println("MQTT_EVENT_CONNECTED");
      {
        String subTopic = "aqua/devices/" + uuid + "/actuators";
        esp_mqtt_client_subscribe(mqtt_client, subTopic.c_str(), 0);
      }
      canSendData = true;
      break;

    case MQTT_EVENT_DISCONNECTED:
      Serial.println("MQTT_EVENT_DISCONNECTED");
      canSendData = false;
      break;

    case MQTT_EVENT_DATA:
      Serial.println("MQTT_EVENT_DATA");
      //Serial.printf("Received topic: %.*s\n", event->topic_len, event->topic);

      processSubscribeData(event->data);
      break;

    default:
      break;
  }
}

void mqtt_app_start(void) {
  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.broker.address.uri = mqtt_server;
  mqtt_cfg.broker.address.port = mqtt_port;

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, mqtt_client);
  Serial.println("Starting MQTT");
  esp_mqtt_client_start(mqtt_client);
}

void processAlimentationTime(String message, int *h, int *min) {
  int hyphenIndex = message.indexOf('-');
  int firstColonIndex = message.indexOf(':');
  String prefix;
  esp_err_t err;
  // Check if both the hyphen and colons are found
  if (hyphenIndex != -1 && firstColonIndex != -1) {
    // Extract prefix (the part before the hyphen)
    prefix = message.substring(0, hyphenIndex);
    // Extract hour and minute substrings
    String hourString = message.substring(hyphenIndex + 1, firstColonIndex);
    String minuteString = message.substring(firstColonIndex + 1);

    // Convert substrings to integers and assign to pointers
    *h = hourString.toInt();
    *min = minuteString.toInt();
    Serial.printf("%s, %s, %s", prefix, hourString, minuteString);
  }

  // Save the alimentation time to nvs
  err = nvs_set_str(nvs, prefix.c_str(), message.c_str());
  if (err != ESP_OK) {
    Serial.printf("Failed to write %s to NVS: %s\n", prefix, esp_err_to_name(err));
    return;
  }
}
