#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Stepper.h>
#include <DS1302.h>
#include <WiFi.h>
#include <WebServer.h>


// Access Point credentials
const char *ap_ssid = "Aqua Access Point";

// Wifi credentials
String ssid = "";
String password = "";

// Web server running on port 80
WebServer server(80);

// Turbidity Sensor
const int turbidityPin = 34;  // Define the pin for the turbidity sensor

//Pinos do modulo de relogio
const int CLK = 25;
const int DAT = 26;
const int RST = 27;

// DS18B20 Sensor
const int oneWireBus = 15;  // GPIO pin where the DS18B20 is connected
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

//Define o relogio
DS1302 rtc(RST, DAT, CLK);
int horaAtual, minutoAtual;

// Boot button
const int buttonPin = 0;  // GPIO0 is the boot button
volatile bool motorState = false;

// ISR for button press
void IRAM_ATTR onButtonPress() {
  motorState = false;
}

// Initialize variables for period tracking
unsigned long lastMillis;
const size_t valuesLength = 2;

void setup() {
  Serial.begin(115200);

  //Create a Access Point
  WiFi.softAP(ap_ssid);
  Serial.print("Access Point \"");
  Serial.print(ap_ssid);
  Serial.println("\" started");

  // Print the IP address
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Define routes for the web server
  server.on("/", handleRoot);
  server.on("/wificonfig", handleWifiConfig);
  server.onNotFound(handleNotFound);

  // Start the web server
  server.begin();
  Serial.println("Web server started");

  // Config the RTC clock
  rtc.halt(false);
  rtc.writeProtect(false);

  // Initialize sensors
  pinMode(turbidityPin, INPUT);
  sensors.begin();

  // Initialize stepper motor
  myStepper.setSpeed(50);  // set the speed to 50 RPM

  // Initialize LCD
  lcd.init();       // Inicializa o LCD 16x2
  lcd.backlight();  // Liga a luz de fundo do LCD
  lcd.print("Initializing...");
  delay(2000);
  // Initialize button
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(buttonPin, onButtonPress, FALLING);
}

int ciclo = 0;
int alterna = 0;
bool flipflop = false;
bool start = false;
void loop() {
  ciclo++;
  if (ciclo > 999) {
    alterna++;
    if (alterna > 5) {
      alterna = 0;
      flipflop = !flipflop;
    }

    // Read turbidity sensor
    int turbidityValue = analogRead(turbidityPin);
    // Read temperature sensor
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    ciclo = 0;
    //Obtem a hora atual a partir do RTC
    Time t = rtc.time();
    horaAtual = t.hr;
    minutoAtual = t.min;
    if (flipflop) {

      //Resets the counters on midnight
      if (horaAtual == 0 && minutoAtual == 0) {
      }

      // Control stepper motor
      if (motorState) {
        motorState = false;  // Reset flag
        alimentarPeixe(stepsPerRevolution);
      }
      //horarioNaTela();
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Turbidez: ");
      lcd.print(turbidityValue, 0);
      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(temperatureC);
      lcd.print(" C");

      // Handle client requests
      server.handleClient();
    }

    // Run every x seconds
    if (start) {
      if (millis() - lastMillis >= 1 * 1000UL) {
        lastMillis = millis();  //get ready for the next iteration
      }
    }
  }
}
// Feed the fish
void alimentarPeixe(int steps) {
  delay(1000);
  // Activate the step motor
  myStepper.step(steps);
  //Desliga qualquer bobina do motor que possa ter ficado acionada
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

//Print the current time
void horarioNaTela() {
  //Obtem a hora atual a partir do RTC
  Time t = rtc.time();
  //Transforma os valores do horario em informacao unificada para o LCD
  char horaRelogioStr[10];
  //%02d indica que o programa colocara, nos numeros menores que 10, um zero a esquerda para manter o padrao de exibicao (ex: 09 e nao 9)
  sprintf(horaRelogioStr, "Hora: %02d:%02d:%02d", t.hr, t.min, t.sec);
  //Imprime no LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(horaRelogioStr);
}

void handleRoot() {
  // Send the connected message
  server.send(200, "text/plain", "Connected to ESP32 AP. Please send your Wi-Fi SSID and Password.");
}

void handleNotFound() {
  //Send 404 message
  server.send(404, "text/plain", "404: Not found");
}

void handleWifiConfig() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    ssid = server.arg("ssid");
    password = server.arg("password");

    // Print received SSID and password
    Serial.print("Received SSID: ");
    Serial.println(ssid);
    Serial.print("Received Password: ");
    Serial.println(password);

    // Respond to the client
    server.send(200, "text/plain", "SSID and Password received. Attempting to connect to Wi-Fi...");

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
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to connect to Wi-Fi.");
    }
  } else {
    server.send(400, "text/plain", "Bad Request: Missing ssid or password");
  }
}
