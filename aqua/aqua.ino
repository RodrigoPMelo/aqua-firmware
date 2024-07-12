#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Stepper.h>
#include <WiFi.h>
#include "FS.h"
#include <LittleFS.h>
#include <Arduino.h>
#include <DS1302.h>

#define FORMAT_LITTLEFS_IF_FAILED true

// Access Point credentials
const char *ap_ssid = "Aqua Access Point";

// Wifi credentials
String ssid = "";
String password = "";

// Socket server
WiFiServer server(80);

// Turbidity Sensor
const int turbidityPin = 34; // Define the pin for the turbidity sensor

// Pinos do modulo de relogio
const int CLK = 25;
const int DAT = 26;
const int RST = 27;

// DS18B20 Sensor
const int oneWireBus = 15; // GPIO pin where the DS18B20 is connected
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Stepper Motor
const int stepsPerRevolution = 500; // change this to fit the number of steps per revolution for your motor
const int motorPin1 = 19;           // IN1 on the ULN2003 driver
const int motorPin2 = 18;           // IN2 on the ULN2003 driver
const int motorPin3 = 17;           // IN3 on the ULN2003 driver
const int motorPin4 = 16;           // IN4 on the ULN2003 driver
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust the I2C address if needed

// Define o relogio
DS1302 rtc(RST, DAT, CLK);
int horaAtual, minutoAtual;

// Boot button
const int buttonPin = 0; // GPIO0 is the boot button
volatile bool motorState = false;

// ISR for button press
void IRAM_ATTR onButtonPress()
{
  motorState = false;
}

// Initialize variables for period tracking
unsigned long lastMillis = 0;
const size_t valuesLength = 2;

// Initialize variable to start sending data
bool canSendData = false;

void setup()
{
  Serial.begin(115200);
  // Initialize LCD
  lcd.init();      // Inicializa o LCD 16x2
  lcd.backlight(); // Liga a luz de fundo do LCD
  lcd.print("Initializing...");

  // Initialize the file system
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED))
  {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("Little FS Mounted Successfully");
  verifyFile("/wifidata.txt");
  if (connectToWifi())
  {
    canSendData = true; // habilitates the sending of data over mqtt
  }
  else
  {
    // Create a Access Point
    WiFi.softAP(ap_ssid);
    Serial.print("Access Point \"");
    Serial.print(ap_ssid);
    Serial.println("\" started");

    // Start the socket server
    server.begin();
    Serial.println("Socket server started");
  }

  // Config the RTC clock
  rtc.halt(false);
  rtc.writeProtect(false);

  // Initialize sensors
  pinMode(turbidityPin, INPUT);
  sensors.begin();

  // Initialize stepper motor
  myStepper.setSpeed(50); // set the speed to 50 RPM

  // Initialize button
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(buttonPin, onButtonPress, FALLING);

  lcd.clear();
}

int ciclo = 0;
int alterna = 0;
bool flipflop = false;

void loop()
{
  ciclo++;
  if (ciclo > 999)
  {
    alterna++;
    if (alterna > 5)
    {
      alterna = 0;
      flipflop = !flipflop;
    }

    // Read turbidity sensor
    int turbidityValue = analogRead(turbidityPin);
    // Read temperature sensor
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    ciclo = 0;
    // Obtem a hora atual a partir do RTC
    Time t = rtc.time();
    horaAtual = t.hr;
    minutoAtual = t.min;
    if (flipflop)
    {

      // Resets the counters on midnight
      if (horaAtual == 0 && minutoAtual == 0)
      {
      }

      // Control stepper motor
      if (motorState)
      {
        motorState = false; // Reset flag
        alimentarPeixe(stepsPerRevolution);
      }
      // horarioNaTela();
    }
    else
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Turbidez: ");
      lcd.print(turbidityValue, 0);
      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(temperatureC);
      lcd.print(" C");

      // Handle client requests
      if (WiFi.status() == WL_CONNECTED)
      {
        handleSocketClient();
      }
    }

    // Run every x seconds
    if (canSendData)
    {
      if (millis() - lastMillis >= 1 * 1000UL)
      {
        lastMillis = millis(); // get ready for the next iteration
      }
    }
  }
} // end loop

// Feed the fish
void alimentarPeixe(int steps)
{
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

// Print the current time
void horarioNaTela()
{
  // Obtem a hora atual a partir do RTC
  Time t = rtc.time();
  // Transforma os valores do horario em informacao unificada para o LCD
  char horaRelogioStr[10];
  //%02d indica que o programa colocara, nos numeros menores que 10, um zero a esquerda para manter o padrao de exibicao (ex: 09 e nao 9)
  sprintf(horaRelogioStr, "Hora: %02d:%02d:%02d", t.hr, t.min, t.sec);
  // Imprime no LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(horaRelogioStr);
}

void handleSocketClient()
{
  WiFiClient client = server.available();

  if (client)
  {
    Serial.println("New Client.");
    while (client.connected())
    {
      if (client.available())
      {
        String received = client.readStringUntil('\n');
        Serial.print("Received: ");
        Serial.println(received);

        // Send a response back to the client
        client.println("Message received: " + received);
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
// Overwrite a file or creates it if it dont exists
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("- file written");
  }
  else
  {
    Serial.println("- write failed");
  }
  file.close();
}
// Save data to a file without overwriting
void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("- message appended");
  }
  else
  {
    Serial.println("- append failed");
  }
  file.close();
}
// Read the file and get the wifi credentials
void readCredentials(fs::FS &fs, const char *path, String &ssid, String &password)
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");
    return;
  }

  String line;
  while (file.available())
  {
    line = file.readStringUntil('\n');
    line.trim(); // Remove any leading/trailing whitespace
    if (line.startsWith("ssid:"))
    {
      line.remove(0, 5); // Remove "ssid:" part
      ssid = line;
      ssid.trim();
    }
    else if (line.startsWith("password:"))
    {
      line.remove(0, 9); // Remove "password:" part
      password = line;
      password.trim();
    }
  }
  file.close();
}
void verifyFile(const char *path)
{
  bool fileexists = LittleFS.exists(path);
  Serial.print(fileexists);
  if (!fileexists)
  {
    Serial.println("File doesnt exist");
    Serial.println("Creating file...");
    // Create File and add header
    writeFile(LittleFS, path, "");
  }
  else
  {
    readCredentials(LittleFS, path, ssid, password);
  }
}

bool connectToWifi()
{
  if (ssid.equals(""))
  {
    return false;
  }
  // Attempt to connect to the received Wi-Fi credentials
  Serial.printf("%s, %s", ssid.c_str(), password.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());

  int timeout = 10; // 10 seconds timeout
  while (WiFi.status() != WL_CONNECTED && timeout > 0)
  {
    delay(1000);
    Serial.print(".");
    timeout--;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nConnected to Wi-Fi!");
    // stops the AP
    if (WiFi.softAPdisconnect(true))
    {
      Serial.print("AP closed");
    }
    // closes the web server
    server.close();

    char message[128]; // Ensure the buffer is large enough to hold the concatenated string
    snprintf(message, sizeof(message), "ssid: %s\r\npassword: %s\r\n", ssid, password);
    writeFile(LittleFS, "/wifidata.txt", message);
    return true;
  }
  else
  {
    Serial.println("\nFailed to connect to Wi-Fi.");
    return false;
  }
}

void sendSensorsData()
{
  // Your implementation for sending sensor data
}
