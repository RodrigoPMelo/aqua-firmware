//Inclusão das bibliotecas
#include <DS1302.h>             //para o modulo de relogio
#include <LiquidCrystal_I2C.h>  //para o LCD
#include <Stepper.h>            //para o motor de passo
#include <Wire.h>               //para comunicacao I2C a utilizada pelo LCD
#include "GravityTDS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>


#define VALUES_LENGTH 2

//Pinos do modulo de relogio
const int CLK = 5;
const int DAT = 6;
const int RST = 7;

//Define o relogio
DS1302 rtc(RST, DAT, CLK);

//Define o endereco do LCD e o tamanho da tela
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Define as entradas onde o motor de passo está conectado
#define in1 1
#define in2 8
#define in3 9
#define in4 4

//Quantidade de passos que o motor é capaz de dar em cada acionamento
int passosPorAcionamento = 32;

//Quantidade de passos para cada alimentacao
//Ajuste o numero necessario
int passosRefeicao = 4;

//Define o motor de passo
// ordenar as entradas em in1, in3, in2, in4
Stepper mp(passosPorAcionamento, in1, in3, in2, in4);

//Parametros de horario que serao atualizados
int horaAtual, minutoAtual;
//Parametros horarios de alimentacao - serao recebidos por bluetooth no formato "1H-12:23", "2H-14:45", "3H-18:55"
int horaAlimentacao1, minutoAlimentacao1, demosComida1;
int horaAlimentacao2, minutoAlimentacao2, demosComida2;
int horaAlimentacao3, minutoAlimentacao3, demosComida3;

unsigned long lastMillis;
const size_t valuesLength = 2;

/* ******************** CONFIGURACAO DE HORARIO PARA ALIMENTACAO ********************/

//Substitua os valores abaixo pelos horarios desejados para alimentacao
//Caso nao queira duas alimentacoes por dia, use um valor de hora acima de 24 e um de minuto acima de 59

//Primeira alimentacao
//#define horaAlimentacao1 12
//#define minutoAlimentacao1 12

//Segunda alimentacao
//#define horaAlimentacao2 12
//#define minutoAlimentacao2 34

#define ONE_WIRE_BUS A1
#define TdsSensorPin A0

OneWire oneWire(ONE_WIRE_BUS);
GravityTDS gravityTds;

DallasTemperature sensors(&oneWire);

SoftwareSerial BT(2, 3);  // RX TX
const int ledPin = 13;    // the pin that the LED is attached to
String incomingByte;

/***************************************************************************** */

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(ledPin, OUTPUT);


  //Prepara o LCD
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  //Define a velocidade do motor
  mp.setSpeed(500);

  //Prepara o relogio e desativa a proteção contra a escrita
  rtc.halt(false);
  rtc.writeProtect(false);

  sensors.begin();
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);       //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();            //initialization

  //ATENÇÃO: Descomente para ajustar a hora inicial do relogio, suba o codigo para o arduino com o modulo RTC conectado, comente novamente as linhas e suba novamente o codigo.
  //Define a data e hora no chip o formato deve ser: ano, mes, dia, hora, minuto, segundo e dia da semana sendo o domingo o dia 1.
  //Time t(2024, 04, 11, 23, 30, 00, 5);
  //rtc.time(t);

  //Determina o status de alimentação. Zero equivale a não e um a sim
  demosComida1 = 0;
  demosComida2 = 0;
}
float tdsValue = 0, temperatureValue = 0;
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
    sensors.requestTemperatures();
    gravityTds.setTemperature(sensors.getTempCByIndex(0));  // set the temperature and execute temperature compensation
    gravityTds.update();                                    //sample and calculate
    tdsValue = gravityTds.getTdsValue();                    // then get the value

    ciclo = 0;
    //Obtem a hora atual a partir do RTC
    Time t = rtc.time();
    horaAtual = t.hr;
    minutoAtual = t.min;
    if (flipflop) {


      //Verifica se e o horário da primeira alimentacao

      if (horaAtual == horaAlimentacao1 && minutoAtual == minutoAlimentacao1 && demosComida1 == 0) {
        alimentarPeixe();
        demosComida1 = 1;  //altera status da comida1
      }
      //Verifica se e o horário da segunda alimentacao
      if (horaAtual == horaAlimentacao2 && minutoAtual == minutoAlimentacao2 && demosComida2 == 0) {
        alimentarPeixe();
        demosComida2 = 1;  //altera status da comida2
      }
      if (horaAtual == horaAlimentacao3 && minutoAtual == minutoAlimentacao3 && demosComida3 == 0) {
        alimentarPeixe();
        demosComida3 = 1;  //altera status da comida2
      }
      horarioNaTela();
      /*
       if (BT.available() > 0) {  //caso o app mande um comando
        // read the oldest byte in the serial buffer:
        incomingByte = BT.readString();
        //  turn on the LED:
        //Serial.println(incomingByte);
        if (incomingByte.length() > 0) {

          if (incomingByte == "HIGH") {
            digitalWrite(ledPin, HIGH);
            //BT.println("LED: ON");
          }
          //  turn off the LED:
          else if (incomingByte == "LOW") {
            digitalWrite(ledPin, LOW);
            //BT.println("LED: OFF");
          } else if (incomingByte == "ALIMENTAR") {
            alimentarPeixe();
          } else if (incomingByte.startsWith("1H")) {
            recebeHorarioAlimentacao(incomingByte, &horaAlimentacao1, &minutoAlimentacao1);
            //Serial.println(horaAlimentacao1);
            //Serial.println(minutoAlimentacao1);

          } else if (incomingByte.startsWith("2H")) {
            recebeHorarioAlimentacao(incomingByte, &horaAlimentacao2, &minutoAlimentacao2);

          } else if (incomingByte.startsWith("3H")) {
            recebeHorarioAlimentacao(incomingByte, &horaAlimentacao3, &minutoAlimentacao3);
          } else if (incomingByte == "start") {
            start = true;
          } else if (incomingByte == "stop") {
            start = false;
          }
        }
      }
    */

      //Imprime o horario da proxima alimentacao, nao funciona mais pq eu n implementei uma verificacao do valor das variaveis, por isso da um erro que trava o processamento. pf nao descomentar o codigo abaixo.
      /*if (demosComida1 == 0 || demosComida1 == 1 && demosComida2 == 1) {
        horarioNaTela();
        alimentacao1Tela();
      }

      if (demosComida1 == 1 && demosComida2 == 0) {
        horarioNaTela();
        alimentacao2Tela();
      }*/

      //Meia noite reseta o status de comida do dia
      if (horaAtual == 0 && minutoAtual == 0) {
        demosComida1 = 0;
        demosComida2 = 0;
        demosComida3 = 0;
      }
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TDS: ");
      lcd.print(tdsValue, 0);
      lcd.print(" PPM");
      //Serial.write((int)(tdsValue));
      temperatureValue = sensors.getTempCByIndex(0);
      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(temperatureValue);
      lcd.print(" C");
    }
  }
  if (start) {
    if (millis() - lastMillis >= 1 * 1000UL) {
      lastMillis = millis();  //get ready for the next iteration
      enviaDadosFormatadosBT(tdsValue, temperatureValue);
    }
  }
}

//Funcao auxiliar que comanda o motor de passo e realiza a alimentacao
void alimentarPeixe() {
  //Movimenta o motor de passo
  for (int i = 0; i < passosRefeicao; i++) {
    mp.step(500);
  }
  //Desliga qualquer bobina do motor que possa ter ficado acionada
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(1000);
}

//Funcao auxiliar que faz o LCD exibir o horario como um relogio
void horarioNaTela() {
  //Obtem a hora atual a partir do RTC
  Time t = rtc.time();
  //Transforma os valores do horario em informacao unificada para o LCD
  char horaRelogioStr[10];
  //%02d indica que o programa colocara, nos numeros menores que 10, um zero a esquerda para manter o padrao de exibicao (ex: 09 e nao 9)
  sprintf(horaRelogioStr, "Hora: %02d:%02d:%02d", t.hr, t.min, t.sec);
  //Imprime no LCD
  lcd.setCursor(0, 0);
  lcd.print(horaRelogioStr);
}

void enviaDadosFormatadosBT(float tdsValue, float temperatureValue) {
  // Create an array to hold the values and the delimiter
  float values[VALUES_LENGTH] = { tdsValue, temperatureValue };
  byte data[sizeof(values) + 1];  // Plus 1 for the delimiter
  memcpy(data, values, sizeof(values));
  data[sizeof(values)] = '!';  // Add the delimiter at the end

  // Send the data over serial
  BT.write(data, sizeof(data));
}

void recebeHorarioAlimentacao(String message, int* h, int* min) {
  int hyphenIndex = message.indexOf('-');
  int firstColonIndex = message.indexOf(':');

  // Check if both the hyphen and colons are found
  if (hyphenIndex != -1 && firstColonIndex != -1) {
    // Extract hour and minute substrings
    String hourString = message.substring(hyphenIndex + 1, firstColonIndex);
    String minuteString = message.substring(firstColonIndex + 1);

    // Convert substrings to integers and assign to pointers
    *h = hourString.toInt();
    *min = minuteString.toInt();
  }
}
