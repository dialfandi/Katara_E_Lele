//==================INCLUDE LIBRARY===================
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <Arduino.h>
//==================INITIALIZE PIN=====================
#define pinServo 13
#define pinRelay 12
#define pinPH 34
const int oneWireBus = 14;
//====================================================
#define FIREBASE_HOST "katara-84403-default-rtdb.firebaseio.com" //Without http:// or https:// schemes
#define FIREBASE_AUTH "WVWkZRARrNIxsz7ZuCKuB1e760hxeZntSmOjemZM"
#define WIFI_SSID "ITS1"
#define WIFI_PASSWORD "12345678"
//=============INIITIALIZE CLASS VARIABLE=============
Servo myservo;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);
FirebaseData firebaseData;
//================INITIALIZE VARIABLE==================
//float coba1, coba2;
long previousMillis = 0;
unsigned long interval = 100;
int pos = 0;
// kalman variables
float varVolt = 0.051364211;  // variance determined using excel and reading samples of raw sensor data
float varProcess = 1e-6;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

float phValue;
char PH[10];
float temperatureC;
char valueTemperature[10];
//===================END INITIALIZE====================

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(pinServo, 1000, 4000); // Max bit resolution ADC 4096

  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > 300) {
      Serial.print(".");
      previousMillis = currentMillis;
    }
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  sensors.begin();
  pinMode(pinRelay, OUTPUT);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("     KATARA");
  lcd.setCursor(0, 1);
  lcd.print("       ON");
  delay(1000);
  lcd.clear();
}

void phSensor() {
  int sensorVal = analogRead(pinPH);
  float volt = sensorVal * 5.0 / 4096;
  // kalman process
  Pc = P + varProcess;
  G = Pc / (Pc + varVolt);  // kalman gain
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (volt - Zp) + Xp; // the kalman estimate of the sensor voltage

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    phValue = -4.6786 * Xe + 8.3291;
    Serial.println(phValue, 1);
  }
}

void forwardServo(int Fangle, int FspeedTime) {
  for (pos = 0; pos <= Fangle; pos += 1) {
    myservo.write(pos);
    delay(FspeedTime);
  }
}

void reverseServo(int Rangle, int RspeedTime) {
  for (pos = Rangle; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(RspeedTime);
  }
}

void printTemperature() {
  sensors.requestTemperatures();
  temperatureC = sensors.getTempCByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
}

void relayMode(bool onOff) {
  if (onOff == 1) {
    digitalWrite(pinRelay, HIGH);
  }
  else if (onOff == 0) {
    digitalWrite(pinRelay, LOW);
  }
}

void lcdShow() {
  phSensor();
  printTemperature();
  sprintf(PH, "%.1f", phValue);
  sprintf(valueTemperature, "%.2f", temperatureC);

  lcd.setCursor(0, 0); lcd.print("PH   :   "); lcd.print(PH);
  lcd.setCursor(0, 1); lcd.print("SUHU :   "); lcd.print(valueTemperature);
}

void updateFirebase() {
  printTemperature();
  phSensor();

  Firebase.setFloat(firebaseData, "/Sensor%20PH", phValue);
  Firebase.setFloat(firebaseData, "/Sensor%20Suhu", temperatureC);
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > 100) {
      updateFirebase();
      lcdShow();
      previousMillis = currentMillis;
    }
  
    if (Firebase.getInt(firebaseData, "/RELAY_STATUS")) {
      if (firebaseData.dataType() == "int") {
        //Serial.println(firebaseData.intData());
        if (firebaseData.intData() == 0) {
          Serial.println("Relay OFF");
          relayMode(0);
        }
        else {
          Serial.println(firebaseData.intData());
          relayMode(1);
        }
      }
    }
  
    if (Firebase.getInt(firebaseData, "/SERVO_STATUS")) {
      if (firebaseData.dataType() == "int") {
        if (firebaseData.intData() == 1) {
          unsigned long currentMillis = millis();
          if (currentMillis - previousMillis > 500) {
            Serial.println("BUKA SERVO");
            forwardServo(70, 10);
            previousMillis = currentMillis;
          }
        }
        else {
          unsigned long currentMillis = millis();
          if (currentMillis - previousMillis > 500) {
            Serial.println("TUTUP SERVO");
            reverseServo(10, 100);
            Serial.println(firebaseData.intData());
            previousMillis = currentMillis;
          }
        }
      }
    }
}
