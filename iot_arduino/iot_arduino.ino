#include <SPI.h>
#include <WiFi101.h>
#include <ADXL345.h>
#include <Wire.h>
#include "arduino_secrets.h"

#define ALCOHOL_DAT A0
#define HEATER_SEL A1
#define LED_PIN 3

#ifdef NRF52840_XXAA
#endif

ADXL345 adxl;
unsigned long readInterval = 500;
unsigned long printInterval = 1000;
unsigned long pulseInterval = 300;
const unsigned long pulseLength = 50;
bool connected = false;

unsigned long lastReadTime = millis();
unsigned long lastPrintTime = millis();

bool ledState = false; // LED initially OFF
unsigned long lastPulse = 0;


uint8_t pulse = 0;
uint16_t acc = 0;
uint16_t alc = 0;

char ssid[] = SECRET_SSID;
char password[] = SECRET_PASS;
char server[] = SECRET_SERVER;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  initAccelerometer();
  pinsInit();
  switchOnHeater();
  Wire.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

uint8_t readPulse() {
  uint8_t pulse = 0;

  Wire.requestFrom(0x50, 1);
  if (Wire.available()) {
    pulse = Wire.read();
  }

  return pulse;
}

uint16_t readAlcohol() {
  uint16_t sensorValue = analogRead(ALCOHOL_DAT);  //read the analog value

  return 1023 - sensorValue;
}

uint16_t readAcceleration() {
  int x,y,z;  
	adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

	double xyz[3];
	adxl.getAcceleration(xyz);

	float total = sqrt(xyz[0]*xyz[0] +
                     xyz[1]*xyz[1] +
                     xyz[2]*xyz[2]);

  return (uint16_t)(total * 100); 
}

void sendHTTP(uint8_t pul, uint16_t acc, uint16_t alc) {

  if (client.connect(server, 5000)) {

    char postData[48];
    snprintf(postData, sizeof(postData), "p=%d&a=%d&v=%d", pul, acc, alc);

    client.println(F("POST /api/endpoint HTTP/1.1"));
    client.print(F("Host: "));
    client.println(server);
    client.println(F("Connection: close"));
    client.println(F("Content-Type: application/x-www-form-urlencoded"));
    client.print(F("Content-Length: "));
    client.println(strlen(postData));
    client.println();
    client.print(postData);

    client.stop();

    Serial.println("Data sent");
    connected = true;
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("Connection failed");
    connected = false;
    digitalWrite(LED_PIN, HIGH);
  }
}

void loop() {
  static unsigned long lastSend = 0;
  unsigned long currentMillis = millis();

  if(millis() - lastReadTime > readInterval) {
    uint8_t pulse1 = readPulse();
    uint16_t alc1 = readAlcohol();
    uint16_t acc1 = readAcceleration();
    pulse = (pulse1 > pulse) ? pulse1 : pulse;
    alc = (alc1 > alc) ? alc1 : alc;
    acc = (acc1 > acc) ? acc1 : acc;
    lastReadTime = millis();
  }
  if(connected) {
    if (pulse > 0) {
      // Calculate interval based on BPM
      pulseInterval = 60000UL / pulse; // milliseconds between beats
      unsigned long currentMillis = millis();

      if (!ledState && (currentMillis - lastPulse >= pulseInterval)) {
          digitalWrite(LED_PIN, HIGH);
          lastPulse = currentMillis;
          ledState = true;
      }

      if (ledState && (currentMillis - lastPulse >= pulseLength)) {
         digitalWrite(LED_PIN, LOW);
          ledState = false;
      }
    } else {
      // If pulse = 0, make sure LED is off
      digitalWrite(LED_PIN, LOW);
      ledState = false;
      }
  }
  if (millis() - lastSend > 5000) { // send every 5 seconds
    lastSend = millis();
    sendHTTP(pulse, acc, alc);
    pulse = 0;
    acc = 0;
    alc = 0;
  }
}

void pinsInit() {
  pinMode(HEATER_SEL, OUTPUT);  // set the HEATER_SEL as digital output.
  switchOffHeater();            //when HEATER_SEL is set, heater is switched off.
  pinMode(ALCOHOL_DAT, INPUT);
}
/*switch on the heater of Alcohol sensor*/
void switchOnHeater() {
  digitalWrite(HEATER_SEL, LOW);
}
/*switch off the heater of Alcohol sensor*/
void switchOffHeater() {
  digitalWrite(HEATER_SEL, HIGH);
}

void initAccelerometer() {
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}
