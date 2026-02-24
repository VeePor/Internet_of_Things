#include <Wire.h>
#include <ADXL345.h>
#define ALCOHOL_DAT A0
#define HEATER_SEL A1
#define LED_PIN 3

#ifdef NRF52840_XXAA
#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif
#endif

ADXL345 adxl;
unsigned long readInterval = 1000;
unsigned long printInterval = 1000;
unsigned long pulseInterval = 300;
unsigned long pulseLength = 50;

unsigned long lastReadTime = millis();
unsigned long lastPrintTime = millis();
unsigned long lastPulse = millis();

void setup() {
  Serial.begin(9600);  // open the serial port at 9600 bps
  pinMode(LED_PIN, OUTPUT);
  initAccelerometer();
  pinsInit();
  switchOnHeater();
  Serial.println("heart rate sensor:");
  Serial.println("Start to heat the sensor, please wait 5~10min befor exposure to alcohol");
  Wire.begin();
}


void loop() {
  if(millis() - lastReadTime > readInterval) {
    readPulse();
    readAlcohol();
    readAcceleration();
    Serial.println();
    lastReadTime = millis();
  }
  if(millis() - lastPrintTime > printInterval) {
    lastPrintTime = millis();
  }
  if(millis() - lastPulse > pulseInterval) {
    digitalWrite(LED_PIN, HIGH);
    lastPulse = millis();
  }
  if(millis() - lastPulse > pulseLength) {
    digitalWrite(LED_PIN, LOW);
  }
  // readPulse();
  // readAlcohol();
  // readAcceleration();
  // Serial.println();


  // delay(1000);
}

void readPulse() {
    Wire.requestFrom(0xA0 >> 1, 1);   // request 1 bytes from slave device
  while (Wire.available()) {        // slave may send less than requested
    unsigned char c = Wire.read();  // receive heart rate value (a byte)
    Serial.println(c, DEC);         // print heart rate value
    pulseInterval = 60000 / c;
  }
}

void readAlcohol() {
    int sensorValue;
  sensorValue = analogRead(ALCOHOL_DAT);  //read the analog value
  int value = 1023 - sensorValue;
  //Disply the results in serial monitor.
  Serial.print("sensor test value = ");
  //sensorValue goes down when alcohol is detected. Hence subtracting from 1023.
  Serial.println(value);
  /*The information below is recommended result of the judgment*/
  if (value < 200) {
    Serial.println("No alcohol vapor detected");
  } else if ((value >= 200) && (value < 600)) {
    Serial.println("Could be alcohol idfk");
  } else if ((value > 600) && (value < 750)) {
    Serial.println("High Concentration of Alcohol detected");
  } else {
    Serial.println("Very high Concentration of Alcohol detected");
  }
}

void readAcceleration() {
  int x,y,z;  
	adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z

	double xyz[3];
	double ax,ay,az;
  double total;
	adxl.getAcceleration(xyz);
	ax = xyz[0];
	ay = xyz[1];
	az = xyz[2];
    total = sqrt(ax*ax+ay*ay+az*az);

    if (total > 0.93) {
        Serial.print("Acceleration: ");
        Serial.println(total);
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