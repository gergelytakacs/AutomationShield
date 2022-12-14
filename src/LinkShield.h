/*
  API for the LinkShield didactic hardware.

  The file is a part of the application programmers interface for
  the LinkShield didactic tool for control engineering and
  mechatronics education. The LinkShield implements an a flexible
  rotational link experiment on an Arduino shield.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Martin Vríčan.
  Last update: 20.5.2020.
*/

#ifndef LINKSHIELD_H                     //Include guard
#define LINKSHIELD_H

//include necessary libraries                      
#include <Arduino.h>                
#include <Servo.h>
#include "AutomationShield.h"


#include <Wire.h>  

//Defining pins used by LinkShield
#define LINK_RPIN 0         		//Potentiometer pin
#define LINK_FLEX_PIN 1         	//FlexSensor pin
#define LINK_SERVO_ANGLE_PIN 2      //Servo feedback pin
#define LINK_UPIN 9         		//Servo control pin


#define ADXL345 0x53        //Define default address for ADXL345 sensor

class LinkClass {                         //Creating class
  public:
    void  begin(void);                  //initialization function 
	void  calibrate();                  //Calibration function
	void  actuatorWrite(float);         //u(t)
	
    float referenceRead();              //r(t)
    float flexRead();                  	//y(t)
	float servoRead();
   

  private:
  
    int   _referenceRead;                 
    float _referenceValue;	
	int   _flexRead;                 
    float _flexValue;
    int   _servoRead;                 
    float _servoValue;	

    float _flexSum = 0;
    float _flexBias;
	int   _zeroBendValue = 0;
   
    void  ADXL_POWER_CTL();               //Set ADXL345 to measure mode
    void  ADXL345_BW_RATE();              //Set ADXL345 data and bandwidth rate
    void  ADXL_DATA_FORMAT();             //ADXL345 output data formatting
    void  ADXL345_OFSZ();                 //Set offset to Z axis
    float ADXL345_DATAZ();                //Read Z accel
	float sensorRead();
   
    float _accelZ;
    float _angle;
    float _sensorBias;                    // Sensor bias from 0
    float _Z_out;
    float flexBias(int);                // Takes N measurements and computes average

};
LinkClass LinkShield;
Servo myservo;

// declaring PIN and initializing sensor library
void LinkClass::begin() {
  myservo.attach(LINK_UPIN, 1000, 2000);    // Set Servo pin and PWM range
  
  #ifdef ARDUINO_ARCH_AVR
  	Wire.begin();	// Starts the "Wire" library for I2C
	analogReference(EXTERNAL); // Set reference voltage (3v3)
  #elif ARDUINO_ARCH_SAM
	//analogReadResolution(12);
	Wire1.begin(); // Initialize I2C communication
  #elif ARDUINO_ARCH_SAMD
        //analogReadResolution(12);
	Wire.begin(); // Initialize I2C communication
  #endif

  /* 
  LinkShield.ADXL_POWER_CTL();
  LinkShield.ADXL_DATA_FORMAT();
  LinkShield.ADXL345_BW_RATE(); 
  */
}

void LinkClass::calibrate() {
  AutomationShield.serialPrint("Calibration...");
  LinkShield.actuatorWrite(45.0);                    // Go to middle and wait
  delay(500);
  _zeroBendValue = flexBias(1000);
  
  
  
  
/*   _sensorBias = -(LinkShield.sensorBias(1000)/4);   // Calculate offset to LSB/g
  ADXL345_OFSZ();  */                                  
  AutomationShield.serialPrint(" successful.\n");
}

float LinkClass::flexBias(int testLength) {
  for (int i = 0; i < testLength; i++ ) {                 //Make N measurements
    _flexSum  = _flexSum + analogRead(LINK_FLEX_PIN);    
  }
  return _flexSum / testLength;                         //Compute average
}

//values from potentiometer in degrees , for fututre use
float LinkClass::referenceRead() {
  _referenceRead = analogRead(LINK_RPIN) - _zeroBendValue;
  _referenceValue = AutomationShield.mapFloat((float)_referenceRead, 0.00, 1023.00, 0.00, 90.00);
  return _referenceValue;
}

float LinkClass::flexRead() {
  _flexRead = analogRead(LINK_FLEX_PIN);
  _flexValue = AutomationShield.mapFloat((float)_flexRead, 0.00, 1023.00, 0.00, 90.00);
  return _flexValue;
}

float LinkClass::servoRead() {
  _servoRead = analogRead(LINK_SERVO_ANGLE_PIN);
  _servoValue = AutomationShield.mapFloat((float)_servoRead, 0.00, 1023.00, 0.00, 90.00);
  return _servoValue;
}

void LinkClass::actuatorWrite(float _angle) {		//Turn servo to desired angle
  int _modAngle = map((int)_angle, 0, 90, 0, 180);
  myservo.write(_modAngle);
}

float LinkClass::sensorRead() {			//Execute reading from sensor
  _accelZ = ADXL345_DATAZ();
  return _accelZ;
}

void LinkClass::ADXL_POWER_CTL() {    	//Power-saving features control
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);
  Wire.write(0b1000);
  Wire.endTransmission();
  delay(10);
}

void LinkClass::ADXL_DATA_FORMAT() {    //Data format function - Range, justify, etc...
  Wire.beginTransmission(ADXL345);
  Wire.write(0x31);
  Wire.write(0b1101);
  Wire.endTransmission();
  delay(10);
}

void LinkClass::ADXL345_BW_RATE() {     //BandWidth and data rate setup - 1600Hz
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2C);
  Wire.write(0b1111);
  Wire.endTransmission();
  delay(10);
}

void LinkClass::ADXL345_OFSZ() {        //Offset calibration
  Wire.beginTransmission(ADXL345);
  Wire.write(0x20);
  Wire.write((int)(_sensorBias));
  Wire.endTransmission();
  delay(10);
}

float LinkClass::ADXL345_DATAZ() {      //Reading of Z-axis value(other axis not needed)
  Wire.beginTransmission(ADXL345);
  Wire.write(0x36); // Start with register 0x36 (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 2, true);
  _Z_out = ( Wire.read() | Wire.read() << 8)/4096; // Z-axis value
  return _Z_out;
}

#endif
