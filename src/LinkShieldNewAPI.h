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
  Last update: 21.2.2020.
*/

#ifndef LINKSHIELD_H_                     //Include guard
#define LINKSHIELD_H_

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include "AutomationShield.h"

//Defining pins used by LinkShield
#define LINK_RPIN 0   //Potentiometer pin
#define LINK_UPIN 9   //Servo pin

#define ADXL345 0x53  //Define default addres for ADXL345 sensor

class LinkClass {
  public:
    void  begin(void);
    float referenceRead();    //r(t)
    void  actuatorWrite(float);          //y(t)
    float sensorRead();  
    void calibrate();
    void ADXL345_BW_RATE();
    void ADXL_DATA_FORMAT();
    void ADXL345_OFSZ();
    float ADXL345_DATAZ();
  
  private:
    int _referenceRead;
    float _referenceValue;
    float _accelZ;
    float _angle;
    float _sensorBias;                   // Sensor bias from 0
    bool _wasCalibrated;                 // Variable for storing calibration status
    float _Z_out;
    
    float sensorBias(int); // Takes N measurements and computes average

};
#endif

extern LinkClass LinkShield;
Servo myservo;

// declaring PIN and initializing sensor library
void LinkClass::begin() {
  myservo.attach(LINK_UPIN,1000,2000);      // set Servo pin
  analogReference(EXTERNAL);
  Wire.begin();
 ADXL_DATA_FORMAT();
ADXL345_BW_RATE();
}


float LinkClass::sensorBias(int testLength){
  float _accelZsum = 0;  
  for (int i=0; i<testLength; i++ ){
    _accelZsum=_accelZsum+LinkShield.sensorRead();
  }
  return (_accelZsum/testLength)*256;
}

void LinkClass::calibrate(){
  AutomationShield.serialPrint("Calibration...");   
  
  LinkShield.actuatorWrite(0.0);          // Go to zero and wait
  delay(500);
      
  _sensorBias=LinkShield.sensorBias(1000);
   ADXL345_OFSZ();                               // Sets the status of calibration to true
  AutomationShield.serialPrint(" sucessful.\n");
  
  }


float LinkClass::sensorRead() {
_accelZ = ADXL345_DATAZ();
  return _accelZ;
}

//values from potentiometer in degrees , for fututre use
float LinkClass::referenceRead() {
  _referenceRead = analogRead(LINK_RPIN);
  _referenceValue = AutomationShield.mapFloat((float)_referenceRead, 0.00, 1023.00, 0.00, 180.00);
  return _referenceValue;
}

void LinkClass::actuatorWrite(float _angle){
  int _modAngle=map((int)_angle,0,90,0,180);
   myservo.write(_modAngle);
  }

  
float LinkClass::ADXL345_DATAZ(){  //Reading of Z-axis value(other axis not needed)
  Wire.beginTransmission(ADXL345);
  Wire.write(0x36); // Start with register 0x36 (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 2, true);
  _Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
  _Z_out = _Z_out/256; //Convert to G-value
  return _Z_out;
}


void LinkClass::ADXL345_BW_RATE(){ //BandWidth rate setup - 1600Hz
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2C); 
  Wire.write(0b1111);
  Wire.endTransmission();
  delay(10);
}
  
  void LinkClass::ADXL345_OFSZ(){ //Offset calibration
  Wire.beginTransmission(ADXL345);
  Wire.write(0x20); 
  Wire.write(0);//((int)-_sensorBias); //Offset set to 0 yet
  Wire.endTransmission();
  delay(10); 
}


void LinkClass::ADXL_DATA_FORMAT(){ //Data format function - Range, justify, etc...
  Wire.beginTransmission(ADXL345);
  Wire.write(0x31); 
  Wire.write(0b1101);
  Wire.endTransmission();
  delay(10);
  
}

LinkClass LinkShield;
