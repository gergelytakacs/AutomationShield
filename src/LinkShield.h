#ifndef LINKSHIELD_H_                     //Include guard
#define LINKSHIELD_H_

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "AutomationShield.h"

//Defining pins used by LinkShield
#define LINK_RPIN 0   //Potentiometer pin
#define LINK_UPIN 9   //Servo pin

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

class LinkClass {
  public:
    void  begin(void);
    float referenceRead();    //r(t)
    void  actuatorWrite(float);          //y(t)
    float sensorRead();  
    

    void calibrate();
  
  private:
    int _referenceRead;
    float _referenceValue;
    float _accelZ;
    float _angle;
    float _sensorBias;                   // Sensor bias from 0
    bool _wasCalibrated;                 // Variable for storing calibration status

    float sensorBias(int); // Takes N measurements and computes average

};
#endif

extern LinkClass LinkShield;
Servo myservo;

// declaring PIN and initializing sensor library
void LinkClass::begin() {
  myservo.attach(LINK_UPIN);      // set Servo pin
  analogReference(EXTERNAL);
  AutomationShield.serialPrint("Accelerometer Test. \n");

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    AutomationShield.serialPrint("No accelerometer Test.");
    while (1);  
  }
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_4_G);
}


float LinkClass::sensorBias(int testLength){
  float _accelZsum = 0;  
  for (int i=0; i<testLength; i++ ){
    _accelZsum=_accelZsum+LinkShield.sensorRead();
  }
  return _accelZsum/testLength;
}

void LinkClass::calibrate(){
  AutomationShield.serialPrint("Calibration...");   
  
  LinkShield.actuatorWrite(0.0);          // Go to zero and wait
  delay(500);
      
  _sensorBias=LinkShield.sensorBias(1000);
  _wasCalibrated = true;                                 // Sets the status of calibration to true
  AutomationShield.serialPrint(" sucessful.\n");
  
  }


float LinkClass::sensorRead() {
  sensors_event_t event;
  accel.getEvent(&event);
  _accelZ = event.acceleration.z;
  if(_wasCalibrated){
    _accelZ=_accelZ-_sensorBias;
    }
  return _accelZ;
}

//values from potentiometer in degrees , for fututre use
float LinkClass::referenceRead() {
  _referenceRead = analogRead(LINK_RPIN);
  _referenceValue = AutomationShield.mapFloat((float)_referenceRead, 0.00, 1023.00, 0.00, 180.00);
  return _referenceValue;
}

void LinkClass::actuatorWrite(float _angle){
   myservo.write((int)_angle);
  }

LinkClass LinkShield;