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
    void initialize();
    float referenceRead();    //r(t)
    void actuatorWrite(float);          //y(t)
  float sensorRead();  
  private:
    int _referenceRead;
    float _referenceValue;
    float _accelZ;
    float _angle;
};
#endif


extern LinkClass LinkShield;
Servo myservo;


// declaring PIN and initializing sensor library
void LinkClass::begin() {
  myservo.attach(LINK_UPIN);      // set Servo pin
  analogReference(EXTERNAL);
}

void LinkClass::initialize() {
 # if ECHO_TO_SERIAL 
  Serial.println("Accelerometer Test"); Serial.println("");
   # endif 
  /* Initialise the sensor */
  if (!accel.begin())
  {
    # if ECHO_TO_SERIAL 
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
    # endif 
  }
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_4_G);
}

float LinkClass::sensorRead() {
  sensors_event_t event;
  accel.getEvent(&event);
  _accelZ = event.acceleration.z
           return _accelZ;
}

//values from potentiometer in degrees , for fututre use
float LinkClass::referenceRead() {
  _referenceRead = analogRead(LINK_RPIN);
  _referenceValue = AutomationShield.mapFloat((float)_referenceRead, 0.00, 1023.00, 0.00, 180.00);
  return _referenceValue;
}

void LinkClass::actuatorWrite(float _angle){
   myservo.write((int)angle);
  }
