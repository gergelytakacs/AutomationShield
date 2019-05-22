#ifndef BOBSHIELD_H_
#define BOBSHIELD_H_

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
extern Servo myservo;
#include "lib/Adafruit_VL6180X/Adafruit_VL6180X.h"
#include "AutomationShield.h"


/* #ifndef ADAFRUIT_VL6180X_H 				// If library not installed somewhere
#if __has_include("src/lib/Adafruit_VL6180X/Adafruit_VL6180X.h")  // If library present from GIT
	#include "src/lib/Adafruit_VL6180X/Adafruit_VL6180X.h" // Include it from there
#endif
#endif*/

#define BOB_RPIN 0
#define BOB_UPIN 9



class BOBClass {
  public:
    Adafruit_VL6180X sens =  Adafruit_VL6180X();   	  //symbolic name for sensor library
    void initialize();                  // sets up the sensor - required to run in setup!
    void  begin(void);                      		  //sets up sensor
    void calibration();		                 	  //calibration
    float referenceRead();		          	  // Read reference pot in %
    float sensorReadPerc();
    float sensorRead();
    float sensorReadCal();
    int degree;
    void actuatorWrite(int deg);
	
  private:
      float _referenceRead;
      float _referenceValue;
      uint8_t range;
      float _servoValue;
      int pos,posCal, position, posperc;
      int maxRange, minRange, ballPos;
      byte calibrated = 0;
      int minCalibrated = 1488;			// Actual minimum value
      int maxCalibrated = 14; 			// Actual maximum value
      int minimum ;
      int maximum ;
  };
extern BOBClass BOBShield;

#endif

