#ifndef BOBSHIELD_H_
#define BOBSHIELD_H_

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
#define MAX_CALIBRATED_DEFAULT 10           //TODO 
#define MIN_CALIBRATED_DEFAULT 100          //TODO

class BOBClass {
  public:
    Adafruit_VL6180X sens = Adafruit_VL6180X();   	  //symbolic name for sensor library
    void  begin(void);                      		  //sets up sensor
    void calibration();		                 	  //calibration
    float referenceRead();		          	  // Read reference pot in %
    float sensorReadPerc();
    float sensorRead();
    void actuatorWrite(float degree);


  private:

      float _referenceRead;
      float _referenceValue;
      uint8_t range;
      float _servoValue;
      int pos;
      byte calibrated = 0;
      int minCalibrated;			// Actual minimum value
      int maxCalibrated;
      int minimum ;
      int maximum ;

  };
extern BOBClass BOBShield;

#endif
