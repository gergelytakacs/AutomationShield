/*
  API for the TugShield hardware.
  
  The file is a part of the application programming interface for
  the TugShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Eva Vargová. 
  Last update: 13.5.2019.
*/

#ifndef TUGSHIELD_H_			// Include guard
#define TUGSHIELD_H_			

#include "Arduino.h"			// Required Arduino API in libraries
#include "AutomationShield.h"
#include "Servo.h"


// Defining pins used by the Tugshield 
#define TUG_YPIN 0  			// Flexi sensor
#define TUG_UPIN 10 			// Servo motor
#define K_MIN 800;				// Initial value for variable K_MIN
#define K_MAX 0;				// Initial value for variable k_max
#define DELAY_VALUE 100;		// Initial value for variable delay_value
#define SERVO_MAX 180;
#define SERVO_MIN 0;

class TugShieldClass{			// Class for the TugShield device
  
  public:
    void begin();				// Begin metod
    void calibration();			// Calibration metod
	void actuatorWrite();		// Write value to servo metod
	void sensorRead();			// Read value from flexi metod
  private:   
	bool  _wasCalibrated; 	// Initializing the variable _wasCalibrated
};

extern TugShieldClass TugShield;

#endif