/*
  API for the HeatShield hardware.
  
  The file is a part of the application programming interface for
  the HeatShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Richard Köplinger. 
  Last update: 10.10.2018.
*/

#ifndef TUGSHIELD_H_			// Include guard
#define TUGSHIELD_H_			

#include "Arduino.h"			// Required Arduino API in libraries



// Defining pins used by the Tugshield 
#define TUG_YPIN 0  			// Flexi sensor
#define TUG_UPIN 10 			// Servo motor
#define K_MIN = 800;			// Initial value for variable k_min
#define K_MAX = 0;				// Initial value for variable k_max



class TugShieldClass{			// Class for the TugShield device
  
  public:
    void begin();
    void calibration();
	void actuatorWrite();
	void sensorRead();
  private:   

};

extern TugShieldClass TugShield;

#endif