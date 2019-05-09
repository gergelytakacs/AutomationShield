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
  Last update: 6.5.2019.
*/

#include "TugShield.h"
#include "AutomationShield.h"
#include "Servo.h"
// Initializes hardware pins
void TugShieldClass::begin(){  
      pinMode(TUG_YPIN, INPUT);  		// Flexi pin
      pinMode(TUG_UPIN, OUTPUT); 		// Servo pin
	  servo.attach(TUG_UPIN);			// Set pin for servo
	  servo.write(0);					// Set zero position 
}

// Calibration
void TugShieldClass::calibration()
{  # if ECHO_TO_SERIAL                        
	Serial.println("Calibration is running...");
  # endif
  servo.write(0);
  delay(1000);
  for (int ii=0; ii <= 10; ii++)
  {
    _sensorRead=analogRead(0);
    if(_sensorRead<k_min)
    {
      k_min=_sensorRead;
    }
    delay(100);
  }
  servo.write(180);
  delay(1000);
  for (ii=0; ii <= 10; ii++)
  {
    _sensorRead=analogRead(0);
    if(_sensorRead>k_max)
    {
      k_max=_sensorRead;
    }
   delay(100);
  }
	# if ECHO_TO_SERIAL
	Serial.println("Calibration is done");		
  # endif
}
void TugShieldClass::actuatorWrite(bbb){  
      analogWrite(HEAT_UPIN,bbb); // Write to actuator
}

float TugShieldClass::sensorRead(){
   _sensorRead = analogRead(TUG_YPIN);
   _sensorValue = AutomationShield.mapFloat(_sensorRead, k_min, k_max, 0.00, 100.00); 
   return _sensorValue;
}
TugShieldClass TugShield;