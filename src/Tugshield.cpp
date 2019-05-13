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

#include "TugShield.h"

// Initializes hardware pins
void TugShieldClass::begin(){  
      pinMode(TUG_YPIN, INPUT);  		// Flexi pin 
	  pinMode(TUG_UPIN, OUTPUT); 		// Servo pin 
	  pinMode(POT_YPIN, INPUT); 		// Potenciometer pin
	  servo.attach(TUG_UPIN);			// Set pin for servo
	  servo.write(servo_min);					// Set zero position 
}

// Calibration
void TugShieldClass::calibration()				
{  # if ECHO_TO_SERIAL                        	
	Serial.println("Calibration is running...");	// Calibration is in progress
  # endif
  servo.write(servo_min);									// Write 0 to servo 
  delay(delay_value);								
  for (int ii=0; ii <= 10; ii++)					
  {
    _sensorRead=analogRead(0);						// Read value from flexi
    if(_sensorRead<k_min)							
    {
      k_min=_sensorRead;							// Checking the lowest value
    }
    delay(delay_value);
  }
  servo.write(servo_max);									// Write 180 to servo 
  delay(delay_value);								
  for (ii=0; ii <= 10; ii++)						
  {
    _sensorRead=analogRead(0);						// Read value from flexi
    if(_sensorRead>k_max)							//
    {
      k_max=_sensorRead;							// Checking the highest value
    }
   delay(delay_value);
  }
  _wasCalibrated = true; 							
	# if ECHO_TO_SERIAL
	Serial.println("Calibration is done");			// Calibration is completed	
  # endif
}
void TugShieldClass::actuatorWrite(write_to_servo){  
    servo.write(write_to_servo);  		 			// Write to actuator
}

float TugShieldClass::sensorRead(){
    _sensorRead = analogRead(TUG_YPIN);				// Read value from flexi
   
   if(_wasCalibrated){                  			// Was calibration done?
   _sensorValue = AutomationShield.mapFloat(_sensorRead, k_min, k_max, 0.00, 100.00); 
   }
   else{ // Use default typical values
   _sensorValue = AutomationShield.mapFloat(_sensorRead, 20.00, 800.00, 0.00, 100.00);
   }
	return _sensorValue;
   
}
TugShieldClass TugShield;