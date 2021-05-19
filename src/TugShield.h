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
  Last update: 5.5.2021.
*/

#ifndef TUGSHIELD_H			// Include guard
#define TUGSHIELD_H			

#include <Arduino.h>                
#include <Servo.h>
#include "AutomationShield.h"
#include "PID.h"
#include "PIDAbs.h"
#include "PIDInc.h"

// Defining pins and constants used by the Tugshield 
#define TUG_YPIN 0  			// Flexi sensor
#define TUG_UPIN 10 			// Servo motor
#define K_MIN 675				// Initial value for variable K_MIN
#define K_MAX 0					// Initial value for variable K_MAX
#define DELAY_VALUE 500			// Initial value for variable delay_value
#define SERVO_MAX_FLEX 0
#define SERVO_MIN_FLEX 180
#define DEFAULT_MIN 0.0
#define DEFAULT_MAX 675.0

class TugShieldClass 						// Class for the TugShield device
{			
  public:
      void begin();							// Begin method
      void calibration();					// Calibration method
	  void actuatorWrite(int servo_angle);	// Write value to servo method
	  float sensorRead();					// Read value from flexi method
  private:   
	  bool  _wasCalibrated; 				// Initializing the variable _wasCalibrated
	  int   _sensorRead ;
	  int   k_maximal;
	  int   k_minimal;
};
 TugShieldClass TugShield;
 Servo servo;

// Hardware initialization
void TugShieldClass::begin()
{  												
    servo.attach(TUG_UPIN);						// Servo pin setting
	servo.write(SERVO_MIN_FLEX);				// Set the servo position to zero

}

// Calibration
void TugShieldClass::calibration()				
{ 
  Serial.println("Begin calibration");		
  servo.write(SERVO_MIN_FLEX);					// Set the servo position to zero
  delay(DELAY_VALUE);								
  for (int ii=0; ii <= 10; ii++)				// 11 x	repeat
  {
    _sensorRead=analogRead(TUG_YPIN);			// Read value from flexi sensor
	if (ii == 00)								// First repeat
	{
		if(_sensorRead<K_MIN)					// Comparison of measured value with determined value		
		{
			k_minimal=_sensorRead;					// Value assignment
		}
		delay(DELAY_VALUE);
	}
	else
	{
		if(_sensorRead<k_minimal)				// Comparison of k_minimal with the measured value		
		{
			k_minimal=_sensorRead;					// Value assignment
		}
		delay(DELAY_VALUE);
	}
  }
  
  servo.write(SERVO_MAX_FLEX);					// Set the servo position to 180 degrees
  delay(DELAY_VALUE);								
  for (int ii=0; ii <= 10; ii++)				// 11 x	repeat	
  {
    _sensorRead=analogRead(TUG_YPIN);			// Read value from flexi sensor
	if (ii == 00)								// First repeat
	{
		if(_sensorRead>K_MAX)					// Comparison of measured value with determined value			
		{			
			k_maximal=_sensorRead;					// Value assignment
		}
		delay(DELAY_VALUE);
	}
	else
	{
		if(_sensorRead>k_maximal)				// Comparison of k_maximal with the measured value				
		{
		k_maximal=_sensorRead;					// Value assignment
		}
		delay(DELAY_VALUE);
	}
  }
  _wasCalibrated = true; 							
	Serial.println("TugShield calibrated.");
}

void TugShieldClass::actuatorWrite(int servo_angle){  
     int real_servo_angle = 180 - servo_angle;           // Count real angle of servo
	 servo.write(real_servo_angle);  		 			// Set angle on servo
}

float TugShieldClass::sensorRead()
{
    _sensorRead = analogRead(TUG_YPIN);					// read senzor value
    float _sensorValue;
    if(_wasCalibrated == true)
	{                  										// Remap in case the calibration has been done
      _sensorValue = AutomationShield.mapFloat((float)_sensorRead,(float)k_minimal,(float)k_maximal, 0.00, 100.00); 
    }
    else													// Remap in case the calibration has not been done
	{ 
      _sensorValue = AutomationShield.mapFloat((float)_sensorRead, DEFAULT_MIN, DEFAULT_MAX, 0.00, 100.00);
    }
    return _sensorValue;
}
#endif
