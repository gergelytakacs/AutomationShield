/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education. The AeroShield implements an air-based,
  pendulum angle positioning experiment on an Arduino microcontroller.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Peter Tibenský, Erik Mikuláš and Ján Boldocký
  Last update: 09.06.2023
*/

// Defining c++ library used by the AeroShield 
#ifndef AEROSHIELD_H			       // Include guard
#define AEROSHIELD_H	

// Defining libraries used by the AeroShield 
#include "AutomationShield.h"    // Include the main library
#include <Wire.h>                // Include I2C protocol library
#include <Arduino.h>			 // Required Arduino API in libraries
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "AS5600_AS.h"

#ifndef SHIELDRELEASE
  #define SHIELDRELEASE 3   					//  Use number only: e.g. for R3 is 3
#endif

// Defining pins used by the AeroShield 
#define AERO_RPIN A3             // Input from potentiometer
#define AERO_VOLTAGE_SENSOR_PIN A2    // Input pin for measuring Vout
#define AERO_UPIN 5              // Motor (Actuator)

class AeroClass{		    	                           // Class for the AeroShield device
 
 public:
 #include "getGainLQ.inl"
 #include "getKalmanEstimate.inl"
   void begin(void);                     // Board initialisation - initialisation of pin modes and variables                             
   void actuatorWrite(float PotPercent);             // Write actuator - function takes input 0.0-100.0% and sets motor speed accordingly
   void actuatorWriteVolt(float);
   bool calibrate(void);                 // sensor calibration
   float referenceRead(void);                        // Read value of potentiometer and converts it to percentual value 

   float sensorRead(float maximumDegrees = 90.0);
   float sensorReadDegree(void);           // Measure pendulum angle in degrees
   float sensorReadRadian(void);     // Measure pendulum angle in radians   
   word getRawAngle();              // AS5600 angle value 

   #if SHIELDRELEASE == 2 							         
      float sensorReadCurrent(void);		  				      // Read current from actuator 
      uint16_t sensorReadCurrentRaw(void);
   #endif
   
 private: 
   //int startAngle = -1;                                 // Variable for storing zero angle position 
   bool wasCalibrated = false;
   float ShuntRes = 0.1;						   	      // Value of shunt resistor in Ohms 
   
   int _ams5600_Address = 0x36;						      // AS5600 address
   int _stat = 0x0b;										   // AS5600 communication variable 
   int _raw_ang_hi = 0x0c;								   // AS5600 communication variable 
   int _raw_ang_lo = 0x0d;								   // AS5600 communication variable

   float _min_angle = 60;                            // minimal measured angle in negative direction (relative to zero position)
   float _max_angle = 210;                           // maximal measured angle in positive direction (relative to zero position)

   AS5600 as5600;
};
extern AeroClass AeroShield; 

#endif
