/*
  API for the BiCopShield hardware.
  
  The file is a part of the application programming interface for
  the BiCopShield didactic tool for control engineering and 
  mechatronics education. The BiCopShield implements an air-based,
  pendulum angle positioning experiment on an Arduino microcontroller.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Nemcek, Jan Boldocky
  Last update: 20.5.2024
*/

// Defining c++ library used by the BiCopShield

#ifndef BICOPSHIELD_H			       // Include guard
#define BICOPSHIELD_H	

// Defining libraries used by the BiCopShield 
#include "AutomationShield.h"    // Include the main library
#include <Wire.h>                // Include I2C protocol library
#include <Arduino.h>			 // Required Arduino API in libraries
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h" // Basic Linear Algebra library		
#include "AS5600_AS.h"			 // Include Hall sensor library
#include "lib/Adafruit_VL6180X/Adafruit_VL6180X.h"	// Include TOF sensor library


#ifndef SHIELDRELEASE
  #define SHIELDRELEASE 1   					//  Use number only: e.g. for R3 is 3
#endif

// Defining pins used by the BiCopShield 
#define BICOP_RPIN A3				// Input from potentiometer
#define BICOP_VOLTAGE_SENSOR_PIN A2    // Input pin for measuring Volt
#define BICOP_UPIN1 5				// Motor (Actuator)
#define BICOP_UPIN2 6				// Motor 2 (Actuator)
#define MIN_CALIBRATED_DEFAULT 7	// Minimum ball distance
#define ZERO_COMPENSATION 0
class BicopClass{		    	                           // Class for the BiCopShield device
 
 public:
 #include "getGainLQ.inl"
 #include "getKalmanEstimate.inl"
   float begin(void);                     // Board initialisation - initialisation of pin modes and variables                             
   void actuatorWrite(float inputPercentageM1, float inputPercentageM2);             // Write actuator - function takes input 0.0-100.0% and sets motor speed accordingly
   void actuatorWriteVolt(float inputVoltageM1, float inputVoltageM2);
   bool calibrate(void);                 // sensor calibration
   float referenceRead(void);                        // Read value of potentiometer and converts it to percentual value 

   uint16_t sensorReadDegreeAbsolute();
   float sensorRead(void);
   float sensorReadDegree(void);           // Measure pendulum angle in degrees
   float sensorReadRadian(void);     // Measure pendulum angle in radians   
   word getRawAngle();              // AS5600 angle value 
   // BoB
   Adafruit_VL6180X sens = Adafruit_VL6180X();   	  //symbolic name for sensor library
   void TOFinitialize();                                // sets up the TOF sensor - required to run in setup!
   void TOFcalibration();
   float TOFsensorReadPerc();
   float TOFsensorRead();
   float deg2rad(float u);
   float rad2deg(float u);

 private: 
   //int startAngle = -1;                                 // Variable for storing zero angle position 
   bool wasCalibrated = false; 
   
   int _ams5600_Address = 0x36;						      // AS5600 address
   int _stat = 0x0b;										   // AS5600 communication variable 
   int _raw_ang_hi = 0x0c;								   // AS5600 communication variable 
   int _raw_ang_lo = 0x0d;								   // AS5600 communication variable

   float _min_angle = 10;                            // minimal measured angle in negative direction (relative to zero position)
   float _max_angle = 280;                           // maximal measured angle in positive direction (relative to zero position)

   //BoB private
   float _referenceRead;
   float _referenceValue;
   uint8_t range;
   float _servoValue, pos, posCal, position, ballPos;
   int  posperc;
   float maxRange, minRange;
   byte calibrated = 0;
   float minCalibrated = 0;			                // Actual minimum value
   float maxCalibrated = 100; 		                	// Actual maximum value
   float minimum;
   float maximum;
   float deg;
   //BoB end

   AS5600 as5600;
};
extern BicopClass BicopShield; 

#endif