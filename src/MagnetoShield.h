/*
  API for the MagnetoShield didactic hardware.
  
  The file is a part of the application programmers interface for
  the MagnetoShield didactic tool for control engineering and 
  mechatronics education. The MagnetoShield implements a magnetic
  levitation experiment on an Arduino shield.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Jakub Mihalík. 
  Last update: 14.11.2018.
*/

#ifndef MAGNETOSHIELD_H							// Include guard
#define MAGNETOSHIELD_H

#include "AutomationShield.h"           		// Includes the core header
#include "Wire.h" 								// I2C library for the DAC chip
#define VIN 12.0 							    // Input voltage (Assumed)
#define PCF8591 (0x90 >> 1)			    		// Address of the DAC chip
#define MAGNETO_YPIN A3							// Defines the location of the Hall sensor 
#define A1302_SENSITIVITY	1.3					// [mV/G] Sensitivity of the Hall sensor
class MagnetoShieldClass						// Class for MagnetoShield API
{
public:
	void begin(); 								// Initializes "Wire" library
	void dacWrite(byte dacIn);			    	// Writes dacIn to DAC chip
	void calibration();							// Finds out lowest and highest possitions of magnet
	void actuatorWrite(float u);         		// Sets actuation voltage (not actual Voltage)
	void actuatorWritePercents(float u);      // Sets output power in percent
	float readSensor(); 						// Returns current "position" in percent
	float readSensorPercents(); 						// Returns current "position" in percent
    float readSensorGauss(); 				    // Returns current Hall sensor reading in Gauss
	
	float adcToGauss(short adc);					// Computes Gauss from the ADC of the Hall sensor
	// Get functions to extract private variables
	int getMinCalibrated();						// Returns the minimum calibrated value for the magnetic field (10-bit ADC levels).
	int getMaxCalibrated();						// Returns the maximum calibrated value for the magnetic field (10-bit ADC levels).
	byte getSaturation();		                // Returns the saturation level of the magnet (8-bit DAC levels).
	
private:							
	short minCalibrated = 0;					// ADC on Hall for ground
	short maxCalibrated = 1024;					// ADC on Hall for ceiling
	byte  dacSaturate = 255;					// Saturation level of the DAC	
};

extern MagnetoShieldClass MagnetoShield;		//declare external instance
#endif