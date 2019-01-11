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

#ifndef SHIELDRELEASE
	#define SHIELDRELEASE 2   					//  Use number only: e.g. for R2 is 2
#endif

#define VIN 11.7 							    // [V] Input voltage from source
#define EMAGNET_HEIGHT 20.0				        // [mm] Location of electromagnet above ground
#define PCF8591 (0x90 >> 1)			    		// Address of the DAC chip
#define MAGNETO_YPIN A3							// Defines the location of the Hall sensor 
#define A1302_SENSITIVITY	769.23	    		// [G/V] = 1.3 mV/G Sensitivity of the A1302 Hall sensor
#define LOAD_RESISTANCE	196.6					// [Ohm] Load resistance
#define LOAD_HSAT	204							// [8-bit DAC] Upper saturation of the magnets before dropping
#define IRF520_LSAT	170							// [8-bit DAC] Lower saturation of the IRF520
#define IRF520_HSAT	220							// [8-bit DAC] Hight (upper) saturation of the IRF520	

#if SHIELDRELEASE == 1	
	#define A1302_LSAT	19							// [10-bit ADC] Lower saturation of the Hall sensor
	#define A1302_HSAT 382							// [10-bit ADC] Higher (upper) saturation of the Hall sensor
#elif SHIELDRELEASE == 2
	#define A1302_LSAT	29							// [10-bit ADC] Lower saturation of the Hall sensor
	#define A1302_HSAT 577							// [10-bit ADC] Higher (upper) saturation of the Hall sensor
#endif

// Functionality specific for R2
#if SHIELDRELEASE == 2
    #define MAGNETO_RPIN A0                        // Defines the location of reference pot
	#define MAGNETO_VPIN A1						   // Defines the location of input voltage sensing
	#define MAGNETO_IPIN A2						   // Defines the location of input current sensing
	#define VGAIN 4								   // Defines the voltage sensing gain (voltage divider)
    #define IGAIN 33.33333333 					   // Defines the current sensing gain mA/V
#endif

// Power model of the input-output voltage DAC->Vout
#define P1 0.01131							    // Power function constant (f(y) = P1*x^P2+P3*x^P4+P5) for DAC vs. Output voltage
#define P2 2.554								// Power function constant (f(y) = P1*x^P2+P3*x^P4+P5) for DAC vs. Output voltage
#define P3 -447.4							    // Power function constant (f(y) = P1*x^P2+P3*x^P4+P5) for DAC vs. Output voltage
#define P4 -0.01247								// Power function constant (f(y) = P1*x^P2+P3*x^P4+P5) for DAC vs. Output voltage
#define P5 638.6								// Power function constant (f(y) = P1*x^P2+P3*x^P4+P5) for DAC vs. Output voltage

// Distance model based on magnetic flux density
// As it is hard to make exact measurements a two-point 
// calibration of a power function seems to work best
#define P6 470.6 								//Distance function constant (f(y) = P6*x^P7) for Flux vs. distance from magnet
#define P7 -0.6719								// Distance function constant (f(y) = P6*x^P7) for Flux vs. distance from magnet

class MagnetoShieldClass						// Class for MagnetoShield API
{
public:
	void begin(); 								// Initializes "Wire" library
	void dacWrite(byte dacIn);			    	// Writes dacIn to DAC chip
    byte voltageToDac(float vOut);		   		// Computes DAC levels for magnet voltage
	void calibration();							// Finds out lowest and highest positions of magnet
	void actuatorWrite(float u);         		// Default actuator handling method
	void actuatorWriteVoltage(float u);	   		// Sends voltage to magnet
	void actuatorWritePercents(float u);        // Sets output voltage in percent
	float sensorRead(); 						// Returns current "position" in percent
	float sensorReadPercents(); 				// Returns current "position" in percent
    float sensorReadGauss(); 				    // Returns current Hall sensor reading in Gauss
    float sensorReadDistance(); 			    // Returns current distance reading estimate from the magnet
	
	float adcToGauss(short adc);			    // Computes Gauss from the ADC of the Hall sensor
    float gaussToDistance(float g);			    // Computes distance from magnetic flux
	// Get functions to extract private variables
	int getMinCalibrated();						// Returns the minimum calibrated value for the magnetic field (10-bit ADC levels).
	int getMaxCalibrated();						// Returns the maximum calibrated value for the magnetic field (10-bit ADC levels).
	byte getSaturation();		                // Returns the saturation level of the magnet (8-bit DAC levels).
	
	#if SHIELDRELEASE == 2
		float auxReadVoltage(); 			    // Returns voltage potential of the electromagnet
	    float auxReadCurrent(); 			    // Returns current through the electromagnet
		float referenceRead();					// Returns potentiometer value in percents
	#endif
	
private:							
	short minCalibrated = 0;					// ADC on Hall for ground
	short maxCalibrated = 1024;					// ADC on Hall for ceiling
	byte  magnetSaturate = 255;					// Saturation level of the magnet before its dropped	
    byte calibrated = 0;						// If the calibration routine was completed
};

extern MagnetoShieldClass MagnetoShield;		//declare external instance
#endif