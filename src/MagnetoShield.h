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
  Last update: 21.08.2020.
*/

#ifndef MAGNETOSHIELD_H							// Include guard
#define MAGNETOSHIELD_H

#include "AutomationShield.h"           		// Includes the core header
#include "Wire.h" 								// I2C library for the DAC chip

#ifndef SHIELDRELEASE
	#define SHIELDRELEASE 4   					//  Use number only: e.g. for R3 is 3
#endif


#ifdef ARDUINO_ARCH_AVR
  #define VIN 11.6 							    // [V] Input voltage from source
#elif ARDUINO_ARCH_SAMD
  #define VIN 12.5 							    // [V] Input voltage from source 
#elif ARDUINO_ARCH_SAM
  #define VIN 11.6 							    // [V] Input voltage from source 
#endif

// Addresses of the DAC chip
#if SHIELDRELEASE==1 || SHIELDRELEASE==2
	#define PCF8591 (0x90 >> 1)					// 8-bit DAC, Chip: PCF8591
#elif SHIELDRELEASE==3 || SHIELDRELEASE==4
	#define MCP4725 (0x60)						// 12-bit DAC, Chip: MCP4725
#endif

// Maximal levels of the DAC
#if SHIELDRELEASE==1 || SHIELDRELEASE==2
  #define DACMAX 255 // Maximal decimal DAC value for 8 bits
#else
  #define DACMAX 4095 // Maximal decimal DAC value for 12 bits
#endif

// Input pins on Adruino
#define MAGNETO_YPIN A3							// Defines the location of the Hall sensor
#if SHIELDRELEASE==2 || SHIELDRELEASE==3 || SHIELDRELEASE==4
	#define MAGNETO_RPIN A0                        // Defines the location of reference pot
	#define MAGNETO_VPIN A1						   // Defines the location of input voltage sensing
	#define MAGNETO_IPIN A2						   // Defines the location of input current sensing
#endif

// Physical dimensions and constants of devices
#define EMAGNET_HEIGHT 20.0				        // [mm] Location of electromagnet above ground
#define MAGNET_LOW	3.0  					    // [mm] Top of the magnet from ground - distance from Hall element
#define MAGNET_HIGH 8.0						    // [mm] Top of the magnet from ground - distance from Hall element
#define LOAD_RESISTANCE	196.6					// [Ohm] Load resistance

#if SHIELDRELEASE==4
	#define HALL_SENSITIVITY	800.00	    		// [G/V] = 1.25 mV/G Sensitivity of the TI DRV5055Z4 Hall sensor
#else
	#define HALL_SENSITIVITY	769.23	    		// [G/V] = 1.3 mV/G Sensitivity of the A1302 Hall sensor (typical, datasheet)
#endif

#if SHIELDRELEASE==1 || SHIELDRELEASE==2
	#define LOAD_HSAT	204							// [8-bit DAC] Upper saturation of the magnets before dropping
	#define IRF520_LSAT	170							// [8-bit DAC] Lower saturation of the IRF520
	#define IRF520_HSAT	220							// [8-bit DAC] Hight (upper) saturation of the IRF520
#endif

#if SHIELDRELEASE == 1
	#define HALL_LSAT 19							// [10-bit ADC] Lower saturation of the Hall sensor
	#define HALL_HSAT 382							// [10-bit ADC] Higher (upper) saturation of the Hall sensor
#elif SHIELDRELEASE == 2
	#ifdef ARDUINO_ARCH_AVR
		#define HALL_LSAT	30							// [10-bit ADC] Lower saturation of the Hall sensor
		#define HALL_HSAT 586							// [10-bit ADC] Higher (upper) saturation of the Hall sensor
	#elif ARDUINO_ARCH_SAMD || ARDUINO_ARCH_SAM
		#define HALL_LSAT	120							// [12-bit ADC] Lower saturation of the Hall sensor
		#define HALL_HSAT 2346							// [12-bit ADC] Higher (upper) saturation of the Hall sensor
	#endif
#elif SHIELDRELEASE == 3
	#ifdef ARDUINO_ARCH_AVR
		#define HALL_LSAT	28							// [10-bit ADC] Lower saturation of the Hall sensor
		#define HALL_HSAT 631							// [10-bit ADC] Higher (upper) saturation of the Hall sensor
	#elif ARDUINO_ARCH_SAMD || ARDUINO_ARCH_SAM
		#define HALL_LSAT	112							// [12-bit ADC] Lower saturation of the Hall sensor
		#define HALL_HSAT 2526							// [12-bit ADC] Higher (upper) saturation of the Hall sensor
	#endif
#elif SHIELDRELEASE == 4
	#ifdef ARDUINO_ARCH_AVR
		#define HALL_LSAT	35							// [10-bit ADC] Lower saturation of the Hall sensor
		#define HALL_HSAT 621							// [10-bit ADC] Higher (upper) saturation of the Hall sensor
	#elif ARDUINO_ARCH_SAMD || ARDUINO_ARCH_SAM
		#define HALL_LSAT	112							// [12-bit ADC] Lower saturation of the Hall sensor
		#define HALL_HSAT 2526							// [12-bit ADC] Higher (upper) saturation of the Hall sensor
	#endif
#endif

// Gain constants for current and voltage measurements
#if SHIELDRELEASE == 2 ||  SHIELDRELEASE==3 ||  SHIELDRELEASE==4
	#define VGAIN 4.2256  							   // Defines the voltage sensing gain (voltage divider), theoretical value 4.0
    #define IGAIN 33.33333333 					   	   // Defines the current sensing gain mA/V
	//#define IBIAS 3.01 	   					   	   // Current sensing bias mA
#endif

// Power model of the input-output voltage DAC->Vout (MOSFET in/out characteristic)
#if SHIELDRELEASE==1 || SHIELDRELEASE==2
	#ifdef ARDUINO_ARCH_AVR
		#define P1 193.7						        // Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P2 0.02833								// Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P3 0.9068							    // Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P4 0.1418								// Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
	#elif ARDUINO_ARCH_SAMD
		#define P1 193.7						        // Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P2 0.02833								// Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P3 0.9068							    // Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P4 0.1418								// Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
	#elif ARDUINO_ARCH_SAM
	    #define P1 191.2						        // Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P2 0.02718								// Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P3 2.464							    // Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
		#define P4 0.08407								// Power function constant (f(y) = P1*x^P2+P3*exp(x^P4)) for DAC vs. Output voltage
	#endif
#elif SHIELDRELEASE==3 || SHIELDRELEASE==4
	#define P1    1.41353993			      		// Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
	#define P2  -15.4070873						    // Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
    #define P3  389.266686						    // Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
	#define P4  -11.7613432						    // Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
#endif

// Distance model based on magnetic flux density
// As it is hard to make exact measurements a two-point 
// calibration of a power function seems to work best
#define D_P1_DEF  3.233100   					//Default distance function constant (f(y) = D_P1*x^D_P2) for Flux vs. distance from magnet
#define D_P2_DEF  0.220571 					    // Default distance function constant (f(y) = D_P1*x^D_P2) for Flux vs. distance from magnet

class MagnetoShieldClass						// Class for MagnetoShield API
{
public:
	void begin(); 								// Initializes "Wire" library
	void calibration();							// Finds out lowest and highest positions of magnet
	#if SHIELDRELEASE == 1 || SHIELDRELEASE == 2
	void dacWrite(uint8_t DAClevel);			    	// Writes dacIn to DAC chip
	uint8_t voltageToDac(float vOut);		   		// Computes DAC levels for magnet voltage based on MOSFET characteristic
	#endif
	
	#if SHIELDRELEASE == 2 || SHIELDRELEASE == 3 || SHIELDRELEASE == 4
	float auxReadVoltage(); 			    // Returns voltage potential of the electromagnet
	float auxReadCurrent(); 			    // Returns current through the electromagnet
	float referenceRead();					// Returns potentiometer value in percents
	#endif
	
	#if SHIELDRELEASE == 3 || SHIELDRELEASE == 4
	void dacWrite(uint16_t DAClevel);			// Writes DAClevel (DAC levels) to DAC register
	uint16_t voltageToDac(float vOut);
	float getVoltageRef();						// Returns voltage reference for DAC to Voltage conversion (4095 = voltageRef = default 10.3V)
	void setVoltageRef(float v_ref);			// Change voltage reference for DAC to Voltage conversion
	#endif
	
		
	// Functions common for all releases
	void actuatorWrite(float u);         		// Default actuator handling method
	void actuatorWriteVoltage(float u);	   		// Sends voltage to magnet
	void actuatorWritePercents(float u);        // Sets output voltage in percent
	float sensorRead(); 						// Returns current "position" in percent
	float sensorReadPercents(); 				// Returns current "position" in percent
    float sensorReadGauss(); 				    // Returns current Hall sensor reading in Gauss
    float sensorReadDistance(); 			    // Returns current distance reading estimate from the magnet
	
	float adcToGauss(uint16_t adc);			    // Computes Gauss from the ADC of the Hall sensor
    float gaussToDistance(float g);			    // Computes distance from magnetic flux
	// Get functions to extract private variables
	uint16_t getMinCalibrated();						// Returns the minimum calibrated value for the magnetic field (10-bit ADC levels).
	uint16_t getMaxCalibrated();						// Returns the maximum calibrated value for the magnetic field (10-bit ADC levels).
	
	
private:							
    float d_p1;			     					// Calibrated distance function constant (f(y) = d_p1*x^d_p2) for Flux vs. distance from magnet
	float d_p2;									// Calibrated distance function constant (f(y) = d_p1*x^d_p2) for Flux vs. distance from magnet
	uint16_t minCalibrated = HALL_LSAT;		// ADC on Hall for ground
	uint16_t maxCalibrated = HALL_HSAT;		// ADC on Hall for ceiling
    bool calibrated = 0;						// If the calibration routine was completed
    #if SHIELDRELEASE == 3 || SHIELDRELEASE == 4
		float voltageRef = 10.0; 			    // [V] Maximal voltage measured directly on electromagnet, possibility to change manually with changeVoltageRef()
    #endif
};

extern MagnetoShieldClass MagnetoShield;		//declare external instance
#endif
