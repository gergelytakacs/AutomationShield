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

#include "MagnetoShield.h"

// An initialization method for the MagnetoShield
void MagnetoShieldClass::begin()
{	
	Wire.begin();	// Starts the "Wire" library for I2C
	#if SHIELDRELEASE == 1	
		analogReference(DEFAULT);
	#elif SHIELDRELEASE == 2
		analogReference(EXTERNAL);
	#endif
}

// Write DAC levels (8-bit) to the PCF8591 chip
void MagnetoShieldClass::dacWrite(byte dacIn)
{	
	Wire.beginTransmission(PCF8591);		// I2C addressing, transmission begin
	Wire.write(0x40);						// Use DAC (0100 0000)
	Wire.write(dacIn);						// 8-bit value of DAC output
	Wire.endTransmission();					// I2C transmission end
}

// Calibrates the output readings and measures the input_iterator
// saturation for the MagnetoShield
void MagnetoShieldClass::calibration()
{
	short meas_t, meas_tt; 				   		// Temporary measurements
	
	// Selects maximum ADC value to filter for any noise.
	// Magnet cannot get physically lower than ground level.
	MagnetoShield.dacWrite(0);						// No power to magnet
	delay(500);					   			// Wait for things to settle
	for (int i=1; i<=100; i++) {	    		// Perform 100 measurements
		meas_t = analogRead(MAGNETO_YPIN); 		// Measure
		if (meas_t > minCalibrated){ 			// If higher than already
			minCalibrated=meas_t; 		  		// Save new minimum
		}
	}
	// Selects maximum ADC value to filter for any noise.
	// Magnet cannot get physically higher than ceiling
	MagnetoShield.dacWrite(255);						// Full power to magnet
	delay(500);					   			// Wait for things to settle	
	for (int i=1; i<=100; i++) {				// Perform 100 measurements
		meas_t = analogRead(MAGNETO_YPIN);		// Measure
		if (meas_t < maxCalibrated){			// If lower than already
			maxCalibrated=meas_t;				// Save new maximum
		}	
	}
	
	// Lower DAC until reaches saturation point
	while (maxCalibrated>=meas_t) {				// While we reach maxCalibrated
	    magnetSaturate--; 		     			// Lower magnet power
		MagnetoShield.dacWrite(magnetSaturate); // Set magnet power
	    delay(500);							    // Wait for things to settle
		for (int i=1; i<=10; i++) {			    // Perform 10 measurements
			meas_tt = analogRead(MAGNETO_YPIN);	// Measure
			if (meas_t < meas_tt){				// If lower than already
				meas_t=meas_tt;			    	// Save measurement
			}
		}
	}
	
	calibrated = 1;								// The calibration routine was launched
	MagnetoShield.dacWrite(0); 					// Fall back to ground
	delay(500);							    	// Wait for things to settle
}	
		
// Writes input to actuator as desired voltage on magnet
void MagnetoShieldClass::actuatorWriteVoltage(float u)
{
    byte dacIn = voltageToDac(u);								// Re-computes DAC levels according to Voltage
    dacIn = constrain(dacIn,IRF520_LSAT,IRF520_HSAT); 			// Constrains to IRF520 saturation
	dacWrite(dacIn);  	   									  // Writes to DAC
}		
		
// Default actuator write function (just a call)
void MagnetoShieldClass::actuatorWrite(float u)
{ 
	actuatorWriteVoltage(u); 			// Calls preferred routine			
}

// Writes input to actuator as a percentage in the range of 0-100%
void MagnetoShieldClass::actuatorWritePercents(float u)
{
    u = AutomationShield.mapFloat(u,0.0,VIN,0.0,100.0); 
	actuatorWriteVoltage(u); 		// Writes as Voltage
}

// Default sensor reading method
float MagnetoShieldClass::sensorRead()
{	
	return  sensorReadPercents();
}

// Reads sensor and returns percentage of voltage from Hall sensor
// effectively giving an indirect percentual distance
float MagnetoShieldClass::sensorReadPercents()
{	
	if (~calibrated){
		minCalibrated=A1302_LSAT;
		maxCalibrated=A1302_HSAT;		
	}
	return  (AutomationShield.mapFloat(analogRead(MAGNETO_YPIN),minCalibrated,maxCalibrated,0.0,100.0));
}

// Converts a 10-bit ADC reading from the A1302 Hall sensor
// to Gauss. The Hall sensor picks up on the levitating magnet
float MagnetoShieldClass::adcToGauss(short adc)
{	
 	#if SHIELDRELEASE == 1
		float y=(2.5-(float)adc*ARES)*A1302_SENSITIVITY;
	#elif SHIELDRELEASE == 2
		float y=(2.5-(float)adc*ARES3V3)*A1302_SENSITIVITY;
	#endif
	return  y;
}

// Converts the magnetic flux reading from the A1302 Hall sensor
// to distance measured from the bottom of the magnet
float MagnetoShieldClass::gaussToDistance(float g)
{	
	float h = (EMAGNET_HEIGHT)-P6*pow(g,P7);
	return  h;
}

// Computes DAC levels for equivalent magnet voltage
byte MagnetoShieldClass::voltageToDac(float vOut)
{
	 float dacOut = P1*pow(vOut,P2)+P3*pow(vOut,P4)+P5;
	 return (byte)dacOut;
}			

// Reads sensor and returns the Hall sensor reading in Gauss
float MagnetoShieldClass::sensorReadGauss()
{	
	return  MagnetoShield.adcToGauss(analogRead(MAGNETO_YPIN));
}

// Reads sensor and returns the Hall sensor reading in Gauss
float MagnetoShieldClass::sensorReadDistance()
{	
	return  MagnetoShield.gaussToDistance(MagnetoShield.sensorReadGauss());
}


// Approximate distance based on magnetism
// float MagnetoShieldClass::sensorReadDistance()
// {	
	// return  MagnetoShield.adcToGauss(analogRead(MAGNETO_YPIN));
// }

// Returns the saturation of the magnet (8-bit levels)
byte MagnetoShieldClass::getSaturation()
{
	return magnetSaturate;
}	

// Returns the lower limit of the Hall sensor reading (10-bit levels)
int MagnetoShieldClass::getMinCalibrated()
{
	return minCalibrated;;
}	

// Returns the upper limit of the Hall sensor reading (10-bit levels)
int MagnetoShieldClass::getMaxCalibrated() 
{
	return maxCalibrated;
}	

MagnetoShieldClass MagnetoShield;	//construct instance (define)
