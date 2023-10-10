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
  Last update: 17.09.2020.
*/

#include "MagnetoShield.h"

// An initialization method for the MagnetoShield
void MagnetoShieldClass::begin(){	    
	#ifdef ARDUINO_ARCH_AVR
		Wire.begin();	// Starts the "Wire" library for I2C
		#if SHIELDRELEASE == 1	
			analogReference(DEFAULT);
		#elif SHIELDRELEASE == 2 || SHIELDRELEASE == 3 || SHIELDRELEASE == 4
			analogReference(EXTERNAL);
		#endif
	#elif ARDUINO_ARCH_SAM
		#if SHIELDRELEASE == 1
			#error "MagnetoShield R1 is not compatible with 3.3V CMOS logic"
		#else
			analogReadResolution(12);
			Wire1.begin();
		#endif
	#elif ARDUINO_ARCH_SAMD
		#if SHIELDRELEASE == 1
			#error "MagnetoShield R1 is not compatible with 3.3V CMOS logic"
		#else
			analogReadResolution(12);
			Wire.begin();
		#endif
	#elif ARDUINO_ARCH_RENESAS_UNO
		#if SHIELDRELEASE == 1	
			analogReference(DEFAULT);
		#elif SHIELDRELEASE == 2 || SHIELDRELEASE == 3 || SHIELDRELEASE == 4
			analogReference(AR_EXTERNAL);
		#endif
		analogReadResolution(12);
		Wire.begin();
	#endif
}

// Write DAC levels (8-bit) to the PCF8591 chip
#if SHIELDRELEASE == 1 || SHIELDRELEASE == 2
void MagnetoShieldClass::dacWrite(uint8_t DAClevel){
	#ifdef ARDUINO_ARCH_AVR || ARDUINO_ARCH_SAMD || ARDUINO_ARCH_RENESAS_UNO
		Wire.beginTransmission(PCF8591);		// I2C addressing, transmission begin
		Wire.write(0x40);						// Use DAC (0100 0000)
		Wire.write(DAClevel);					// 8-bit value of DAC output
		Wire.endTransmission();					// I2C transmission end
	#elif ARDUINO_ARCH_SAM
		Wire1.beginTransmission(PCF8591);		// I2C addressing, transmission begin
		Wire1.write(0x40);						// Use DAC (0100 0000)
		Wire1.write(DAClevel);					// 8-bit value of DAC output
		Wire1.endTransmission();				// I2C transmission end 
	#endif
}
// Write DAC levels (12-bit) to the MCP4725 chip
#elif SHIELDRELEASE == 3 || SHIELDRELEASE == 4
void MagnetoShieldClass::dacWrite(uint16_t DAClevel){	// 16 bits in the form (0,0,0,0,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0)
	#if ARDUINO_ARCH_AVR || ARDUINO_ARCH_SAMD || ARDUINO_ARCH_RENESAS_UNO 
		Wire.beginTransmission(MCP4725); 					//addressing
	    Wire.write(0x40); 								// write dac(DAC and EEPROM is 0x60)
	    uint8_t firstbyte=(DAClevel>>4);					//(0,0,0,0,0,0,0,0,D11,D10,D9,D8,D7,D6,D5,D4) of which only the 8 LSB's survive
	    DAClevel = DAClevel << 12;  						//(D3,D2,D1,D0,0,0,0,0,0,0,0,0,0,0,0,0) 
	    uint8_t secndbyte=(DAClevel>>8);					//(0,0,0,0,0,0,0,0,D3,D2,D1,D0,0,0,0,0) of which only the 8 LSB's survive.
	    Wire.write(firstbyte); //first 8 MSB's
	    Wire.write(secndbyte); //last 4 LSB's
	    Wire.endTransmission();
	#elif ARDUINO_ARCH_SAM
		Wire1.beginTransmission(MCP4725); 					//addressing
	    Wire1.write(0x40); 								// write dac(DAC and EEPROM is 0x60)
	    uint8_t firstbyte=(DAClevel>>4);					//(0,0,0,0,0,0,0,0,D11,D10,D9,D8,D7,D6,D5,D4) of which only the 8 LSB's survive
	    DAClevel = DAClevel << 12;  						//(D3,D2,D1,D0,0,0,0,0,0,0,0,0,0,0,0,0) 
	    uint8_t secndbyte=(DAClevel>>8);					//(0,0,0,0,0,0,0,0,D3,D2,D1,D0,0,0,0,0) of which only the 8 LSB's survive.
	    Wire1.write(firstbyte); //first 8 MSB's
	    Wire1.write(secndbyte); //last 4 LSB's
	    Wire1.endTransmission();
	#endif
}
#endif

// Calibrates the output readings and measures the input_iterator
// saturation for the MagnetoShield
void MagnetoShieldClass::calibration(){
	uint16_t min, max, minV; 					// Temporary measurements
	
	// Selects maximum ADC value to filter for any noise.
	// Magnet cannot get physically lower than ground level.
	dacWrite(0);					            // No power to magnet
	delay(500);					   			    // Wait for things to settle
	minCalibrated = analogRead(MAGNETO_YPIN);	// overwrite default values -> independence of the measurement from default saturation
	for (int i=1; i<=100; i++) {	    		// Perform 100 measurements
		min = analogRead(MAGNETO_YPIN); 		// Measure
		if (min < minCalibrated){ 				// If higher than already
			minCalibrated=min; 		  			// Save new minimum
		}
	}
	//Selects maximum ADC value to filter for any noise.
	// Magnet cannot get physically higher than ceiling
	dacWrite(DACMAX);				// Full power to magnet for 12-bit DAC
	delay(500);					   			    // Wait for things to settle
	maxCalibrated = analogRead(MAGNETO_YPIN);	// overwrite default values -> independence of the measurement from default saturation	
	for (int i=1; i<=100; i++) {				// Perform 100 measurements
		max = analogRead(MAGNETO_YPIN);			// Measure
		if (max > maxCalibrated){				// If lower than already
			maxCalibrated=max;					// Save new maximum
		}	
	}	
	
	// Measures true coil voltage with magnet fully turned on. Upgrades pre-determined variable.
	#if SHIELDRELEASE == 3 || SHIELDRELEASE == 4
		for (int i=1; i<=100; i++) {	    	// Perform 100 measurements
		 minV = auxReadVoltage(); 				// Measure
		 if (minV < voltageRef){ 				// If higher than already
			voltageRef = minV; 		  			// Save new minimum
		 }
		}
		setVoltageRef(voltageRef);		// Upgrades voltage level
	#endif
	
	// Recalibrate distance based on these
	d_p2=log((EMAGNET_HEIGHT-MAGNET_LOW)/(EMAGNET_HEIGHT-MAGNET_HIGH))/log(adcToGauss(minCalibrated)/adcToGauss(maxCalibrated));	// Power
	d_p1=(EMAGNET_HEIGHT-MAGNET_HIGH)/(pow(adcToGauss(maxCalibrated),d_p2)); //Multiplier	
	
	dacWrite(0); 								// Fall back to ground
	delay(500);							    	// Wait for things to settle
	calibrated = 1;							    // State of the things is now calibrated
}	
		
// Writes input to actuator as desired voltage on magnet
void MagnetoShieldClass::actuatorWriteVoltage(float u){
	#if SHIELDRELEASE == 1 || SHIELDRELEASE == 2
    uint8_t dacIn = voltageToDac(u);								// Re-computes DAC levels according to Voltage
    dacIn = constrain(dacIn,IRF520_LSAT,IRF520_HSAT); 				// Constrains to IRF520 saturation
	dacWrite(dacIn);  	   									  		// Writes to DAC
	#elif SHIELDRELEASE == 3 || SHIELDRELEASE == 4
	uint16_t dacIn = voltageToDac(u);							    // Re-computes DAC levels according to Voltage
	dacIn = constrain(dacIn,0,DACMAX);								// Constrain input into acceptable range
	dacWrite(dacIn);  	   											// Writes to DAC
	#endif
}
		
// Default actuator write function (just a call)
void MagnetoShieldClass::actuatorWrite(float u){ 
	actuatorWriteVoltage(u); 										// Calls preferred routine			
}

// Writes input to actuator as a percentage in the range of 0-100%
void MagnetoShieldClass::actuatorWritePercents(float u){
	
	#if SHIELDRELEASE == 1 || SHIELDRELEASE == 2
		float maxV = VIN
	#elif SHIELDRELEASE == 3 || SHIELDRELEASE == 4	
		float maxV = getVoltageRef();	
	#endif
	u = AutomationShield.mapFloat(u,0.0,100.0,0.0,maxV); 
	actuatorWriteVoltage(u); 										// Writes as Voltage - needed because of the nonlinearity DAC/voltage	
}

// Default sensor reading method
float MagnetoShieldClass::sensorRead(){	
	return  sensorReadDistance();
}

// Reads sensor and returns the Hall sensor reading in Gauss
float MagnetoShieldClass::sensorReadDistance(){	
	return  gaussToDistance(sensorReadGauss());
}

// Reads sensor and returns percentage of voltage from Hall sensor
// effectively giving an indirect percentual distance
float MagnetoShieldClass::sensorReadPercents(){	
	if (!calibrated){
		minCalibrated=HALL_LSAT;
		maxCalibrated=HALL_HSAT;		
	}
	// Recalculates measured value in interval (minCalibrated, maxCalibrated) to percents 0-100%
	return  (AutomationShield.mapFloat(analogRead(MAGNETO_YPIN),minCalibrated,maxCalibrated,0.0,100.0));
}

// Reads sensor and returns the Hall sensor reading in Gauss
float MagnetoShieldClass::sensorReadGauss(){	
	return  adcToGauss(analogRead(MAGNETO_YPIN));
}

// Converts a 10-bit ADC reading from the A1302 Hall sensor
// to Gauss. The Hall sensor picks up on the levitating magnet
float MagnetoShieldClass::adcToGauss(uint16_t adc){	
	#if SHIELDRELEASE == 1
	float y=(2.5-(float)adc*ARES)*HALL_SENSITIVITY;
	#elif SHIELDRELEASE == 2 || SHIELDRELEASE == 3 || SHIELDRELEASE == 4
	float y=(2.5-(float)adc*ARES3V3)*HALL_SENSITIVITY;
	#endif
	return  y;
}

// Converts the magnetic flux reading from the A1302 Hall sensor
// to distance measured from the bottom of the magnet
float MagnetoShieldClass::gaussToDistance(float g){	
	if (!calibrated){
		d_p1=D_P1_DEF;
		d_p2=D_P2_DEF;		
	}
	return  d_p1*pow(g,d_p2);
}


	// Computes DAC levels for equivalent magnet voltage. For release R3 is the conversion linear
#if SHIELDRELEASE == 1 || SHIELDRELEASE == 2
	uint8_t MagnetoShieldClass::voltageToDac(float vOut){
		uint8_t dacOut = (uint8_t)(round(P1*pow(vOut,P2)+P3*exp((vOut*P4))));
		return dacOut;
	}
#elif SHIELDRELEASE == 3 || SHIELDRELEASE == 4
	uint16_t MagnetoShieldClass::voltageToDac(float vOut){
		float dacOutTemp=round((P1 * pow(vOut,3)) + (P2 * pow(vOut,2)) + (P3 * vOut) + P4);
		if (dacOutTemp<=0.0) dacOutTemp=0.0;		// Prevent negative values
		uint16_t dacOut = (int16_t)(dacOutTemp);
		return dacOut;
	}
#endif
	

#if SHIELDRELEASE == 2 || SHIELDRELEASE == 3 || SHIELDRELEASE == 4
	// Default sensor reading method - returns value from 0 to 100
	float MagnetoShieldClass::referenceRead(){	
		return  (AutomationShield.mapFloat(analogRead(MAGNETO_RPIN),0.0,ADCREF,0.0,100.0));
	}
	
	// Reads the voltage on the Electromagnet in V
	float MagnetoShieldClass::auxReadVoltage(){	
		float v = ((float)analogRead(MAGNETO_VPIN))*ARES3V3*VGAIN;
		return  v;
	}
	
	
	// Reads the current on the Electromagnet in mA
	float MagnetoShieldClass::auxReadCurrent(){	
		float i =  ((float)analogRead(MAGNETO_IPIN))*ARES3V3*IGAIN;//-IBIAS;
		return  i;
	}
#endif

#if SHIELDRELEASE == 3 || SHIELDRELEASE == 4
// Returns reference voltage for calculation DAC levels to voltage (4095 = voltageRef)
float MagnetoShieldClass::getVoltageRef(){
	return voltageRef;
}

// Possibility to change what is maximal voltage we can get on the emag
void MagnetoShieldClass::setVoltageRef(float v_ref){
	voltageRef = v_ref;
}
#endif

// Returns the lower limit of the Hall sensor reading (10-bit levels)
uint16_t MagnetoShieldClass::getMinCalibrated(){
	return minCalibrated;;
}	

// Returns the upper limit of the Hall sensor reading (10-bit levels)
uint16_t MagnetoShieldClass::getMaxCalibrated(){
	return maxCalibrated;
}	

MagnetoShieldClass MagnetoShield;	//construct instance (define)
