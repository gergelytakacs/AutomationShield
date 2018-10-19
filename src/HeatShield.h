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
  Last update: 10.10.2018.
*/

#ifndef HEATSHIELD_H_			// Include guard
#define HEATSHIELD_H_			

#include "Arduino.h"			// Required Arduino API in libraries



// Defining pins used by the OptoShield board
#define HEAT_YPIN 0   // NTC Thermistor (Sensor)
#define HEAT_UPIN 3   // Cartridge (Actuator)
#define REF_TEMP (25.0 + ABSZERO) // Thermistor reference temperature
#define NTC_RES 100000.0 // Resistance of the thermistor
#define VD_RES 100000.0 // Resistance of voltage divider arm
#define VD_REF 5.0 		// Input for the voltage divider
#define NTC_BETA 3950.0 // value of Beta factor from datasheet MF58


class HeatShieldClass{			// Class for the HeatShield device
  
  public:
    void begin();
    float sensorRead();
    void actuatorWrite(float percent);
	float getThermistorVoltage();
    float getThermistorResistance();  
    
  private:   

};

extern HeatShieldClass HeatShield;

#endif