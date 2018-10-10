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

#include "HeatShield.h"
#include "AutomationShield.h"
// Initializes hardware pins
void HeatShieldClass::begin(){  
      pinMode(HEAT_YPIN, INPUT);  		// Thermistor pin
      pinMode(HEAT_UPIN, OUTPUT); 		// Cartridge
}

// Writes PWM signal to Cartridge, input is in %
void HeatShieldClass::actuatorWrite(float percent){  
      analogWrite(HEAT_UPIN,AutomationShield.percToPwm(percent)); // Write to actuator
}

// Calculates equivalent temperature by computing the
// Steinhart-hart equation in the simplified Beta parameter
// form
float HeatShieldClass::sensorRead() {
    return (1 / ((1 / REF_TEMP) + (log(getThermistorResistance() / NTC_RES) / NTC_BETA))) - ABSZERO;    
}

// Reads thermistor voltage
float HeatShieldClass::getThermistorVoltage() { 
      return (float)analogRead(HEAT_YPIN) * ARES;
}

// Reads thermistor resistance
// Voltage divider circuit (Kirchhoffs law)
float HeatShieldClass::getThermistorResistance() {
	float Vterm = getThermistorVoltage();
	 return ((Vterm*VD_RES)/(VD_REF-Vterm));
}

HeatShieldClass HeatShield;