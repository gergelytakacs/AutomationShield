/*
  MagnetoShield calibration experiment

  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and starts a 
  predetermined reference trajectory for the heating block
  temperature. 
  
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács. 
  Last update: 19.11.2018.
*/
#include <MagnetoShield.h>     // Include header for hardware API
void setup() {
   Serial.begin(2000000);          // Starts serial communication
   MagnetoShield.begin();       // Initializes shield
   MagnetoShield.actuatorWrite(0);
   Serial.print(MagnetoShield.auxReadVoltage());
	
}

void loop() {    
}