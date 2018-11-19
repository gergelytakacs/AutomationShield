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

  Created by Gergely Takács and Jakub Mihalík. 
  Last update: 19.11.2018.
*/
#define SHIELDVERSION 2       // 
#include <MagnetoShield.h>     // Include header for hardware API


int Minimum;
int Maximum;
short Saturation;

void setup() {
   Serial.begin(9600);          // Starts serial communication
   MagnetoShield.begin();       // Initializes shield
   MagnetoShield.calibration();       // Measures maximum and minimum
   
   Minimum=MagnetoShield.getMinCalibrated();     //  Getting borders for flying
   Maximum=MagnetoShield.getMaxCalibrated(); 
   Saturation=MagnetoShield.getSaturation(); 
   
   Serial.print("Hall sensor minimum: ");  
   Serial.print(Minimum);
   Serial.print(" of 10-bit ADC, that is ");   
   Serial.print(MagnetoShield.adcToGauss(Minimum));
   Serial.println(" G"); 
   
   Serial.print("Hall sensor maximum: ");  
   Serial.print(Maximum);
   Serial.print(" of 10-bit ADC, that is ");  
   Serial.print(MagnetoShield.adcToGauss(Maximum));
   Serial.println(" G"); 
  
   Serial.print("Magnet saturation: ");  
   Serial.print(Saturation);
   Serial.println(" of 8-bit ADC"); 
}

void loop() {                                  
}