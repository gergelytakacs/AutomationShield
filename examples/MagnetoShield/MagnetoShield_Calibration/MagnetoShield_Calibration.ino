/*
  MagnetoShield calibration experiment

  This example initializes the MagnetoShield and measures
  the Hall sensor output with the electromagnet turned 
  on and off. It returns the minimal and maximal ADC levels, 
  equivalent magnetic flux in Gauss and the position as
  estimated by a power curve.
  
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács. 
  Last update: 11.01.2019.
*/
#include <MagnetoShield.h>     // Include header for hardware API


int Minimum;
int Maximum;
short Saturation;

void setup() {
   Serial.begin(2000000);                        // Starts serial communication
   Serial.print("Calibration in progress...");   // Begin note
   MagnetoShield.begin();                        // Initializes shield
   MagnetoShield.calibration();                  // Calibrates shield
  
   Minimum=MagnetoShield.getMinCalibrated();     //  Getting borders for flying
   Maximum=MagnetoShield.getMaxCalibrated(); 
   Saturation=MagnetoShield.getSaturation(); 
   

   Serial.print(Minimum);
   Serial.print(" of 10-bit ADC, that is ");   
   Serial.print(MagnetoShield.adcToGauss(Minimum));
   Serial.print(" G, estimated distance is "); 
   Serial.print(MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(Minimum)));
   Serial.println(" mm");
   
   Serial.print("Hall sensor maximum: ");  
   Serial.print(Maximum);
   Serial.print(" of 10-bit ADC, that is ");  
   Serial.print(MagnetoShield.adcToGauss(Maximum));
   Serial.print(" G"); 
   Serial.print(" G, estimated distance is "); 
   Serial.print(MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(Maximum)));
   Serial.println(" mm");
  
   Serial.print("Magnet saturation: ");  
   Serial.print(Saturation);
   Serial.println(" of 8-bit ADC"); 
}

void loop() {                                  
}