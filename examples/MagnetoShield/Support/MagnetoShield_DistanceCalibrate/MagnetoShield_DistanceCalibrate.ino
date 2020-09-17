/*
  MagnetoShield automatic distance calibration experiment

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
  Last update: 17.9.2020.
*/
#include <AutomationShield.h> 
#include <MagnetoShield.h>     // Include header for hardware API

// Maximum and minimum mean voltage, ADC or position (not magnetic)
int Minimum;
int Maximum;
short Saturation;

void setup() {
   Serial.begin(9600);                           // Starts serial communication
   AutomationShield.printSeparator("=");          //
   Serial.println("Calibration in progress...");   // Begin note
   MagnetoShield.begin();                        // Initializes shield
   MagnetoShield.calibration();                  // Calibrates shield
   Serial.println("Done.");   // Begin note
   AutomationShield.printSeparator("="); 
     
   Minimum=MagnetoShield.getMinCalibrated();     //  minimum ADC, maximum Gauss
   Maximum=MagnetoShield.getMaxCalibrated();     //  maximum ADC, minimum Gauss
   
   Serial.print("Hall sensor maximum magnetic reading at: ");  
   Serial.print(Minimum);
   Serial.print(" ADC levels when magnet at bottom, that is ");   
   Serial.print(MagnetoShield.adcToGauss(Minimum));
   Serial.print(" G, estimated distance from electromagnet is "); 
   Serial.print(MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(Maximum)));
   Serial.println(" mm");
   
   Serial.print("Hall sensor minimum magnetic reading at: ");  
   Serial.print(Maximum);
   Serial.print(" ADC levels when magnet at top, that is ");  
   Serial.print(MagnetoShield.adcToGauss(Maximum));
   Serial.print(" G, estimated distance from electromagnet is "); 
   Serial.print(MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(Minimum)));
   Serial.println(" mm");
  
}

void loop() {                                  
}
