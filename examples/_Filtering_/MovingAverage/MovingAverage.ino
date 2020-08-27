 /*
  Moving average filtering using a circular FIR filter
  Possible to convert easily to a generic FIR filter class
  
  Tested with Arduino Mega, but there is no reason why it should 
  not work on other types of MCU.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 27.8.2020
*/

#include "FIR.h"                       // Include FIR header

FIR myFIR;                             // Create myFIR object from FIR class

void setup() {
  Serial.begin(9600);                  // Initialize serial print
  myFIR.begin(8);                      // Create moving average filter of a certain order
}

void loop() {
 float x=100+(float)random(-100, 100); // Create a random number between +-100 and shift it up to 100
 delay(100);                           // A bit of delay for the visual effect in the plotter
 Serial.print(x);                      // Print the value 100 with random noise
 Serial.print(",");                    // Separator
 Serial.println(x-myFIR.filter(x));    // Filtered value 
}
