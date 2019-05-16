/*
  TugShield system identification experiment
  Creates a pulse train to extract input-output data.
  
  This example initializes the sampling subsystem from the 
  AutomationShield library and starts an open-loop experiment
  with the intention of extracting useful data for a system
  identification procedure. Upload the code to your board, then
  open the Serial Monitor or log the data to an external program.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács. 
  Last update: 13.5.2019.
*/

#include "TugShield.h"             // Include the library

float y = 0.0;                    // Output
int i = 0;                        // Section counter

void setup() 
{
  Serial.begin(9600);               // Initialize serial
  TugShield.calibration();         // Initialize and calibrate board
  TugShield.begin();               // Define hardware pins
  Serial.print("Testing flexi sensor...0 degree ");
  TugShield.actuatorWrite(0);
  delay(100);
  y = TugShield.sensorRead();
  if (y >= 0.0 && y <= 5.0)
  {
    Serial.println(" Ok.");
  }
  else 
  {
    Serial.println(" Fail.");
  }

  Serial.print("Testing flexi sensor...180 degree ");
  TugShield.actuatorWrite(180);
  delay(100);
  y = TugShield.sensorRead();
  if (y >= 95.0 && y <= 100.0) 
  {
    Serial.println(" Ok.");
  }
  else 
  {
    Serial.println(" Fail.");
  }
}

// Main loop launches a single step at each enable time
void loop() {
  
}
