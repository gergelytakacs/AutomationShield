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
  Created by Eva Vargova and Gergely Takacs. 
  Last update: 6.5.2019.
*/

#include <TugShield.h>             // Include the library

float y = 0.0;                    // Output
int i = 0;                        // Section counter

void setup() 
{
  Serial.begin(9600);               // Initialize serial
  TugShield.begin();               // Define hardware pins
  TugShield.calibration();         // Initialize and calibrate board
  delay(100);
  Serial.println("Testovanie flexi snímača...poloha serva: 0 ");
  TugShield.actuatorWrite(0);
  delay(500);
  y = TugShield.sensorRead();
  if (y >= 0.0 && y <= 15.0)
  {
    Serial.println(" V poriadku.");
    Serial.println(y);
  }
  else 
  {
    Serial.println(" Nevyhovuje.");
    Serial.println(y);
  }
  delay(1000);
  Serial.println("Testovanie flexi snímača...poloha serva: 180");
  TugShield.actuatorWrite(180);
  delay(500);
  y = TugShield.sensorRead();
  if (y >= 85.0 && y <= 100.0) 
  {
    Serial.println(" V poriadku.");
    Serial.println(y);
  }
  else 
  {
    Serial.println(" Nevyhovuje.");
    Serial.println(y);
  }
}

// Main loop launches a single step at each enable time
void loop() {
  
}
