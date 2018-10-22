/*
  HeatShield Step Response

  Performs a simple step response test on the HeatShield hardware
  
  The example initializes the HeatShield, then a 50% power input 
  is sent to the heating cartridge. The thermal response of the
  block is sent to the serial port. The response can be logged
  for system identification purposes. 
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács. 
  Last update: 10.10.2018.
*/

#include <HeatShield.h>              // Include library

float u = 50.0;                      // Open-loop input (%)
int Ts = 3000;						 // Sampling (ms)
void setup() {
  Serial.begin(9600);                // Initialize serial 
  HeatShield.begin();                // Initialize shield
  HeatShield.actuatorWrite(u);       // Step change input  
}

void loop() {
  float y = HeatShield.sensorRead(); // Read sensor
  Serial.print(u);                   // Print input
  Serial.print(", ");                // Comma
  Serial.println(y);                 // Print output 
  delay(Ts);						     // Approximate sampling
}