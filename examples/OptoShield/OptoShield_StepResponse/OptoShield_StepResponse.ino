/*
  OptoShield Step Response

  Performs a step response test on the OptoShield hardware.
  
  The example initializes the OptoShield, then performs 
  calibration. A 50% input is sent to the actuator and the 
  response is sent to the serial port. The response can be logged
  for system identification purposes. 
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Tibor Konkoly. 
  Last update: 28.09.2018.
*/

#include <OptoShield.h>              // Include library

float u = 50.0;                      // Open-loop input

void setup() {
  Serial.begin(9600);                // Initialize serial 
  OptoShield.begin();                // Initialize shield
  OptoShield.calibration();          // Calibrate shield
  delay(1000);                       // Wait
  OptoShield.actuatorWrite(u);       // Step change input  
}

void loop() {
  float y = OptoShield.sensorRead(); // Read sensor
  Serial.print(u);                   // Print input
  Serial.print(", ");                // Comma
  Serial.println(y);                 // Print output
}