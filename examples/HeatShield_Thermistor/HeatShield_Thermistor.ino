/*
  HeatShield Thermistor Test

  Reads the temperature of the block on the HeatShield hardware.
  
  The example initializes the HeatShield, then reads data 
  from the thermistor which is then sent to the serial port. 
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 10.10.2018.
*/


#include <HeatShield.h>              // Include library

void setup() {
  Serial.begin(9600);                // Initialize serial 
  HeatShield.begin();                // Initialize shield
  
  // Annotate heading
  Serial.print("U (V)");                
  Serial.print("\t");                
  Serial.print("R (kOhm)");               
  Serial.print("\t");                
  Serial.println("T (oC)");               
}

void loop() {
  Serial.print(HeatShield.getThermistorVoltage());            // Print voltage
  Serial.print("\t");                                
  Serial.print(HeatShield.getThermistorResistance()/1000,1);  // Print resistance
  Serial.print("\t");   
  Serial.print("\t");  
  Serial.println(HeatShield.sensorRead(),1);                  // Print temperatures
  delay(500);
}