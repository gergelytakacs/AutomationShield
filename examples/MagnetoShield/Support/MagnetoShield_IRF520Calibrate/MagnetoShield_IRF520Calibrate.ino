/*
  MagnetoShield IRF520 calibration curve experiment

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

void wait() {
  delay(1000);
};

void setup() {
  Serial.begin(2000000);       // Starts serial communication
  Serial.println("Pair the DAC level with current and low-side voltage reading:");
  Serial.print("DAC: ");
  Serial.print("\t");  
  Serial.print("i: ");
  Serial.print("\t");  
  Serial.println("v: (External measurement)");
  MagnetoShield.begin();       // Initializes shield
  MagnetoShield.dacWrite(0);   // Turns off magnet

  for(int i=170; i<220; i++){
  MagnetoShield.dacWrite(i);   // Turns off magnet
  wait();
  Serial.print(i);  
  Serial.print("\t");  
  Serial.println(MagnetoShield.auxReadCurrent());
  }
  Serial.print("End of experiment.");
  MagnetoShield.dacWrite(0);   // Turns off magnet
}


  void loop() {
}