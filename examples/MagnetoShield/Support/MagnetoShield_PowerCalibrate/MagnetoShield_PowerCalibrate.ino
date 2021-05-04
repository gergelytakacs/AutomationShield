/*
  MagnetoShield MOSFET or BJT calibration curve experiment

  This example gradually increases the DAC levels that
  are supplied to the power element (MOSFET, or BJT) and
  outputs an average coil voltage and current to create
  calibration curves. You shall use a terminal program to
  log the data, then use it externally, e.g. in MATLAB.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 19.11.2018.
*/
#include <MagnetoShield.h>     // Include header for hardware API

float averaging = 10;         // Create average from this many measurements
int waitDAC = 10;            // This many milliseconds to wait before moving to next level
float Uavg;                   // Declare average voltage variable
float Iavg;                    // Declare average current variable

void setup() {
  Serial.begin(4800);         // Starts serial communication

  MagnetoShield.begin();      // Initializes shield
  MagnetoShield.dacWrite(0);  // Turns off magnet
  delay(waitDAC);             // Waits a bit for things to settle

  // Supplies DAC level from zero to maximal resolution
  for (int i = 0; i < DACMAX + 1; i++) {
    MagnetoShield.dacWrite(i);      // Supplies certain equivalent voltage

    Uavg = 0;                       // Reset average voltage
    Iavg = 0;                       // Reset average current
    delay(waitDAC);                 // Wait for things to settle

    // Cycle through averages
    for (int j = 0; j < (int)averaging; j++) {
      Uavg = (Uavg + MagnetoShield.auxReadVoltage()); // Measure sum of voltages
      Iavg = (Iavg + MagnetoShield.auxReadCurrent()); // Measure sum of currents
    }
      Uavg = Uavg /averaging; // Compute average voltage
      Iavg = Iavg /averaging; // Compute average current

    // Print to serial
    Serial.print(i);
    Serial.print(", ");
    Serial.print(Uavg,4);
    Serial.print(", ");
    Serial.println(Iavg,4);
    delay(100);
  }

  MagnetoShield.dacWrite(0);   // Turns off magnet
}


void loop() {
}
