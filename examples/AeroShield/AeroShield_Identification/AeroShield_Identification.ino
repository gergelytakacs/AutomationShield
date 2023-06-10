/*
  AeroShield open loop experiment for system identification.


    The following code is an example of MotoShield API in use with
    two modes of reference value setting. For LQ computation, 
    LQ and Kalman Estimate library has been used.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 16.1.2023.
*/

#include "AeroShield.h"
#include "Sampling.h"
#include "aprbsU.h"
#define TS 15


float startAngle = 0;
float y;
float u;
float r;
float i;
bool flag = false;
bool realTimeViolation = false;
int k;
void setup() {
  Serial.begin(500000);
  AeroShield.begin();
  AeroShield.calibrate();
  Serial.println("u, y, i"); //--Print header
  Sampling.interrupt(stepEnable);
  Sampling.period(TS * 1000);
}

void loop() {
  if (flag) {
    step();
    flag = false;
  }
}

void stepEnable() {                                    // ISR
  if (flag) {                            // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    Serial.println("Real-time samples violated."); // Print error message
    AeroShield.actuatorWrite(0.0);                // Turn off the fan
    while (1);                                     // Stop program execution
  }
  flag = true;                                   // Enable step flag
}

void step() {
  if (k > aprbsU_length) {                   // If at end of trajectory
    AeroShield.actuatorWrite(0.0);      // Turn off the fan
    while (1);                           // Stop program execution
  } else {                                 // Otherwise
    u = pgm_read_float_near(&aprbsU[k]);
  }
  y = AeroShield.sensorReadRadian();   // Read sensor
  AeroShield.actuatorWriteVolt(u);           // Actuate
  i = AeroShield.sensorReadCurrent();
  Serial.print(y, 5);          // Print output
  Serial.print(" ");
  Serial.print(i, 5);
  Serial.print(" ");
  Serial.println(u, 5);        // Print input
  k++;                       // Increment index
}