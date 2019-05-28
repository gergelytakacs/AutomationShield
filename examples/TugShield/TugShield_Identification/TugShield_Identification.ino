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

unsigned long Ts = 10;             // Sampling in milliseconds
unsigned long k = 0;               // Sample index
bool enable=false;                 // Flag for sampling 

float y = 0.0;                    // Output
int u = 0;                        // Input (open-loop)
int U[]={0,90,45,100,180,0};      // Input trajectory
int T = 100;                      // Section length 
int i = 0;                        // Section counter

void setup() {
  Serial.begin(9600);               // Initialize serial
  TugShield.begin();               // Define hardware pins
  TugShield.calibration();
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {               // If ISR enables
    step();                 // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}

// A signle algoritm step
void step(){ 

if (k % (T*i) == 0){        
  u = U[i];                // Set reference
  i++;
}

TugShield.actuatorWrite(u);           // Actuate                  
y = TugShield.sensorRead();           // Read sensor 

   
Serial.print(y);            // Print output  
Serial.print(" ");
Serial.println(u);            // Print input
k++;                  // Increment k
}
