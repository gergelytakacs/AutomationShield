/*
  PressureShield system identification experiment
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
  Last update by Martin Staron
  Last update: 2.12.2020.
*/

#include "PressureShield.h"     // Include the library
#include <Sampling.h>            // Include sampling

#include "aprbsU.h"                    // Include aprbs sequence from .h file
int aprbs;                             // Variable for storing APRBS signal

unsigned long Ts = 10;              // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 

float y = 0.0;            // Output
float u = 0.0;            // Input (open-loop)

int T = 300;            // Section length
int i = 0;              // Section counter

void setup() {
  Serial.begin(2000000);               // Initialize serial
  
  // Initialize and calibrate board
  PressureShield.begin();               // Define hardware pins
  PressureShield.calibration();         // Calibration
  
  // Initialize sampling function
  Sampling.period(Ts * 10);       // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {                 // If ISR enables
    step();                     // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}

// A signle algoritm step
void step(){ 

 if(k>7200) {                                // If at end of trajectory
        PressureShield.actuatorWrite(0.0);   // Turn off
        while(1);                            // Stop program execution
    } else {                                 // Otherwise
        
      u=pgm_read_word(&aprbsU[k]); // Progress in trajectory from .h file
    }
                  
y = PressureShield.sensorRead();           // Read sensor 
PressureShield.actuatorWrite(u);           // Actuate
   
Serial.print(y);        // Print output  
Serial.print(", ");
Serial.println(u);      // Print input
k++;                    // Increment k
}
