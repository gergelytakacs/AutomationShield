/*
  HeatShield system identification experiment

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
  Last update: 10.10.2018.
*/

#include <HeatShield.h> 	    // Include the library
#include <Sampling.h>            // Include sampling

long noise;                         // Noise component in input
unsigned long Ts = 2000;            // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool enable = false;                // Flag for sampling

float y = 0.0;						// Output
float u = 0.0;						// Input (open-loop)
float U[] = {25.0, 85.0, 50.0, 15.0, 75.0}; // Input trajectory
int nU = sizeof(U) / sizeof(U[0]);  // Number of elements
int T = 1800;						// Section length (60 min)
int i = 0;							// Section counter

void setup() {
  Serial.begin(9600);               // Initialize serial
  randomSeed(analogRead(1));        // Seed random numbers
  // Initialize and calibrate board
  HeatShield.begin();               // Define hardware pins

  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) { 							// If ISR enables
    step();									  // Algorithm step
    enable = false;  					 // Then disable
  }
}

void stepEnable() {  					// ISR
  enable = true;							 // Change flag
}

// A single algorithm step
void step() {

  if (i <= nU) {                 // If experiment is still on
    if (k % (T * i) == 0) {      // If new section
      u = U[i];     						 // Set reference
      i++;                       // Increment section
    }
  }
  else {								         // Experiment done
    HeatShield.actuatorWrite(0);    // Turn off heat shield
    while (1);
    }

  y = HeatShield.sensorRead();           // Read sensor
  noise = ((float)random(-50, 50)) / 10;
  u = u + noise;
  
  if (u>=100.0) {  
	u=100.0;
  }
  
  if (u<=0.0) {  
	u=0.0;
  }
  
  HeatShield.actuatorWrite(u);           // Actuate

  Serial.print(y);						// Print output
  Serial.print(", ");
  Serial.println(u);						// Print input
  k++;									// Increment k
}
