/*
  LinkShield identification example.

  Example used to acquire data for LinkShield system identification.
  
  The LinkShield implements an a flexible rotational link experiment
  on an Arduino shield. This example initialises the sampling 
  subsystem from the AutomationShield library and allows user
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Martin Vríčan. 
  Last update: 23.1.2020.
*/

#include "LinkShield.h"               // Include the library
#include <SamplingServo.h>            // Include sampling

unsigned long Ts = 3;                 // Sampling in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep=false;                  // Flag for sampling 
bool realTimeViolation = false;       // Flag for real-time sampling violation
bool endExperiment = false;           // Boolean flag to end the experiment

float y1 = 0.0;                        // Output variable
float y2 = 0.0;                        // Output variable
float u = 0.0;                        // Input (open-loop), initialized to zero
float U[]={45.0, 20.0, 75.0, 90.0};   // Input trajectory
int T = 2000;                         // Section length (appr. '/.+2 s)
int i = 0;                            // Section counter

void setup() {
 Serial.begin(2000000);               // Initialize serial

 // Initialize linkshield hardware
 LinkShield.begin();                  // Define hardware pins
 LinkShield.calibrate();              // Remove sensor bias
 
 // Initialize sampling function
 Sampling.period(Ts *1000);           // Sampling init.
 Sampling.interrupt(stepEnable);      // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (nextStep) {                     // If ISR enables
    step();                           // Algorithm step
    nextStep=false;                   // Then disable
  }  
}

void stepEnable(){                                     // ISR 
  if(endExperiment == true){                           // If the experiment is over
    while(1);                                          // Do nothing
  }
  if(nextStep == true) {                               // If previous sample still running
        realTimeViolation = true;                      // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
        while(1);                                      // Stop program execution
  }   
  nextStep=true;                                       // Change flag
}

// A single algorithm step
void step(){ 

// Switching between experiment sections

if(i>(sizeof(U)/sizeof(U[0]))) {      // If at end of trajectory
        endExperiment=true;           // Stop program execution at next ISR
    } else if (k % (T*i) == 0) {      // If at the end of section
        u = U[i];                     // Progress in trajectory
        i++;                          // Increment section counter
    }
                 
y1 = LinkShield.flexRead();          // Read sensor 
y2 = LinkShield.servoRead();
LinkShield.actuatorWrite(u);          // Actuate
   
Serial.print(y1);                      // Print output  
Serial.print(", ");
Serial.print(y2);                      // Print output  
Serial.print(", ");
Serial.println(u);                    // Print input

k++;                                  // Sample counter
}
