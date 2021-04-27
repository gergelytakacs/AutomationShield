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

unsigned long Ts = 5;                 // Sampling in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep=false;                  // Flag for sampling 
bool realTimeViolation = false;       // Flag for real-time sampling violation
bool endExperiment = false;           // Boolean flag to end the experiment

float y = 0.0;                        // Output variable
float u = 0.0;                        // Input (open-loop), initialized to zero
float r = 0.0;                        // Reference
float R[]={0.0, 90.0}; // Input trajectory
int T = 1000;                         // Section length (appr. '/.+2 s)
int i = 0;                            // Section counter

// PPF Trial
float up = 0.0; 
float upp = 0.0;

float e = 0.0;
float ep = 0.0;
float epp = 0.0;


//Not bad
float g=2;

float b0 = 0.0;
float b1 = 0.121845865729993;
float b2 = 0.120211333659273;

float a0 = 1.0;
float a1 = -1.718562876079719;
float a2 =  0.960620075468985;

// PPF Trial end

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
        Serial.println("Real-time samples violated."); // Print error message
        realTimeViolation = true;                      // Real-time has been violated
        while(1);                                      // Stop program execution
  }   
  nextStep=true;                                       // Change flag
}

// A single algorithm step
void step(){ 

// Switching between experiment sections

if(i>(sizeof(R)/sizeof(R[0]))) {      // If at end of trajectory
        endExperiment=true;           // Stop program execution at next ISR
    } else if (k % (T*i) == 0) {      // If at the end of section
        r = R[i];                     // Progress in trajectory
        i++;                          // Increment section counter
    }

                 
y = LinkShield.sensorRead();          // Read sensor 

// PPF trial

u = -((g*(b1*ep+b2*epp)-a1*up-a2*upp)/a0);
e=-y;
ep=e;
epp=ep;
up=u;
upp=up;

// PPF trial

LinkShield.actuatorWrite(u+r);          // Actuate
   
Serial.print(y);                      // Print output  
Serial.print(", ");
Serial.println(u);                    // Print input

k++;                                  // Sample counter
}
