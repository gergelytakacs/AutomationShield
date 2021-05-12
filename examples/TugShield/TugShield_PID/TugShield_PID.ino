
/*
  TugShield closed-loop PID response example
  PID feedback control of the bend.
  
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
  Created by Eva Vargova and Gergely Takacs. 
  Last update: 6.5.2019.
*/

#include "TugShield.h"              // Include the library
#include "SamplingServo.h"               // Include sampling subsystem

unsigned long Ts = 5;               // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool nextStep=false;                // Flag for sampling 
float r = 0.0;                      // Reference
float R[]={20.0,30.0,25.0,30.0,15.0,25.0};    // Reference trajectory
int T = 200;            
int i = i;                          // Section counter
float y = 0.0;                      // Output
float u ;                           // Input          

#define KP 0.5                      // PID Kp
#define TI 0.1                      // PID Ti
#define TD 0.0                      // PID Td

void setup() {
  Serial.begin(250000);              // Initialize serial
  
  // Initialize and calibrate board
  TugShield.begin();                // Define hardware pins
  TugShield.calibration();
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);       // Sampling init.
  Sampling.interrupt(stepEnable);   // Interrupt fcn.

 // Set the PID constants
 PIDAbs.setKp(KP);
 PIDAbs.setTi(TI);
 PIDAbs.setTd(TD); 
}

// Main loop launches a single step at each enable time
void loop() {
  if (nextStep) {               // If ISR enables
    step();                 // Algorithm step
    nextStep=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  nextStep=true;                  // Change flag
}

// A single algorithm step

void step(){ 

if (k % (T*i) == 0){        
  r = R[i]+40;                // Set reference
  i++;
}
                  

u = PIDAbs.compute(r-y,0,180,0,100);   // PID
TugShield.actuatorWrite((int)u);           // Actuate
y = TugShield.sensorRead();           // Read sensor 

Serial.print(r);            // Print reference
Serial.print(", ");            
Serial.print(y);            // Print output  
Serial.print(", ");
Serial.println(u);            // Print input
k++;                  // Increment k

}
