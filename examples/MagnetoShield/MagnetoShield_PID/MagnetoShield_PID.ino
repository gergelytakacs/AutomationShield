/*
  MagnetoShield closed-loop PID response example

  PID feedback control for magnet levitation.
  
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
  Last update: 14.19.2018.
*/
#include <MagnetoShield.h>     // Include header for hardware API

// PID Tuning
#define KP 2.2            // PID Kp
#define TI 0.1            // PID Ti
#define TD 0.04           // PID Td

unsigned long Ts = 4000;            // Sampling in microseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 
bool realTimeViolation=false;       // Flag for real-time violations

float r = 0.0;            // Reference
float R[]={15.0,15.0,15.0,15.0,15.0};    // Reference trajectory
int T = 1000;           // Section length (steps) 1 hrs
int i = i;              // Section counter
float y = 0.0;            // Output
float u = 0.0;            // Input          

void setup() {
  Serial.begin(2000000);               // Initialize serial
  // Initialize and calibrate board
  MagnetoShield.begin();               // Define hardware pins
  //MagnetoShield.calibration();                  // Calibrates shield
     
  // Initialize sampling function
  Sampling.interruptInitialize(Ts);   // Sampling init.
  Sampling.setInterruptCallback(stepEnable); // Interrupt fcn.

 // Set the PID constants
 PIDAbs.setKp(KP);
 PIDAbs.setTi(TI);
 PIDAbs.setTd(TD); 
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {               // If ISR enables
    step();                 // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  if(enable){
  realTimeViolation=true;
  Serial.println("Real-time samples violated.");
  while(1);
  }
  enable=true;                  // Change flag
}

// A single algoritm step

void step(){ 

if (k % (T*i) == 0){        
  r = R[i];                // Set reference
  i++;
}

y = MagnetoShield.sensorRead();           // Sensor Read (~mm)
u = PIDAbs.compute(r-y,0,12,0,20);   // PID
MagnetoShield.actuatorWrite(u);       // Actuate

Serial.print(r);               // Print reference
Serial.print(", ");            
Serial.print(y);               // Print output  
Serial.print(", ");
Serial.println(u);            // Print input
k++;                          // Increment k
}