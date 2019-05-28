/*
  HeatShield closed-loop PID response example

  PID feedback control of the heating block temperature.
  
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
  Last update: 12.10.2018.
*/

#include <BOBShield.h>     // Include the library
#include <Sampling.h>            // Include sampling

unsigned long Ts = 10;            // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 

float r = 0.0;            // Reference
float R[]={30.0,50.0,8.0,65.0,15.0,40.0};;    // Reference trajectory
int T = 1000;           // Section length
int i = i;              // Section counter
float y = 0.0;            // Output
float u = 0.0;            // Input          

#define KP 2                  // PID Kp
#define TI 100000                  // PID Ti
#define TD 0.001                    // PID Td

void setup() {
  Serial.begin(115200);               // Initialize serial
  
  // Initialize and calibrate board
  BOBShield.begin();               // Define hardware pins
  BOBShield.initialize();
  BOBShield.calibration();
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

 PIDAbs.setKp(KP); // Proportional
 PIDAbs.setTi(TI); // Integral
 PIDAbs.setTd(TD); // Derivative
 PIDAbs.setTs(Sampling.samplingPeriod); // Sampling
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
  r = R[i];                // Set reference
  i++;
}
                  
y = BOBShield.sensorRead();           // Read sensor 
u = PIDAbs.compute(r-y,-30,30,-10,10);   // PID
BOBShield.actuatorWrite(u);           // Actuate

Serial.print(r);            // Print reference
Serial.print(", ");            
Serial.print(y);            // Print output  
Serial.print(", ");
Serial.println(u);            // Print input
k++;                  // Increment k

}
