/*
  BoBShield closed-loop PID response example

  PID feedback control of the servo.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and starts a 
  predetermined reference trajectory for the ball position. 
  
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács. 
  Last update: 10.2.2021.
*/

#include <BOBShield.h>
#include <SamplingServo.h>

unsigned long Ts = 10;            // Sampling in milliseconds
unsigned long k = 0;              // Sample index
bool enable=false;                // Flag for sampling 

float r = 0.0;                    // Reference
float R[]={30.0,50.0,8.0,65.0,15.0,40.0};;    // Reference trajectory

int T = 1000;           // Section length
int i = 0;              // Section counter
float y = 0.0;            // Output
float u = 0.0;            // Input          


#define KP 0.3                    // PID Kp
#define TI 600.0                  // PID Ti
#define TD 0.22                   // PID Td

void setup() {
  Serial.begin(115200);               // Initialize serial
  
  // Initialize and calibrate board
  BOBShield.begin();               // Define hardware pins
  BOBShield.initialize();
  BOBShield.calibration();
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

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
  enable=true;                  // Change flag
}


void step(){               // A single algorithm step
if (k % (T*i) == 0){       // End of reference section 
  r = R[i];                // Set reference
  i++;                     // Increment reference counter
}
// Measure, compute, actuate:                  
y = BOBShield.sensorRead();              // Read sensor 
u = PIDAbs.compute(r-y,-30,30,-10,10);   // PID
BOBShield.actuatorWrite(u);              // Actuate

Serial.print(r);            // Print reference
Serial.print(", ");            
Serial.print(y);            // Print output  
Serial.print(", ");
Serial.println(u);            // Print input
k++;                  // Increment k

}
