/*
  FloatShield automatic PID response example

  PID feedback control of ball altitude in the FloatShield.

  This example initializes the sampling and PID control
  subsystems from the AutomationShield library and starts a
  predetermined reference trajectory for alitiude of the ball
  inside the tube. Upload the code to your board, then open
  the Serial Plotter tool in your Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 2.7.2019.
*/

#include <FloatShield.h>              // Include main library  
#include <Sampling.h>                 // Include sampling library

unsigned long Ts = 25;                // Sampling period in miliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function

float r = 0.0;                               // Reference (Wanted ball altitude)
float R[]= {80.0,40.0,75.0,20.0,55.0,10.0};  // Reference trajectory
float y = 0.0;                               // Output (Current ball altitude)
float u = 0.0;                               // Input (Fan power)

int T = 400;              // Section length
int i = 0;                // Section counter

#define KP 0.25           // PID Kp constant
#define TI 5              // PID Ti constant
#define TD 0.1            // PID Td constant

void setup() {                         // Setup - runs only once
    Serial.begin(250000);              // Begin serial communication                            
   
    FloatShield.begin();               // Initialise FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board

    Sampling.period(Ts*1000);          // Set sampling period in microseconds
    Sampling.interrupt(stepEnable);    // Set interrupt function

    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                      // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID

    while(1) {                                // Wait for ball to lift off from ground
        r = R[0];                             // Refenrece set to first point in trajectory
        y = FloatShield.sensorRead();         // Read sensor
        u = PIDAbs.compute(r-y,0,100,0,100);  // PID
        FloatShield.actuatorWrite(u);         // Actuate
        if(y > 30.0) {                        // If the ball sucessfully lifted off
            break;                            // Continue program
        }
    }
}

void loop() {                       // Loop - runs indefinitely
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {                 // ISR
    nextStep = true;                // Enable step flag
}

void step() {                       // Define step function
    if (k % (T*i) == 0) {           // Way of progressing in trajectory        
        r = R[i%6];                 // Progress in reference trajectory
        i++;                        // Increment counter     
    }
    y = FloatShield.sensorRead();         // Read sensor
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID
    FloatShield.actuatorWrite(u);         // Actuate
    
    Serial.print(r);           // Print reference
    Serial.print(" ");
    Serial.print(y);           // Print output
    Serial.print(" ");
    Serial.println(u);         // Print input
    
    k++;                       // Increment index
}
