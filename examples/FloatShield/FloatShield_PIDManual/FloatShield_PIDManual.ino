/*
  FloatShield manual PID response example

  PID feedback control of ball altitude in the FloatShield.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and allows the
  user to manually create a reference alitiude for the ball
  inside the tube. Upload the code to your board, then open
  the Serial Plotter tool in your Arduino IDE. Adjust the 
  reference by turning the potentiometer knob.
  
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
bool nextStep = false;                // Flag for step function

float r = 0.0;            // Reference (Wanted ball altitude)
float y = 0.0;            // Output (Current ball altitude)
float u = 0.0;            // Input (Fan power)

#define KP 0.25           // PID Kp constant
#define TI 5              // PID Ti constant
#define TD 0.1            // PID Td constant

void setup() {                         // Setup - runs only once
    Serial.begin(250000);              // Begin serial communication
    
    FloatShield.begin();               // Initialize FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board

    Sampling.period(Ts*1000);          // Set sampling period in microseconds
    Sampling.interrupt(stepEnable);    // Set interrupt function

    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                      // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID
}

void loop() {                       // Loop - runs indefinitely                     
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {               // ISR  
    nextStep = true;              // Enable step flag
}

void step() {                             // Define step function
    r = FloatShield.referenceRead();      // Read reference
    y = FloatShield.sensorRead();         // Read sensor    
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID    
    FloatShield.actuatorWrite(u);         // Actuate

    Serial.print(r);           // Print reference
    Serial.print(" ");
    Serial.print(y);           // Print output   
    Serial.print(" ");
    Serial.println(u);         // Print input    
}
