/*
  FloatShield closed-loop PID response example

  PID feedback control of ball altitude in the FloatShield.

  This example initialises the sampling and PID control
  subsystems from the AutomationShield library and allows user
  to select wheter the reference is given by the potentiometer
  or by a predetermined reference trajectory. Upload the code to 
  your board and open the Serial Plotter tool in Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 17.7.2019.
*/

#include <FloatShield.h>              // Include main library  
#include <Sampling.h>                 // Include sampling library
#include "U.h"                        // Include PRBS sequence

unsigned long Ts = 25;                // Sampling period in miliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation
float y = 0.0;                        // Output (Current ball altitude)
float u = 0.0;                        // Input (Fan power)

float pStart = 46.0;                   // Starting power [%]
float center = 45.0;                  // Center of PRBS, power [%]
float span = 6.0;                     // Span +/- from center of PRBS, power [%]
int PRBS;                             // PRBS signal in the header

void setup() {                         // Setup - runs only once
    Serial.begin(250000);              // Begin serial communication
    
    FloatShield.begin();               // Initialise FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board
    
    FloatShield.actuatorWrite(pStart);   // Starting power to starting position
    delay(15000);                      // Wait 10 s to settle
    
    Sampling.period(Ts*1000);          // Set sampling period in microseconds
    Sampling.interrupt(stepEnable);    // Set interrupt function
}

void loop() {                       // Loop - runs indefinitely
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {                                    // ISR
    if(nextStep == true) {                             // If previous sample still running
        realTimeViolation = true;                      // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
        FloatShield.actuatorWrite(0.0);                // Turn off the fan
        while(1);                                      // Stop program execution
    }
    nextStep = true;                                   // Enable step flag
}

void step() {                               // Define step function

    if(k>U_length) {        // If at end of trajectory
        FloatShield.actuatorWrite(0.0);     // Turn off the fan
        while(1);                           // Stop program execution
    } else {                                // Otherwise
        PRBS = pgm_read_word(&U[k]);
        u = center+(float)PRBS*span;           // Progress in trajectory
    }

    y = FloatShield.sensorReadAltitude();   // Read sensor
    FloatShield.actuatorWrite(u);           // Actuate

    Serial.print(y);           // Print output
    Serial.print(" ");
    Serial.println(u);         // Print input
    k++;                       // Increment index
}