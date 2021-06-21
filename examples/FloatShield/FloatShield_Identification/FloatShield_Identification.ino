/*
  FloatShield Identification example.

  Example used to acquire data for FloatShield system identification.

  This example initialises the sampling and PID control
  subsystems from the AutomationShield library and allows user
  to select whether to use PRBS or APRBS signal for making small 
  changes in input value. Example stabilises the ball at selected
  position using PID control and then uses selected signal to cause
  small changes in the stabilised input, with the goal of monitoring
  system behaviour while avoiding saturated positions of the ball.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 29.8.2019.
*/

#include <FloatShield.h>              // Include main library  
#include <Sampling.h>                 // Include sampling library

#define PRBS 1                        // Write 1 to use PRBS, 0 to use APRBS
                                      // For APRBS use MEGA or larger, too large for UNO

#if PRBS
#include "prbsU.h"                    // Include PRBS sequence from .h file
float prbs;                             // Variable for storing PRBS signal
#else
#include "aprbsU.h"                   // Include APRBS sequence from .h file
float aprbs;                          // Variable for storing APRBS signal
#endif

unsigned long Ts = 25;                // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation
float y = 0.0;                        // Output (Current ball altitude)
float u = 0.0;                        // Input (Fan power)

float stabilisedPower;                // Variable for storing stabilised value of power [%]
float stabilisationAltitude = 150;     // Altitude where the ball should be stabilised (range 0-320mm)
float powerSpan = 6;                // Span +/- from stabilised value of power [%]

#if SHIELDRELEASE == 1
  #define KP 0.25           // PID Kp constant
  #define TI 5              // PID Ti constant
  #define TD 0.01           // PID Td constant
#elif SHIELDRELEASE == 2
  #define KP 0.01           // PID Kp constant
  #define TI 2              // PID Ti constant
  #define TD 0.01           // PID Td constant
  #elif SHIELDRELEASE == 4
  #define KP 0.25           // PID Kp constant
  #define TI 3              // PID Ti constant
  #define TD 0.01           // PID Td constant
#endif

void setup() {                         // Setup - runs only once
    Serial.begin(250000);              // Begin serial communication

    FloatShield.begin();               // Initialise FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board

    Sampling.period(Ts*1000);          // Set sampling period in microseconds

    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                         // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod);   // Set sampling period for PID

    int stabilisationCounter=0;            // Counter to ensure that ball has been stabilised    

    while(1) {                                                               // Use PID control to stabilise the ball
        y = FloatShield.sensorReadAltitude();                                // Read sensor altitude
        u = PIDAbs.compute(stabilisationAltitude-y,10,100,10,100);           // PID
        FloatShield.actuatorWrite(u);                                        // Actuate
        if(y >= stabilisationAltitude-2.5 && y <= stabilisationAltitude+2.5) {   // If the ball is near the wanted altitude
            stabilisationCounter++;                                          // Increment counter
            if (stabilisationCounter==100) {                                 // If the ball has been moving around wanted position for a while
                stabilisedPower=u;                                           // Save value of input (u) at stabilised position
                break;                                                       // Continue program
            }
        }
        delay(Ts);                                                           // Wait before repeating the loop
    }
    Sampling.interrupt(stepEnable);                                          // Set interrupt function
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

#if PRBS
    if(k>prbsU_length) {                     // If at end of trajectory
        FloatShield.actuatorWrite(0.0);      // Turn off the fan
        while(1);                            // Stop program execution
    } else {                                 // Otherwise
        prbs = pgm_read_float_near(&prbsU[k]);
        u = stabilisedPower+prbs*powerSpan;  // Progress in trajectory
    }
#else
    if(k>aprbsU_length) {                    // If at end of trajectory
        FloatShield.actuatorWrite(0.0);      // Turn off the fan
        while(1);                            // Stop program execution
    } else {                                 // Otherwise
        aprbs = pgm_read_float_near(&aprbsU[k]);
        u = stabilisedPower+aprbs*powerSpan; // Progress in trajectory
    } 
#endif
    y = FloatShield.sensorReadAltitude();   // Read sensor
    FloatShield.actuatorWrite(u);           // Actuate

    Serial.print(y);           // Print output
    Serial.print(" ");
    Serial.println(u);         // Print input
    k++;                       // Increment index
}