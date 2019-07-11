/*
  FloatShield closed-loop PID response example

  PID feedback control of ball altitude in the FloatShield.

  This example initializes the sampling and PID control
  subsystems from the AutomationShield library. You may
  select wheter the reference is given by the potentiometer
  or you want to test a predetermined reference trajectory.
  Upload the code to your board, then open  the Serial Plotter
  tool in your Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 11.7.2019.
*/

#include <FloatShield.h>              // Include main library  
#include <Sampling.h>                 // Include sampling library

#define MANUAL 0                      // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

unsigned long Ts = 25;                // Sampling period in miliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

float r = 0.0;                                                    // Reference (Wanted ball altitude)
float R[]= {65.0,50.0,30.0,45.0,65.0,70.0,60.0,40.0,20.0,35.0};   // Reference trajectory
float y = 0.0;                                                    // Output (Current ball altitude)
float u = 0.0;                                                    // Input (Fan power)

int T = 1000;             // Section length
int i = 0;                // Section counter

#define KP 0.25           // PID Kp constant
#define TI 5              // PID Ti constant
#define TD 0.01           // PID Td constant

void setup() {                         // Setup - runs only once
    Serial.begin(250000);              // Begin serial communication

    FloatShield.begin();               // Initialise FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board

    Sampling.period(Ts*1000);          // Set sampling period in microseconds

    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                      // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID

    while(1) {                                  // Wait for ball to lift off from ground
#if MANUAL                                      // If Manual mode is active
        r = FloatShield.referenceRead();        // Read reference from potentiometer
#else                                           // If Automatic mode is active
        r = R[0];                               // Reference set to first point in trajectory
#endif
        y = FloatShield.sensorRead();           // Read sensor
        u = PIDAbs.compute(r-y,30,100,30,100);  // PID
        FloatShield.actuatorWrite(u);           // Actuate
        if(y >= r*2/3) {                          // If the ball is getting close to reference
            break;                              // Continue program
        }
        delay(50);                              // Wait 50 miliseconds before repeating
    }
    Sampling.interrupt(stepEnable);             // Set interrupt function
}

void loop() {                       // Loop - runs indefinitely
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {                                    // ISR
    if(nextStep == true) {                             // If previous sample still running
        realTimeViolation=true;                        // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
        FloatShield.actuatorWrite(0.0);                // Turn off the fan
        while(1);                                      // Stop program execution
    }
    nextStep = true;                                   // Enable step flag
}

void step() {                               // Define step function
#if MANUAL                                  // If Manual mode is active
    r = FloatShield.referenceRead();        // Read reference from potentiometer
#else                                       // If Automatic mode is active
    if(i>(sizeof(R)/sizeof(R[0]))) {        // If at end of trajectory
        FloatShield.actuatorWrite(0.0);     // Turn off the fan
        while(1);                           // Stop program execution
    } else if (k % (T*i) == 0) {            // If at the end of section
        r = R[i];                           // Progress in trajectory
        i++;                                // Increment section counter
    }
#endif
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
