/*
  MagnetoShield closed-loop MPC control example

  Model Predictive Control (MPC) of ball altitude in the 
  MagnetoShield open-source air flotation device.

  This example initialises the sampling subsystem from
  the AutomationShield library and then realises MPC control of
  the altitude of the permanent magnet. MPC control is implemented
  by using the muAO-MPC software, see MagnetoShield_Export_muAOMPC.m
  for the problem definition. The example allows teh user to select
  wheter the reference altitude is given by the potentiometer or by 
  a predetermined reference trajectory. Upload the code to your board
  and open the Serial Plotter tool in Arduino IDE.

  Tested on an Arduino Mega, as the memory of the Uno is insufficient for
  the given horizon length.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takacs.
  Created on:  9.11.2020
  Last update: 11.11.2020.
*/

#include <MagnetoShield.h>                            // Include main library  
#include <Sampling.h>                                 // Include sampling library

#ifdef ARDUINO_ARCH_AVR
  #include "AVR\ectrl.h"                                    // include EMPC matrices
#else
  #include "ectrl.h"
#endif
#include <empcSequential.h>

#define MANUAL 0                    // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

unsigned long Ts =5;                // Sampling period in miliseconds
unsigned long k = 0;                // Sample index
bool nextStep = false;              // Flag for step function
bool realTimeViolation = false;     // Flag for real-time sampling violation

float Ref[] = {14.0,13.0,14.0,15.0,14.0};   // Reference trajectory
float y = 0.0;                      // [mm] Output (Current object height)
float yp= 0.0;                      // [mm] Previous output (Object height in previous sample)
float u = 0.0;                      // [V] Input (Magnet voltage)
float I = 0.0;                      // [mA] Input (Magnet current)

float y_0= 14.3;                     // [mm] Linearization point based on the experimental identification
float I0= 21.9;                     // [mA] Linearization point based on the experimental identification
float u0= 4.6234;                   // [V] Linearization point based on the experimental identification

int T = 1000;                       // Section length
int s = 0;                          // Section counter

float X[4]  = {0.0, 0.0, 0.0, 0.0}; // Estimated initial state vector
float Xr[4] = {0.0, 0.0, 0.0, 0.0}; // Initial reference state vector
static float u_opt[MPT_RANGE];      // predicted inputs 
  
extern struct mpc_ctl ctl;          // Object representing presets for MPC controller

void setup() {                      // Setup - runs only once
  Serial.begin(250000);             // Begin serial communication

  MagnetoShield.begin();            // Initialize MagnetoShield device
  MagnetoShield.calibration();      // Calibrate MagnetoShield device

  Sampling.period(Ts * 1000);       // Set sampling period in microseconds
  Sampling.interrupt(stepEnable);   // Set interrupt function
}

void loop() {                       // Loop - runs indefinitely
  if (nextStep) {                   // If ISR enables step flag
    step();                         // Run step function
    nextStep = false;               // Disable step flag
  }
}

void stepEnable() {                                // ISR
  if (nextStep == true) {                          // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    noInterrupts();
    Serial.println("Real-time samples violated."); // Print error message
    MagnetoShield.actuatorWrite(0.0);              // Turn off the fan
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                 // Enable step flag
}

void step() {                                      // Define step function
#if MANUAL                                         // If Manual mode is active
    Xr[1]  = MagnetoShield.referenceReadAltitude()/1000;  // Read reference from potentiometer
#else                                              // If Automatic mode is active
  if (s > (sizeof(Ref) / sizeof(Ref[0]))) {        // If at end of trajectory
    MagnetoShield.actuatorWrite(0.0);              // Turn off the magnet
    while (1);                                     // Stop program execution
  } else if (k % (T * s) == 0) {                   // If at the end of section
    Xr[1]  = (Ref[s]-y_0)/1000.0;                   // Progress in reference trajectory
    s++;                                           // Increment section counter
  }
#endif

  y = MagnetoShield.sensorRead();                  // Read Hall sensor
  I = MagnetoShield.auxReadCurrent();              // Read current sensor
  
// Direct position and current measurement, simple difference for speed
// Saving valuable milliseconds for MPC algorithm
  X[0] = X[0] + (Xr[1]-X[1]);                      // Add error to the summation state
  X[1] = (y-y_0)/1000.0;                            // [m] First state, position measurement/approximation
  X[2] = (y-yp)/(1000.0*(float(Ts)/1000.0));       // [m/s] Second state, speed - approximation
  X[3]=  (I-I0)/1000.0;                            // [A] Third state, current measurement  
  yp=y;

  empcSequential(X, u_opt);                         // solve Explicit MPC problem 
  u = u_opt[0]+u0;                                 // Save system input into input variable
  MagnetoShield.actuatorWrite(u);                  // Actuate

  Serial.print((Xr[1]*1000+y_0));                   // Print reference
  Serial.print(", ");
  Serial.print(y);                                 // Print output
  Serial.print(", ");
  Serial.println(u);                               // Print input
 // Serial.println(AutomationShield.quality((Xr[1]-X[1])*1000,"ISE"));

  k++;                                             // Increment sample index
}
