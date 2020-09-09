/*
  MagnetoShield closed-loop MPC control example

  Model Predictive Control (MPC) of ball altitude in the 
  MagnetoShield open-source air flotation device.

  This example initialises the sampling subsystem from
  the AutomationShield library and then with the help of
  Kalman filtering realises MPC control of the ball altitude.
  It allows user to select wheter the reference altitude is
  given by the potentiometer or by a predetermined reference
  trajectory. Upload the code to your board and open the
  Serial Plotter tool in Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takacs.
  Created on:  9.9.2020
  Last update: 9.9.2020.
*/

#include <MagnetoShield.h>          // Include main library  
#include <Sampling.h>               // Include sampling library
//#include "MagnetoShield_muAO-MPC/MagnetoShield_MPC.h" // Include muAO-MPC compiled files

#define MANUAL 0                    // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

unsigned long Ts =5;                // Sampling period in miliseconds
unsigned long k = 0;                // Sample index
bool nextStep = false;              // Flag for step function
bool realTimeViolation = false;     // Flag for real-time sampling violation

float Ref[] = {14.0,13.0,14.0,15.0,14.0};   // Reference trajectory
float y = 0.0;                      // [mm] Output (Current object height)
float yp= 0.0;
float u = 0.0;                      // [V] Input (Magnet voltage)
float I = 0.0;                      // [mA] Input (Magnet current)

float y0= 14.3;                     // [mm] Linearization point based on the experimental identification
float I0= 21.9;                     // [mA] Linearization point based on the experimental identification
float u0= 4.6234;                   // [V] Linearization point based on the experimental identification

int T = 1500;                       // Section length
int i = 0;                          // Section counter

float K[4] = {0.2106E3,   -5.7575E3,   -0.1264E3,    0.0458E3};
float X[4] = {0, 0, 0, 0};          // Estimated state vector
float Xr[4] = {0, 0, 0, 0};         // Reference state vector
//extern struct mpc_ctl ctl;          // Object representing presets for MPC controller

void setup() {                       // Setup - runs only once
  Serial.begin(250000);              // Begin serial communication

  MagnetoShield.begin();             // Initialize MagnetoShield device
  MagnetoShield.calibration();         // Calibrate MagnetoShield device

  Sampling.period(Ts * 1000);        // Set sampling period in microseconds
  Sampling.interrupt(stepEnable);    // Set interrupt function
}

void loop() {                        // Loop - runs indefinitely
  if (nextStep) {                    // If ISR enables step flag
    step();                          // Run step function
    nextStep = false;                // Disable step flag
  }
}

void stepEnable() {                                // ISR
  if (nextStep == true) {                          // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    Serial.println("Real-time samples violated."); // Print error message
    MagnetoShield.actuatorWrite(0.0);              // Turn off the fan
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                 // Enable step flag
}

void step() {                                      // Define step function
#if MANUAL                                         // If Manual mode is active
  Xr[0]  = MagnetoShield.referenceReadAltitude();  // Read reference from potentiometer
#else                                              // If Automatic mode is active
  if (i > (sizeof(Ref) / sizeof(Ref[0]))) {        // If at end of trajectory
    MagnetoShield.actuatorWrite(0.0);              // Turn off the magnet
    while (1);                                     // Stop program execution
  } else if (k % (T * i) == 0) {                   // If at the end of section
    Xr[0]  = Ref[i]/1000;                          // Progress in reference trajectory
    i++;                                           // Increment section counter
  }
#endif

  y = MagnetoShield.sensorRead();           // Read sensor
  I = MagnetoShield.auxReadCurrent();
  //mpc_ctl_solve_problem(&ctl, X);                   // Calculate MPC system input
  //u = ctl.u_opt[0];                                 // Save system input into input variable
  u = -(K[0]*X[0]+K[1]*X[1]+K[2]*X[2]+K[3]*X[3])+u0;
  
  MagnetoShield.actuatorWrite(u);                   // Actuate

  X[1] = (y-y0)/1000;                               // [m] First state, position measurement/approximation
  X[2] = (y-yp)/(1000*(Ts/1000));                   // [m/s] Second state, speed - approximation
  X[3]=  (I-I0)/1000;                               // [A] Third state, current measurement
  //MagnetoShield.getKalmanEstimate(X, u, y, A, B, C, Q_Kalman, R_Kalman);    // Estimate internal states X
  
  X[0] = X[0] + (Xr[1] - X[1]);                                           // Add error to the summation state
  
  yp=y;
  Serial.print(Xr[0]*1000);       // Print reference
  Serial.print(" ");
  Serial.print(y);           // Print output
  Serial.print(" ");
  Serial.println(u);         // Print input

  k++;                       // Increment index
}
