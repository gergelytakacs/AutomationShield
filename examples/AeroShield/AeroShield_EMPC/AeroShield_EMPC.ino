/*
  AeroShield closed-loop eMPC control example

  Explicit Model Predictive Control (eMPC) of the pendulum arm in 
  AeroShield open-source aero pendulum device.

  This example initialises the sampling subsystem from
  the AutomationShield library and then realises eMPC control of
  the angle of the arm. MPC control is implemented
  by using the MPT3 software, see AeroShield_Export_EMPC.m
  for the problem definition. The example allows the user to select
  whether the reference altitude is given by the potentiometer or by 
  a predetermined reference trajectory. Upload the code to your board
  and open the Serial Plotter tool in Arduino IDE.

  Tested on an Arduino Mega, DUE and UNO(max prediction horizon 3).

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Erik Mikuláš.
  Created on:  8.8.2023
  Last update: 9.8.2023
*/

#include <AeroShield.h>                            // Include main library  
#include <Sampling.h>                              // Include sampling library

#include "ectrl.h"

#include <empcSequential.h>

#define MANUAL 0                    // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

#define USE_KALMAN 0

#define Ts 10.0                // Sampling period in milliseconds
unsigned long k = 0;                // Sample index
bool nextStep = false;              // Flag for step function
bool realTimeViolation = false;     // Flag for real-time sampling violation

float R[]={0.8,1.0,0.35,1.3,1.0,0.35,0.8,1.2,0.0};    // Reference trajectory 
float y = 0.0;                      // [rad] Output (Current object height)
float yprev= 0.0;                   // [rad] Previous output (Object height in previous sample)
float u = 0.0;                      // [V] Input (Magnet voltage)

int T = 1000;               // Section length
int i = 0;                  // Section counter

// Kalman filter
#if USE_KALMAN 
  BLA::Matrix<3, 3> A = {-1, 0, 1, 0.9998, 0.003, 0, -0.1355, 0.9896, 0};
  BLA::Matrix<3, 1> B = {0, 0.00014989, 0.0997};
  BLA::Matrix<1, 3> C = {0, 1, 0};
  BLA::Matrix<3, 3> Q_Kalman = {1, 0, 0, 0, 1, 0 ,0, 0, 0};    //--process noise  covariance matrix
  BLA::Matrix<3, 3> R_Kalman = {1,0,0,1};                    //--measurement noise covariance matrix
  BLA::Matrix<3, 1> xIC = {0, 0, 0};                                //--Kalman filter initial conditions
#endif

float X[3]  = {0.0, 0.0, 0.0};    // Estimated initial state vector
float Xr[3] = {0.0, 0.0, 0.0};    // Initial reference state vector

static float u_opt[MPT_RANGE];      // predicted inputs 
  
extern struct mpc_ctl ctl;          // Object representing presets for MPC controller

void setup() {
  Serial.begin(250000);                 //--Initialize serial communication
  AeroShield.begin();                   //--Initialize AeroShield
  AeroShield.calibrate();               //  Calibrate AeroShield board + store the 0° value of the pendulum
  Serial.println("r, y, u");            //--Print header
  Sampling.period(Ts*1000.0);
  Sampling.interrupt(stepEnable);
  //delay(1000);
}

void loop(){
  if (y > PI) {                   // If pendulum agle too big
    AeroShield.actuatorWrite(0);  // Turn off motor
    while (1);                    // Stop program
  }
  if (nextStep) {      // Running the algorithm once every sample
    step();               
    nextStep = false;  // Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}

void stepEnable() {                                // ISR
  if (nextStep) {                                  // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    noInterrupts();
    Serial.println("Real-time samples violated."); // Print error message
    AeroShield.actuatorWrite(0.0);                 // Turn off the fan
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                 // Enable step flag
}

void step() {                                      // Define step function
  #if MANUAL
    r = AeroShield.referenceRead();          //--Sensing Pot reference
  #elif !MANUAL
    if(i >= sizeof(R)/sizeof(float)){ // If trajectory ended
      AeroShield.actuatorWriteVolt(0.0); // Stop the Motor
      while(true); // End of program execution
    }
    if (k % (T*i) == 0){ // Moving through trajectory values    
    //r = R[i];        
    Xr[1] = R[i];
      i++;             // Change input value after defined amount of samples
    }
    k++;                              // Increment
  #endif

  y = AeroShield.sensorReadRadian(); // Angle in radians
  
  // Direct angle and simple difference for speed
  // Saving valuable milliseconds for MPC algorithm
  X[1] = y;                      // Arm angle
  X[2] = (y-yprev)/(float(Ts)/1000.0);  // Angular speed of the arm
  #if USE_KALMAN
    BLA::Matrix<2, 1> Y = {X(1), X(2)};
    AeroShield.getKalmanEstimate(X, u, Y, A, B, C, Q_Kalman, R_Kalman, xIC);   // State estimation using Kalman filter
  #endif
  X[0] = X[0] + (Xr[1] - X[1]);  // Integral state 
  yprev=y;

  empcSequential(X, u_opt);                      // solve Explicit MPC problem 
  u = u_opt[0];                                 // Save system input into input variable
  AeroShield.actuatorWriteVolt(u);              // Actuation

  Serial.print(Xr[1],3);            // Printing reference
  Serial.print(" ");            
  Serial.print(X[1],3);             // Printing output
  Serial.print(" ");
  Serial.println(u,3);              // Print input

  k++;                                             // Increment sample index
}
