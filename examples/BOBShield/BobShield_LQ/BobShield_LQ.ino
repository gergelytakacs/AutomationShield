
/*
  FloatShield closed-loop LQ control example

  Linear quadratic feedback control of ball altitude in the
  BoBShield open-source air flotation device.

  This example initialises the sampling subsystem from
  the AutomationShield library and then with estimates states
  by Kalman filtering and implements a linear quadratic (LQ)
  control of the ball altitude. It allows the user to select
  wheter the reference altitude is given by the onboard
  potentiometer or by a predetermined reference
  trajectory. Upload the code to your board and open the
  Serial Plotter tool in Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Anna Vargova.
  Last update: 24.4.2021.
*/



#include <BOBShield.h>              // Include API for BobShield  
#include <SamplingServo.h>
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h">
#include <getKalmanEstimate.inl>
#include <getGainLQ.inl>

unsigned long Ts = 10;                // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool enable = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation


float R[] = {30,50,8, 65,15,4};    // [m] Reference trajectory
float y = 0.0;                                                                        // [mm] Output (Current ball altitude)
float u = 0.0;                                                                        // [%]  Input (Fan power)

int T = 1000;             // Section length
int i = 0;                // Section counter for pre-set reference


//Defining State-Space matrices of the system

BLA::Matrix<2, 2> A = {1.0, 0.01, 0.0,  1.0}; // State matrix A
BLA::Matrix<2, 1> B = {0.0002, 0.0422};                              // Input matrix B
BLA::Matrix<1, 2> C = {1.0, 0.0};                                                // Output matrix C

// Kalman process and measurement error covariances
BLA::Matrix<2, 2> Q_Kalman = {100, 0.0,0.0,1.0};     // Process noise covariance matrix
BLA::Matrix<1, 1> R_Kalman = {100};                                  // Measurement noise covariance matrix

// LQ gain with integrator
BLA::Matrix<1, 3> K = {5.7589, 1.6407, -0.0965};         // Pre-calculated LQ gain K
BLA::Matrix<2, 1> X = {0.0, 0.0};                                 // Estimated state vector
BLA::Matrix<3, 1> XI = {0.0, 0.0, 0.0};                                 // Estimated state vector including integrator state
BLA::Matrix<3, 1> Xr = {0.0, 0.0, 0.0};                                // Reference state vector 

void setup() { // Setup - runs only once
  Serial.begin(115200);               // Initialize serial

  // Initialize and calibrate board
  BOBShield.begin();               // Define hardware pins (with no sensor troubleshooting) 
  //BOBShield.initialize(); //only the sensor, including some troubleshooting
  BOBShield.calibration();
  BOBShield.actuatorWrite(0.0); //should be included in callibration

  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.
                                                                     // If Automatic mode is active

  //initial step
  Xr(0)  = R[0]; 
  y = BOBShield.sensorRead();           // Read sensor 
  u =( -(K * (XI - Xr))(0));                                                 // Calculate LQ system input
  u=rad2deg(u);
  if (u>30.0) {u=30.0;} //could be integrated tu actuatorwrite function
  if (u<-30.0) {u=-30.0;}
  BOBShield.actuatorWrite(u);                                           // Actuate
  getKalmanEstimate(X, deg2rad(u), y, A, B, C, Q_Kalman, R_Kalman);    // Estimate internal states X
  XI(2) = XI(2) + (Xr(0) - XI(0));    
 
}

void loop() {
  // put your main code here, to run repeatedly:
   if (enable) {                   // If ISR enables step flag
    step();                         // Run step function
    enable = false;               // Disable step flag
  }
}

void step(){ // Define step function
 if (i > (sizeof(R) / sizeof(R[0]))) {       // If at end of trajectory

    // Serial.println("End");                //only for debuging  
    while (1);                                // Stop program execution
  } 
  else if (k % (T * i) == 0) {              // If at the end of section
    Xr(0)  = R[i];                            // Progress in trajectory
    i++;                                      // Increment section counter
  }  
  
  y = BOBShield.sensorRead();           // Read sensor 
  u =( -(K * (XI - Xr))(0));                                                 // Calculate LQ system input
  u=rad2deg(u);
  if (u>30.0) {u=30.0;}
  if (u<-30.0) {u=-30.0;}
  BOBShield.actuatorWrite(u);                                           // Actuate
  getKalmanEstimate(XI, deg2rad(u), y, A, B, C, Q_Kalman, R_Kalman);    // Estimate internal states X
  XI(2) = XI(2) + (Xr(0) - XI(0));                                           // Add error to the summation state

  Serial.print(Xr(0));       // Print reference
  Serial.print(" ");
  Serial.print(y);           // Print output
  Serial.print(" ");
  Serial.println(u);         // Print input

  k++;                       // Increment index
  }


void stepEnable() {                                // ISR
  if (enable == true) {                          // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    Serial.println("Real-time samples violated."); // Print error message
    BOBShield.actuatorWrite(0.0);                // Turn off the fan
    while (1);                                     // Stop program execution
  }
  enable = true;                                 // Enable step flag
}



//transformations between radians and degrees - will be integrated to AutomationShield library
  float deg2rad(float u){
    u=u*(3.14/180.0);
    return u;
    }

    float rad2deg(float u){
    u=u*(180.0/3.14);
    return u;
    }
