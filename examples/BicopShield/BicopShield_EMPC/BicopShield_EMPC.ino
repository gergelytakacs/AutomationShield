/*
  BicopShield closed-loop eMPC control example

  Explicit Model Predictive Control (eMPC).

  This example initialises the sampling subsystem from
  the AutomationShield library and then realises eMPC control of
  the angle of the arm. MPC control is implemented
  by using the MPT3 software, see BicopShield_Export_EMPC.m
  for the problem definition. The example allows the user to select
  whether the reference altitude is given by the potentiometer or by 
  a predetermined reference trajectory. Upload the code to your board
  and open the Serial Plotter tool in Arduino IDE.

  Tested on an UNO(max prediction horizon 1).

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Nemček.
  Created on:  6.5.2024
  Last update: 24.5.2024
*/

#include <BicopShield.h>              // Include main library  
#include <Sampling.h>                // Include sampling library

#include "ectrl.h"               // Include header file for EMPC controller

#include <empcSequential.h>

#define MANUAL 0                 // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

#define USE_KALMAN 0              // (1) use Kalman filter for state estimation, (0) use direct measurement and simple difference for speed

#define Ts 10.0                  // Sampling period in milliseconds
unsigned long k = 0;             // Sample index
bool nextStep = false;           // Flag for step function
bool realTimeViolation = false;  // Flag for real-time sampling violation

float R[]={0.8,1.0,0.35,1.3,1.0,0.35,0.8,1.2,0.0};    // Reference trajectory 
float y = 0.0;                   // [rad] Output (Current object height)
float yprev= 0.0;                // [rad] Previous output (Object height in previous sample)
float u1,u2;
int T = 1000;                     // Section length
int i = 0;                       // Section counter


// Kalman filter
#if USE_KALMAN 
BLA::Matrix<2, 2> A = {0.99999, 0.00998, -0.00246, 0.99525};
BLA::Matrix<2, 2> B = {7.78747048702292e-06, -9.62042244147616e-06, 0.00156, -0.00192};
BLA::Matrix<1, 2> C = {1, 0};
BLA::Matrix<2, 2> Q_Kalman = {1, 0, 0, 100};    
BLA::Matrix<1, 1> R_Kalman = {0.01};            
#endif

float X[3] = {0.0, 0.0, 0.0};    // Estimated initial state vector
float Xr[3] = {0.0, 0.0, 0.0};    // Initial reference state vector
//float Xk[2] = {0.0, 0.0};

float u = 0.0;                   // [V] Input (Magnet voltage)
static float u_opt[MPT_RANGE];      // predicted inputs 
float BaseU=50.0;                   // Base input
float ref_read;                     // Variable for potentiometer
extern struct mpc_ctl ctl;          // Object representing presets for MPC controller

void setup() {
  Serial.begin(115200);                 //--Initialize serial communication
  BicopShield.begin();                   //--Initialize AeroShield
  delay(1000);
  BicopShield.calibrate();               //  Calibrate AeroShield board + store the 0° value of the pendulum
  Serial.println("r, y, u");            //--Print header
  Sampling.period(Ts*1000.0);
  Sampling.interrupt(stepEnable);
}

void loop(){
  if (nextStep) {                  // Running the algorithm once every sample
    step();               
  nextStep = false;              // Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}

void stepEnable() {                                // ISR
 // if (nextStep) {                                  // If previous sample still running
  //  realTimeViolation = true;                      // Real-time has been violated
 //   noInterrupts();
 //   Serial.println("Real-time samples violated."); // Print error message
 //   BicopShield.actuatorWrite(0.0,0.0);                 // Turn off the fan
 //   while (1);                                     // Stop program execution
  //}
  nextStep = true;                                 // Enable step flag
}

void step() {                                      // Define step function
  #if MANUAL
  ref_read=BicopShield.referenceRead();
    Xr[1] =  AutomationShield.mapFloat(ref_read,0,100,0,100);          //--Sensing Pot reference
  #elif !MANUAL
    if(i >= sizeof(R)/sizeof(float)){     // If trajectory ended
      BicopShield.actuatorWriteVolt(0.0,0.0);  // Stop the Motor
      while(true);                        // End of program execution
    }
    if (k % (T*i) == 0){                  // Moving through trajectory values    
    //r = R[i];        
    Xr[1] = R[i];
      i++;                                // Change input value after defined amount of samples
    }
    k++;                                  // Increment
  #endif

  y = BicopShield.sensorReadRadian();      // Angle in radians
  
  #if USE_KALMAN
  BLA::Matrix<2, 1> Xk = {X[1], X[2]};
  BLA::Matrix<2, 1> U = {u_opt[0], u_opt[1]};
  BicopShield.getKalmanEstimate(Xk, U, y, A, B, C, Q_Kalman, R_Kalman);   // State estimation using Kalman filter
  X[1] = Xk(0);
  X[2] = Xk(1);
  X[0] = X[0] + (Xr[1]- X[1]); 
  #else
    // Direct angle and simple difference for Angular speed
    // Saving valuable milliseconds for MPC algorithm

    X[1] = y;                            // Arm angle
    X[2] = (y-yprev)/(float(Ts)/1000.0); // Angular speed of the arm
  #endif

  X[0] = X[0] + (Xr[1] - X[1]);          // Integral state 
  yprev=y;

  empcSequential(X, u_opt);              // solve Explicit MPC problem 
  u1 =BaseU+u_opt[0];                          // Save system input into input variable
  u2 =BaseU+u_opt[1];
 
  BicopShield.actuatorWrite(u1,u2);       // Actuation

  Serial.print(Xr[1],3);                 // Printing reference
  Serial.print(", ");            
  Serial.print(X[1],3);                  // Printing output
  Serial.print(", ");
  Serial.print(X[2],4); 
  Serial.print(", ");
  Serial.print(u1,3);                  // Printing input
  Serial.print(", ");
  Serial.print(u2,3);                   // Printing input
  Serial.print(", ");
  Serial.println(X[1],4); 
}
