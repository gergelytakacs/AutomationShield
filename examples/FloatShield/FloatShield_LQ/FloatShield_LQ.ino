/*
  FloatShield closed-loop LQ control example

  Linear quadratic feedback control of ball altitude in the
  FloatShield open-source air flotation device.

  This example initialises the sampling subsystem from
  the AutomationShield library and then with estimates states
  by Kalman filtering and implements a linear quadratic (LQ)
  control of the ball altitude. It allows the user to select
  whether the reference altitude is given by the onboard
  potentiometer or by a predetermined reference
  trajectory. Upload the code to your board and open the
  Serial Plotter tool in Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Peter Chmurčiak.
  Last update: 24.4.2020.
*/

#include <FloatShield.h>              // Include API for FloatShield  
#include <Sampling.h>                 // Include interrupt-based sampling framework

#define MANUAL 0                      // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)

unsigned long Ts = 25;                // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

float R[] = {210.0, 160.0, 110.0, 145.0, 195.0, 245.0, 180.0, 130.0, 65.0,  95.0};    // [mm] Reference trajectory
float y = 0.0;                                                                        // [mm] Output (Current ball altitude)
float u = 0.0;                                                                        // [%]  Input (Fan power)

int T = 1200;             // Section length
int i = 0;                // Section counter for pre-set reference

// Linear, discrete state-space matrices for FloatShield model
BLA::Matrix<3, 3> A = {1, 0.02437, 0.00061, 0, 0.9502, 0.04721, 0, 0, 0.89871}; // State matrix A
BLA::Matrix<3, 1> B = {0.00011, 0.01321, 0.51677};                              // Input matrix B
BLA::Matrix<1, 3> C = {1, 0, 0};                                                // Output matrix C

// Kalman process and measurement error covariances
BLA::Matrix<3, 3> Q_Kalman = {5, 0, 0, 0, 1000, 0, 0, 0, 1000};     // Process noise covariance matrix
BLA::Matrix<1, 1> R_Kalman = {25};                                  // Measurement noise covariance matrix

// LQ gain with integrator
BLA::Matrix<1, 4> K = {0.45815, 0.1997, 0.63981, -0.00252};         // Pre-calculated LQ gain K
BLA::Matrix<4, 1> X = {0, 0, 0, 0};                                 // Estimated state vector
BLA::Matrix<4, 1> Xr = {0, 0, 0, 0};                                // Reference state vector

void setup() {                       // Setup - runs only once
  Serial.begin(115200);              // Begin serial communication

  FloatShield.begin();               // Initialise FloatShield board
  FloatShield.calibrate();           // Calibrate FloatShield board

  Sampling.period(Ts * 1000);        // Set sampling period in microseconds (Ts in ms)

  while (1) {                                                                 // Wait for ball to lift off from ground
#if MANUAL                                                                    // If Manual mode is active
    Xr(0)  = FloatShield.referenceReadAltitude();                             // Read reference from potentiometer
#else                                                                         // If Automatic mode is active
    Xr(0)  = R[0];                                                            // Reference set to first point in trajectory
#endif
    y = FloatShield.sensorReadAltitude();                                     // Read sensor
    u = -(K * (X - Xr))(0);                                                   // Calculate LQ system input
    FloatShield.actuatorWrite(u);                                             // Actuate
    FloatShield.getKalmanEstimate(X, u, y, A, B, C, Q_Kalman, R_Kalman);      // Estimate internal states X
    X(3) = X(3) + (Xr(0) - X(0));                                             // Add error to the integral state
    if (y >= Xr(0) * 2 / 3) {                                                 // If the ball is getting close to reference
      break;                                                                  // Continue program
    }
    delay(Ts);                                                                // Wait before repeating the loop
  }
  Sampling.interrupt(stepEnable);                                             // Set interrupt function
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
    Serial.println("Real-time samples violated."); // Print error message
    FloatShield.actuatorWrite(0.0);                // Turn off the fan
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                 // Enable step flag
}

void step() {                                        // Define step function
#if MANUAL                                           // If Manual mode is active
  Xr(0)  = FloatShield.referenceReadAltitude();      // Read reference from potentiometer
#else                                         // If Automatic mode is active
  if (i > (sizeof(R) / sizeof(R[0]))) {       // If at end of trajectory
    FloatShield.actuatorWrite(0.0);           // Turn off the fan
    while (1);                                // Stop program execution
  } else if (k % (T * i) == 0) {              // If at the end of section
    Xr(0)  = R[i];                            // Progress in trajectory
    i++;                                      // Increment section counter
  }
#endif

  y = FloatShield.sensorReadAltitude();                                   // Read sensor
  u = -(K * (X - Xr))(0);                                                 // Calculate LQ system input
  FloatShield.actuatorWrite(u);                                           // Actuate
  FloatShield.getKalmanEstimate(X, u, y, A, B, C, Q_Kalman, R_Kalman);    // Estimate internal states X
  X(3) = X(3) + (Xr(0) - X(0));                                           // Add error to the summation state

  Serial.print(Xr(0));       // Print reference
  Serial.print(" ");
  Serial.print(y);           // Print output
  Serial.print(" ");
  Serial.println(u);         // Print input

  k++;                       // Increment index
}
