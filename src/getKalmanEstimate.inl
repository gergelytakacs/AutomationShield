/*
   Function used to estimate internal states of linear SISO system
   through Kalman filter equations, using system state-space matrices.
   Requires inclusion of "BasicLinearAlgebra" library to work.

   Usage:
   X = getKalmanEstimate(u,y,A,B,C,Q,R);

   Where A,B,C,Q,R are BLA::Matrix<rows,columns> matrix objects,
   and u,y is current value of system input and output respectively.

   This code is part of the AutomationShield hardware and software
   ecosystem. Visit http://www.automationshield.com for more
   details. This code is licensed under a Creative Commons
   Attribution-NonCommercial 4.0 International License.

   Created by Peter Chmurčiak
   Last update: 4.4.2020
*/

template <int n>                                             // Template function definition
BLA::Matrix<n, 1> getKalmanEstimate(float systemInput, float measuredOutput, BLA::Matrix<n, n> &A, BLA::Matrix<n, 1> &B, BLA::Matrix<1, n> &C, BLA::Matrix<n, n> &Q, BLA::Matrix<1, 1> &R) {

  static BLA::Matrix <n, 1> xEstimate;                       // State matrix
  static BLA::Matrix <n, n> P;                               // Covariance matrix
  static BLA::Matrix <n, n> I;                               // Eye matrix
  static bool wasInitialised = false;                        // Boolean used to initialize static variables at the start

  if (!wasInitialised) {                             // Initialise static variables
    for (int i = 0; i < n; i++) {
      if (C(i) == 1) {
        xEstimate(i) = measuredOutput;               // Use first measured value as initial value of the corresponding state
      }
    }
    P.Fill(0);                                       // Initialise covariance matrix with zeros
    for (int i = 0; i < n; i++) {
      for ( int j = 0; j < n; j++) {
        if (i == j) {
          I(i, j) = 1.0;                             // Create eye matrix I
        } else {
          I(i, j) = 0.0;
        }
      }
    }
    wasInitialised = true;                           // Flag initialisation as complete
  }
  
  // Prediction
  xEstimate = A * xEstimate + B * systemInput;                            // Calculate initial "a priori" state estimate
  P = A * P * ~A + Q;                                                     // Calculate error covariance (variance of error in state estimate due to process noise)
  
  // Update
  BLA::Matrix <n, 1> K = P * ~C / (C * P * ~C + R)(0);                    // Calculate the Kalman gain
  xEstimate = xEstimate + K * (measuredOutput - (C * xEstimate)(0));      // Update the state estimate - calculate corrected "a posteriori" state estimate
  P = (I - K * C) * P;                                                    // Update error covariance matrix

  return xEstimate;                                 // Return vector of estimated states
}