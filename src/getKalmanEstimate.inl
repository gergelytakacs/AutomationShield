/*
   Function used to estimate internal states of linear SISO system
   through Kalman filter equations, using system state-space matrices.
   Requires inclusion of "BasicLinearAlgebra" library to work.

   Usage:
   X = getKalmanEstimate(u,y,A,B,C,Q,R);

   Where X,A,B,C,Q,R are BLA::Matrix<rows,columns> matrix objects,
   and u,y is current value of system input and output respectively.

   This code is part of the AutomationShield hardware and software
   ecosystem. Visit http://www.automationshield.com for more
   details. This code is licensed under a Creative Commons
   Attribution-NonCommercial 4.0 International License.

   Created by Peter Chmurčiak
   Last update: 5.5.2020
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


/*
   Overloaded function that saves the estimated states directly to the specified vector X.
   Vector X has to have at least "n" rows, but also can have more. In that case, only
   its first "n" rows will get overwritten with the rest left intact.

   Function was created to be implemented in LQ with integrator - where the summation
   state is not being estimated, but is still part of state vector.

   Usage:
   getKalmanEstimate(X,u,y,A,B,C,Q,R);

   Where X,A,B,C,Q,R are BLA::Matrix<rows,columns> matrix objects,
   and u,y is current value of system input and output respectively.
*/

template <int n, int m>                                      // Template function definition
void getKalmanEstimate(BLA::Matrix<m, 1> &stateMatrix, float systemInput, float measuredOutput, BLA::Matrix<n, n> &A, BLA::Matrix<n, 1> &B, BLA::Matrix<1, n> &C, BLA::Matrix<n, n> &Q, BLA::Matrix<1, 1> &R) {
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

  for (int i = 0; i < n; i++ ) {
    stateMatrix(i) = xEstimate(i);                  // Save the "n" estimated states into output vecror with length "m" where m>=n.
  }
}

/*
   Overloaded function that saves the estimated states directly to the specified vector X.
   Vector X has to have at least "n" rows, but also can have more. In that case, only
   its first "n" rows will get overwritten with the rest left intact.

   Function was created to be implemented in MPC with integrator - where the summation
   state is not being estimated, but is still part of state vector.

   Usage:
   getKalmanEstimate(X,u,y,A,B,C,Q,R);

   Where A,B,C,Q,R are BLA::Matrix<rows,columns> matrix objects,
   X is an array of floating values, and u,y is current value of 
   system input and output respectively.   
*/

template <int n>                                      // Template function definition
void getKalmanEstimate(float* stateVector, float systemInput, float measuredOutput, BLA::Matrix<n, n> &A, BLA::Matrix<n, 1> &B, BLA::Matrix<1, n> &C, BLA::Matrix<n, n> &Q, BLA::Matrix<1, 1> &R) {
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

  for (int i = 0; i < n; i++ ) {
    stateVector[i] = xEstimate(i);                  // Save the "n" estimated states into output vecror with length "m" where m>=n.
  }
}