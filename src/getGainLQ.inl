/*
   Function used to calculate K gain for LQ control applications.
   Requires inclusion of "BasicLinearAlgebra" library to work.

   Usage:
   K = getGainLQ(A,B,Q,R);

   Where K,A,B,Q,R are BLA::Matrix<rows,columns> matrix objects.

   This code is part of the AutomationShield hardware and software
   ecosystem. Visit http://www.automationshield.com for more
   details. This code is licensed under a Creative Commons

   Created by Peter Chmurčiak based on work of Anna Vargová
   Last update: 11.03.2020
*/

template <int m, int n>                    // Template function definition
BLA::Matrix<m, n> getGainLQ(BLA::Matrix<n, n> &A, BLA::Matrix<n, m> &B, BLA::Matrix<n, n> &Q, BLA::Matrix<m, m> &R) {

  BLA::Matrix <m, n> K;                    // K gain matrix
  K.Fill(0.0);                             // Initialised with zeros

  BLA::Matrix <m, n> difference;           // difference matrix used to store K changes in between iterations
  K.Fill(-1.0);                            // Initialised with negative ones

  float differenceSum = -1.0;              // differenceSum variable used as while-stop condition

  BLA::Matrix <n, n> P;                    // P matrix
  for (int i = 0; i < n; i++) {            // Initialised as identity matrix
    for ( int j = 0; j < n; j++) {
      if (i == j) {
        P(i, j) = 1.0;
      } else {
        P(i, j) = 0.0;
      }
    }
  }

  BLA::Matrix <m, m> Inv ;                 // Auxiliary Inv matrix for inversion calculation
  Inv.Fill(0.0);                           // Initialised with zeros

  while (differenceSum != 0.0) {                                // While K differences between iterations are not 0, continue iterating
    differenceSum = 0.0;
    difference = K;
    P = (~(A - B * K) * P) * (A - B * K) + Q + ~K * R * K;      // Lyapunov equation
    Inv = (R + (~ B * P) * B);
    K = Invert (Inv) * ((~ B * P) * A);                         // Calculate gain K
    difference -= K;                                            // Get difference matrix of current K and previous K
    for (int i = 0; i < m; i++) {                               // Get sum of individual element differences
      for ( int j = 0; j < n; j++) {
        differenceSum = differenceSum + abs(difference(i, j));
      }
    }
  }
  return K;                                                     // Return gain K
}