/*
  Moving average filtering using a circular FIR filter
  Possible to convert easily to a generic FIR filter class
  
  Tested with Arduino Mega, but there is no reason why it should 
  not work on other types of MCU.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 26.8.2020
*/

// So far onlxFiltered moving average FIR filter,
// Should be expanded to general FIR filtering later.

// - N is the filter order
// - B is an array containing filter coefficints b(0), b(1), b(2), ..., b(N-1)

#include "FIR.h"
#include <stdlib.h>

// Creates a circular moving average FIR filter of order N
void FIR::begin(int userN)
{
     N=userN;											// User supplies the desired filter order
	 arrayInit(N);
     movingAverage(N);
     p=0;
}


void FIR::movingAverage(int userN){
	B[0] = 1.0-(1.0/(float)userN);
	for (int i=1; i<userN+1; i++){
		B[i]=-(1.0/(float)userN);
	}
}

void FIR::arrayInit(int userN){
	 B       = (float*)malloc(N * sizeof(float));
	 xBuffer = (float*)malloc(N * sizeof(float));
}

float FIR::filter(float x)
{
  p++;
  if (p > N) {									// If position reached end,
    p = 1;									    // Re-set the position to beginning
  }
  xBuffer[(int)p - 1] = x;   				    // Circulate position p
  float xAccumulator = 0.0;   					// Circulate actual x in buffer
  int k = p;  									// Reset xAccumulatorumulator
  for (int j = 0; j < N; j++) {
    xAccumulator += B[j] * xBuffer[(int)k - 1];     /* MultiplyFiltered buffer with FIR constants */
    k--;
    if (k < 1) {
      k = N;
    }
  }
  return xAccumulator;						// The final result is the last xAccumulator (this is the filtered value)
}
