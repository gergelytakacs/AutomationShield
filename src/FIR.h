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

#ifndef FIR_H
#define FIR_H

 class FIR{ 
	public:
		void  begin(int);		// Creates a moving average filter of order N
		float filter(float);
		
	private:
		void movingAverage(int);
		void arrayInit(int);
		
		int p;					// Filter position
		int N; 					// FIR filter order
		float* xBuffer;  		// Circular buffer storing filter states
		float* B;				// Array of filter coefficients B = [b(0), b(1), ..., b(N-1)]
 };

#endif