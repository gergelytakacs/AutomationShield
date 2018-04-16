/*
  AutomationShield.h
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
*/

/* ======================================== N O T E ======================================================  
I deleted the whole part of the branch. This branch was created for finalizing the sampling function, 
and just contains my version of the function. After assembling and finalizing the final function can be put into the 
AutomationShield class or into another one. 
The ISR function is now hidden, but it has some problems with the accuracy of sampling. Not every sample is accurate, some of 
them are triggered sooner or later by 1ms. I must figure out what causes this problem.
===========================================================================================================
*/


#ifndef Sampling_h  // Include guard - prevents multiple header inclusions
#define Sampling_h  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  



class Sampling {
  
public:   

       // methods
       void samplingTime(float sampleTime); 
       

       
       
private:
       float _testValue;
       int   _i;




};

extern Sampling testSample;

extern bool StepEnable;
extern float Ts;    // external variable for the sampling time




#endif 
