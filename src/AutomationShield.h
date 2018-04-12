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
The only problem is that I can not hide the interrupt service routine (yet). 
I think the solution might be using the same method which was also used by Richard.
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
       void samplingTime(float Ts); 
       float test;
       int i;
       
       
private:




};

extern Sampling testSample;

extern bool StepEnable;



#endif 
