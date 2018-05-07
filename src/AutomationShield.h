/*
  AutomationShield.h
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
*/

#ifndef AutomationShield_h  // Include guard - prevents multiple header inclusions
#define AutomationShield_h  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  


// Headers for essential functionality
#include "Sampling.h" 
#include "PIDInc.h"
#include "PIDAbs.h"

// Headers for individual shields
//#include "HeatShield.h" 
//#include "OptoShield.h"



// Diagnostics
#define ECHO_TO_SERIAL      1                // echo data to serial port
#define ERRORPIN            13               // Overload Signal

// class(es) .h part of the library
 class AutomationShieldClass{
   
  public:

    
  
   
  // Methods
 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

 void error(char *str);
float constrainFloat(float x, float min_x, float max_x); 

 
  private:

    
   
 }; // end of the class

extern AutomationShieldClass AutomationShield; // Declare external instance

#endif // End of AutomationShield library.
