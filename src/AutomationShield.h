/*
  AutomationShield API

  This library serves as an API for the AutomationShield 
  ecosystem of Arduino Shields used for control engineering and
  mechatronics education.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács, Tibor Konkoly, Gábor Penzinger
  Last update: 31.09.2018.
*/

#ifndef AUTOMATIONSHIELD_H_  // Include guard - prevents multiple header inclusions
#define AUTOMATIONSHIELD_H_  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  

#include <sampling\SamplingCore.h>
#include <PIDAbs.h>


// THESE ARE NOT VISIBLE OUTSIDE THE SCOPE OF THE FILE, DO WE NEED THEM?
// Common definitions
#define AREF 5.0 // ADC reference voltage for 5 V logic
#define ARES AREF/1023.0 // ADC resolution for 5 V logic
#define AREF3V3 3.3 // ADC reference voltage for 3.3 V logic
#define ARES3V3 AREF3V3/1023.0 // ADC resolution for 3.3 V logic
#define ABSZERO 273.15 // Absolute zero in Celsius 

// Diagnostics
#define ECHO_TO_SERIAL      0                // echo data to serial port
#define ERRORPIN            13               // Overload Signal

// class(es) .h part of the library
 class AutomationShieldClass{   
  public:    
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    void error(char *str);
    float constrainFloat(float x, float min_x, float max_x); 
    byte percToPwm(float perc); 
}; // end of the class

extern AutomationShieldClass AutomationShield; // Declare external instance

#endif // End of AutomationShield library.
