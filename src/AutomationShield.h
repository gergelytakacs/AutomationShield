/*
  AutomationShield API
  This library serves as an API for the AutomationShield 
  ecosystem of Arduino Shields used for control engineering and
  mechatronics education.
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takacs, Tibor Konkoly, Gabor Penzinger
  Last update: 31.09.2018.
*/

#ifndef AUTOMATIONSHIELD_H  // Include guard - prevents multiple header inclusions
#define AUTOMATIONSHIELD_H  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  

#include "sampling/SamplingCore.h"
#include <PIDAbs.h>


// THESE ARE NOT VISIBLE OUTSIDE THE SCOPE OF THE FILE, DO WE NEED THEM?
// Common definitions
#ifdef ARDUINO_ARCH_AVR                     // Chip uses 10-bit ADC

	#define ADCREF 1023.0					// 10-bit resolution for AD converter
#elif ARDUINO_ARCH_SAMD || ARDUINO_ARCH_SAM || ARDUINO_ARCH_RENESAS_UNO  // Chip uses 12-bit ADC
	#define ADCREF 4095.0					// 12-bit resolution for AD converter
#endif

#define AREF 5.0                              // ADC reference voltage for 5 V logic
#define ARES AREF/ADCREF                      // ADC resolution for 5 V logic
#define AREF3V3 3.3                           // ADC reference voltage for 3.3 V logic
#define ARES3V3 AREF3V3/ADCREF                // ADC resolution for 3.3 V logic
#define ABSZERO 273.15 // Absolute zero in Celsius 

// Diagnostics
#define ECHO_TO_SERIAL      0                // echo data to serial port
#define ERRORPIN            13               // Overload Signal

// class(es) .h part of the library
 class AutomationShieldClass{   
 public:    
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    void error(const char *str);
    float constrainFloat(float x, float min_x, float max_x); 
    byte percToPwm(float perc); 
	float quality(float, char *method); 															// Quality metric for feedback control input
	
	// Printing functions
  void serialPrint(const char *str); 														// Should be renamed to diagnostic printLowHigh
	void print(float, float, float);
	void printLowHigh(char *named, float low, float high, char *unit, short int precision); // Prints a single line for range measurements in an ordered form
	void printSeparator(char); 																// Prints a line of dashes, 60 characters wide, then a new line.
	void printLowHighFirst(void); 															// Creates a header for displaying numeric ranges with a label and unit
	bool printTestResults(char *text,float value, float low, float high);	 				// Evaluates and prints if a number fits into a range	


  private:
	float qualityVal;																		// Variable to store quality at all times
}; // end of the class




extern AutomationShieldClass AutomationShield; // Declare external instance

#endif // End of AutomationShield library.

