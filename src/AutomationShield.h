/*
  AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
  Last change by Tibor Konkoly on 26.03.2018 at 20:51 .
*/

#ifndef AutomationShield_h  // Include guard - prevents multiple header inclusions
#define AutomationShiled_h  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  

// Defining the pins used by the Optical board
  #define OPTICAL_YPIN 1   // defining the pin of the LDR
  #define OPTICAL_UPIN 3   // defining the pin of the Led diodes (pwm)
  #define OPTICAL_RPIN 0  // defining the pin of the potentiometer's runner

// classes
 class Optical{

  public:
  // Constructor
  Optical();
  
  // Methods
  void actuatorWrite(int value);
  int sensorRead();
  int referenceRead();
  void begin(void);

  private:
  int _valueRead;
  
 }; // end of the class


#endif // End of AutomationShield library.

