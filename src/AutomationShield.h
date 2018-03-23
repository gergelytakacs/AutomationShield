/*
  AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
*/

#ifndef AutomationShield_h  // Include guard - prevents multiple header inclusions
#define AutomationShiled_h  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif       

#define VERSION 2       // Version of this library
// Pin definitions
#define OPTICAL_YPIN  0 // Output (sensor) pin
#define OPTICAL_UPIN  9 // Input (actuator) pin
#define OPTICAL_RPIN  1 // Reference pin

// #define MOTOR_YPIN 0 
// ETC...
// ...

/*
---Common functions for all shields---
*/

class AutomationShield
{
  public:
    // void Common functions;  // 
    // PID, LQ, filters, etc.
  private:
    //   int x;
};


extern AutomationShield AutomationShield;

/*
---Optical Shield---
*/
class OpticalClass
{
  public:
    void  begin(void);           // Initialize for OpticalShield
    void  actuatorWrite(float);  // Write to actuator
    float sensorRead(void);      // Read from sensor
    float referenceRead(void);   // Read from reference pot
  //...
  private:
    //   int x;
};

extern OpticalClass Optical;

/*
---Motor Shield---
*/


// Here comes the motor library


#endif // End of AutomationShield library.
