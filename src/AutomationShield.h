/*
  AutomationShield.h - Library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger [...], and Gergely Takacs
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

// Pin definitions
#define OPTICAL_YPIN 0 // Output (sensor) pin
#define OPTICAL_UPIN 9 // Input (actuator) pin
#define OPTICAL_RPIN 1 // Reference pin

// #define MOTOR_YPIN 0 
// ETC...
// ...

//--------------------------------------
// AutomationShield - common functions
//--------------------------------------
class AutomationShield
{
  public:
    // void Common functions;  // 
    // PID, LQ, filters, etc.
  private:
    //   int x;
};


extern AutomationShield AutomationShield;
//--------------------------------------
// AutomationShield - common functions
//--------------------------------------


//--------------------------------------
// Optical Shield
//--------------------------------------
class OpticalClass
{
  public:
    void begin(void);  // Initialize for OpticalShield
    void input(float); 
    float output(void);
  //float reference(void);
  //...
  private:
    //   int x;
};

extern OpticalClass Optical;

//--------------------------------------
// Optical Shield
//--------------------------------------
#endif // End of AutomationShield library.
