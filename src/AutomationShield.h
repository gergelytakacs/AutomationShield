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
#include "Timer.h" 
#include "PID.h"

// Headers for individual shields
#include "HeatShield.h" 
#include "OptoShield.h"



// Diagnostics
#define ECHO_TO_SERIAL      1                // echo data to serial port
#define ERRORPIN            13               // Overload Signal

// Defining pins used by the Optoshield board
  #define OPTO_RPIN 0   // defining the pin of the potentiometer's runner
  #define OPTO_YPIN 1   // defining the pin of the LDR
  #define OPTO_YAUX 2   // defining the pin of the auxiliary LDR
  #define OPTO_UPIN 3   // defining the pin of the Led diodes (pwm)

 // class(es) .h part of the library
 class AutomationShieldClass{
   
  public:
    float delta;
    float integral;
    float derivative;
    float lasterr;
    float e[3] = {0,0,0};
    float out[2] = {0,0};
    float r_p;
    float r_i;
    float r_d;
    float q0;
    float q1;
    float q2;
    float output;
    float T = (float)Ts/1000; //convert ms to s
    
  
   
  // Methods
 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

 void error(char *str);
float constrainFloat(float x, float min_x, float max_x); 

  float pid(float err,float Kp,float Ki,float Kd,float outMin,float outMax);
  float pid(float err,float Kp,float Ki,float Kd);
  float pidInc(float err,float Kp,float Ti,float Td,float outMin, float outMax); 

    AutomationShieldClass(); 
    float pidExp(float e, float Kp, float Ti, float Td);
  private:

    float eSum;
    float eArray[2]; 
   
 }; // end of the class
 extern AutomationShieldClass AutomationShield; // Declare external instance


 
 #endif // End of AutomationShield library.
