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

// Defining the pins used by the Opto board
  #define OPTO_YPIN 1   // defining the pin of the LDR
  #define OPTO_UPIN 3   // defining the pin of the Led diodes (pwm)
  #define OPTO_RPIN 0  // defining the pin of the potentiometer's runner

// Diagnostics
#define ECHO_TO_SERIAL      1                // echo data to serial port
#define ERRORPIN            13               // Overload Signal



// classes
 class AutomationShield{

  public:
    #define REV 1 //reverse acting
    #define DIR 0
  // Constructor
  AutomationShield();
  
  // Methods
 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
//constrainFloat?
 void error(char *str);
  float constrain(float x, float min_x, float max_x); 
  float pid(float err,float input,float Kp,float Ki,float Kd,float outMin,float outMax,int direct);
  float pid1(float err,float Kp,float Ti,float Td,float outMin, float outMax); 
  private:
    float integral;
    float derivative;
    float lastinput;
    float lastoutput = 0;
    float lastlasterror = 0;
    float lasterror = 0;
    float r_p;
    float r_i;
    float r_d;
    float q0;
    float q1;
    float q2;
 }; // end of the class




 class Opto{

  public:
  // Constructor
  Opto();
  
  // Methods
  void actuatorWrite(int value);
  int sensorRead();
  int referenceRead();
  void begin(void);

  private:
  int _valueRead;
  
 }; // end of the class


#endif // End of AutomationShield library.

