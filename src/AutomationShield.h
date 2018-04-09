/*
  AutomationShield.h
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
  Last change by Tibor Konkoly on 09.04.2018 at 21:15 .
*/

#ifndef AutomationShield_h  // Include guard - prevents multiple header inclusions
#define AutomationShield_h  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  

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

   public:
    #define REV 1 //reverse acting
    #define DIR 0
  // Methods
 float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
 void error(char *str);

     

  private:

  
 }; // end of the AutomationShield class
 

 extern AutomationShieldClass Automationshield; // Declare external instance
 
 
 
 class OptoClass{

  public:
  
  // Methods

  void  begin(void);
  void  calibration();
  void  actuatorWrite(float value);
  float sensorRead();
  float sensorReadVoltage();
  float sensorAuxRead();
  float referenceRead();
 
  
  private:

  float _convertedValue;
  float _valueRead;
  float _sensorVoltage;
  float _auxRead;
  float _auxVoltage;
  float _referenceRead;
  float _referenceValue;
  
  float _sensorRead;
  float _sensorValue;
  float _minVal;
  float _maxVal;
  
  bool indicator;
  
 }; // end of the Opto class

 extern OptoClass OptoShield; // Declare external instance
 
 
 
 #endif // End of AutomationShield library.
 
