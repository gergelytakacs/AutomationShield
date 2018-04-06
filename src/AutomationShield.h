/*
  AutomationShield.h
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
  Last change by Tibor Konkoly on 06.04.2018 at 21:25 .
*/

#ifndef AutomationShield_h  // Include guard - prevents multiple header inclusions
#define AutomationShield_h  // If not already there, include

#if (ARDUINO >= 100)  // Libraries don't include this, normal Arduino sketches do
 #include "Arduino.h" // For new Arduino IDE
#else
 #include "WProgram.h" // For old Arduino IDE
#endif  

// functions defined by us
// 1.
 extern float mapFloat(float x, float in_min, float in_max, float out_min, float out_max); // same as Arudino map() but with floating point numbers



class AutomationShield{

  public:

   // Constructor
   AutomationShield();

   // Methods

  private:

  
 }; // end of the AutomationShield class




// Defining pins used by the Optoshield board
  #define OPTO_RPIN 0   // defining the pin of the potentiometer's runner
  #define OPTO_YPIN 1   // defining the pin of the LDR
  #define OPTO_YAUX 2   // defining the pin of the auxiliary LDR
  #define OPTO_UPIN 3   // defining the pin of the Led diodes (pwm)
  
 
  class Opto{

  public:
  
  // Constructor
  Opto();
  
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
 
 #endif // End of AutomationShield library.
 
