#ifndef OPTOSHIELD_H_
#define OPTOSHIELD_H_

#include "AutomationShield.h"


// Defining pins used by the Optoshield board
  #define OPTO_RPIN 0   // defining the pin of the potentiometer's runner
  #define OPTO_YPIN 1   // defining the pin of the LDR
  #define OPTO_YAUX 2   // defining the pin of the auxiliary LDR
  #define OPTO_UPIN 3   // defining the pin of the Led diodes (pwm)
  
 class OptoClass{
  public:
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
  bool _indicator;
 }; // end of the Opto class

extern OptoClass OptoShield; // Declare external instance

#endif

