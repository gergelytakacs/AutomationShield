#ifndef OPTOSHIELD_H_
#define OPTOSHIELD_H_

#include "Arduino.h"

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

