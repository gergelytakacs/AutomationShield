/*
  API for the OptoShield hardware.
  
  The file is a part of the application programmers interface for
  the OptoShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Tibor Konkoly. 
  Last update: 28.09.2018.
*/

#ifndef OPTOSHIELD_H_                // Include guard
#define OPTOSHIELD_H_

#include "AutomationShield.h"        // Include the main library

// Defining pins used by the OptoShield board
  #define OPTO_RPIN 0   // Potentiometer runner (Reference)
  #define OPTO_YPIN 1   // LDR (Sensor)
  #define OPTO_YAUX 2   // Auxiliary LDR
  #define OPTO_UPIN 3   // LED (Actuator)

  // Other constants
  #define LDRDELAY 1000 // Calibration delays
  
 class OptoClass{
  public:
  void  begin(void);				 // Board initialization
  void  calibration();				 // Board calibration
  void  actuatorWrite(float value);  // Write actuators
  float sensorRead();                // Read mean sensor in %
  float sensorReadVoltage();		 // Read mean sensor in V
  float sensorAuxRead();			 // Read aux. sensor in V
  float referenceRead();			 // Read reference pot in %
  bool returnCalibrated();           // Is calibrated?
  float returnMinVal();              // Minimum main LDR ADC 
  float returnMaxVal();				 // Maximum main LDR ADC 
  
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
  bool _wasCalibrated;
 }; 

extern OptoClass OptoShield; // Declare external instance

#endif
