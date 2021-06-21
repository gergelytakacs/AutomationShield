/*
  OptoShield Step Response

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

#include "OptoShield.h"

// calibration method determines the maximal and minimal output
// from the LDR, which is crucial for accurate measurements.
void OptoClass::calibration(){   
  # if ECHO_TO_SERIAL                        
	Serial.println("Calibration is running...");
  # endif
  delay(LDRDELAY);             		    // Wait LDR to settle
  _minVal = analogRead(OPTO_YPIN);       // Read minimal value

  analogWrite(OPTO_UPIN,255);     		// Send maximal value
  delay(LDRDELAY);					    // Wait to settle
  _maxVal = analogRead(OPTO_YPIN);      // Read input
  analogWrite(OPTO_UPIN,0);             // Switch off input
  delay(LDRDELAY);					    // Wait LDR to settle
  _wasCalibrated = true;                // Calibration performed
  # if ECHO_TO_SERIAL
	Serial.println("Calibration is done");// Send message to serial
  # endif
} 

// begin method initializes the pins used by the hardware.
// OptoShield uses three pins, pin A1 is used by the LDR, pin A0
// is connected to the runner of the potentiometer and digital 
// pin 3 is for setting the intensity of the light.
 void OptoClass::begin(void){                                                          
  pinMode(OPTO_YPIN, INPUT);
  pinMode(OPTO_UPIN, OUTPUT);
  pinMode(OPTO_RPIN, INPUT); 
  pinMode(OPTO_YAUX, INPUT);
}

// actuatorWrite method sets the LED brightness, using a floating
// point number from 0 to 100 %.
void OptoClass::actuatorWrite(float value){          // 
  _convertedValue = AutomationShield.mapFloat(value,0.00,100.00,0.00,255.00);
  analogWrite(OPTO_UPIN,_convertedValue); 
}

// sensorRead method reads the main LDR and returns the result
// in percents depending on whether it has been calibrated or not
// Returns a floating point number.
float OptoClass::sensorRead(){
   _sensorRead = analogRead(OPTO_YPIN);
   
   if(_wasCalibrated){                  // Was calibration done?
   _sensorValue = AutomationShield.mapFloat(_sensorRead, _minVal, _maxVal, 0.00, 100.00); 
   }
   else{ // Use default typical values
   _sensorValue = AutomationShield.mapFloat(_sensorRead, 20.00, 800.00, 0.00, 100.00);
   }
  return _sensorValue;
}

// sensorReadVoltage method reads the main LDR and returns the result
// in the unit of Volts, as a floating point number.
float OptoClass::sensorReadVoltage(){   
  float k = (5.00 / 1023.00);     
  _valueRead = analogRead(OPTO_YPIN);
  _sensorVoltage = _valueRead * k;
  return _sensorVoltage;  
}

// sensorAuxRead method reads the auxiliary LDR and returns the
// result in the unit of Volts, as a floating point number.
float OptoClass::sensorAuxRead(){      
 float k = (5.00 / 1023.00);      // constant that allows us to convert the analog values from the sensor into voltage 
  _auxRead = analogRead(OPTO_YAUX);
  _auxVoltage = _auxRead * k;
  return _auxVoltage;  
}

// Reads the reference potentiometer and returns a floating 
// point number with the setting scaled to per cents.
float OptoClass::referenceRead(){           
   _referenceRead = analogRead(OPTO_RPIN);
   _referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return _referenceValue;
}

// returnMinVal method return returns calibration state.
bool OptoClass::returnCalibrated(){           
  return _wasCalibrated;
}

// returnMinVal method returns maximum calibrated ADC level of
// the main LDR.
float OptoClass::returnMinVal(){
 return _minVal; 
}

// returnMaxVal method returns minimum calibrated ADC level of
// the main LDR.
float OptoClass::returnMaxVal(){
 return _maxVal; 
}      

OptoClass OptoShield; // Construct instance
