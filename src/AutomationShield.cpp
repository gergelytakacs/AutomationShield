/* 
 AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
  Last change by Tibor Konkoly on 09.04.2018 at 21:15 .
*/


#include "AutomationShield.h"
#include "Arduino.h"

float AutomationShieldClass::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) // same as Arudino map() but with floating point numbers
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // linear mapping, same as Arduino map()
}

void AutomationShieldClass::error(char *str) // Error handler function
{
  #if ECHO_TO_SERIAL                    // If Serial Echo is turned on in the DEFINE (by advanced user)
  Serial.print("Error: ");
  Serial.println(str);
  #endif
  digitalWrite(ERRORPIN, HIGH);           // Enable generic error pin, should be 13 w/ the onboard LED
  while (1);                              // Stop all activity in busy cycle, so user definitely knows what's wrong.
}



AutomationShieldClass AutomationShield; // Construct instance (define)

// Opto

void OptoClass::calibration(){                           // a function determines the maximal and minimal value of the LDR, which is crutial for accurate measurements
  Serial.println("Calibration is running...");
  delay(3000);                                      // we start with a 3s delay, which is needed for the value of LDR to get steady
  
  _minVal = analogRead(OPTO_YPIN);             // determining the minimal value

  analogWrite(OPTO_UPIN,255);                        // determining the maximal value
  delay(3000);
  _maxVal = analogRead(OPTO_YPIN);

  analogWrite(OPTO_UPIN,0);                          // this line switches off the LEDs

  indicator = true;

  // summary
  Serial.println("_______________________________________");
  Serial.println("__________CALIBRATION RESULTS__________");
  Serial.print("Minimal value of the LDR is: ");
  Serial.println(_minVal);
  Serial.print("Maximal value of the LDR is: ");
  Serial.println(_maxVal);
  Serial.println("_______________________________________"); 
} // end of calibration



void OptoClass::begin(void){                  // begin function initializes the pins used by the hardware. Optoshield uses three pins, pin A1 is used by the LDR,                                          //pin A0 is connected to the runner of the potentiometer and digital pin 3 is for setting the intensity of the leds' light                                            
  pinMode(OPTO_YPIN, INPUT);             // analog pin A2 is connected to the auxiliary LDR
  pinMode(OPTO_UPIN, OUTPUT);
  pinMode(OPTO_RPIN, INPUT); 
  pinMode(OPTO_YAUX, INPUT);
}


void OptoClass::actuatorWrite(float value){          // actuatorWrite() sets the LEDs' brightness, using a floating point number from 0 (min) to 100 (max)
  _convertedValue = AutomationShield.mapFloat(value,0.00,100.00,0.00,255.00);
  analogWrite(OPTO_UPIN,_convertedValue); 
}


float OptoClass::sensorRead(){
   _sensorRead = analogRead(OPTO_YPIN);
   
   if(indicator){                                                   // with an if statement I can check, if the calibration function was called (recommended)
   _sensorValue = AutomationShield.mapFloat(_sensorRead, _minVal, _maxVal, 0.00, 100.00); 
   }
   else{
   _sensorValue = AutomationShield.mapFloat(_sensorRead, 515.00, 1000.00, 0.00, 100.00); // when the calibration was not called we use static values, which were the most common values during the tests
   }
  return _sensorValue;
}


float OptoClass::sensorReadVoltage(){   // sensorReadVoltage returns the Voltage on the LDR in the tube
  float k = (5.00 / 1023.00);     // constant that allows us to convert the analog values from the sensor into voltage 
  _valueRead = analogRead(OPTO_YPIN);
  _sensorVoltage = _valueRead * k;
  return _sensorVoltage;  
}


float OptoClass::sensorAuxRead(){      // sensorAuxRead returns the Voltage on the auxiliary LDR as a floating point number
 float k = (5.00 / 1023.00);      // constant that allows us to convert the analog values from the sensor into voltage 
  _auxRead = analogRead(OPTO_YAUX);
  _auxVoltage = _auxRead * k;
  return _auxVoltage;  
}



float OptoClass::referenceRead(){           // referenceRead function returns the reference value of the potentiometer, which was set by the user, in percents
   _referenceRead = analogRead(OPTO_RPIN);
   _referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return _referenceValue;
}

OptoClass OptoShield; // Construct instance (define) 
