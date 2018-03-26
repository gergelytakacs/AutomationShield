/* 
 AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
  Last change by Tibor Konkoly on 25.03.2018 at 21:05.
*/


#include "AutomationShield.h"
#include "Arduino.h"

        
Optical::Optical(){
  
}

void Optical::begin(int pin1 , int pin2, int pin3){
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3; 
  
  pinMode(_pin1, INPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pin3, INPUT); 
}


void Optical::actuatorWrite(int value){
 
  if(value <= 255){                                                 // nested if statement, if the condition is true check the following
      if(value > 0){                                                  // if the second condition is also true, write the value of the sensor
        analogWrite(_pin2,value);
      }
    }
    else {Serial.println("The number You added doesn't fit the conditions, please choose a number between 0-255");} // if any of the statements is true, you receive a report 
}

int Optical::sensorRead(){
  int _valueRead = analogRead(_pin1);
  return _valueRead;
}

int Optical::referenceRead(){
  int _valueRead = analogRead(_pin3);
  return _valueRead;
}
            

                     

