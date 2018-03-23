/*
  AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
*/

#include "AutomationShield.h"

/*
---Optical Shield---
*/

void OpticalClass::begin(void){
// Example begin function
  pinMode(OPTICAL_UPIN,OUTPUT); // Set output pin
}

void OpticalClass::actuatorWrite(float u){
  u = constrain(u,0.0,100.0); //Constrains to 0-100%
  int uPWM = map((int)u,0,100,0,255);
  analogWrite(OPTICAL_UPIN,uPWM);
}

float OpticalClass::sensorRead(void){
  //
  // return y;
}

float OpticalClass::referenceRead(void){
  //
  // return r;
}

OpticalClass Optical;

/*
---Motor Shield---
*/


// Motor shield....:

