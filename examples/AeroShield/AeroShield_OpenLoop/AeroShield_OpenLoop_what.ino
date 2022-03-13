/*
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by 
  1ast update: 13.03.2022.
*/

#include "AeroShield.h"           //  Include main library

  float startangle=0;           //  Variable for storing 0° angle in raw format
  float lastangle=0;            //  Variable needed for mapping of the angle
  float pendulumAngle;          //  Variable used for storing the actual angle of the pendulum
  float referencePercent;       //  Variable for potentiometer value in %
  float CurrentMean;
 
void setup() {                // Setup - runs only once

  Serial.begin(115200);       // Begin serial communication
  AeroShield.begin(AeroShield.detectMagnet());         // Initialise AeroShield board
  startangle = AeroShield.calibration(AeroShield.getRawAngle());   //  Calibrate AeroShield board + store the 0° value of the pendulum
  lastangle=startangle+1024;      //  Callculation of second angle needed for map function
}

void loop() {
  
pendulumAngle= AutomationShield.mapFloat(AeroShield.getRawAngle(),startangle,lastangle,0.00,90.00);    //  mapping the pendulum angle 
Serial.print("pendulum angle is: ");
Serial.print(pendulumAngle);      //  Printing the mapped angle value
Serial.print("° || ");

referencePercent= AeroShield.referenceRead();   //  Function for mapping the potentiometer input
Serial.print("pot value is: ");
Serial.print(referencePercent);    // Printing potentiometer value in %
Serial.print("% || ");

AeroShield.actuatorWrite(referencePercent);     //  Actuate

CurrentMean= AeroShield.currentMeasure();
Serial.print("current value is: ");
Serial.print(CurrentMean);    // Printing current value in A
Serial.println("A || ");
}
