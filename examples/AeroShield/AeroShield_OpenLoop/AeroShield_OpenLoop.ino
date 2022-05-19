/*
  AeroShield Open Loop
  
  Performs continuous sensor readings with manual motor speed control.
  
  The example initializes the AeroShield and performs
  calibration. Then it reads the current position
  of the potentiometer and sets the motor speed accordingly.
  
  The current position of the potentiometer and current
  angle of the pendulum are periodically (with specified
  sampling period) sent out through the serial communication,
  and can be displayed using Serial Plotter tool of Arduino IDE.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  
  Created by Peter Tibenský.
  Last update: 17.5.2022.
*/

#include "AeroShield.h"           //  Include main library

  float startAngle=0;           //  Variable for storing 0° angle in raw format
  float lastAngle=0;            //  Variable needed for mapping of the angle
  float pendulumAngle;          //  Variable used for storing the actual angle of the pendulum
  float referencePercent;       //  Variable for potentiometer value in %
  float currentMean;
 
void setup() {                // Setup - runs only once

  Serial.begin(115200);       // Begin serial communication
  AeroShield.begin();         // Initialise AeroShield board
  startAngle = AeroShield.calibration(AeroShield.getRawAngle());   //  Calibrate AeroShield board + store the 0° value of the pendulum
  lastAngle=startAngle+1024;      //  Callculation of second angle needed for map function
}

void loop() {
  
     if(pendulumAngle>120){        // If pendulum agle too big 
      AeroShield.actuatorWrite(0);  // Turn off motor 
      while(1);                     // Stop program 
      } 
  
pendulumAngle= AutomationShield.mapFloat(AeroShield.getRawAngle(),startAngle,lastAngle,0.00,90.00);    //  mapping the pendulum angle 
referencePercent= AeroShield.referenceRead();   //  Function for mapping the potentiometer input
AeroShield.actuatorWrite(referencePercent);     //  Actuate
currentMean= AeroShield.currentMeasure();       //  Read current drawn by motor 
  
Serial.print(pendulumAngle);      //  Printing the mapped angle value
Serial.print(" ");
Serial.print(referencePercent);    // Printing potentiometer value in %
Serial.print(" ");
Serial.println(currentMean);    // Printing current value in A

}
