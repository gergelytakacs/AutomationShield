/*
  MotoShield API exaple - Open loop.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 23.4.2020.
*/
#include <MotoShield.h>
#define TS 100.0 //--Sampling period in milliseconds
void setup() {
  Serial.begin(2000000);//--Initialize serial communication # 2 000 000 baudrate
  MotoShield.begin(TS);//--Initialize MotoShield with sampling period 100 millis
  MotoShield.calibration();//--Calibration method
}
  float r;
void loop() {
  r = MotoShield.referenceRead(); //--Reading potentiometers (reference) value in percents
  MotoShield.actuatorWrite(r); //--Defining potentiometers value as input
  Serial.print(MotoShield.sensorReadRPMPerc()); //--Printing angular velocity in percents to serial monitor
  Serial.print(" ");
  Serial.println(r); //--Printing potentiometers value to serial monitor
}
