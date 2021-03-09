/*
  MotoShield step response example.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 23.4.2020.
*/
#include <MotoShield.h>
#define TS 10.0 //--Sampling period in milliseconds
#define u  100.0 //--Step value in percent
void setup() {
  Serial.begin(2000000); //--Initialize serial communication # 9600 baudrate
  MotoShield.begin(TS); //--Initialize MotoShield with sampling period 100 millis
  MotoShield.calibration(); //--Calibration method
  MotoShield.actuatorWrite(u); //--Step value in percents
  Serial.println("y, u");  //--Print header 
}

void loop() {
  if (MotoShield.stepEnable) {      //--Running the algorithm once every sample
    step();               
    MotoShield.stepEnable=false;  //--Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}

void step() {
  Serial.print(MotoShield.sensorReadRPMPerc()); //--Printing angular velocity in percents
  Serial.print(", ");
  Serial.println(u); //--Printing Step (reference) value
}
