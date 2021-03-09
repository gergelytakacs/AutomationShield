/*
  MotoShield Identification data acquisition.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 23.4.2020.
*/
#include<MotoShield.h> //--Include API
#define TS 20.0 //--Sampling period # symbolic constant

#define PRBS 0 //--APRBS data selected

#if PRBS
#include "prbsU.h"
float prbs;
#else
#include "aprbsU.h"
float aprbs;
#endif
int i;
float u;

void setup() {
  Serial.begin(500000); //--initialize Serial
  MotoShield.begin(TS); //--initialize MotoShield
  Serial.println("u, pps, I"); //--Print header
}

void loop() {
  if (MotoShield.stepEnable) {//--Flag condition 
    step();     //--algorithm
    MotoShield.stepEnable = false; //--disable Flag
  }
}

void step() { //--Algorythm
#if PRBS  //--case PRBS 
  if (i > prbsU_length) { //--end of data
    MotoShield.actuatorWrite(0); //--Stop the Motor 
    while (1); //--Stop the program
  }
  else { 
    u = pgm_read_word(&prbsU[i]); //--read actuating value
  }
#else //--Case ARPBS
  if (i > aprbsU_length) { //--end of data
    MotoShield.actuatorWrite(0); //--Stop the Motor
    while (1); //--Stop the program
  }
  else {
    u = pgm_read_float_near(&aprbsU[i]); //--read actuating value
  }
#endif
MotoShield.actuatorWriteVolt(u); //--Actuate
Serial.print(u,5); //--Print actuating value
Serial.print(", "); //--CSV
Serial.print(MotoShield.counted); //--Print Number of ticks per sample 
Serial.print(", ");
Serial.println(MotoShield.sensorReadCurrent(),5); //--Print Current
i++; //--Sample #data# index
}
