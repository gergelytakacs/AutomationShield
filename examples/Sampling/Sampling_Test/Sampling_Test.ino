/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR, SAMD and SAM-based Arduino 
  prototyping boards. This module should be compatible with
  all AVR boards (Uno, Mega 2560 etc.), Arduino Zero and Arduino
  Due. Take care of timer conflicts when using the Servo library.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Koplinger 2018-2019
  AVR Timer: 	 Richard Koplinger, 2018
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 11.02.2019.
*/

#include "Sampling.h"

unsigned long int Ts = 1000; // Sampling in microseconds

#define PIN 12                  // For oscilloscope test
bool enable=false;
unsigned long int curTime=0;
unsigned long int prevTime=0;
bool state = 0;                 // pin state

void setup() {
  pinMode(PIN,OUTPUT);
  Serial.begin(2000000); 
  
  Sampling.period(Ts);
  Sampling.interrupt(stepEnable);
}

void loop() {
  curTime=micros();
  if (enable) {
    step();
    enable=false;    
  }  
}

void stepEnable(){
  enable=true;
}

void step(){
  // Here comes your control application

  // This is just for testing
    Serial.print((float)(curTime-prevTime)/1000.0);
    Serial.println(" ms");
    prevTime=curTime;
    if(state == true) {
      digitalWrite(PIN,HIGH);
    } else {
      digitalWrite(PIN,LOW);
    }
    state = !state;
}