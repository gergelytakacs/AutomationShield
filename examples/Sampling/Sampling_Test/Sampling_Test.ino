/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR, SAMD and SAM-based Arduino 
  prototyping boards with the R3 pinout. This module should be 
  compatible with the Uno, Mega 2560, Arduino Zero and Arduino
  Due. There should be no timer conflicts when using the Servo library.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Koplinger 2018-2019
  AVR Timer:      Gergely Takacs, Richard Koplinger, Matus Biro, Lukas Vadovic 2018-2019
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 11.02.2019.
*/

#include "Sampling.h"
#define SERIAL_OUT           // Output on Serial, comment for oscilloscope only
#define PIN 12               // For oscilloscope test

unsigned long int Ts = 10000; // Sampling in microseconds
bool enable=false;
unsigned long int curTime=0;
unsigned long int prevTime=0;
bool state = 0;                 // pin state

void setup() {
  pinMode(PIN,OUTPUT);

  #ifdef SERIAL_OUT
    Serial.begin(2000000); 
    Serial.println("Sampling test begin:");
  #endif
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
  #ifdef SERIAL_OUT
    Serial.print((float)(curTime-prevTime)/1000.0);
    Serial.println(" ms");
  #endif
    prevTime=curTime;
  if(state == true) {
    digitalWrite(PIN,HIGH);
   } else {
     digitalWrite(PIN,LOW);
   }
    state = !state;
}
