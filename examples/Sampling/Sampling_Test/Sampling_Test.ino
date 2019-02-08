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