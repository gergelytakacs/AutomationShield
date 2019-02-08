#include "AutomationShield.h"

unsigned long int Ts = 1000000; // Sampling in microseconds

bool enable=false;
unsigned long int curTime=0;
unsigned long int prevTime=0;

void setup() {
  
  Serial.begin(9600);
  
  Sampling.period(Ts);
  Sampling.interrupt(stepEnable);

}

void loop() {
  curTime=millis();
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
  Serial.print((curTime-prevTime));
  Serial.println(" ms");
  prevTime=curTime;
}