#include "AutomationShield.h"

bool enable=false;
unsigned long int curTime=0;
unsigned long int prevTime=0;

void setup() {
  
  Serial.begin(9600);
  Sampling.interruptInitialize(1000000);
  Sampling.setInterruptCallback(stepEnable);

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
  
  Serial.println(curTime-prevTime);
  prevTime=curTime;
}
