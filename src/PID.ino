#include "AutomationShield.h"

bool stepEnable=false;

void setup() {
  
  Serial.begin(9600);
  Sampling.interruptInitialize(1000000);
  Sampling.setInterruptCallback(stepTimer);
}

void loop() {

    if (stepEnable) {
    Serial.println("Hello world");
    stepEnable=false;
    }  
}

void stepTimer(){
  stepEnable=true;
}
