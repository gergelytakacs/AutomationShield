#include "AutomationShield.h"

float r;
float y;
float u=0.0;
float e;

bool enable = false;

void setup(void) {

  Serial.begin(9600);          
  HeatShield.begin(); 
  Sampling.interruptInitialize(1000000); //period in us
  Sampling.setInterruptCallback(stepEnable); 
  PIDAbs.setKp(0.5);
  PIDAbs.setTi(0.5);
  PIDAbs.setTd(0.5);
}

void loop(void) {

  if (enable) {
    
    step();
    enable=false;
  }
}

void stepEnable() {

  enable=true;
}

void step(){
  
  HeatShield.actuatorWrite(u);
  r=40.0;
  y=HeatShield.sensorReadTemperature();
  e=r-y;
  u=PIDAbs.compute(e,0.0,100.0,-100.0,100.0);
    
  Serial.print(y);
  Serial.print(" ");
  Serial.print(u);
  Serial.print(" ");
  Serial.println(r);
}


