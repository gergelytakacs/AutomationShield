#include "AutomationShield.h"

AutomationShieldClass::AutomationShieldClass(){

  eSum=0.0;
  eArray[0]=0.0;
  eArray[1]=0.0;  
}

float AutomationShieldClass::pid(float e, float Kp, float Ti, float Td){     
  eArray[1]=e;
  eSum+=e;
  float Ts=Timer.getSamplingPeriod();
  float u=(Kp*eArray[1])+((Kp*Ts/Ti)*eSum)+((Kp*Td/Ts)*(eArray[1]-eArray[0]));
  if (u>100)
    u=100.0;
  if (u<0)
    u=0.0;
  eArray[0]=eArray[1];
  return u;
}   

AutomationShieldClass AutomationShield; // Construct instance (define)





