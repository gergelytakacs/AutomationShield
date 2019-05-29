#include "PIDInc.h"

PIDIncClass::PIDIncClass(){
  e[0]=0.0;
  e[1]=0.0;
  e[2]=0.0;    
  u[0]=0.0;
  u[1]=0.0;
}

void PIDIncClass::loadVariables(float err){
  e[2]=err;
}


void PIDIncClass::shiftVariables(){
  e[0]=e[1];
  e[1]=e[2];
  u[0]=u[1];
}

float PIDIncClass::getU(){
  return u[1];
}

void PIDIncClass::setU(float u){
  this->u[1]=u;
}

float PIDIncClass::computeU(){      
  return u[0]+((Kp+(Kp*Ts/Ti)+(Kp*Td/Ts))*e[2])+((-Kp-(2*Kp*Td/Ts))*e[1])+((Kp*Td/Ts)*e[0]);
}

PIDIncClass PIDInc; // Construct instance (define)
