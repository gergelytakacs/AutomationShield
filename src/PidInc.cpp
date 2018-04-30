#include "PidInc.h"

PidIncClass::PidIncClass(){

  e[0]=0.0;
  e[1]=0.0;
  e[2]=0.0;    
  u[0]=0.0;
  u[1]=0.0;
}

void PidIncClass::loadVariables(float err){
  
  PidClass::loadVariables(err);
  e[2]=err;
}


void PidIncClass::shiftVariables(){

  e[0]=e[1];
  e[1]=e[2];
  u[0]=u[1];
}

float PidIncClass::getU(){

  return u[1];
}

void PidIncClass::setU(float u){

  this->u[1]=u;
}

float PidIncClass::computeU(){      

  return u[0]+((Kp+(Kp*Ts/Ti)+(Kp*Td/Ts))*e[2])+((-Kp-(2*Kp*Td/Ts))*e[1])+((Kp*Td/Ts)*e[0]);
}

PidIncClass PidInc; // Construct instance (define)
