#include "PidAbs.h"

PidAbsClass::PidAbsClass(){

  eSum=0.0;
  e[0]=0.0;
  e[1]=0.0;  
  u=0.0;
}


void PidAbsClass::loadVariables(float err){
  
  PidClass::loadVariables(err);
  e[1]=err;
  eSum+=e[1];
}

void PidAbsClass::shiftVariables(){

  e[0]=e[1];
}

float PidAbsClass::getU(){

  return u;
}

void PidAbsClass::setU(float u){

  this->u=u;
}

float PidAbsClass::compute(float err,float saturationMin,float saturationMax,float antiWindupMin,float antiWindupMax){    
  
  loadVariables(err);
  eSum=constrainFloat((Kp*Ts/Ti)*eSum,antiWindupMin,antiWindupMax)/(Kp*Ts/Ti);    
  setU(constrainFloat(computeU(),saturationMin,saturationMax));
  shiftVariables();
  return getU();
}

float PidAbsClass::computeU(){      

  return (Kp*e[1])+((Kp*Ts/Ti)*eSum)+((Kp*Td/Ts)*(e[1]-e[0]));
}

PidAbsClass PidAbs; // Construct instance (define)
