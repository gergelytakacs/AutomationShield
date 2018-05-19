#include "PIDAbs.h"

PIDAbsClass::PIDAbsClass(){

  eSum=0.0;
  e[0]=0.0;
  e[1]=0.0;  
  u=0.0;
}


void PIDAbsClass::loadVariables(float err){
  
  Ts = Sampling.getSamplingPeriod();
  e[1]=err;
  eSum+=e[1];
}

void PIDAbsClass::shiftVariables(){

  e[0]=e[1];
}

float PIDAbsClass::getU(){

  return u;
}

void PIDAbsClass::setU(float u){

  this->u=u;
}

float PIDAbsClass::compute(float err,float saturationMin,float saturationMax,float antiWindupMin,float antiWindupMax){    
  
  loadVariables(err);
  eSum=constrainFloat(eSum,antiWindupMin,antiWindupMax);    
  setU(constrainFloat(computeU(),saturationMin,saturationMax));
  shiftVariables();
  return getU();
}

float PIDAbsClass::computeU(){      
  // How about q0, q1, q2
  return (Kp*e[1])+((Kp*Ts/Ti)*eSum)+((Kp*Td/Ts)*(e[1]-e[0]));
}

PIDAbsClass PIDAbs; // Construct instance (define)
