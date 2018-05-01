#include "Pid.h"

PidClass::PidClass(){

  Kp=1.0;
  Ki=1.0;
  Kd=1.0;
  Ti=1.0;
  Td=1.0;
  Ts=1.0;
}

void PidClass::setKp(float Kp){
  this->Kp=Kp;
}

void PidClass::setKi(float Ki){
  this->Ki=Ki;
  Ti=Kp/Ki;
}

void PidClass::setKd(float Kd){
  this->Kd=Kd;
  Td=Kd/Kp;
}

void PidClass::setTi(float Ti){
  this->Ti=Ti;
  Ki=Kp/Ti;
}

void PidClass::setTd(float Td){
  this->Td=Td;
  Kd=Kp*Td;
}

float PidClass::getKp(){
  return Kp;
}

float PidClass::getKi(){
  return Ki;
}

float PidClass::getKd(){
  return Kd;
}

float PidClass::getTi(){
  return Ti;
}

float PidClass::getTd(){
  return Td;
}

float PidClass::constrainFloat(float x, float min_x, float max_x){
  if (x<=min_x)
    return min_x;
  else if (x>=max_x)
    return max_x;  
  return x;
}

void PidClass::loadVariables(float err){
  
  Ts=Timer.getSamplingPeriod();
}

float PidClass::compute(float err){    

  loadVariables(err);
  setU(computeU());
  shiftVariables();
  return getU();  
}

float PidClass::compute(float err,float saturationMin,float saturationMax){
  loadVariables(err);   
  setU(constrainFloat(computeU(),saturationMin,saturationMax));
  shiftVariables();
  return getU();

}
