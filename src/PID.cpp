#include "PID.h"

PIDClass::PIDClass(){ // base class constructor without parameters will be called automatically in child constructors 
  Kp=1.0;
  Ki=1.0;
  Kd=1.0;
  Ti=1.0;
  Td=1.0;
  Ts=1.0;
}

void PIDClass::setKp(float Kp){
  this->Kp=Kp;
}

void PIDClass::setKi(float Ki){
  this->Ki=Ki;
  Ti=Kp/Ki;
}

void PIDClass::setKd(float Kd){
  this->Kd=Kd;
  Td=Kd/Kp;
}

void PIDClass::setTi(float Ti){
  this->Ti=Ti;
  Ki=Kp/Ti;
}

void PIDClass::setTd(float Td){
  this->Td=Td;
  Kd=Kp*Td;
}

void PIDClass::setTs(float Ts){
  this->Ts=Ts;
}

float PIDClass::getKp(){
  return Kp;
}

float PIDClass::getKi(){
  return Ki;
}

float PIDClass::getKd(){
  return Kd;
}

float PIDClass::getTi(){
  return Ti;
}

float PIDClass::getTd(){
  return Td;
}

float PIDClass::getTs(){
  return Td;
}


float PIDClass::constrainFloat(float x, float min_x, float max_x){
  if (x<=min_x)
    return min_x;
  else if (x>=max_x)
    return max_x;  
  return x;
}

float PIDClass::compute(float err){    
  loadVariables(err);
  setU(computeU());
  shiftVariables();
  return getU();  
}

float PIDClass::compute(float err,float saturationMin,float saturationMax){
  loadVariables(err);   
  setU(constrainFloat(computeU(),saturationMin,saturationMax));
  shiftVariables();
  return getU();
}
