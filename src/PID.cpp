#include "PID.h"

PIDClass::PIDClass(){

  eSum=0.0;
  e[0]=0.0;
  e[1]=0.0;  
  e[2]=0.0;
  u[0]=0.0;
  u[1]=0.0;

  Kp=1.0;
  Ki=1.0;
  Kd=1.0;
  Ti=1.0;
  Td=1.0;
  Ts=1.0;

  pidForm=absolute;
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

void PIDClass::beginAbsolute(){
  pidForm=absolute;
}
  
void PIDClass::beginIncremental(){
  pidForm=incremental;
}


float PIDClass::constrainFloat(float x, float min_x, float max_x){
  if (x<=min_x)
    return min_x;
  else if (x>=max_x)
    return max_x;  
  return x;
}

void PIDClass::loadVariables(float err){

  e[2]=err;
  eSum+=e[2];
  Ts=Timer.getSamplingPeriod();
}

void PIDClass::shiftVariables(){

  e[0]=e[1];
  e[1]=e[2];
  u[0]=u[1];
}

float PIDClass::compute(float err){    

  loadVariables(err);
  u[1]=computeU();
  shiftVariables();  
  return u[1];
}

float PIDClass::compute(float err,float saturationMin,float saturationMax){    
  
  loadVariables(err);
  u[1]=computeU();    
  u[1]=constrainFloat(u[1],saturationMin,saturationMax);
  shiftVariables();
  return u[1];
}

float PIDClass::compute(float err,float saturationMin,float saturationMax,float antiWindupMin,float antiWindupMax){    
  
  loadVariables(err);
  eSum=constrainFloat((Kp*Ts/Ti)*eSum,antiWindupMin,antiWindupMax)/(Kp*Ts/Ti);  
  u[1]=computeU();   
  u[1]=constrainFloat(u[1],saturationMin,saturationMax);
  shiftVariables();
  return u[1];
}

float PIDClass::computeU(){
  
  if(pidForm==absolute)
    return computeAbsForm();
  else
    return computeIncForm();
}

float PIDClass::computeAbsForm(){      

  return (Kp*e[2])+((Kp*Ts/Ti)*eSum)+((Kp*Td/Ts)*(e[2]-e[1]));
}

float PIDClass::computeIncForm(){     

  return u[0]+((Kp+(Kp*Ts/Ti)+(Kp*Td/Ts))*e[2])+((-Kp-(2*Kp*Td/Ts))*e[1])+((Kp*Td/Ts)*e[0]);
}


PIDClass PID; // Construct instance (define)
