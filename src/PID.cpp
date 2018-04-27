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

  pidForm=absolute;
  saturation=false;
  antiWindup=false;

  saturationMin=0.0;
  saturationMax=0.0;
  antiWindupMin=0.0;
  antiWindupMax=0.0;
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

void PIDClass::beginSaturation(float saturationMin,float saturationMax){
  saturation=true;
  this->saturationMin=saturationMin;
  this->saturationMax=saturationMax;
}
  
void PIDClass::stopSaturation(){
  saturation=false;
}
  
void PIDClass::beginAntiWindup(float antiWindupMin,float antiWindupMax){
  antiWindup=true;
  this->antiWindupMin=antiWindupMin;
  this->antiWindupMax=antiWindupMax;
}
  
void PIDClass::stopAntiWindup(){
  antiWindup=false;
}

float PIDClass::constrainFloat(float x, float min_x, float max_x){
  if (x<=min_x)
    return min_x;
  else if (x>=max_x)
    return max_x;  
  return x;
}

float PIDClass::compute(float err){    
  e[2]=err;
  eSum+=e[2];
  float Ts=Timer.getSamplingPeriod();
  
  if(pidForm==absolute)
    u[1]=computeAbsForm(Ts);
  else
    u[1]=computeIncForm(Ts);

  if(saturation)
    u[1]=constrainFloat(u[1],saturationMin,saturationMax);
    
  e[0]=e[1];
  e[1]=e[2];
  u[0]=u[1];
  return u[1];
}

float PIDClass::computeAbsForm(float Ts){      

  if(antiWindup)
    return (Kp*e[2])+constrainFloat((Kp*Ts/Ti)*eSum,antiWindupMin,antiWindupMax)+((Kp*Td/Ts)*(e[2]-e[1]));
  else
    return (Kp*e[2])+((Kp*Ts/Ti)*eSum)+((Kp*Td/Ts)*(e[2]-e[1]));
}

float PIDClass::computeIncForm(float Ts){     

  return u[0]+((Kp+(Kp*Ts/Ti)+(Kp*Td/Ts))*e[2])+((-Kp-(2*Kp*Td/Ts))*e[1])+((Kp*Td/Ts)*e[0]);
}



PIDClass PID; // Construct instance (define)
