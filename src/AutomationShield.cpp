/* 
 AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs to be continued
  2017-2018.
  Released into the public domain.
*/

#include "AutomationShield.h"
#include "Arduino.h"

float AutomationShieldClass::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) // same as Arudino map() but with floating point numbers
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // linear mapping, same as Arduino map()
}

float AutomationShieldClass::constrainFloat(float x, float min_x, float max_x){
 if (x>=max_x){x=max_x;} // if x larger than maximum, it's maximum
 if (x<=min_x){x=min_x;} // if x smaller than minimum, it's minimum
 return x; // return x
}

void AutomationShieldClass::error(char *str) // Error handler function
{
  #if ECHO_TO_SERIAL                    // If Serial Echo is turned on in the DEFINE (by advanced user)
  Serial.print("Error: ");
  Serial.println(str);
  #endif
  digitalWrite(ERRORPIN, HIGH);           // Enable generic error pin, should be 13 w/ the onboard LED
  while (1);                              // Stop all activity in busy cycle, so user definitely knows what's wrong.
}

float AutomationShieldClass::pid(float err,float Kp,float Ki,float Kd,float outMin, float outMax)   //PID function with anti wind-up and saturation limits
 {     
  
   integral = integral + (err)*T;     //integral section
  if (integral > outMax)              //anti wind-up
  {
    integral = outMax;
  }
  else if( integral < outMin)    
  {
    integral = outMin;
  }

   derivative = (err - lasterr)/T;                   //derivative section
   output = Kp *err + Ki*integral + Kd*derivative;   //PID action

  if (output > outMax)                               //saturation limits
  {
    output = outMax;
  }
  else if( output < outMin)
  {
    output = outMin;
  }

 lasterr = err;                                   
  return output;
 }
    
float AutomationShieldClass::pid(float err,float Kp,float Ki,float Kd)    //PID function without anti wind-up and saturation limits
 {     
  
    integral = integral + (err)*T;
  

    derivative = (err - lasterr)/T; 
    output = Kp *err + Ki*integral + Kd*derivative;


    lasterr = err;
    return output;
 }

 AutomationShieldClass::AutomationShieldClass(){

  eSum=0.0;
  eArray[0]=0.0;
  eArray[1]=0.0;  
}

float AutomationShieldClass::pidExp(float e, float Kp, float Ti, float Td){     
  eArray[1]=e;
  eSum+=e;
  float Ts=Timer.getSamplingPeriod();
  float u=(Kp*eArray[1])+((Kp*Ts/Ti)*eSum)+((Kp*Td/Ts)*(eArray[1]-eArray[0]));
  if (u>100)
    u=100.0;
  else if (u<0)
    u=0.0;
  eArray[0]=eArray[1];
  return u;
}   
float AutomationShieldClass::pidInc(float err,float Kp,float Ti,float Td,float outMin, float outMax)   //incremental PID function
 {
    
  e[0] = err;
  r_p = Kp;
  r_i = Kp/Ti;
  r_d = Kp*Td;

  q0 = r_p+r_i*T+r_d/T;
  q1 = -r_p - (2*r_d)/T;
  q2 = r_d/T;
  delta = q0*e[0] + q1*e[1] + q2*e[2];
  out[1] = out[0] + delta;              //difference eq.

    
  if (out[1] > outMax)                  //saturation limits

  {
    
    out[1] = outMax;
  }
  else if( out[1] < outMin)
  {
    
    out[1] = outMin;
  }
  out[0] = out[1];                     //last output
  
  e[2] = e[1];                       //last last error
  e[1]= e[0];                        //last error

  return out[1];
 }

AutomationShieldClass AutomationShield; // Construct instance (define)


