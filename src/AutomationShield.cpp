/* 
 AutomationShield.cpp
  Arduino library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger, [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
  Last change by Tibor Konkoly on 26.03.2018 at 20:51.
*/


#include "AutomationShield.h"
#include "Arduino.h"

AutomationShield::AutomationShield(){
}

float AutomationShield::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) // same as Arudino map() but with floating point numbers
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // linear mapping, same as Arduino map()
}

float AutomationShield::constrain(float x, float min_x, float max_x){
//
}

void AutomationShield::error(char *str) // Error handler function
{
  #if ECHO_TO_SERIAL                    // If Serial Echo is turned on in the DEFINE (by advanced user)
  Serial.print("Error: ");
  Serial.println(str);
  #endif
  digitalWrite(ERRORPIN, HIGH);           // Enable generic error pin, should be 13 w/ the onboard LED
  while (1);                              // Stop all activity in busy cycle, so user definitely knows what's wrong.
}

void AutomationShield::initializePID(float _KP, float _KI, float _KD, int _Ts, float _min_u, float _max_u)      // Function run in setup to input user set values
{                                                                                                               // (for simplicity sake of not calling too many parameters in main PID function)
    KP = _KP;
    KI = _KI;
    KD = _KD;
    Ts = _Ts;
    min_u = _min_u;
    max_u = _max_u;
}

float AutomationShield::PID(float request_val, float current_val)               // computing function of PID output
{                                                                               // it is supposed to run loop and it will decide whether it's time for new sample
    
    now = milis();                                                              // Recording the current time and the timechange from last calculation.
    time_change = now - time_prior;                                             // Bottleneck of this method is that it will overflow after 52 days,
    if (time_change >= Ts)                                                      // however for didactic purposes I suppose it is acceptable.
    {
        real_Ts = (now - time_prior)/1000;                                      // Calculating real sample time, as there might be some other code running apart this one.
        
        error_pid = request_val - current_val;
        
        integral += error_pid*real_Ts;
        
        if (integral < min_u/KI)                                                // Limiting the integral part based on user parameters.
        {
            integral = min_u/KI;
        }
        else if (integral > max_u/KI)
        {
            integral = max_u/KI;
        }
        
        derivative = (error_pid - error_prior)/real_Ts;
        u = KP*error_pid + KI*integral + KD*derivative;
        
        if (u < min_u)                                                          // Limiting the output based on user set parameters.
        {
            u = min_u;
        }
        else if (u > max_u)
        {
            u = max_u;
        }
        
        error_prior = error_pid;                                                // Recycling new values into old.
        time_prior = milis();
        return u;
    }
    else
    {                                                                           // If the function determines it is not the right time for the next sample,
        return u;                                                               // it will just return previous output.
    }
}

    // same function, but with only one calling parameter

float AutomationShield::PID(float user_error)                                   // computing function of PID output
{                                                                               // it is supposed to run loop and it will decide whether it's time for new sample
    
    now = milis();                                                              // Recording the current time and the timechange from last calculation.
    time_change = now - time_prior;                                             // Bottleneck of this method is that it will overflow after 52 days,
    if (time_change >= Ts)                                                      // however for didactic purposes I suppose it is acceptable.
    {
        real_Ts = (now - time_prior)/1000;                                      // Calculating real sample time, as there might be some other code running apart this one.
        
        error_pid = user_error;
        
        integral += error_pid*real_Ts;
        
        if (integral < min_u/KI)                                                // Limiting the integral part based on user parameters.
        {
            integral = min_u/KI;
        }
        else if (integral > max_u/KI)
        {
            integral = max_u/KI;
        }
        
        derivative = (error_pid - error_prior)/real_Ts;
        u = KP*error_pid + KI*integral + KD*derivative;
        
        if (u < min_u)                                                          // Limiting the output based on user set parameters.
        {
            u = min_u;
        }
        else if (u > max_u)
        {
            u = max_u;
        }
        
        error_prior = error_pid;                                                // Recycling new values into old.
        time_prior = milis();
        return u;
    }
    else
    {                                                                           // If the function determines it is not the right time for the next sample,
        return u;                                                               // it will just return previous output.
    }
}

Opto::Opto(){ 
}


void Opto::begin(void){                  // begin function initializes the pins used by the hardware. Optoshield uses three pins, pin A1 is used by the LDR, 
                                            //pin A0 is connected to the runner of the potentiometer and digital pin 3 is for setting the intensity of the leds' light                                            
  pinMode(OPTO_YPIN, INPUT);
  pinMode(OPTO_UPIN, OUTPUT);
  pinMode(OPTO_RPIN, INPUT); 
}


void Opto::actuatorWrite(int value){
 // Do it in percents @TiborKonkoly :)
  if(value <= 255){                                                 // nested if statement, if the condition is true check the following
      if(value > 0){                                                  // if the second condition is also true, write the value of the sensor
        analogWrite(OPTO_UPIN,value);
      }
    }
    else {AutomationShield.error("Input must be between 0-100.");} // if any of the statements is true, you receive a report 
}

int Opto::sensorRead(){
  int _valueRead = analogRead(OPTO_YPIN);
  return _valueRead;
}

int Opto::referenceRead(){
  int _valueRead = analogRead(OPTO_RPIN);
  return _valueRead;
}
            

                     

