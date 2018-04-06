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

float AutomationShield::PID(float err,int input, float SampleTime, float Kp,float Ki, float Kd, float outMin, float outMax)
{
  float output;
  
    if (direction == 1) // reverse acting system (FloatShield)
    {
        Kp = (0 - Kp);
        Ki = (0 - Ki);
        Kd = (0 - Kd);
    }
  integral = integral + (err)*SampleTime;
  if (integral > outMax)
  {
    integral = outMax;
  }
  else if( integral < outMin)
  {
    integral = outMin;
  }

   derivative = (input - lastinput)/SampleTime; //removes Derivative kick
   output = Kp *err + Ki*integral - Kd*derivative;

  if (output > outMax)
  {
    output = outMax;
  }
  else if( output < outMin)
  {
    output = outMin;
  }

 lastinput = input;
  return output;
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
            

                     

