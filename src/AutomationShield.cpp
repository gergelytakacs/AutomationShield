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

        
Optical::Optical(){
 
}

void Optical::begin(void){                  // begin function initializes the pins used by the hardware. Optoshield uses three pins, pin A1 is used by the LDR, 
                                            //pin A0 is connected to the runner of the potentiometer and digital pin 3 is for setting the intensity of the leds' light                                            
  pinMode(OPTICAL_YPIN, INPUT);
  pinMode(OPTICAL_UPIN, OUTPUT);
  pinMode(OPTICAL_RPIN, INPUT); 
}


void Optical::actuatorWrite(int value){
 
  if(value <= 255){                                                 // nested if statement, if the condition is true check the following
      if(value > 0){                                                  // if the second condition is also true, write the value of the sensor
        analogWrite(OPTICAL_UPIN,value);
      }
    }
    else {Serial.println("The number You added doesn't fit the conditions, please choose a number between 0-255");} // if any of the statements is true, you receive a report 
}

int Optical::sensorRead(){
  int _valueRead = analogRead(OPTICAL_YPIN);
  return _valueRead;
}

int Optical::referenceRead(){
  int _valueRead = analogRead(OPTICAL_RPIN);
  return _valueRead;
}
            

                     

