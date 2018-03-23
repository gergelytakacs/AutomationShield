/*
  AutomationShield.h - Library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger [...] and Gergely Takacs
  2017-2018.
  Released into the public domain.
*/

#ifndef AutomationShield_h
#define AutomationShiled_h

#include "Arduino.h"

class AutomationShield
{
public:
 void input(float u);
private:
 int x;
}

// Just an example
void AutomationShield::input(float u)
{
  //...
  constrain(u, 0, 100)		   // Constrain input to 0-100 %
  int upwm=map((int)u,0,100,0,255) // Map percents to 8-bit PWM
  analogWrite(_pin, upwm); 	   // Write to actuator pin
  //...
}


#endif