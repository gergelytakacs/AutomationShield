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


AutomationShieldClass AutomationShield; // Construct instance (define)
