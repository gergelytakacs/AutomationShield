#ifndef AUTOMATIONSHIELD_H_
#define AUTOMATIONSHIELD_H_

#include "Timer.h"

class AutomationShieldClass{
  
  public:

    AutomationShieldClass(); 
    float pid(float e, float Kp, float Ti, float Td);
  
  private:
    
    float eSum;
    float eArray[2];   
};
extern AutomationShieldClass AutomationShield; // Declare external instance
#endif
 
