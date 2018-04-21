#ifndef HEATSHIELD_H_
#define HEATSHIELD_H_

#include "Arduino.h"

class HeatShieldClass{
  
  public:

    float getTemperature();
    void pinInitialization();
    void power(float percent);
    
  private:

    const float referenceTemperature = 25 + 273.15;       
    const int beta = 3950;       // value of Beta factor from datasheet MF58 
    const int pwmOutputPin = 10;
    const int thermPin = 0;       
    const float powerSupply = 5.0;  
    const unsigned long int resistorResistance = 100000; 

    float getThermistorVolage();
    float getThermistorResist();  
};

extern HeatShieldClass HeatShield;
#endif


