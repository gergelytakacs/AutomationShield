#ifndef FLOATSHIELD_H_
#define FLOATSHIELD_H_

#include "Arduino.h"
#include "src/lib/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h"

#define vent 3
#define pot A0


class FloatShieldClass{
  
   
    
    public:
    void initialize();
    int referencePercent();             // returns potenciometer value in percent
    int positionPercent();              // returns position value in percent
    void ventInPercent(int _value);     // sets ventilator output to value in percent
    float manualControl();              // returns potenciometer value in percent

    private:
    
    int lastValue;
    int value;
    int ref;
    int u;
    int in;
    int pos;
    };

extern FloatShieldClass FloatShield;
#endif
