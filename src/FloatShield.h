#ifndef FLOATSHIELD_H_
#define FLOATSHIELD_H_

#include "Arduino.h"
#include "Adafruit_VL53L0X.h"

#define vent 3
#define pot A0;


class FloatShieldClass{
  
    FloatShieldClass();                 // Constructor - setting up the sensor
    
    public:
    
    int referencePercent();             // returns potenciometer value in percent
    int positionPercent();              // returns position value in percent
    void ventInPercent(int _value);     // sets ventilator output to value in percent
    float manualControl();              // returns potenciometer value in percent

    private:
    
    float _lastValue;
    
    };

extern FloatShieldClass FloatShield;
#endif
