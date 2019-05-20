#ifndef FLOATSHIELD_H_
#define FLOATSHIELD_H_

#include "Arduino.h"

#ifndef ADAFRUIT_VL53L0X_H 				// If library not installed somewhere
#if __has_include("lib/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h")  // If library present from GIT
	#include "lib/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h" // Include it from there
#endif
#endif	


#define vent 3
#define pot A0

#ifdef ADAFRUIT_VL53L0X_H 				// If header present

class FloatShieldClass{
  
    public:
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
    void initialize();                  // sets up the sensor - required to run in setup!
    void debug();                       // prints out sensor debug data in monitor - must be called before initialize
    int referencePercent();             // returns potenciometer value in percent
    int positionPercent();              // returns position value in percent
    void ventInPercent(int value);      // sets ventilator output to value in percent
    float manualControl();              // returns potenciometer value in percent
    int  positionMillimeter();          // returns position value in millimeters
    void calibrate();                   // calibration
    private:
    
    uint8_t i2c_addr = 0x29;
    boolean _debug = false;
    int lastValue;
    int value;
    int ref;
    int u;
    int in;
    int pos;
    int minimum = 20;
    int maximum = 350; 
    };

extern FloatShieldClass FloatShield;

#endif	
#endif	
