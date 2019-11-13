/*
  API for the FloatShield didactic hardware.

  The file is a part of the application programmers interface for
  the FloatShield didactic tool for control engineering and
  mechatronics education. The FloatShield implements an air-flow
  levitation experiment on an Arduino shield.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 13.11.2019.
*/

#ifndef FLOATSHIELD_H_             // Include guard
#define FLOATSHIELD_H_

#include "AutomationShield.h"      // Include the main library
#include <Wire.h>                  // Include the I2C protocol library

#ifndef SHIELDRELEASE              // Define release version of used hardware
  #define SHIELDRELEASE 2          // Latest version by default
#endif

#ifndef VL53L0X_h                                           // If library for distance sensor is not already included
  #if __has_include("lib/vl53l0x-arduino/VL53L0X.h")        // If said library is present in main library file
    #include "lib/vl53l0x-arduino/VL53L0X.h"                // Include it from there
  #endif
#endif

// Defining pins used by the FloatShield board
#if SHIELDRELEASE == 1              
  #define FLOAT_UPIN 3              // Fan (Actuator)
  #define FLOAT_RPIN A0             // Potentiometer runner (Reference)
#elif SHIELDRELEASE == 2            
  #define FLOAT_UPIN 5              // Fan (Actuator)
  #define FLOAT_RPIN A0             // Potentiometer runner (Reference)
  #define FLOAT_YPIN 3              // Hall sensor signal (Output)
#endif

#ifdef VL53L0X_h                    // If library for distance sensor was sucessfully included

class FloatClass {                                               // Class for FloatShield device
public:
    VL53L0X distanceSensor;                                      // Create object for distance sensor
    void begin(void);                                            // Board initialisation - initialisation of distance sensor, pin modes and variables
    void calibrate(void);                                        // Board calibration - finding out the minimal and maximal values measured by distance sensor
    void actuatorWrite(float);                                   // Write actuator - function takes input 0.0-100.0 and sets fan speed accordingly
    float referenceRead(void);                                   // Reference read - returns potentiometer position in percentual range 0.0-100.0
    float sensorRead(void);                                      // Sensor read - returns the altitude of the ball in tube in percentual range 0.0(ball is on the fan)-100.0(ball is on the tube ceiling)
    float sensorReadAltitude(void);                              // Sensor read altitude - returns the altitude of the ball in tube in milimetres
    float sensorReadDistance(void);                              // Sensor read distance - returns raw reading of distance between sensor and ball in milimetres
    bool returnCalibrated(void);                                 // Returns calibration status, true if sensor was calibrated
    float returnMinDistance(void);                               // Returns value of minimal distance measured by sensor in milimetres
    float returnMaxDistance(void);                               // Returns value of maximal distance measured by sensor in milimetres
    float returnRange(void);                                     // Returns range of measured distances between minimal and maximal values in milimetres
  #if SHIELDRELEASE == 2                                         
    int sensorReadRPM(void);                                     // Returns fan RPM calculated from hall sensor signal
    volatile int hundredthsOfMillisecond;                        // Variable for storing custom time unit created using Timer2
    volatile int hallPeriod;                                     // Period of hall signal in custom time units
  #endif
  
private:
    float _minDistance;                  // Variable for storing minimal distance measured by sensor in milimetres
    float _maxDistance;                  // Variable for storing maximal distance measured by sensor in milimetres
    float _range;                        // Variable for storing range of measured distances by sensor in milimetres
    bool _wasCalibrated;                 // Variable for storing calibration status
    float _referenceValue;               // Variable for storing potentiometer runner position as analog value 0.0-1023.0
    float _referencePercent;             // Variable for storing potentiometer runner position as percentual range 0.0-100.0
    float _sensorValue;                  // Variable for storing measured distance by sensor in milimetres
    float _sensorPercent;                // Variable for percentual altitude of the ball in the tube 0.0-100.0
    float _ballAltitude;                 // Variable for altitude of the ball in tube in milimetres
  #if SHIELDRELEASE == 2                 
    int _rpm;                            // Variable for storing current value of fan rotations per minute
  #endif
};

extern FloatClass FloatShield;           // Creation of external FloatClass object

#endif                                   
#endif                                   // End of guard
