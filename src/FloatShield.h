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
  Last update: 11.3.2020.
*/

#ifndef FLOATSHIELD_H_             // Include guard
#define FLOATSHIELD_H_

#include "AutomationShield.h"      // Include the main library
#include <Wire.h>                  // Include the I2C protocol library
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h"     // Include library for matrix operations

#ifndef SHIELDRELEASE              // Define release version of used hardware
  #define SHIELDRELEASE 2          // Latest version by default
#endif

#ifndef VL53L0X_h                                           // If library for distance sensor is not already included  
  #include "lib/vl53l0x-arduino/VL53L0X.h"                  // Include it from the library folder  
#endif

// Defining pins used by the FloatShield board
#if SHIELDRELEASE == 1
  #define FLOAT_UPIN 3              // Fan (Actuator)
  #define FLOAT_RPIN A0             // Potentiometer runner (Reference)
#elif SHIELDRELEASE == 2
  #define FLOAT_UPIN 5              // Fan (Actuator)
  #define FLOAT_RPIN A0             // Potentiometer runner (Reference)
  #define FLOAT_YPIN 3              // Hall sensor signal (Output)
  #define FLOAT_RPM_CONST 68.1      // Constant representing the slope of RPM to PWM fan dependency graph 
#endif

#ifdef VL53L0X_h                    // If library for distance sensor was sucessfully included

class FloatClass {                                               // Class for FloatShield device
  public:
    VL53L0X distanceSensor;                                      // Create object for distance sensor
    void begin(void);                                            // Board initialisation - initialisation of distance sensor, pin modes and variables
    void calibrate(void);                                        // Board calibration - finding out the minimal and maximal values measured by distance sensor
    void actuatorWrite(float);                                   // Write actuator - function takes input 0.0-100.0 and sets fan speed accordingly
    float referenceRead(void);                                   // Reference read - returns potentiometer position in percentual range 0.0-100.0
    float referenceReadAltitude(void);                           // Reference read altitude - returns potentiometer position in calibrated altitude range 17.0-341.0 (mm)
    float sensorRead(void);                                      // Sensor read - returns the altitude of the ball in tube in percentual range 0.0(ball is on the fan)-100.0(ball is on the tube ceiling)
    float sensorReadAltitude(void);                              // Sensor read altitude - returns the altitude of the ball in tube in milimetres
    float sensorReadDistance(void);                              // Sensor read distance - returns raw reading of distance between sensor and ball in milimetres
    bool returnCalibrated(void);                                 // Returns calibration status, true if sensor was calibrated
    float returnMinDistance(void);                               // Returns value of minimal distance measured by sensor in milimetres
    float returnMaxDistance(void);                               // Returns value of maximal distance measured by sensor in milimetres
    float returnRange(void);                                     // Returns range of measured distances between minimal and maximal values in milimetres
    #include "getGainLQ.inl"                                     // Include template function for calculating K gain for LQ control applications
#if SHIELDRELEASE == 2
    void actuatorWriteRPM(float);                                // Write actuator RPM - function takes input 0.0-17000.0 (RPM) and sets fan speed accordingly
    volatile unsigned int pulseCount;                            // Variable for storing the amount of pulses that were measured within set time interval
    volatile bool pulseMeasured;                                 // Variable for storing information if pulse was sucessfully measured
    void setSamplingPeriod(float);                               // Sets the sampling period used for accurate RPM calculation (default 25ms)
    float sensorReadRPM(void);                                   // Returns fan RPM calculated from hall sensor signal
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
    float _rpm;                                                          // Variable for storing current value of fan rotations per minute
    float _samplingPeriod = 25.0;                                        // Variable for storing sampling period in milliseconds used for accurate RPM calculation (default 25ms)
    float _nOfSamples = ceil(150.0 / _samplingPeriod);                   // Variable for storing number of samples required for accurate RPM calculation (default 6)
    float _pulseCountToRPM = 15000.0 / (_samplingPeriod * _nOfSamples);  // Variable for storing constant used to calculate RPM from number of pulses (default 100)
#endif
};

extern FloatClass FloatShield;           // Creation of external FloatClass object

#endif
#endif                                   // End of guard
