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

  Created by Gergely Takács, Peter Chmurčiak and Erik Mikuláš.
  Last update: 2.12.2020.
*/

#ifndef FLOATSHIELD_H             // Include guard
#define FLOATSHIELD_H

#include "AutomationShield.h"      // Include the main library
#include <Wire.h>                  // Include the I2C protocol library
#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h"     // Include library for matrix operations

#ifndef SHIELDRELEASE              // Define release version of used hardware
  #define SHIELDRELEASE 4          // Latest version by default
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
#elif SHIELDRELEASE == 4
  #define MCP4725 (0x60)             // 12-bit DAC, Chip: MCP4725A0
  #define DACMAX 4095                // Maximal decimal DAC value for 12 bits
  #define FLOAT_RPIN A0             // Potentiometer runner (Reference)
  #define FLOAT_AVPIN A1            // Pin for reading actuator voltage
#endif

#ifdef VL53L0X_h                    // If library for distance sensor was successfully included

class FloatClass {                                               // Class for FloatShield device
  public:
    VL53L0X distanceSensor;                                      // Create object for distance sensor
    void begin(void);                                            // Board initialisation - initialisation of distance sensor, pin modes and variables
    void calibrate(void);                                        // Board calibration - finding out the minimal and maximal values measured by distance sensor
    void actuatorWrite(float);                                   // Write actuator - function takes input 0.0-100.0 and sets fan speed accordingly
    float referenceRead(void);                                   // Reference read - returns potentiometer position in percentual range 0.0-100.0
    float referenceReadAltitude(void);                           // Reference read altitude - returns potentiometer position in calibrated altitude range 0.0-324.0 (mm)
    float sensorRead(void);                                      // Sensor read - returns the altitude of the ball in tube in percentual range 0.0(ball is on the fan)-100.0(ball is on the tube ceiling)
<<<<<<< HEAD
    float sensorReadAltitude(void);                              // Sensor read altitude - returns the altitude of the ball in tube in milimetres
    float sensorReadDistance(void);                              // Sensor read distance - returns raw reading of distance between sensor and ball in milimetres
    bool wasCalibrated(void);                                    // Returns calibration status, true if sensor was calibrated
    float returnMinDistance(void);                               // Returns value of minimal distance measured by sensor in milimetres
    float returnMaxDistance(void);                               // Returns value of maximal distance measured by sensor in milimetres
    float returnRange(void);                                     // Returns range of measured distances between minimal and maximal values in milimetres
=======
    float sensorReadAltitude(void);                              // Sensor read altitude - returns the altitude of the ball in tube in millimetres
    float sensorReadDistance(void);                              // Sensor read distance - returns raw reading of distance between sensor and ball in millimetres
    bool returnCalibrated(void);                                 // Returns calibration status, true if sensor was calibrated
    float returnMinDistance(void);                               // Returns value of minimal distance measured by sensor in millimetres
    float returnMaxDistance(void);                               // Returns value of maximal distance measured by sensor in millimetres
    float returnRange(void);                                     // Returns range of measured distances between minimal and maximal values in millimetres
>>>>>>> master
    #include "getGainLQ.inl"                                     // Include template function for calculating K gain for LQ control applications
    #include "getKalmanEstimate.inl"                             // Include template function for estimating internal states of linear SISO system (Kalman filter)
#if SHIELDRELEASE == 2
    void actuatorWriteRPM(float);                                // Write actuator RPM - function takes input 0.0-17000.0 (RPM) and sets fan speed accordingly
    volatile unsigned int pulseCount;                            // Variable for storing the amount of pulses that were measured within set time interval
    volatile bool pulseMeasured;                                 // Variable for storing information if pulse was successfully measured
    void setSamplingPeriod(float);                               // Sets the sampling period used for accurate RPM calculation (default 25ms)
    float sensorReadRPM(void);                                   // Returns fan RPM calculated from hall sensor signal
#elif  SHIELDRELEASE == 4                                  
    void dacWrite(uint16_t DAClevel);                 // Writes DAClevel (DAC levels) to DAC register
    float actuatorReadVoltage(void);
#endif

  private:
    float _minDistance;                  // Variable for storing minimal distance measured by sensor in millimetres
    float _maxDistance;                  // Variable for storing maximal distance measured by sensor in millimetres
    float _range;                        // Variable for storing range of measured distances by sensor in millimetres
    bool _wasCalibrated;                 // Variable for storing calibration status
    float _referenceValue;               // Variable for storing potentiometer runner position as analog value 0.0-1023.0
    float _referencePercent;             // Variable for storing potentiometer runner position as percentual range 0.0-100.0
    float _sensorValue;                  // Variable for storing measured distance by sensor in millimetres
    float _sensorPercent;                // Variable for percentual altitude of the ball in the tube 0.0-100.0
    float _ballAltitude;                 // Variable for altitude of the ball in tube in millimetres
#if SHIELDRELEASE == 2
    float _rpm;                                                          // Variable for storing current value of fan rotations per minute
    float _samplingPeriod = 25.0;                                        // Variable for storing sampling period in milliseconds used for accurate RPM calculation (default 25ms)
    float _nOfSamples = ceil(150.0 / _samplingPeriod);                   // Variable for storing number of samples required for accurate RPM calculation (default 6)
    float _pulseCountToRPM = 15000.0 / (_samplingPeriod * _nOfSamples);  // Variable for storing constant used to calculate RPM from number of pulses (default 100)
#endif
#if SHIELDRELEASE == 4
    float _actuatorVoltage;               // Variable for storing voltage measured at the output pin to actuator
    float _actuatorVoltageADC;            // Variable for storing RAW voltage measured at the output pin to actuator 0.0-1023.0
#endif
};

extern FloatClass FloatShield;           // Creation of external FloatClass object

#endif
#endif                                   // End of guard