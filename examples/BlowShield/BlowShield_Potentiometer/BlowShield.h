/*
  tu príde pokec, ktorý napíše Vladko. niečo v zmysle: 
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
  Last update: 4.4.2020.
*/
#ifndef BLOWSHIELD_H_             // Include guard
#define BLOWSHIELD_H_

#include <AutomationShield.h>      // Include the main library
#include <Wire.h>                  // Include the I2C protocol library

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#ifndef SHIELDRELEASE              // Define release version of used hardware
  #define SHIELDRELEASE 1          // Latest version by default
#endif


// Defining pins used by the FloatShield board
#if SHIELDRELEASE == 1
  #define BLOW_UPIN 11              // Pump (Actuator)
  #define BLOW_RPIN A0             // Potentiometer runner (Reference)
#endif

#define BMP280_ADRESA (0x76)        // Sensor adress 
  
class BlowClass {           // Class for BlowShield device
  public:
//toto neviem, ci moze byt tu, ale skusim
    Adafruit_BMP280 pressureSensor;                    // Create object for pressure sensor
    void begin(void);                                            // Board initialisation - initialisation of pressure sensor, pin modes and variables
    void Step(void);
    void calibration(void);                                        // Board calibration - finding out the minimal and maximal values measured by pressure sensor
    void actuatorWrite(float);                                   // Write actuator - function takes input 0.0-100.0 and sets pump speed accordingly
    float referenceRead(void);                                   // Reference read - returns potentiometer position in percentual range 0.0-100.0
    float sensorRead(void);                                      // Sensor read - returns the pressure in percentual range 0.0-100.0
    //toto je vytahovanie z private asi, teraz nam netreba: float returnMinPressure(void);                               // Returns value of minimal distance measured by sensor in milimetres
    //toto je vytahovanie z private asi, teraz nam netreba: float returnMaxPressure(void);                               // Returns value of maximal distance measured by sensor in milimetres
  private:
    // tieto asi by bolo fajn ponastavovat pre pripad, ze nekalibrujeme
    float _minPressure;                  // Variable for storing minimal distance measured by sensor in milimetres
    float _maxPressure;                  // Variable for storing maximal distance measured by sensor in milimetres
    //float _referencePercent;             // Variable for storing potentiometer runner position as percentual range 0.0-100.0
    float _sensorPercent;                // Variable for percentual altitude of the ball in the tube 0.0-100.0
    
    //float _sensorValue;                  // Variable for storing measured distance by sensor in milimetres
    //float _referenceValue;               // Variable for storing potentiometer runner position as analog value 0.0-1023.0
    
    bool _wasCalibrated;                 // Variable for storing calibration status
    };


    extern BlowClass BlowShield;           // Creation of external FloatClass object

    
#endif                                   // End of guard
