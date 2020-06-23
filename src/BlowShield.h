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
#include <Arduino.h>

// Sensor definitions

#define BMP280_DIG_T1_LSB_REG     0x88
#define BMP280_DIG_T1_MSB_REG     0x89
#define BMP280_DIG_T2_LSB_REG     0x8A
#define BMP280_DIG_T2_MSB_REG     0x8B
#define BMP280_DIG_T3_LSB_REG     0x8C
#define BMP280_DIG_T3_MSB_REG     0x8D
#define BMP280_DIG_P1_LSB_REG     0x8E
#define BMP280_DIG_P1_MSB_REG     0x8F
#define BMP280_DIG_P2_LSB_REG     0x90
#define BMP280_DIG_P2_MSB_REG     0x91
#define BMP280_DIG_P3_LSB_REG     0x92
#define BMP280_DIG_P3_MSB_REG     0x93
#define BMP280_DIG_P4_LSB_REG     0x94
#define BMP280_DIG_P4_MSB_REG     0x95
#define BMP280_DIG_P5_LSB_REG     0x96
#define BMP280_DIG_P5_MSB_REG     0x97
#define BMP280_DIG_P6_LSB_REG     0x98
#define BMP280_DIG_P6_MSB_REG     0x99
#define BMP280_DIG_P7_LSB_REG     0x9A
#define BMP280_DIG_P7_MSB_REG     0x9B
#define BMP280_DIG_P8_LSB_REG     0x9C
#define BMP280_DIG_P8_MSB_REG     0x9D
#define BMP280_DIG_P9_LSB_REG     0x9E
#define BMP280_DIG_P9_MSB_REG     0x9F


//BlowShield definitions 

#ifndef SHIELDRELEASE              // Define release version of used hardware
  #define SHIELDRELEASE 1          // Latest version by default
#endif


// Defining pins used by the FloatShield board
#if SHIELDRELEASE == 1
  #define BLOW_UPIN 11              // Pump (Actuator)
  #define BLOW_RPIN A0             // Potentiometer runner (Reference)
#endif

#define BMP280_addr          0x76       // Sensor address 
  
class BlowClass {           // Class for BlowShield device
  public:
    // BlowShield public function
    void begin(void);                                            // Board initialisation - initialisation of pressure sensor, pin modes and variables
    void Step(void);
    void calibration(void);                                        // Board calibration - finding out the minimal and maximal values measured by pressure sensor
    void actuatorWrite(float);                                   // Write actuator - function takes input 0.0-100.0 and sets pump speed accordingly
    float referenceRead(void);                                   // Reference read - returns potentiometer position in percentual range 0.0-100.0
    float sensorRead(void);                                      // Sensor read - returns the pressure in percentual range 0.0-100.0
    
    // Sensor public funcions
    void readCoefficients();
    void begin_config();
    void measure_config();
    float readPressure();
    void readTemperature();

  private:
    // BlowShield private 
    float _minPressure;                  // Variable for storing minimal distance measured by sensor in milimetres
    float _maxPressure;                  // Variable for storing maximal distance measured by sensor in milimetres
    float _sensorPercent;                // Variable for percentual altitude of the ball in the tube 0.0-100.0
    bool _wasCalibrated;                 // Variable for storing calibration status
    
    // Sensor private
    uint8_t readRegister(uint8_t RegSet);
    uint32_t press_read();
    uint32_t temp_read();

    int64_t t_fine;
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint32_t temp_raw_data;
    uint32_t press_raw_data;
    };


    extern BlowClass BlowShield;           // Creation of external FloatClass object

    
#endif                                   // End of guard
