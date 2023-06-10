/*
  Library for AS5600 magnetic rotary position sensor.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education. The AeroShield implements an air-based,
  pendulum angle positioning experiment on an Arduino microcontroller.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  This library is inspired by work of Rob Tillaart 
  https://github.com/RobTillaart/AS5600

  Created by Erik Mikulas
  Last update: 09.05.2023.
*/

#ifndef AS5600AS_H			       // Include guard
#define AS5600AS_H	


#include "Arduino.h"
#include "Wire.h"

#define MODE_180_180 0
#define MODE_0_360 1

//  I2C address of the sensor
const uint8_t AS5600_ADDRESS    = 0x36;

//  setSlowFilter
const uint8_t AS5600_SLOW_FILT_16X      = 0;
const uint8_t AS5600_SLOW_FILT_8X       = 1;
const uint8_t AS5600_SLOW_FILT_4X       = 2;
const uint8_t AS5600_SLOW_FILT_2X       = 3;

//  setFastFilter
const uint8_t AS5600_FAST_FILT_NONE     = 0;
const uint8_t AS5600_FAST_FILT_LSB6     = 1;
const uint8_t AS5600_FAST_FILT_LSB7     = 2;
const uint8_t AS5600_FAST_FILT_LSB9     = 3;
const uint8_t AS5600_FAST_FILT_LSB18    = 4;
const uint8_t AS5600_FAST_FILT_LSB21    = 5;
const uint8_t AS5600_FAST_FILT_LSB24    = 6;
const uint8_t AS5600_FAST_FILT_LSB10    = 7;


//  0.087890625;
const float   AS5600_RAW_TO_DEG     = 360.0 / 4096;
const float   AS5600_DEGTO_RAW     = 4096 / 360.0;
//  0.00153398078788564122971808758949;
const float   AS5600_RAW_TO_RAD     = PI * 2.0 / 4096;





class AS5600
{
    public:

        AS5600(TwoWire *wire = &Wire);

        bool begin(void);
        bool isConnected();

        //  set start position of measuring range in raw angle (0-4095)
        //  returns false if parameter out of range
        bool setStartPositionRaw(uint16_t value);

        //  set end position of measuring range in raw angle (0-4095)
        //  returns false if parameter out of range
        bool setEndPositionRaw(uint16_t value);

        // set zero of the senzor at currnet position
        void zeroSensor(void);

        // calibrate the sensor, zero is set to curent position. 
        // Start and end represents angle measurement range relative to zero.
        // when star and end are not provided the renge will be full circle and 
        // optional argument mode can be 180_180_MODE (-180, 180> deg. or 0_360_MODE <0, 360) deg.

        // calibrate the sensor zero and range with angles in raw values of sensor relative to zero
        bool calibrate(double start, double end);

        // calibrate the sensor zero and range with angles in degrees relative to zero
        bool calibrate(int start, int end);

        // calibrate the sensor zero and select the mode (full circle operation)
        bool calibrate(int8_t mode = MODE_180_180);

        //  READ OUTPUT REGISTERS
        uint16_t rawAngle();
        uint16_t readAngleReg();


        // Read scaled angle in degrees
        float readAngleDeg();
        // Read scaled angle in radians
        float readAngleRad();

        //  READ STATUS REGISTERS
        uint8_t  readStatus();
        uint8_t  readAGC();
        uint16_t readMagnitude();

        //  access detail status register
        bool     detectMagnet();
        bool     magnetTooStrong();
        bool     magnetTooWeak();




        //  0 = 16x    1 = 8x     2 = 4x     3 = 2x
        //  returns false if parameter out of range
        bool     setSlowFilter(uint8_t mask);

        //  0 = none   1 = LSB6   2 = LSB7   3 = LSB9
        //  4 = LSB18  5 = LSB21  6 = LSB24  7 = LSB10
        //  returns false if parameter out of range
        bool     setFastFilter(uint8_t mask);


    private:
        
        int _zero = 0;
        int _raw_zero = 0; // zero position of the senzor in sensor raw raw
        int _raw_min = 0; // start of measurement range in raw angle
        int _raw_max = 4096; // end of measurement range in raw angle
        float _deg_per_unit = 360.0/4096.0;
        float _rad_per_unit = (2*PI)/4096.0;



    protected:

        uint8_t  _address         = AS5600_ADDRESS;

        TwoWire*  _wire;
        bool _calibrated = false;

        uint8_t  _error           = 0;

        uint8_t  readReg(uint8_t reg);
        uint16_t readReg2(uint8_t reg);
        uint8_t  writeReg(uint8_t reg, uint8_t value);
        uint8_t  writeReg2(uint8_t reg, uint16_t value);

};
#endif