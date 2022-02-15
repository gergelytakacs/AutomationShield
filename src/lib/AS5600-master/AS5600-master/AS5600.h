/*
  AS5600.h - Library for interacting with the AS5600.
  Created by Kane C. Stoboi, 22 August, 2018.
  Released into the public domain.
  https://github.com/kanestoboi/AS5600
*/

#ifndef AS5600_h
#define AS5600_h

#include "Arduino.h"
#include <Wire.h>

class AS5600
{
  public:
    AS5600();

    enum {
      POWER_MODE_NORM = 0,
      POWER_MODE_LPM1 = 1,
      POWER_MODE_LPM2 = 2,
      POWER_MODE_LPM3 = 3,

      HYSTERESIS_OFF = 0,
      HYSTERESIS_1LSB = 1,
      HYSTERESIS_2LSB = 2,
      HYSTERESIS_3LSB = 3,

      OUTPUT_STAGE_ANALOG_FULL = 0,
      OUTPUT_STAGE_ANALOG_REDUCED = 1,
      OUTPUT_STAGE_DIGITAL_PWM = 2,

      SLOW_FILTER_16X = 0,
      SLOW_FILTER_8X = 1,
      SLOW_FILTER_4X = 2,
      SLOW_FILTER_2X = 3,

      FAST_FILTER_THRESHOLD_SLOW = 0,
      FAST_FILTER_THRESHOLD_6LSB = 1,
      FAST_FILTER_THRESHOLD_7LSB = 2,
      FAST_FILTER_THRESHOLD_9LSB = 3,
      FAST_FILTER_THRESHOLD_18LSB = 4,
      FAST_FILTER_THRESHOLD_21LSB = 5,
      FAST_FILTER_THRESHOLD_24LSB = 6,
      FAST_FILTER_THRESHOLD_10LSB = 7,

    };

    uint16_t getPosition();
    uint16_t getAngle();
    uint16_t getRawAngle();
    float getScaledAngle();

    uint8_t getStatus();
    uint8_t getGain();
    uint8_t getMagnet();
    uint16_t getMagnitude();
    void setZero(); // still to be implemented

    bool isMagnetTooStrong();
    bool isMagnetTooWeak();
    bool isMagnetDetected();

    bool setPowerMode(uint8_t powerMode);
    bool setHysteresis(uint8_t hysteresis);
    bool setOutputStage(uint8_t outputStage);
    bool setPWMFrequency(uint8_t frequency);
    bool setSlowFilter(uint8_t slowFilter);
    bool setFastFilterThreshold(uint8_t fastFilterThreshold);


    uint8_t getCONF();

    

    private:
      int _AS5600Address = 0x36; // I2C address for AS5600

      byte _ZMCOAddress = 0x00;
      byte _ZPOSAddressMSB = 0x01;
      byte _ZPOSAddressLSB = 0x02;
      byte _MPOSAddressMSB = 0x03;
      byte _MPOSAddressLSB = 0x04;
      byte _MANGAddressMSB = 0x05;
      byte _MANGAddressLSB = 0x06;
      byte _CONFAddressMSB = 0x07;
      byte _CONFAddressLSB = 0x08;
      byte _RAWANGLEAddressMSB = 0x0C;
      byte _RAWANGLEAddressLSB = 0x0D;
      byte _ANGLEAddressMSB = 0x0E;
      byte _ANGLEAddressLSB = 0x0F;
      byte _STATUSAddress = 0x0B;
      byte _AGCAddress = 0x1A;
      byte _MAGNITUDEAddressMSB = 0x1B;
      byte _MAGNITUDEAddressLSB = 0x1C;
      byte _BURNAddress = 0xFF;

      uint16_t _getRegisters2(byte registerMSB, byte registerLSB);
      uint8_t _getRegister(byte register1);

      void _writeRegister(byte registerAddress, byte value);
};

#endif
