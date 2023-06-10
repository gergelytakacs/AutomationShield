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
  Last update: 09.06.2023.
*/



#include "AS5600_AS.h"




//  OUTPUT REGISTERS
const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

// START AND END ANGLE(raw) REGISTERS
const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
const uint8_t AS5600_CONF = 0x07;   //  + 0x08

//  STATUS REGISTERS
const uint8_t AS5600_STATUS    = 0x0B;
const uint8_t AS5600_AGC       = 0x1A;
const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C

//  STATUS BITS
const uint8_t AS5600_MAGNET_HIGH   = 0x08;
const uint8_t AS5600_MAGNET_LOW    = 0x10;
const uint8_t AS5600_MAGNET_DETECT = 0x20;

//  CONFIGURATION BIT MASKS - byte level
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;


AS5600::AS5600(TwoWire *wire)
{
  _wire = wire;
}

bool AS5600::begin(void)
{
  _wire->begin();
  if (! isConnected()) return false;
  return true;
}


bool AS5600::isConnected()
{
  _wire->beginTransmission(_address);
  return ( _wire->endTransmission() == 0);
}


/////////////////////////////////////////////////////////
//
//  OUTPUT REGISTERS
//

// read raw angle register
uint16_t AS5600::rawAngle()
{
  int16_t value = readReg2(AS5600_RAW_ANGLE) & 0x0FFF;
  return value;
}

// read scaled angle register 
uint16_t AS5600::readAngleReg()
{
  uint16_t value = readReg2(AS5600_ANGLE) & 0x0FFF;
  return value;
}


////////////////////////////////////////////////////
//
// READ ANGLE IN DEG or RAD
//

// read angle in degrees
float AS5600::readAngleDeg()
{
  int angle = readAngleReg();
  return (angle - _zero) * _deg_per_unit;
}

// read angle in radians
float AS5600::readAngleRad()
{
  int angle = readAngleReg();
  return float(angle - _zero) * _rad_per_unit;
}

////////////////////////////////////////
//
// Calibration functions
//

bool AS5600::calibrate(int start, int end){
  _raw_zero = rawAngle();
  _raw_min = _raw_zero - start & 0x0FFF;
  _raw_max = _raw_zero + end  & 0x0FFF;
  setStartPositionRaw(_raw_min);
  delay(200);
  setEndPositionRaw(_raw_max);
  delay(200);
  _zero = readAngleReg();

  _deg_per_unit = (360.0 * float(start + end))/(4096.0 * 4096.0);
  _rad_per_unit = (2 * PI * float(start + end))/(4096.0 * 4096.0);

  return true;
}

bool AS5600::calibrate(double start, double end){
  int _raw_start = int(round(abs(start)*(4096.0/360.0)));
  int _raw_end = int(round(abs(end)*(4096.0/360.0)));
  
  return calibrate(_raw_start, _raw_end);
}

bool AS5600::calibrate(int8_t mode){
  _raw_zero = rawAngle();
  if(mode == MODE_180_180){
    _raw_min = (_raw_zero - (4096/2)) & 0x0FFF;
    _raw_max = (_raw_zero + (4096/2)) & 0x0FFF;
    _zero = 4096/2;
  }
  else if(mode == MODE_0_360){
    _raw_min = _raw_zero;
    _raw_max = (_raw_zero + 4096) & 0x0FFF;    
    _zero = 0;
  }
  else{
    return false;
  }

  setStartPositionRaw(_raw_min);
  delay(200);
  setEndPositionRaw(_raw_max);
  delay(200);

  _deg_per_unit = 360.0/4096.0;
  _rad_per_unit = (2 * PI)/4096.0;

  return true;
}

// set slow filter 
bool AS5600::setSlowFilter(uint8_t mask)
{
  if (mask > 3) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_SLOW_FILTER;
  value |= mask;
  writeReg(AS5600_CONF, value);
  return true;
}


// set fast filter
bool AS5600::setFastFilter(uint8_t mask)
{
  if (mask > 7) return false;
  uint8_t value = readReg(AS5600_CONF);
  value &= ~AS5600_CONF_FAST_FILTER;
  value |= (mask << 2);
  writeReg(AS5600_CONF, value);
  return true;
}



bool AS5600::setStartPositionRaw(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_ZPOS, value);
  return true;
}


bool AS5600::setEndPositionRaw(uint16_t value)
{
  if (value > 0x0FFF) return false;
  writeReg2(AS5600_MPOS, value);
  return true;
}

/////////////////////////////////////////////////////////
//
//  STATUS REGISTERS
//
uint8_t AS5600::readStatus()
{
  uint8_t value = readReg(AS5600_STATUS);
  return value;
}


uint8_t AS5600::readAGC()
{
  uint8_t value = readReg(AS5600_AGC);
  return value;
}


uint16_t AS5600::readMagnitude()
{
  uint16_t value = readReg2(AS5600_MAGNITUDE) & 0x0FFF;
  return value;
}


bool AS5600::detectMagnet()
{
  return (readStatus() & AS5600_MAGNET_DETECT) > 1;
}


bool AS5600::magnetTooStrong()
{
  return (readStatus() & AS5600_MAGNET_HIGH) > 1;
}


bool AS5600::magnetTooWeak()
{
  return (readStatus() & AS5600_MAGNET_LOW) > 1;
}



/////////////////////////////////////////////////////////
//
// reading and writing registers
//
uint8_t AS5600::readReg(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _error = _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)1);
  uint8_t _data = _wire->read();
  return _data;
}


uint16_t AS5600::readReg2(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _error = _wire->endTransmission();

  _wire->requestFrom(_address, (uint8_t)2);
  uint16_t _data = _wire->read();
  _data <<= 8;
  _data += _wire->read();
  return _data;
}


uint8_t AS5600::writeReg(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  _error = _wire->endTransmission();
  return _error;
}


uint8_t AS5600::writeReg2(uint8_t reg, uint16_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value >> 8);
  _wire->write(value & 0xFF);
  _error = _wire->endTransmission();
  return _error;
}