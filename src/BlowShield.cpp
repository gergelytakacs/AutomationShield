/*
  API for the BlowShield didactic hardware.
  The file is a part of the application programmers interface for
  the BlowShield didactic tool for control engineering and
  mechatronics education. The goal of this particular shield is 
  to obtain and keep the required value of pressure in the vessel 
  despite its leakage.Arduino shield.
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Martin Staroň, Anna Vargová, Eva Vargová and Martin Vričan.
  Last update: 24.6.2020.
*/

#include "BlowShield.h"                      // Include header file

// Sensor library

void BlowClass::readCoefficients() {                   // read calibration coefficient
  dig_T1 = ((uint16_t)((BlowShield.readRegister(BMP280_DIG_T1_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_T1_LSB_REG)));
  dig_T2 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_T2_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_T2_LSB_REG)));
  dig_T3 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_T3_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_T3_LSB_REG)));

  dig_P1 = ((uint16_t)((BlowShield.readRegister(BMP280_DIG_P1_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P1_LSB_REG)));
  dig_P2 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P2_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P2_LSB_REG)));
  dig_P3 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P3_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P3_LSB_REG)));
  dig_P4 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P4_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P4_LSB_REG)));
  dig_P5 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P5_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P5_LSB_REG)));
  dig_P6 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P6_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P6_LSB_REG)));
  dig_P7 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P7_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P7_LSB_REG)));
  dig_P8 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P8_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P8_LSB_REG)));
  dig_P9 = ((int16_t)((BlowShield.readRegister(BMP280_DIG_P9_MSB_REG) << 8) + BlowShield.readRegister(BMP280_DIG_P9_LSB_REG)));
}


void BlowClass::begin_config() {        //configuration -> do begin
  Wire.beginTransmission(BMP280_addr);
  Wire.write(0xF4);
  Wire.write(0b111111);   //temp -> full, press -> full, 20-bit (oversampling x 16), mode -> normal
  Wire.write(0xF5);
  Wire.write(0b0);     //standby time, filter
  Wire.endTransmission();
  delay(10);
}

void BlowClass::measure_config() {       //configuration -> do begin
  Wire.beginTransmission(BMP280_addr);
  Wire.write(0xF4);
  Wire.write(0b1111);   //temp -> full, press -> full, 20-bit (oversampling x 16), mode -> normal
  Wire.write(0xF5);
  Wire.write(0b1000);     //standby time, filter
  
  Wire.endTransmission();
  delay(10);
}

uint8_t BlowClass::readRegister(uint8_t RegSet) {
  uint8_t _result = 0;
  Wire.beginTransmission(BMP280_addr);
  Wire.write(RegSet);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_addr, 1, true);
  _result = Wire.read();
  return _result;
}

uint32_t BlowClass::press_read() {        //value from sensor
  Wire.beginTransmission(BMP280_addr);       //sensor address
  Wire.write(0xF7);             //first reg
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_addr, 3, true);
   press_raw_data = Wire.read();
    press_raw_data <<= 8;
    press_raw_data |= Wire.read();
    press_raw_data <<= 8;
    press_raw_data |= Wire.read();
  return press_raw_data;
}

uint32_t BlowClass::temp_read() {        //value from sensor
  Wire.beginTransmission(BMP280_addr);   //sensor address
  Wire.write(0xFA);                      //first register
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_addr, 3, true);
  temp_raw_data = Wire.read();
    temp_raw_data <<= 8;
    temp_raw_data |= Wire.read();
    temp_raw_data <<= 8;
    temp_raw_data |= Wire.read();
  temp_raw_data >>= 4;                 //remove zeroes from last bit
  return temp_raw_data;
}

void BlowClass::readTemperature() {    //tempriture counting
  int32_t _var1, _var2;

  int32_t adc_T = temp_read();

  _var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  _var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

  t_fine = _var1 + _var2 ;
}

float BlowClass::readPressure() {        //pressure counting
  int64_t _var1, _var2, _press;

  int32_t adc_P = BlowShield.press_read();
  adc_P >>= 4;

  _var1 = (t_fine) - 128000;
  _var2 = _var1 * _var1 * (int64_t)dig_P6;
  _var2 = _var2 + ((_var1 * (int64_t)dig_P5) << 17);
  _var2 = _var2 + (((int64_t)dig_P4) << 35);
  _var1 = ((_var1 * _var1 * (int64_t)dig_P3) >> 8) + ((_var1 * (int64_t)dig_P2) << 12);
  _var1 = (((((int64_t)1) << 47) + _var1)) * ((int64_t)dig_P1) >> 33;

  if (_var1 == 0) {
    return 0;               // avoid exception caused by division by zero
  }
  _press = 1048576 - adc_P;
  _press = (((_press << 31) - _var2) * 3125) / _var1;
  _var1 = (((int64_t)dig_P9) * (_press >> 13) * (_press >> 13)) >> 25;
  _var2 = (((int64_t)dig_P8) * _press) >> 19;

  _press = ((_press + _var1 + _var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)_press / 256;            //pressure in Pa
}


//BlowShield library

void BlowClass::begin(void) {  
#ifdef ARDUINO_ARCH_AVR  
  #if SHIELDRELEASE == 1                     // For shield version 1
    pinMode(BLOW_UPIN,OUTPUT);               // Set pump pin
    pinMode(BLOW_RPIN,INPUT);                // Set sensor pin
    digitalWrite(BLOW_UPIN,LOW);             // Turn off pump 
  #endif
 #endif
Wire.begin();
  BlowShield.begin_config();
  BlowShield.readCoefficients();
  BlowShield.readTemperature();             //necessary for t_fine
delay(5000);
}

void BlowClass::calibration(void) {                       // Board calibration
_minPressure=BlowShield.readPressure();               // Read pressure during initialization
_maxPressure=105000.00;                                   // Maximum sensor pressure
_wasCalibrated=true;
}

float BlowClass::referenceRead(void) {                              // Reference read
int val=analogRead(BLOW_RPIN);                                      // Potentiometer value read
float percento = map((float)val, 0.0, 674.0, 0.0, 100.0);           // Mapping potentiometer on 3.3V value to 0 - 100
return percento;
}

void BlowClass::actuatorWrite(float aPercent) {                     // Pump activator
float inPercento = aPercent*255/100;                                
analogWrite(BLOW_UPIN,inPercento);                                  // Turn on pump
}

float BlowClass::sensorRead(void) {                                 // Sensor read
unsigned long int val=BlowShield.readPressure();                // Read sensor value
if (val>105000) {                                                   // Sensor max value safety control
  digitalWrite(BLOW_UPIN,LOW);
}
  float percento = map((long int)val, _minPressure,_maxPressure, 0.00, 100.00);   // Mapping sensor value to 0 - 100         
  return percento;
}

BlowClass BlowShield;                                   // Creation of BlowClass object
