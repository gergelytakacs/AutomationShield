/*
  API for the PressureShield didactic hardware.
  The file is a part of the application programmers interface for
  the PressureShield didactic tool for control engineering and
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

#include "PressureShield.h"                      // Include header file

// Sensor library

void PressureClass::readCoefficients() {
  dig_T1 = ((uint16_t)((PressureShield.readRegister(BMP280_DIG_T1_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_T1_LSB_REG)));
  dig_T2 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_T2_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_T2_LSB_REG)));
  dig_T3 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_T3_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_T3_LSB_REG)));

  dig_P1 = ((uint16_t)((PressureShield.readRegister(BMP280_DIG_P1_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P1_LSB_REG)));
  dig_P2 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P2_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P2_LSB_REG)));
  dig_P3 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P3_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P3_LSB_REG)));
  dig_P4 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P4_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P4_LSB_REG)));
  dig_P5 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P5_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P5_LSB_REG)));
  dig_P6 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P6_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P6_LSB_REG)));
  dig_P7 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P7_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P7_LSB_REG)));
  dig_P8 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P8_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P8_LSB_REG)));
  dig_P9 = ((int16_t)((PressureShield.readRegister(BMP280_DIG_P9_MSB_REG) << 8) + PressureShield.readRegister(BMP280_DIG_P9_LSB_REG)));
}


void PressureClass::begin_config() {        //configuration -> do begin
  Wire.beginTransmission(BMP280_addr);
  Wire.write(0xF4);
  Wire.write(0b111111);   //temp -> full, press -> full, 20-bit (oversampling x 16), mode -> normal
  Wire.write(0xF5);
  Wire.write(0b0);     //standby time, filter
  Wire.endTransmission();
  delay(10);
}

void PressureClass::measure_config() {       //configuration -> do begin
  Wire.beginTransmission(BMP280_addr);
  Wire.write(0xF4);
  Wire.write(0b1111);   //temp -> full, press -> full, 20-bit (oversampling x 16), mode -> normal
  Wire.write(0xF5);
  Wire.write(0b1000);     //standby time, filter
  
  Wire.endTransmission();
  delay(10);
}

uint8_t PressureClass::readRegister(uint8_t RegSet) {
  uint8_t _result = 0;
  Wire.beginTransmission(BMP280_addr);
  Wire.write(RegSet);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_addr, 1, true);
  _result = Wire.read();
  return _result;
}

uint32_t PressureClass::press_read() {        //value from sensor
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

uint32_t PressureClass::temp_read() {        //value from sensor
  Wire.beginTransmission(BMP280_addr);       //sensor address
  Wire.write(0xFA);         //first reg
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_addr, 3, true);
  temp_raw_data = Wire.read();
    temp_raw_data <<= 8;
    temp_raw_data |= Wire.read();
    temp_raw_data <<= 8;
    temp_raw_data |= Wire.read();
  temp_raw_data >>= 4;   //remove zeroes from last bit
  return temp_raw_data;
}

void PressureClass::readTemperature() {   //tempriture counting
  int32_t _var1, _var2;

  int32_t adc_T = temp_read();

  _var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  _var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

  t_fine = _var1 + _var2 ;
}

float PressureClass::readPressure() {  //pressure counting
  int64_t _var1, _var2, _press;

  int32_t adc_P = PressureShield.press_read();
  adc_P >>= 4;

  _var1 = (t_fine) - 128000;
  _var2 = _var1 * _var1 * (int64_t)dig_P6;
  _var2 = _var2 + ((_var1 * (int64_t)dig_P5) << 17);
  _var2 = _var2 + (((int64_t)dig_P4) << 35);
  _var1 = ((_var1 * _var1 * (int64_t)dig_P3) >> 8) + ((_var1 * (int64_t)dig_P2) << 12);
  _var1 = (((((int64_t)1) << 47) + _var1)) * ((int64_t)dig_P1) >> 33;

  if (_var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  _press = 1048576 - adc_P;
  _press = (((_press << 31) - _var2) * 3125) / _var1;
  _var1 = (((int64_t)dig_P9) * (_press >> 13) * (_press >> 13)) >> 25;
  _var2 = (((int64_t)dig_P8) * _press) >> 19;

  _press = ((_press + _var1 + _var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)_press / 256;    //pressure in Pa
}


//PressureShield library

void PressureClass::begin(void) {  
#ifdef ARDUINO_ARCH_AVR  
  #if SHIELDRELEASE == 1                     // For shield version 1
    pinMode(PRESSURE_UPIN,OUTPUT);               // Set pump pin
    pinMode(PRESSURE_RPIN,INPUT);                // Set sensor pin
    pinMode(PRESSURE_MSPIN,INPUT);               // Set manual switch pin
    digitalWrite(PRESSURE_UPIN,LOW);             // Turn off pump 
    analogReference(EXTERNAL);        
  #endif
 #endif
Wire.begin();
  PressureShield.begin_config();
  PressureShield.readCoefficients();
  PressureShield.readTemperature();       //potrebne pre t_fine
delay(5000);
}

void PressureClass::calibration(void) {                       // Board calibration
_minPressure=PressureShield.readPressure();               // Read pressure during initialization
_maxPressure=102000.00;                                   // Maximum sensor pressure
_wasCalibrated=true;
}

float PressureClass::referenceRead(void) {                              // Reference read
int val=analogRead(PRESSURE_RPIN);                                      // Potentiometer value read
float percento = map((float)val, 0.0, 1023.0, 0.0, 100.0);           // Mapping potentiometer on 3.3V value to 0 - 100
return percento;
}

void PressureClass::actuatorWrite(float aPercent) {                     // Pump activator
float inPercento = aPercent*255/100;                                
analogWrite(PRESSURE_UPIN,inPercento);                                  // Turn on pump
}

float PressureClass::sensorRead(void) {                                 // Sensor read
unsigned long int val=PressureShield.readPressure();                // Read sensor value
if (val>102000) {                                                   // Sensor max value safety control
  digitalWrite(PRESSURE_UPIN,LOW);
}
  float percento = map((long int)val, _minPressure,_maxPressure, 0.00, 100.00);   // Mapping sensor value to 0 - 100         
  return percento;
}

PressureClass PressureShield;                                   // Creation of PressureClass object
