/*
Dalsi pokec a'la Vlado tu. 
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

#include "BlowShield.h"                      // Include header file

void BlowClass::begin(void) {  
#ifdef ARDUINO_ARCH_AVR  
  #if SHIELDRELEASE == 1                     // For shield version 1
    pinMode(BLOW_UPIN,OUTPUT);               // Set pump pin
    pinMode(BLOW_RPIN,INPUT);                // Set sensor pin
    digitalWrite(BLOW_UPIN,LOW);             // Turn off pump 
  #endif
 #endif
 
 if (!pressureSensor.begin(BMP280_ADRESA)) {                              // Setting up I2C pressure sensor
    AutomationShield.error("BlowShield failed to initialise!");           // Error if sensor is not connected
  while (1);
 }
}

void BlowClass::calibration(void) {                       // Board calibration
_minPressure=pressureSensor.readPressure();               // Read pressure during initialization
_maxPressure=106000.00;                                   // Maximum sensor pressure
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
unsigned long int val=pressureSensor.readPressure();                // Read sensor value
if (val>106000) {                                                   // Sensor max value safety control
  digitalWrite(BLOW_UPIN,LOW);
}
  float percento = map((long int)val, _minPressure,_maxPressure, 0.00, 100.00);   // Mapping sensor value to 0 - 100         
  return percento;
}

BlowClass BlowShield;                                   // Creation of BlowClass object
