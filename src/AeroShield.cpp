/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education. The AeroShield implements an air-based,
  pendulum angle positioning experiment on an Arduino microcontroller.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Peter Tibenský
  Last update: 24.02.2022
*/

#include "AeroShield.h"         // Include header file

// Initializes hardware pins
float AeroClass::begin(void){                             // Board initialisation
  #ifdef ARDUINO_ARCH_AVR                                             // For AVR architecture boards
    Wire.begin();                                                     // Use Wire object
  #elif ARDUINO_ARCH_SAM                                              // For SAM architecture boards
    Wire1.begin();                                                    // Use Wire1 object
  #elif ARDUINO_ARCH_SAMD                                             // For SAMD architecture boards
    Wire.begin();                                                     // Use Wire object
  #endif
  bool isDetected = AeroShield.detectMagnet();
  pinMode(AERO_UPIN, OUTPUT);  		                                  // Actuator pin
  pinMode(AERO_RPIN, INPUT);
  pinMode(AERO_VOLTAGE_SENSOR_PIN, INPUT);
  if(isDetected == 0){                                           // If magnet not detected go on
    while(1){                                                           // Go forever until magnet detected 
        if(isDetected == 1){                                           // If magnet detected
            AutomationShield.serialPrint("Magnet detected \n");         // Print information then break
            break;
        }
        else{                                                           // If magnet not detected 
            AutomationShield.serialPrint("Can not detect magnet \n");   // Print information then go back to check while statement
        }
    }
  }       
} 

float AeroClass::calibration(void){                          // Calibration 
  AutomationShield.serialPrint("Calibration running...\n");             // Print info 
  startAngle = AeroShield.getRawAngle();
    for(int i=0;i<3;i++){                                               // Simple sound indication of successful calibration 3 beeps
      analogWrite(AERO_UPIN,1);                                         // Actuator powereded just a bit so the rotor doesn't turn just beep 
      delay(200);                                                       // wait 
      analogWrite(AERO_UPIN,0);                                         // Actuator powered off
      delay(200);                                                       // wait 
      }
  wasCalibrated = true;
  AutomationShield.serialPrint("Calibration done");
  AutomationShield.serialPrint("\n");
  AutomationShield.serialPrint("The starting angle value is: ");
  Serial.print(startAngle);
  return startAngle;                                                  // Return start angle
}

float AeroClass::referenceRead(void) {                                                  // Reference read
  return AutomationShield.mapFloat(analogRead(AERO_RPIN), 0.0, 1024.0, 0.0, 100.0);   // Returns the percentual position of potentiometer runner
}

void AeroClass::actuatorWrite(float percentValue) {                     // Actuator write - expected input 0 - 100 %
  analogWrite(AERO_UPIN, AutomationShield.percToPwm(percentValue));          // Write remapped value to actuator pin 
}

void AeroClass::actuatorWriteVolt(float voltageValue){        // Expects Root Mean Square Voltage value as input - maximum 3.7
  analogWrite(AERO_UPIN, sq(AutomationShield.constrainFloat(voltageValue, 0.0, 3.7))*255.0/sq(3.7)); //--Convert Urms -> PWM duty 8-bit
}

float AeroClass::sensorRead(float maximumDegrees){          // Read relative angle in percentage. Maximum angle defined by the argument. Default maximum 90 degrees.
 if(wasCalibrated) return AutomationShield.mapFloat((float)((int)readTwoBytes(_raw_ang_hi, _raw_ang_lo) - startAngle)*360.0/4096.0, -maximumDegrees, maximumDegrees, -100.0, 100.0);  //  Relative angle degrees to percentage conversion.
 else{ 
  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
  return -1;
 }
}

float AeroClass::sensorReadDegree(){
 if(wasCalibrated) return (float)((int)readTwoBytes(_raw_ang_hi, _raw_ang_lo) - startAngle)*360.0/4096.0; // Read relative angle in degrees
 else{ 
  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
  return -1;
 }
}

float AeroClass::sensorReadRadian(){
 if(wasCalibrated) return (float)((int)readTwoBytes(_raw_ang_hi, _raw_ang_lo) - startAngle)*3.14159*2.0/4096.0; // Read relative angle in radians
 else{
  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
  return -1;
 }
}

float AeroClass::sensorReadAbsolute(){
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo)*360.0/4096.0; // Read absolute angle of pendulum arm in degrees
}

float AeroClass::sensorReadCurrent(){ // Measure current in A
  return analogRead(AERO_VOLTAGE_SENSOR_PIN)*ARES/10/ShuntRes;
}

uint16_t AeroClass::sensorReadCurrentRaw(){
  return analogRead(AERO_VOLTAGE_SENSOR_PIN); // Read voltage indicating the current draw in 10-bit scale
}

word AeroClass::getRawAngle()                                                             // Function for getting raw pendulum angle data 0-4096
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);                                           // Another function for communication with senzor, called from this library 
}

int AeroClass::detectMagnet()                                                             // Function for detecting presence of magnet 
{
  int magStatus;                                                                           // Auxiliary variable
  int retVal = 0;                                                                          // Auxiliary variable
  magStatus = readOneByte(_stat);                                                          // Another function for communication with senzor, called from this library                         

  if (magStatus & 0x20)
    retVal = 1;
  return retVal;                                                                           // Return value 
}

int AeroClass::getMagnetStrength()                 // Function for getting the strength of magnet 
{
  int magStatus;                                    // Auxiliary variable
  int retVal = 0;                                   // Auxiliary variable
  magStatus = readOneByte(_stat);                   // Another function for communication with senzor, called from this library     

  if (detectMagnet() == 1)                          // Return 0 if no magnet is detected
  {
    retVal = 2;                                     // Return 2 if magnet is just right
    if (magStatus & 0x10)
      retVal = 1;                                   // Return 1 if magnet is too weak
    else if (magStatus & 0x08)
      retVal = 3;                                   // Return 3 if magnet is too strong
  }

  return retVal;                                    // Return value 
}

int AeroClass::readOneByte(int in_adr)             // Function for communicating with the senzor using 1 Byte 
{
  int retVal = -1;
  Wire.beginTransmission(_ams5600_Address);         // Initialise wire transmission 
  Wire.write(in_adr);                               // Write 4 bits 
  Wire.endTransmission();                           // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);            // Request answer  
  while (Wire.available() == 0);                    // Wait for returning bits 

  retVal = Wire.read();                             // Store returning bits 

  return retVal;                                    // Return stored bits 
}

word AeroClass::readTwoBytes(int in_adr_hi, int in_adr_lo)          // Function for communicating with the senzor using 2 Bytes 
{
  word retVal = -1;

  /* Read Low Byte */
  Wire.beginTransmission(_ams5600_Address);        // Initialise wire transmission 
  Wire.write(in_adr_lo);                           // Write 4 bits 
  Wire.endTransmission();                          // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);           // Request answer  
  while (Wire.available() == 0);                    // Wait for returning bits 

  int low = Wire.read();                           // Store first returning bits 

  /* Read High Byte */
  Wire.beginTransmission(_ams5600_Address);        // Initialise wire transmission 
  Wire.write(in_adr_hi);                           // Write 4 bits   
  Wire.endTransmission();                          // End wire transmission 
  Wire.requestFrom(_ams5600_Address, 1);           // Request answer  
  while (Wire.available() == 0);                    // Wait for returning bits 

  word high = Wire.read();                         // Store second returning bits 

  high = high << 8;                                // bitwise left shift
  retVal = high | low;

  return retVal;                                   // Return stored bits 
}

AeroClass AeroShield; 
