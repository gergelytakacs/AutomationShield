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

  Created by Peter Tibenský, Erik Mikuláš and Ján Boldocký
  Last update: 03.10.2023
*/

#include "AeroShield.h"         // Include header file

// Initializes hardware pins
void AeroClass::begin(void){                                 // Board initialisation
  #if  defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RENESAS_UNO) || defined(ARDUINO_ARCH_STM32) // For AVR, SAMD, Renesas architecture boards
    Wire.begin();                                                     // Use Wire object
    as5600.setWirePtr(&Wire);                                                    
  #elif ARDUINO_ARCH_SAM                                              // For SAM architecture boards
    Wire1.begin();                                                    // Use Wire1 object
    as5600.setWirePtr(&Wire1);                                                   
  #endif
                                                        
  
  as5600.begin();
  bool isDetected = as5600.detectMagnet();
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

bool AeroClass::calibrate(void){                          // Calibration 
  AutomationShield.serialPrint("Calibration running...\n");             // Print info 
  wasCalibrated = as5600.calibrate(_min_angle,_max_angle);                         // Calibrate sensor range for -50 deg to 200 deg
    for(int i=0;i<3;i++){                                               // Simple sound indication of successful calibration 3 beeps
      analogWrite(AERO_UPIN,1);                                         // Actuator powereded just a bit so the rotor doesn't turn just beep 
      delay(200);                                                       // wait 
      analogWrite(AERO_UPIN,0);                                         // Actuator powered off
      delay(200);                                                       // wait 
      }
  AutomationShield.serialPrint("Calibration done");
  AutomationShield.serialPrint("\n");
  return wasCalibrated;                                                  // Return start angle
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
 if(wasCalibrated) return AutomationShield.mapFloat(as5600.readAngleDeg(), -maximumDegrees, maximumDegrees, -100.0, 100.0);  //  Relative angle degrees to percentage conversion.
 else{ 
  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
  return -1;
 }
}

float AeroClass::sensorReadDegree(){
 if(wasCalibrated) return as5600.readAngleDeg(); // Read relative angle in degrees
 else{ 
  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
  return -1;
 }
}

float AeroClass::sensorReadRadian(){
 if(wasCalibrated) return as5600.readAngleRad(); // Read relative angle in radians
 else{
  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
  return -1;
 }
}

#if SHIELDRELEASE == 2
  float AeroClass::sensorReadCurrent(){ // Measure current in A
    return analogRead(AERO_VOLTAGE_SENSOR_PIN)*ARES/10/ShuntRes;
  }

  uint16_t AeroClass::sensorReadCurrentRaw(){
    return analogRead(AERO_VOLTAGE_SENSOR_PIN); // Read voltage indicating the current draw in 10-bit scale
  }
#endif

word AeroClass::getRawAngle()                                                             // Function for getting raw pendulum angle data 0-4096
{
  return as5600.readAngleReg();                                           
}

AeroClass AeroShield; 
