<<<<<<< Updated upstream
/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by .... 
  Last update: 05.02.2022.
*/

// Defining c++ library used by the AeroShield 
#ifndef AEROSHIELD_H			       // Include guard
#define AEROSHIELD_H			

// Defining libraries used by the AeroShield 
#include "AutomationShield.h"    // Include the main library
#include <Wire.h>                // Include I2C protocol library
#include <AS5600.h>              // Include hall sensor library 
#include "Arduino.h"			       // Required Arduino API in libraries


// Defining pins used by the AeroShield 
#define AERO_RPIN A3             // Input from potentiometer
#define VOLTAGE_SENSOR_PIN A2            // Input pin for measuring Vout
#define AERO_UPIN 5              // Motor (Actuator)
#define AERO_SR 0.1              // Shunt resistor value (in ohms)
#define VOLTAGE_REF 5.0 	       // Voltage reference for current measurement



class AeroShieldClass{		    	                       // Class for the AeroShield device

 public:

    void begin();                                      // Board initialisation - initialisation of pin modes and variables                             
    float ams5600_initialization(bool isDetected);     // Hall senzor initialisation - looking for magnet 
    void actuatorWrite(float PotPercent);              // Write actuator - function takes input 0.0-100.0% and sets motor speed accordingly
    float calibration(word RawAngle);                  // Board calibration - finding out the 0° value in raw format 
    float convertRawAngleToDegrees(word newAngle);     // Convert raw angle from hall senzor to degrees
    float referenceRead(void);                         // Read value of potentiometer and converts it to percentual value 
    float currentMeasure(void);


 private:

    int ang;                                           // Variable for angle reading in degrees 
    float startangle;                                  // Variable for storing zero angle position 
    float referenceValue;                              // Variable for potentiometer value in percent
    float referencePercent;                            // Percentual value of potentiometer 
    float correction1= 4.1220;
    float correction2= 0.33;
    int repeatTimes= 100;
    float voltageReference= 5.0;
    float ShuntRes= 0.1;
    float current;
    float voltageValue;
   
    
};

extern AeroShieldClass AeroShield;                     // Creation of external AeroShieldClass object
#endif
=======
/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by .... 
  Last update: 05.02.2022.
*/

// Defining c++ library used by the AeroShield 
#ifndef AEROSHIELD_H			       // Include guard
#define AEROSHIELD_H			

// Defining libraries used by the AeroShield 
#include "AutomationShield.h"    // Include the main library
#include <Wire.h>                // Include I2C protocol library
#include <AS5600.h>              // Include hall sensor library 
#include "Arduino.h"			       // Required Arduino API in libraries


// Defining pins used by the AeroShield 
#define AERO_RPIN A3             // Input from potentiometer
#define VOLTAGE_SENSOR_PIN A2            // Input pin for measuring Vout
#define AERO_UPIN 5              // Motor (Actuator)
#define AERO_SR 0.1              // Shunt resistor value (in ohms)
#define VOLTAGE_REF 5.0 	       // Voltage reference for current measurement



class AeroShieldClass{		    	                       // Class for the AeroShield device

 public:

    void begin();                                      // Board initialisation - initialisation of pin modes and variables                             
    float ams5600_initialization(bool isDetected);     // Hall senzor initialisation - looking for magnet 
    void actuatorWrite(float PotPercent);              // Write actuator - function takes input 0.0-100.0% and sets motor speed accordingly
    float calibration(word RawAngle);                  // Board calibration - finding out the 0° value in raw format 
    float convertRawAngleToDegrees(word newAngle);     // Convert raw angle from hall senzor to degrees
    float referenceRead(void);                         // Read value of potentiometer and converts it to percentual value 
    float currentMeasure(void);


 private:

    int ang;                                           // Variable for angle reading in degrees 
    float startangle;                                  // Variable for storing zero angle position 
    float referenceValue;                              // Variable for potentiometer value in percent
    float referencePercent;                            // Percentual value of potentiometer 
    float correction1= 4.1220;
    float correction2= 0.33;
    int repeatTimes= 100;
    float voltageReference= 5.0;
    float ShuntRes= 0.1;
    float current;
    float voltageValue;
   
    
};

extern AeroShieldClass AeroShield;                     // Creation of external AeroShieldClass object
#endif
>>>>>>> Stashed changes
