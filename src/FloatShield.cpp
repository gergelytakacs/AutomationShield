/*
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
  Last update: 9.7.2019.
*/

#include "FloatShield.h"         // Include header file

#ifdef VL53L0X_h                 // If library for distance sensor was sucessfully included

void FloatClass::begin(void) {                                        // Board initialisation
    AutomationShield.serialPrint("FloatShield initialisation...");
    Wire.begin();
    if (!distanceSensor.init()) {                                     // Setting up I2C distance sensor
        AutomationShield.error("FloatShield failed to initialise!");
    }
    distanceSensor.setMeasurementTimingBudget(20000);                 // Setting high-speed mode for laser sensor
    distanceSensor.startContinuous();                                 // Setting continuous mode for laser sensor
    _minDistance = 17.0;                                              // Initializing min,max variables by approximate values so the functions can be used even without calibration but with lower precision
    _maxDistance = 341.0;
    _range = _maxDistance - _minDistance;    
    _wasCalibrated = false;
    AutomationShield.serialPrint(" successful.\n");
}

void FloatClass::calibrate(void) {                         // Board calibration
    AutomationShield.serialPrint("Calibration...");        
    float sum = 0.0;
    actuatorWrite(100.0);                                  // Sets fan speed to maximum
    while(sensorReadDistance() > 100.0) {                  // Waits until the ball is at least in the upper third of the tube
        delay(100);                                        // (This is because of varying time it takes for the ball to travel the first half of the tube)
    }
    delay(1000);                                           // Waits unil the ball reaches top and somewhat stabilises itself
    for(int i=0; i<100; i++) {                             // Measures 100 values with sampling 25 miliseconds and calculates the average reading from those values
        sum += sensorReadDistance();                       // (Simply using minimal value would be inconsistent because of shape of the ball combined with its ability to move horizontally in the tube)
        delay(25);
    }
    _minDistance = sum / 100.0;                            // Sets the minimal distance variable equal to the average reading

    sum = 0.0;
    actuatorWrite(0.0);                                    // Turns off the fan
    while(sensorReadDistance() < 300.0) {                  // Waits until the ball is at least in the lower third of the tube
        delay(100);                                        // (This is probably unecessary, because ball has no problem falling down, but for the sake of consistency)
    }
    delay(1000);
    for(int i=0; i<100; i++) {                             // Measures 100 values with sampling 25 miliseconds and calculates the average reading from those values
        sum += sensorReadDistance();                       // (Simply using maximal value would be inconsistent as in previous case)
        delay(25);
    }
    _maxDistance = sum / 100.0;                            // Sets the maximal distance variable equal to the average reading
    _range = _maxDistance - _minDistance;                  // Sets the range variable equal to the difference of the maximal and minimal distance

   
    _wasCalibrated = true;                                 // Sets the status of calibration to true
    AutomationShield.serialPrint(" sucessful.\n");
}

void FloatClass::actuatorWrite(float aPercent) {                                           // Write actuator
    float mappedValue = AutomationShield.mapFloat(aPercent, 0.0, 100.0, 0.0, 255.0);       // Takes the float type percentual value 0.0-100.0 and remapps it to range 0.0-255.0
    mappedValue = AutomationShield.constrainFloat(mappedValue, 0.0, 255.0);                // Constrains the remapped value to fit the range 0.0-255.0 - safety precaution
    analogWrite(FLOAT_UPIN, (int)mappedValue);                                             // Sets the fan speed using the constrained value
}

float FloatClass::referenceRead(void) {                                                        // Reference read
    _referenceValue = (float)analogRead(FLOAT_RPIN);                                           // Reads the actual analog value of potentiometer runner
    _referencePercent = AutomationShield.mapFloat(_referenceValue, 0.0, 1023.0, 0.0, 100.0);   // Remapps the analog value from original range 0.0-1023 to percentual range 0.0-100.0
    return _referencePercent;                                                                  // Returns the percentual position of potentiometer runner
}

float FloatClass::sensorRead(void) {                                                                         // Sensor read
    float readDistance = sensorReadDistance();                                                               // Reads the actual distance between ball and sensor in milimetres
    _sensorPercent = AutomationShield.mapFloat(readDistance, _maxDistance, _minDistance, 0.0, 100.0);        // Remapps the distance based on minimal and maximal distances from calibration to the percentual altitude of the ball
    _sensorPercent = AutomationShield.constrainFloat(_sensorPercent, 0.0, 100.0);                            // Constrains the percentual altitude - safety precaution
    return _sensorPercent;                                                                                   // Returns the percentual altitude of the ball in the tube
}

float FloatClass::sensorReadAltitude(void) {                  // Sensor read altitude
    _ballAltitude = sensorReadDistance();                     // Reads the current distance of the ball from sensor
    _ballAltitude = _maxDistance - _ballAltitude;             // Inverts the reading so the bottom position is 0
    return _ballAltitude;                                     // Returns the current altitude of the ball in milimetres
}

float FloatClass::sensorReadDistance(void) {                // Sensor read distance                          
    _sensorValue = (float)distanceSensor.readRangeContinuousMillimeters();      // Reads the distance between sensor and the ball in milimetres    
    return _sensorValue;                                                        // Returns the measured distance
}

bool FloatClass::returnCalibrated(void) {                   // Returns calibration status
    return _wasCalibrated;
}

float FloatClass::returnMinDistance(void) {                 // Returns value of minimal distance measured by sensor in milimetres
    return _minDistance;
}

float FloatClass::returnMaxDistance(void) {                 // Returns value of maximal distance measured by sensor in milimetres
    return _maxDistance;
}

float FloatClass::returnRange(void) {                       // Returns range of measured distances between minimal and maximal values in milimetres
    return _range;
}

FloatClass FloatShield;                                     // Creation of FloatClass object
#endif
