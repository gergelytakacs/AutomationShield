/*
    Matlab implementation of Pololu arduino library for
    VL53L0X Time-of-Flight distance sensor.

    This code is part of the AutomationShield hardware and software
    ecosystem. Visit http://www.automationshield.com for more
    details. This code is licensed under a Creative Commons
    Attribution-NonCommercial 4.0 International License.

    Created by Peter Chmurciak.
    Last update: 9.7.2019.
*/

// Include necessary libraries
#include "LibraryBase.h"
#include "Wire.h"
#include "../../../../src/lib/vl53l0x-arduino/VL53L0X.cpp"

// Define command IDs for commandHandler method
#define Pololu_VL53L0X_CREATE      0x00
#define Pololu_VL53L0X_BEGIN       0x01
#define Pololu_VL53L0X_READ        0x02
#define Pololu_VL53L0X_DELETE      0x03

// Sensor class definition, inheriting LibraryBase class properties
class Pololu_VL53L0X : public LibraryBase {
public:
    // Sensor object pointer
    VL53L0X *distanceSensor; 
    
    // Constructor
    Pololu_VL53L0X(MWArduinoClass& a) {
        libName = "Pololu/Pololu_VL53L0X";
        a.registerLibrary(this);
    }
    
    // Override the commandHandler method, assigning each command ID to appropriate method
    void commandHandler(byte cmdID, byte* dataIn, unsigned int payloadSize) {
        // Decide based on received command ID
        switch(cmdID) {            
        // Create sensor object
        case Pololu_VL53L0X_CREATE: {
            distanceSensor = new VL53L0X();
            sendResponseMsg(cmdID, 0, 0);
            break;
        }
        
        // Initialise sensor and return 1 or 0 on success or failure
        case Pololu_VL53L0X_BEGIN: {
            Wire.begin();
            byte success[1] = {distanceSensor->init()};
            distanceSensor->setMeasurementTimingBudget(20000);
            distanceSensor->startContinuous();
            sendResponseMsg(cmdID, success, 1);
            break;
        }
        
        // Get measurement from sensor in milimetres and return the information as two bytes
        case Pololu_VL53L0X_READ: {
            uint16_t milliMeter = distanceSensor->readRangeContinuousMillimeters();
            byte high = highByte(milliMeter);
            byte low = lowByte(milliMeter);
            byte mm[2] = {high,low};
            sendResponseMsg(cmdID, mm, 2);
            break;
        }
        
        // Delete sensor object
        case Pololu_VL53L0X_DELETE: {
            delete distanceSensor;
            sendResponseMsg(cmdID, 0, 0);
            break;
        }        
        }    // end of switch statement 
    }        // end of commandHandler method definition
};           // end of class definition