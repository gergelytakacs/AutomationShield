/*
    Matlab implementation of VL6180X arduino library for
    VL6180X Time-of-Flight distance sensor.

    This code is part of the AutomationShield hardware and software
    ecosystem. Visit http://www.automationshield.com for more
    details. This code is licensed under a Creative Commons
    Attribution-NonCommercial 4.0 International License.

    Created by Michal Bíro.
    Last update: 26.2.2022.
*/

// Include necessary libraries
#include "LibraryBase.h"
#include "Wire.h"
#include "../../../../src/lib/Adafruit_VL6180X/Adafruit_VL6180X.cpp"

// Define command IDs for commandHandler method
#define VL6180X_CREATE      0x00
#define VL6180X_BEGIN       0x01
#define VL6180X_READ        0x02
#define VL6180X_DELETE      0x03

// Sensor class definition, inheriting LibraryBase class properties
class VL6180X : public LibraryBase 
{
   public:
      // Sensor object pointer
       VL6180X *distanceSensor; 
    
      // Constructor
       ML_VL6180X(MWArduinoClass& a) {
        libName = "VL6180X/VL6180X";
        a.registerLibrary(this);
        }
    
    // Override the commandHandler method, assigning each command ID to appropriate method
    void commandHandler(byte cmdID, byte* dataIn, unsigned int payloadSize) {
        // Decide based on received command ID
      switch(cmdID) {            
        // Create sensor object
        case VL6180X_CREATE: {
            distanceSensor = new VL6180X();
            sendResponseMsg(cmdID, 0, 0);
            break;
        }
        
        // Initialise sensor and return 1 or 0 on success or failure
        case VL6180X_BEGIN: {
            Wire.begin();
            byte success[1] = {distanceSensor->init()};
            distanceSensor->setMeasurementTimingBudget(20000);
            distanceSensor->startContinuous();
            sendResponseMsg(cmdID, success, 1);
            break;
        }
        
        // Get measurement from sensor in millimetres and return the information as two bytes
        case VL6180X_READ: {
            uint16_t milliMeter = distanceSensor->readRangeContinuousMillimeters();
            byte high = highByte(milliMeter);
            byte low = lowByte(milliMeter);
            byte mm[2] = {high,low};
            sendResponseMsg(cmdID, mm, 2);
            break;
        }
        
        // Delete sensor object
        case VL6180X_DELETE: {
            delete distanceSensor;
            sendResponseMsg(cmdID, 0, 0);
            break;
        }        
        }    // end of switch statement 
    }        // end of commandHandler method definition
};           // end of class definition