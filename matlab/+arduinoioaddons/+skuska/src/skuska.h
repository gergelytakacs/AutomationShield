#include "LibraryBase.h"
#include "Wire.h"
#include "../../../../src/lib/Adafruit_VL6180X/Adafruit_VL6180X.cpp"

// Define command IDs for commandHandler method
#define VL6180X_CREATE      0x00
#define VL6180X_BEGIN       0x01
#define VL6180X_READ        0x02
#define VL6180X_DELETE      0x03


class skuska : public LibraryBase
{
    public:
        // Sensor object pointer
        Adafruit_VL6180X *distanceSensor;

        skuska(MWArduinoClass& a)
        {
            libName = "skuska/skuska";
            a.registerLibrary(this);
        }
    void commandHandler(byte cmdID, byte* inputs, unsigned int payload_size)
    {
        switch (cmdID){

        case VL6180X_CREATE: {
            distanceSensor = new Adafruit_VL6180X();
            sendResponseMsg(cmdID, 0, 0);
            break;
        }
        
        // Initialise sensor and return 1 or 0 on success or failure
        case VL6180X_BEGIN: {
            Wire.begin();
            byte success[1] = {distanceSensor->begin()};
            sendResponseMsg(cmdID, success, 1);
            break;
        }
        
        // Get measurement from sensor in millimetres and return the information as two bytes
        case VL6180X_READ: {
            uint8_t mm = distanceSensor->readRange();
            sendResponseMsg(cmdID, mm, 1);
            break;
        }
        
        // Delete sensor object
        case VL6180X_DELETE: {
            delete distanceSensor;
            sendResponseMsg(cmdID, 0, 0);
            break;
        }    
      
        case 0x04:{  
            byte val [13] = "skuska  ";  
            sendResponseMsg(cmdID, val, 13);
            break;
                      }     
        default:{
            // Do nothing
                 }
        }
    }
};