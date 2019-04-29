#ifndef BOBSHIELD_H_
#define BOBSHIELD_H_

#include "AutomationShield.h"
#include "lib/Adafruit_VL6180X/Adafruit_VL6180X.h"

/* #ifndef ADAFRUIT_VL6180X_H 				// If library not installed somewhere
#if __has_include("src/lib/Adafruit_VL6180X/Adafruit_VL6180X.h")  // If library present from GIT
	#include "src/lib/Adafruit_VL6180X/Adafruit_VL6180X.h" // Include it from there
#endif
#endif*/

#define POT_PIN 0


class BOBClass {
  public:
    Adafruit_VL6180X sens = Adafruit_VL6180X();    //symbolic name for sensor library
    void  initialize(void);                        //sets up sensor
    float referenceRead();			                  // Read reference pot in %
    float sensorRead();
    void printerrors(void);
    void debug();
    float servoWrite();

    //void begin(long baudRate=115200);


  private:

      uint8_t i2c_addr = 0x29;                  //TODO: determine which address is sensor using
      boolean _debug = false;
      float _referenceRead;
      float _referenceValue;
      uint8_t range;
      uint8_t lastrange;
      uint8_t status;
      float _servoValue;
      int pos;
      int minimum = 20;                      //TODO: determine what's minimum of the clipping interval
      int maximum = 350;                     //TODO: determine what's maximum of the clipping interval

  };
extern BOBClass BOBShield;

#endif
