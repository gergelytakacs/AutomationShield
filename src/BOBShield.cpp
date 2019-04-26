#include "BOBShield.h"

#ifdef ADAFRUIT_VL6180X_H

/*void BOBClass::begin(long baudRate) {
  Serial.begin(115200);
}*/


// declaring PIN and initializing sensor library
void BOBClass::initialize() {
  pinMode(POT_PIN, INPUT);
    Serial.println("Adafruit VL6180X test");
    if (!sens.begin(i2c_addr, _debug)) {
        Serial.println(F("Failed to boot VL6180X"));
        while(1);
    }
    Serial.println(F("VL6180X API Simple Ranging example\n\n"));
}

void BOBClass::debug() {
    _debug = true;
}

//values from potenciometer in %
float BOBClass::referenceRead(){
   _referenceRead = analogRead(POT_PIN);
   _referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return _referenceValue;
}

//values from potenciometer computed for servo
float BOBClass::servoWrite(){
   _referenceRead = analogRead(POT_PIN);
   _servoValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,70.00,130.00);
  return _servoValue;
}

//values from sensor in %
void BOBClass::sensorRead(){
 range = sens.readRange();
 status = sens.readRangeStatus();
 if (status == VL6180X_ERROR_NONE) {
   Serial.print("Range: "); Serial.println(range);

   pos = map(range,minimum,maximum,100,0);
 return pos();
 }
 }

// printing some default errors  from sensor library
 void BOBClass::printerrors(){
 status = sens.readRangeStatus();


 if (status == VL6180X_ERROR_NONE) {
   Serial.print("Range: "); Serial.println(range);
 }

if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
  Serial.println("System error");
}
else if (status == VL6180X_ERROR_ECEFAIL) {
  Serial.println("ECE failure");
}
else if (status == VL6180X_ERROR_NOCONVERGE) {
  Serial.println("No convergence");
}
else if (status == VL6180X_ERROR_RANGEIGNORE) {
  Serial.println("Ignoring range");
}
else if (status == VL6180X_ERROR_SNR) {
  Serial.println("Signal/Noise error");
}
else if (status == VL6180X_ERROR_RAWUFLOW) {
  Serial.println("Raw reading underflow");
}
else if (status == VL6180X_ERROR_RAWOFLOW) {
  Serial.println("Raw reading overflow");
}
else if (status == VL6180X_ERROR_RANGEUFLOW) {
  Serial.println("Range reading underflow");
}
else if (status == VL6180X_ERROR_RANGEOFLOW) {
  Serial.println("Range reading overflow");
}
}

BOBClass BOBShield;
#endif
