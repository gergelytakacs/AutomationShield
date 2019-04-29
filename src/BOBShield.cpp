#include "BOBShield.h"



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

void BOBClass::calibration()
{
	short calmeasure; 				   		// Temporary measurement value
	int minCalibrated;						// Actual minimum value

	BOBShield.servoWrite(130);					// Tilt beam to the maximum so the ball falls towards the sensor
	delay(500);					   			    // Wait for things to settle
	for (int i=1; i<=100; i++) {	    		// Perform 100 measurements
		calmeasure = BOBShield.sensorRead(); 		// Measure
		if (calmeasure < minCalibrated){ 		// If lower than already
			minCalibrated = calmeasure; 		// Save new minimum
		}
		delay(10);								// Wait between measurements
	}

    calibrated = 1;

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

BOBClass BOBShield;

