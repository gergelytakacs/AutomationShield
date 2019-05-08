#include "BOBShield.h"



// declaring PIN and initializing sensor library
void BOBClass::begin() {
  pinMode(BOB_RPIN, INPUT);			// set Potentiometer pin
  myservo.attach(BOB_UPIN);				// set Servo pin
}


void BOBClass::calibration()
{
	short calmeasure; 				   		// Temporary measurement value


	BOBShield.servoWrite(130);					// Tilt beam to the minimum so the ball falls towards the sensor
	delay(100);					   		// Wait for things to settle
	for (int i=1; i<=100; i++) {	    				// Perform 100 measurements
		calmeasure = BOBShield.sensorRead(); 			// Measure
		if (calmeasure < minCalibrated){ 			// If lower than already
			minCalibrated = calmeasure; 			// Save new minimum
		}
	}

	BOBShield.servoWrite(70);					// Tilt beam to the maximum so the ball falls towards the sensor
	delay(100);					   		// Wait for things to settle
	for (int i=1; i<=100; i++) {	    				// Perform 100 measurements
		calmeasure = BOBShield.sensorRead(); 			// Measure
		if (calmeasure > maxCalibrated){ 			// If lower than already
			maxCalibrated = calmeasure; 			// Save new maximum
		}
	}

    calibrated = 1;

}


//values from potentiometer in %
float BOBClass::referenceRead(){
   _referenceRead = analogRead(BOB_RPIN);
   _referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return _referenceValue;
}

//values from potentiometer computed for servo
void BOBClass::actuatorWrite(float degree){
if (degree<70.00) {
  degree=70.00;
}
else if (degree<130.00) {
  degree=130.00;
}
   myservo.write(degree);

}

//values from sensor in %
float BOBClass::sensorReadPerc(){
 range = sens.readRange();

    if (range < minCalibrated) {range = minimum;}
    else if (range > maxCalibrated) {range = maximum;}


  pos = map(range,minimum,maximum,0,100);
 return pos;
 }

//values from sensor in mm
float BOBClass::sensorRead(){
 range = sens.readRange()-minCalibrated;
 if (!calibrated) {
   minCalibrated = MIN_CALIBRATED_DEFAULT;
   maxCalibrated = MAX_CALIBRATED_DEFAULT;
 }

    if (range < minCalibrated) {range = minimum;}
    else if (range > maxCalibrated) {range = maximum;}

   pos = AutomationShield.mapFloat(range,minimum,maximum,0,50); //length of the tube considered as 50mm (just for testing)
   pos = range;
 return pos;
 }


BOBClass BOBShield;
