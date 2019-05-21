#include "BOBShield.h"
Servo myservo;



// declaring PIN and initializing sensor library
void BOBClass::begin() {
  pinMode(BOB_RPIN, INPUT);			// set Potentiometer pin
  myservo.attach(BOB_UPIN);				// set Servo pin
}

// Check if Adafruit VL6180 sensor is available
void BOBClass::initialize() {
      Serial.println("Adafruit VL6180x test!");
  if (! sens.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

void BOBClass::calibration()
{
	short calmeasure; 				   		// Temporary measurement value
  Serial.println("Begining calibratione ");

	BOBShield.actuatorWrite(-30);					// Tilt beam to the minimum so the ball falls towards the sensor
	delay(1000);					   		// Wait for things to settle
	for (int i=1; i<=100; i++) {	    				// Perform 100 measurements
		calmeasure = BOBShield.sensorReadCal(); 			// Measure
		if (calmeasure < minCalibrated){ 			// If lower than already
			minCalibrated = calmeasure; 			// Save new minimum
		}
		delay(10);                                  // Measure for one second
	}

	BOBShield.actuatorWrite(30);					// Tilt beam to the maximum so the ball falls towards the sensor
	delay(1000);					   		// Wait for things to settle
	for (int i=1; i<=100; i++) {	    				// Perform 100 measurements
		calmeasure = BOBShield.sensorReadCal(); 			// Measure
		if (calmeasure > maxCalibrated){ 			// If lower than already
			maxCalibrated = calmeasure; 			// Save new maximum
		}
		delay(10);                                  // Measure for one second
	}
	
	BOBShield.actuatorWrite(0);
	
//	maxRange = maxCalibrated + 4;					// Middle of the ball minimum
//	minRange = minCalibrated + 4;					// Middle of the ball maximum
	 
  Serial.print("Measured maximum is: ");
    Serial.print(maxCalibrated);
     Serial.println(" mm");
  Serial.print("Measured minimum is: ");
    Serial.print(minCalibrated);
     Serial.println(" mm");
      calibrated = 1;
}


//values from potentiometer in %
float BOBClass::referenceRead(){
   _referenceRead = analogRead(BOB_RPIN);
   _referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return _referenceValue;
}

//values from potentiometer computed for servo
void BOBClass::actuatorWrite(int deg){

if (deg<-30) {
  deg=-30;
}
else if (deg>30) {
  deg=30;
}

degree = map(deg,-30,30,70,130);

if (degree<70) {
  degree=70;
}
else if (degree>130) {
  degree=130;
}

	myservo.write(degree);

}

//values from sensor in %
float BOBClass::sensorReadPerc(){
 range = sens.readRange();                          //tuta toto chyba nejde

    if (range < minCalibrated) {range = minimum;}
    else if (range > maxCalibrated) {range = maximum;}


  posperc = map(range,minimum,maximum,0,100);
 return posperc;
 }

//values from sensor in mm
/*float BOBClass::sensorRead(){
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
*/

float BOBClass::sensorRead(){
 pos = sens.readRange();
 position = pos - minCalibrated;
 
 ballPos = position + 4;					// middle of the ball position
 
 return ballPos;
}
float BOBClass::sensorReadCal(){
 posCal = sens.readRange();
 return posCal;
}

BOBClass BOBShield;
