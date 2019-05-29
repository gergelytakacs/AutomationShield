#include "BOBShield.h"
Servo myservo;



// declaring PIN and initializing sensor library
void BOBClass::begin() {
  myservo.attach(BOB_UPIN);				// set Servo pin
}

// Check if Adafruit VL6180 sensor is available, prints if usere sets ECHO_TO_SERIAL flag
void BOBClass::initialize() {
	sens.begin();
	 # if ECHO_TO_SERIAL 
      Serial.println("Adafruit VL6180x test!");
  if (! sens.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
    # endif
}

// check minimal and maximal value of sensor for BoB
void BOBClass::calibration()
{
	short calmeasure; 				   		// Temporary measurement value
	  # if ECHO_TO_SERIAL                        
	Serial.println("Calibration is running...");
  # endif


	BOBShield.actuatorWrite(-30);					// Tilt beam to the minimum so the ball falls towards the sensor
	delay(1000);					   		// Wait for things to settle
	for (int i=1; i<=100; i++) {	    				// Perform 100 measurements
		calmeasure = sens.readRange(); 			// Measure
		if (calmeasure < minCalibrated){ 			// If lower than already
			minCalibrated = calmeasure; 			// Save new minimum
		}
		delay(10);                                  // Measure for one second
	}

	BOBShield.actuatorWrite(30);					// Tilt beam to the maximum so the ball falls towards the sensor
	delay(1000);					   		// Wait for things to settle
	for (int i=1; i<=100; i++) {	    				// Perform 100 measurements
		calmeasure = sens.readRange(); 			// Measure
		if (calmeasure > maxCalibrated){ 			// If lower than already
			maxCalibrated = calmeasure; 			// Save new maximum
		}
		delay(10);                                  // Measure for one second
	}
	
	BOBShield.actuatorWrite(0);

  # if ECHO_TO_SERIAL               //if user sets ECHO_TO_SERIAL flag than print    
  Serial.print("Measured maximum is: ");
    Serial.print(maxCalibrated);
     Serial.println(" mm");
  Serial.print("Measured minimum is: ");
    Serial.print(minCalibrated);
     Serial.println(" mm");
  # endif	
   calibrated = 1;
}


//values from potentiometer in % , for fututre use
float BOBClass::referenceRead(){
   _referenceRead = analogRead(BOB_RPIN);
   _referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return _referenceValue;
}

//values inserted to servo and setting boundaries (saturation)
void BOBClass::actuatorWrite(float fdeg){
	deg= (int) fdeg;       // scale input float parameter into integer (servo works only with integers)
if (deg<-30) {             // predefined boundary for servo range, to prevent hardware damage (in degrees) 
  deg=-30;
}
else if (deg>30) {
  deg=30;
}

degree = map(deg,-30,30,70,130);  // maping inputs defined by user in degrees (-30 / 30) into values understandeable for servo (70 / 130)

	myservo.write(degree);      // write values for servo
}

//values from sensor in %
float BOBClass::sensorReadPerc(){
 range = sens.readRange();                          

    if (range < minCalibrated) {range = minimum;}
    else if (range > maxCalibrated) {range = maximum;}


  posperc = map(range,minimum,maximum,0,100);
 return posperc;               //returns the ball distance in 0 - 100 %
 }


float BOBClass::sensorRead(){  // returns the corected value of sensor
	#if calibrated == 1      // if calibration function was already processed (calibrated flag ==1)
 pos = sens.readRange();
 ballPos  = pos - minCalibrated; //set actual position to position - calibrated minimum

   #elif   calibrated ==0                   // if calibration function was not processed (calibrated flag ==0)
  pos = sens.readRange();     
 ballPos  = pos - MIN_CALIBRATED_DEFAULT; //set actual position to position - predefined value

 #endif
  return ballPos;
}


BOBClass BOBShield;
