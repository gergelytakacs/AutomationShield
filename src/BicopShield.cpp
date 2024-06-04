/*
  API for the BiCopShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education. The AeroShield implements an air-based,
  pendulum angle positioning experiment on an Arduino microcontroller.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Nemcek, Jan Boldocky
  Last update: 20.5.2024
*/

#include "BicopShield.h"         // Include header file
int count = 0.0;                  // Counter variable for use in calibration

// Initializes hardware pins
float BicopClass::begin(void){                                 // Board initialisation
  #ifdef ARDUINO_ARCH_AVR                                             // For AVR architecture boards
    Wire.begin();                                                     // Use Wire object
    as5600.setWirePtr(&Wire);                                                    
  #elif ARDUINO_ARCH_SAM                                              // For SAM architecture boards
    Wire1.begin();                                                    // Use Wire1 object
    as5600.setWirePtr(&Wire1);                                                   
  #elif ARDUINO_ARCH_SAMD                                             // For SAMD architecture boards
    Wire.begin();                                                     // Use Wire object
    as5600.setWirePtr(&Wire);
  #endif
                                                        
  
  as5600.begin();
  bool isDetected = as5600.detectMagnet();
  //pinMode(AERO_UPIN1, OUTPUT);  		                                  // Actuator pin
  //pinMode(AERO_RPIN, INPUT);
  //pinMode(AERO_VOLTAGE_SENSOR_PIN, INPUT);
  if(isDetected == 0){                                           // If magnet not detected go on
    while(1){                                                           // Go forever until magnet detected 
        if(isDetected == 1){                                           // If magnet detected
            AutomationShield.serialPrint("Magnet detected \n");         // Print information then break
            break;
        }
        else{                                                           // If magnet not detected 
            AutomationShield.serialPrint("Can not detect magnet \n");   // Print information then go back to check while statement
        }
    }
  }       
} 


    bool BicopClass::calibrate(void) {                                       // Calibration 
        while (count < 20) {
            analogWrite(BICOP_UPIN2, 50);
            delay(100);
            count++;
        }
        wasCalibrated = as5600.calibrate(_min_angle, _max_angle);
        return wasCalibrated;
    }

    float BicopClass::referenceRead(void) {                                                  // Reference read
        return AutomationShield.mapFloat(analogRead(BICOP_RPIN), 0.0, 1024.0, 0.0, 100.0);   // Returns the percentual position of potentiometer runner
    }

    void BicopClass::actuatorWrite(float inputPercentageM1, float inputPercentageM2) {                     // Actuator write - expected input 0 - 100 %
        analogWrite(BICOP_UPIN1, AutomationShield.percToPwm(inputPercentageM1));  // Write remapped value to actuator pin
        analogWrite(BICOP_UPIN2, AutomationShield.percToPwm(inputPercentageM2));  // Write remapped value to actuator pin 
    }

    void BicopClass::actuatorWriteVolt(float voltageValueM1, float voltageValueM2) {        // Expects Root Mean Square Voltage value as input - maximum 3.7
        analogWrite(BICOP_UPIN1, sq(AutomationShield.constrainFloat(voltageValueM1, 0.0, 3.7)) * 255.0 / sq(3.7)); //--Convert Urms -> PWM duty 8-bit
        analogWrite(BICOP_UPIN2, sq(AutomationShield.constrainFloat(voltageValueM2, 0.0, 3.7)) * 255.0 / sq(3.7));
    }

    float BicopClass::sensorRead(void) {          // Read relative angle in percentage. Maximum angle defined by the argument. Default maximum 90 degrees.
        if (wasCalibrated) return as5600.readAngleDeg(); //AutomationShield.mapFloat(as5600.readAngleDeg(), -maximumDegrees, maximumDegrees, -100.0, 100.0);  //  Relative angle degrees to percentage conversion.
        else {
            AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
            return -1;
        }
    }

    float BicopClass::sensorReadDegree() {
        if (wasCalibrated) return as5600.readAngleDeg(); // Read relative angle in degrees
        else {
            AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
            return -1;
        }
    }

    uint16_t BicopClass::sensorReadDegreeAbsolute() {
        return as5600.readAngleReg(); // Read absolute angle in degrees
        // if(wasCalibrated)
        // else{ 
        //  AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
        //  return -1;
        // }
    }

    float BicopClass::sensorReadRadian() {
        if (wasCalibrated) return as5600.readAngleRad(); // Read relative angle in radians
        else {
            AutomationShield.serialPrint("The device was *not calibrated*, please run the calibration() method.");
            return -1;
        }
    }



    word BicopClass::getRawAngle()                                                             // Function for getting raw pendulum angle data 0-4096
    {
        return as5600.readAngleReg();
    }

    //BoB commands
    void BicopClass::TOFinitialize() {
        sens.begin();
# if ECHO_TO_SERIAL 
        Serial.println("Adafruit VL6180x test!");
        if (!sens.begin()) {
            Serial.println("Failed to find sensor");
            while (1);
        }
        Serial.println("Sensor found!");
# endif
    }

    //values from sensor in % - for possible future use
    float BicopClass::TOFsensorReadPerc() {
        range = sens.readRange();

        if (range < minCalibrated) { range = minimum; }
        else if (range > maxCalibrated) { range = maximum; }


        posperc = map(range, minimum, maximum, 0, 100);
        return posperc;               //returns the ball distance in 0 - 100 %
    }

    // returns the corrected value of sensor
    float BicopClass::TOFsensorRead() {
        if (calibrated == 1) {                         // if calibration function was already processed (calibrated flag ==1)
            pos = sens.readRange();
            ballPos = pos - minCalibrated;                   //set actual position to position - calibrated minimum
        }
        else if (calibrated == 0) {                        // if calibration function was not processed (calibrated flag ==0)
            pos = sens.readRange();
            ballPos = pos - MIN_CALIBRATED_DEFAULT;         //set actual position to position - predefined value
        }
# if ECHO_TO_SERIAL 
        Serial.print("pos :"); Serial.print(pos); Serial.print(" ");
        Serial.print("ballPos :"); Serial.print(ballPos); Serial.print(" ");
# endif
        return ballPos;
    }

    float BicopClass::deg2rad(float u) {
        u = u * (3.14 / 180.0);
        return u;
    }

    float BicopClass::rad2deg(float u) {
        u = u * (180.0 / 3.14);
        return u;
    }


    // check minimal and maximal value of sensor for BoB
    void BicopClass::TOFcalibration()
    {
        short calmeasure; 				         		// Temporary measurement value
# if ECHO_TO_SERIAL                        
        Serial.println("Calibration is running...");
# endif

        
            analogWrite(BICOP_UPIN2, 0);
            analogWrite(BICOP_UPIN1, 150);
            delay(1000);
      			
            
            for (int i = 1; i <= 100; i++) {	    			// Perform 100 measurements
                calmeasure = sens.readRange(); 			    // Measure
                if (calmeasure < minCalibrated) { 			// If lower than already
                    minCalibrated = calmeasure; 			// Save new minimum
                }

                delay(10);                                  // Measure for one second
            }

            analogWrite(BICOP_UPIN2, 150);
            analogWrite(BICOP_UPIN1, 0);
            delay(1000);
           
                for (int i = 1; i <= 100; i++) {	    			// Perform 100 measurements
                    calmeasure = sens.readRange(); 		    	// Measure
                    if (calmeasure > maxCalibrated) { 			// If lower than already
                        maxCalibrated = calmeasure; 			// Save new maximum
                    }

                    delay(10);                                  // Measure for one second
                }
                delay(500);

# if ECHO_TO_SERIAL                               //if user sets ECHO_TO_SERIAL flag than print    
                Serial.print("Measured maximum is: ");
                Serial.print(maxCalibrated);
                Serial.println(" mm");
                Serial.print("Measured minimum is: ");
                Serial.print(minCalibrated);
                Serial.println(" mm");
# endif	
                calibrated = 1;
            }
        
      
        //end BoB Commands
    
    BicopClass BicopShield;
    