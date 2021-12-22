/*
  FloatShield hardware self test routine 

  Upload the code to your board, then open the Serial
  Monitor function in your Arduino IDE. Be prepared to turn the
  shaft of the potentiometer.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Designed for R4 and above on AVR boards.
  
  Created by Gergely Takács.
  Last update: 13.9.2021.
*/
#include <AutomationShield.h>  
#include <FloatShield.h>     // Include header for hardware API
#include <Sampling.h>                 // Include sampling library


bool testFail = 0; 


// For PID test


unsigned long Ts = 25;                // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
float e = 0;                          // Error

float r = 0.0;                                                    // Reference (Wanted ball altitude)
float R[] = {65.0,50.0,35.0,45.0,60.0,75.0,55.0,40.0,20.0,30.0};  // Reference trajectory
float y = 0.0;                                                    // Output (Current ball altitude)
float u = 0.0;                                                    // Input (Fan power)
float uv= 0.0;                                                    // Input (Fan Voltage)

int T = 500;            // Section length
int i = 0;                // Section counter

#define KP 0.15           // PID Kp constant
#define TI 3              // PID Ti constant
#define TD 0.1           // PID Td constant


void setup() {
  Serial.begin(9600);       // Starts serial communication
  Serial.println();
  Serial.println();
  AutomationShield.printSeparator('=');
  Serial.println("TESTING FLOATSHIELD COMPONENTS");
  AutomationShield.printSeparator('=');
  FloatShield.actuatorWrite(0.0); 
  
// Reference potentiometer test ----------------------------
#ifdef ARDUINO_ARCH_AVR
  analogReference(EXTERNAL); // Setting external reference for analog components
#endif
  Serial.print("Turn pot to 0%!  ");
  Serial.print("\t\t\t");                // print tab characters
  delay(5000);
  float potReferenceLow = FloatShield.referenceRead();
  if (potReferenceLow  >= 0.0 && potReferenceLow  <= 1.0) {
    Serial.println(" Ok.");
    testFail = 0;
  }
  else {
    Serial.println(" Fail.");
    testFail = 1;
  }
  
  Serial.print("Turn pot to 100%!   ");
  Serial.print("\t\t\t");                // print tab characters
  delay(1000);
  float potReferenceHi = FloatShield.referenceRead();
  if (potReferenceHi >= 99.0 && potReferenceHi <= 100.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

// Laser sensor test -------------------------------------

  VL53L0X distanceSensor; // Create Object
  Wire.begin();
  distanceSensor.setTimeout(1000);                                  // Set sensor timeout to 1 second
  Serial.print("VL53L0X initialization:   ");
  Serial.print("\t\t");        
  if (distanceSensor.init()) {
    Serial.println(" Ok.");
    testFail = testFail || 0;
  }
  else {
    Serial.println(" Fail.");
    testFail = testFail || 1;
  } 

 distanceSensor.setMeasurementTimingBudget(20000); 
  distanceSensor.startContinuous();                                 // Setting continuous mode for laser sensor
  
  float lasFar = (float)distanceSensor.readRangeContinuousMillimeters(); // Read distance
  AutomationShield.printTestResults("Testing VL53L0X sensor far...",lasFar, 370.0, 380.0);
  testFail = testFail || AutomationShield.printTestResults("Testing VL53L0X sensor far...",lasFar, 370.0, 380.0);

  FloatShield.actuatorWrite(100.0); // Turn on fan
  delay(5000); // Wait
  float lasNear = (float)distanceSensor.readRangeContinuousMillimeters(); // Read distance
  float actFull = FloatShield.actuatorReadVoltage();
  AutomationShield.printTestResults("Testing VL53L0X sensor near...",lasNear, 50.0, 70.0);
  testFail = testFail || AutomationShield.printTestResults("Testing VL53L0X sensor near...",lasNear, 50.0, 70.0);

  AutomationShield.printTestResults("Motor voltage at 100% input...",actFull, 8.7, 9.3);
  testFail = testFail || AutomationShield.printTestResults("Motor voltage at 100% input...",actFull, 8.7, 9.3);
  AutomationShield.printTestResults("Fan 100% power... \t",lasNear, 50.0, 70.0); // Motor at 100% must be also ok
  
  FloatShield.actuatorWrite(0.0);
  delay(3000);

  
  FloatShield.actuatorWrite(70.0);
   delay(3000); // Wait
  float motSeventy = (float)distanceSensor.readRangeContinuousMillimeters(); // Read distance
  float actPart = FloatShield.actuatorReadVoltage();
  AutomationShield.printTestResults("Motor voltage at 70% input...",actPart, 7.5, 7.9);
  testFail = testFail || AutomationShield.printTestResults("Motor voltage at 70% input...",actPart, 7.5, 7.9);
  AutomationShield.printTestResults("Fan 70% power...",motSeventy, 200.0, 300.0);
  testFail = testFail || AutomationShield.printTestResults("Fan 70% power...",motSeventy, 200.0, 300.0);
  FloatShield.actuatorWrite(0.0);
  
// PID??? -----------------------------------------

  Serial.print("Running PID routine...  ");
  Serial.print("\t\t\t");                // print tab characters

  
  FloatShield.begin();               // Initialise FloatShield board
  FloatShield.calibrate();           // Calibrate FloatShield board

  Sampling.period(Ts*1000);          // Set sampling period in microseconds

  PIDAbs.setKp(KP);                      // Set Proportional constant
  PIDAbs.setTi(TI);                      // Set Integral constant
  PIDAbs.setTd(TD);                      // Set Derivative constant
  PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID

    while(1) {                                  // Wait for ball to lift off from ground
        r = R[0];                               // Reference set to first point in trajectory
        y = FloatShield.sensorRead();           // Read sensor
        u = PIDAbs.compute(r-y,10,100,10,100);  // PID
        FloatShield.actuatorWrite(u);           // Actuate
        if(y >= r*2/3) {                        // If the ball is getting close to reference
            break;                              // Continue program
        }
        delay(Ts);                              // Wait before repeating the loop
    }
    Sampling.interrupt(stepEnable);             // Set interrupt function
}

void loop() {                       // Loop - runs indefinitely
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {                                    // ISR
    nextStep = true;                                   // Enable step flag
}

void step() {                               // Define step function
    
    if(i>(sizeof(R)/sizeof(R[0]))) {        // If at end of trajectory
        FloatShield.actuatorWrite(0.0);     // Turn off the fan
        if (e <= 600000.0) {
          Serial.println(" Ok.");
          testFail = testFail || 0;
        }
        else {
          Serial.println(" Fail.");
          testFail = testFail || 1;
        } 

  Serial.println("");
if (testFail){
  Serial.println("SOME TESTS FAILED!");
}
else {
  Serial.println("ALL TESTS PASSED.");
}

        while(1);                           // Stop program execution
    } else if (k % (T*i) == 0) {            // If at the end of section
        r = R[i];                           // Progress in trajectory
        i++;                                // Increment section counter
    }
    y = FloatShield.sensorRead();         // Read sensor
    e = e + (r-y)*(r-y);                  // Squared error.
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID
    FloatShield.actuatorWrite(u);         // Actuate
    uv = FloatShield.actuatorReadVoltage(); // Read actuator voltage
    k++;                       // Increment index
}





//// Summary --------------------------------------
//Serial.println(" ");
//AutomationShield.printSeparator('=');
//Serial.println("Details:");
//AutomationShield.printLowHighFirst();
//AutomationShield.printLowHigh("Potentiometer",potReferenceLow,potReferenceHi,"%",0);
//AutomationShield.printLowHigh("Hall sensor",hallLow,hallHi,"10-bit levels",0);
//AutomationShield.printLowHigh("Hall sensor",MagnetoShield.adcToGauss(hallHi),MagnetoShield.adcToGauss(hallLow),"G",0);
//AutomationShield.printLowHigh("Distance",MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(hallHi)),MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(hallLow)),"mm",1);
//AutomationShield.printLowHigh("Coil voltage",coilVoltageLow,coilVoltageHi,"V",1);
//AutomationShield.printLowHigh("Coil current",coilCurrentLow,coilCurrentHi,"mA",1);
