/*
  MagnetoShield hardware self test routine 

  Upload the code to your board, then open the Serial
  Monitor function in your Arduino IDE. Be prepared to turn the
  shaft of the potentiometer.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 24.8.2020.
*/
#include <AutomationShield.h>  
#include <MagnetoShield.h>     // Include header for hardware API

bool testFail = 0; 

void setup() {
  Serial.begin(9600);       // Starts serial communication
  Serial.println();
  Serial.println();
  AutomationShield.printSeparator("=");
  Serial.println("TESTING MAGNETOSHIELD COMPONENTS");
  AutomationShield.printSeparator("=");
  MagnetoShield.begin();       // Initializes shield
  MagnetoShield.dacWrite(0);   // Turns off magnet

  
// Reference potentiometer test ----------------------------

  Serial.print("Turn pot to 0%!  ");
  Serial.print("\t\t\t");                // print tab characters
  delay(5000);
  float potReferenceLow = MagnetoShield.referenceRead();
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
  float potReferenceHi = MagnetoShield.referenceRead();
  if (potReferenceHi >= 99.0 && potReferenceHi <= 100.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

// Hall sensor test --------------------------------------

  MagnetoShield.dacWrite(0);
  float hallLow = (float)analogRead(MAGNETO_YPIN);
  AutomationShield.printTestResults("Testing Hall sensor low...",hallLow, 25.0, 45.0);
  testFail = testFail || AutomationShield.printTestResults("Testing Hall sensor low...",hallLow, 25.0, 45.0);

  MagnetoShield.dacWrite(DACMAX);
  delay(1000);
  float hallHi  = (float)analogRead(MAGNETO_YPIN);
  AutomationShield.printTestResults("Testing Hall sensor high...",hallHi, 550.0, 650.0);
  testFail = testFail || AutomationShield.printTestResults("Testing Hall sensor high...",hallHi, 550.0, 650.0);

// Voltage supply test --------------------------------------
#if SHIELDRELEASE==1 || SHIELDRELEASE==2

#else
  MagnetoShield.dacWrite(0);
  float coilVoltageLow = MagnetoShield.auxReadVoltage();
  AutomationShield.printTestResults("Testing coil voltage (off)...",coilVoltageLow , 0.0, 0.5);
  testFail = testFail || AutomationShield.printTestResults("Testing coil voltage (off)...",coilVoltageLow , 0.0, 0.5);

  MagnetoShield.dacWrite(DACMAX);
  float coilVoltageHi = MagnetoShield.auxReadVoltage();
  AutomationShield.printTestResults("Testing coil voltage (on)...",coilVoltageHi , 9.5, 10.5);
  testFail = testFail || AutomationShield.printTestResults("Testing coil voltage (on)...",coilVoltageHi , 9.0, 11.0);
#endif


// Current sensor test --------------------------------------


  MagnetoShield.dacWrite(0);
  float coilCurrentLow = MagnetoShield.auxReadCurrent();
  AutomationShield.printTestResults("Testing current sensor (off)...",coilCurrentLow , 0.0, 1.0);
  testFail = testFail || AutomationShield.printTestResults("Testing current sensor (off)...",coilCurrentLow , 0.0, 1.0);

  MagnetoShield.dacWrite(DACMAX);
  delay(1000);
  float coilCurrentHi = MagnetoShield.auxReadCurrent();
  MagnetoShield.dacWrite(0);
  AutomationShield.printTestResults("Testing current sensor (on)...",coilCurrentHi , 49.0, 60.0);
  testFail = testFail || AutomationShield.printTestResults("Testing current sensor (on)...",coilCurrentHi , 49.0, 60.0);

Serial.println("");
if (testFail){
  Serial.println("SOME TESTS FAILED!");
}
else {
  Serial.println("ALL TESTS PASSED.");
}

// Summary --------------------------------------
Serial.println(" ");
AutomationShield.printSeparator("=");
Serial.println("Details:");
AutomationShield.printLowHighFirst();
AutomationShield.printLowHigh("Potentiometer",potReferenceLow,potReferenceHi,"%",0);
AutomationShield.printLowHigh("Hall sensor",hallLow,hallHi,"10-bit levels",0);
AutomationShield.printLowHigh("Hall sensor",MagnetoShield.adcToGauss(hallHi),MagnetoShield.adcToGauss(hallLow),"G",0);
AutomationShield.printLowHigh("Distance",MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(hallHi)),MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(hallLow)),"mm",1);
AutomationShield.printLowHigh("Coil voltage",coilVoltageLow,coilVoltageHi,"V",1);
AutomationShield.printLowHigh("Coil current",coilCurrentLow,coilCurrentHi,"mA",1);
}


  void loop() {
  }
