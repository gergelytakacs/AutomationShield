/*
  MagnetoShield calibration experiment

  This example initializes the sampling and PID control
  subsystems from the AutomationShield library and starts a
  predetermined reference trajectory for the heating block
  temperature.

  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 19.11.2018.
*/
#include <MagnetoShield.h>     // Include header for hardware API

void wait() {
  delay(5000);
};

void setup() {
  Serial.begin(2000000);       // Starts serial communication
  Serial.println("Testing MagnetoShield components");
  MagnetoShield.begin();       // Initializes shield
  MagnetoShield.dacWrite(0);   // Turns off magnet
// Reference potentiometer test ----------------------------

  Serial.print("Turn pot to 0%!  ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  wait();
  float x = MagnetoShield.referenceRead();
  if (x >= 0.0 && x <= 1.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

  Serial.print("Turn pot to 100%!   ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  wait();
  x = MagnetoShield.referenceRead();
  if (x >= 99.0 && x <= 100.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

// Hall sensor test --------------------------------------

  Serial.print("Testing Hall sensor low... ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  MagnetoShield.dacWrite(0);
  x = (float)analogRead(MAGNETO_YPIN);
  if (x >= 25.0 && x <= 40.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

  Serial.print("Testing Hall sensor high... ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  MagnetoShield.dacWrite(255);
  wait();
  x = (float)analogRead(MAGNETO_YPIN);
  MagnetoShield.dacWrite(0);
  if (x >= 550 && x <= 650.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

// Voltage supply test --------------------------------------

  Serial.print("Testing voltage supply... ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  MagnetoShield.dacWrite(0);
  x = MagnetoShield.auxReadVoltage();
  if (x >= 10.0 && x <= 11.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }


// Current sensor test --------------------------------------

  Serial.print("Testing current sensor (no current)... ");
  Serial.print("\t");                // print a tab character
  MagnetoShield.dacWrite(0);
  x = MagnetoShield.auxReadCurrent();
  if (x >= 0.0 && x <= 5.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

  Serial.print("Testing current sensor (full current)...");
  MagnetoShield.dacWrite(255);
  wait();
  x = MagnetoShield.auxReadCurrent();
  MagnetoShield.dacWrite(0);
  if (x >= 50.0 && x <= 65.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

}


  void loop() {
  }