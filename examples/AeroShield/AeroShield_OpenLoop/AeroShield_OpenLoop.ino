/*
  AeroShield Open Loop

  Performs continuous sensor readings with manual motor speed control.

  The example initializes the AeroShield and performs
  calibration. Then it reads the current position
  of the potentiometer and sets the motor speed accordingly.

  The current position of the potentiometer and current
  angle of the pendulum are periodically (with specified
  sampling period) sent out through the serial communication,
  and can be displayed using Serial Plotter tool of Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Peter Tibenský.
  Last update: 16.1.2023.
*/

#include <AeroShield.h>           //  Include main library

float pendulumAngle;          //  Variable used for storing the actual angle of the pendulum
float reference;       //  Variable for potentiometer value in %
float current;

void setup() {                // Setup - runs only once
  Serial.begin(115200);       // Begin serial communication
  AeroShield.begin();         // Initialise AeroShield board
  AeroShield.calibrate();   //  Calibrate AeroShield board + store the 0° value of the pendulum
}

void loop() {
  if (pendulumAngle > 190) {    // If pendulum agle too big
    AeroShield.actuatorWrite(0);  // Turn off motor
    while (1);                    // Stop program
  }

  pendulumAngle = AeroShield.sensorReadDegree();   //  mapping the pendulum angle
  reference = AeroShield.referenceRead();   //  Function for mapping the potentiometer input
  AeroShield.actuatorWrite(reference);     //  Actuate
  current = AeroShield.sensorReadCurrent();       //  Read current drawn by motor

  Serial.print(pendulumAngle);      //  Printing the mapped angle value
  Serial.print(" ");
  Serial.print(reference);    // Printing potentiometer value in %
  Serial.print(" ");
  Serial.println(current);    // Printing current value in A

}