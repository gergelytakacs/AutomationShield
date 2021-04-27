/*
  BoBShield calibration experiment

  This example initializes the PID control
  subsystems from the AutomationShield library, which
  is included in BOBShield library and starts a
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
#include <BOBShield.h>                // Includes all the necessary libraries nad hardware components
#include <SamplingServo.h>            // Include special Sampling running on timmer 2

  void wait() {
  delay(5000);
}

Adafruit_VL6180X vl = Adafruit_VL6180X();
void setup() {

 Serial.begin(115200);               // Initialize serial
 BOBShield.begin();
  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  // finds out if sensor is active
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

  
  Serial.print("Turn servo to -30 degrees!  ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  BOBShield.actuatorWrite(-30);         // Tilt beam to the minimum so the ball falls towards the sensor
  wait();                // Wait for things to settle
  int y = vl.readRange();
  if (y >= 80 && y <= 100) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }

  Serial.print("Turn servo to +30 degrees!  ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  BOBShield.actuatorWrite(30);         // Tilt beam to the minimum so the ball falls towards the sensor
  wait();                // Wait for things to settle
  int a = vl.readRange();
  if (a >= -5 && a <= 10) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }
/*
  // potentiometer test
  Serial.print("Turn pot to 0%!  ");
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  Serial.print("\t");                // print a tab character
  wait();
  float z = BOBShield.referenceRead();
  if (z >= 0.0 && z <= 1.0) {
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
float  x = BOBShield.referenceRead();
  if (x >= 99.0 && x <= 100.0) {
    Serial.println(" Ok.");
  }
  else {
    Serial.println(" Fail.");
  }
*/
}

void loop() {

}
