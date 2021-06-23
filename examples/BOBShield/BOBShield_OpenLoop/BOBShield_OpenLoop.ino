/*
  BoBShield open-loop example

  Manual open-loop control.
  
  This example initializes the sampling subsystem 
  from the AutomationShield library and starts a 
  loop where you can manualy control the angle 
  of the tube by turning on board potentiometer. 
  
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  tube angle by turning the potentiometer.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Erik Mikuláš. 
  Last update: 23.6.2021.
*/

#include <BOBShield.h>
#include <SamplingServo.h>

unsigned long Ts = 10;            // Sampling in milliseconds
unsigned long k = 0;              // Sample index
bool enable=false;                // Flag for sampling 


float y = 0.0;            // Output
float u = 0.0;            // Input          


void setup() {
  Serial.begin(115200);               // Initialize serial
  
  // Initialize and calibrate board
  BOBShield.begin();               // Define hardware pins
  BOBShield.initialize();
  BOBShield.calibration();
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {               // If ISR enables
    step();                 // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}


void step(){               // A single algorithm step

    y = BOBShield.sensorRead();              // Read sensor 
    u = AutomationShield.mapFloat(BOBShield.referenceRead(),0,100,-30,30);
    BOBShield.actuatorWrite(u);              // Actuate

    Serial.print(y);            // Print output  
    Serial.print(", ");
    Serial.println(u);            // Print input
}
