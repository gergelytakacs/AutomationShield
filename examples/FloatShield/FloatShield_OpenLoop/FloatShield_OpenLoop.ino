/*
  FloatShield Open Loop

  Performs continuous sensor readings with manual fan speed control.

  The example initializes the FloatShield and performs
  calibration. Then it reads the current position
  of the potentiometer and sets the fan speed accordingly.
  The current position of the potentiometer and current
  position of the ball is periodically (with specified
  sampling period) sent out through the serial communication,
  and can be displayed using Serial Plotter tool of Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 2.7.2019.
*/

#include <FloatShield.h>              // Including the FloatShield library
#include <Sampling.h>                 // Include sampling library

unsigned long Ts = 25;                // Sampling period in miliseconds
bool nextStep = false;                // Flag for step function

void setup() {                        // Setup - runs only once
    Serial.begin(250000);             // Begin serial comunication
    
    FloatShield.begin();              // Initialise FloatShield
    FloatShield.calibrate();          // Calibrate FloatShield
    
    Sampling.period(Ts*1000);         // Set sampling period in microseconds
    Sampling.interrupt(stepEnable);   // Set interrupt function
}

void loop() {                    // Loop - runs indefinitely
    if (nextStep) {              // If ISR enables step flag
        step();                  // Run step function
        nextStep = false;        // Disable step flag
    }
}

void stepEnable() {              // ISR   
    nextStep = true;             // Enable step flag
}

void step() {                                                       // Define step function
    float potentiometerReference = FloatShield.referenceRead();     // Save current percentual position of potentiometer runner to "potentiometerReference" variable
    float ballPositionInTube = FloatShield.sensorRead();            // Save current percentual position of ball to "ballPositionInTube" variable
    FloatShield.actuatorWrite(potentiometerReference);              // Set the power output of the fan based on the current potentiometer position  
                              
    Serial.print(potentiometerReference);                           // Print out to the Serial Plotter current potentiometer percentual position
    Serial.print(" ");                                              // Print out whitespace character to differentiate data for Serial Plotter
    Serial.println(ballPositionInTube);                             // Print out to the Serial Plotter current percentual position of the ball
}
