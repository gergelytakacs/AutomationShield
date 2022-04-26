/*
  AeroShield closed-loop PID response example
  
  PID feedback control of pendulum angle of the AeroShield.
  
  This example initialises the sampling and PID control
  subsystems from the AutomationShield library and allows user
  to select whether the reference is given by the potentiometer
  or by a predetermined reference trajectory. Upload the code to 
  your board and open the Serial Plotter tool in Arduino IDE.
  
  Tested on Arduino UNO Rev3 and Mega 2560 Rev3. 
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  
  Created by Peter Tibenský.
  Last update: 24.4.2022.
*/

#include "AeroShield.h"               // Include main library  
#include <Sampling.h>                 // Include sampling library

#define MANUAL 1                      // Choose manual reference using potentiometer (1)  automatic reference trajectory (0) 
#define KP 2.2                        // PID Kp constant
#define TI 3.0                        // PID Ti constant
#define TD 0.25                       // PID Td constant

float startAngle=0;           //  Variable for storing 0° angle in raw format
float lastAngle=0;            //  Variable needed for mapping of the pendulum's angle
float pendulumAngle;          //  Variable used for storing the actual angle of the pendulum in degrees

unsigned long Ts = 3;                 // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

int i=i;                      // Section counter
int T=1000;                    // Section length in ms
float R[]={55.0,33.0,85.0,42.0,68.0,20.0,45.0,29.0,19.0,53.0,33.0,75.0,25.0,90.0};   // Reference trajectory
float r=0.0;                  // Reference (Wanted pendulum angle)
float y = 0.0;                // Output (Current pendulum angle)
float u = 0.0;                // Input (motor power)

void setup() {                                                  //  Setup - runs only once
  Serial.begin(250000);                                         //  Begin serial communication
  AeroShield.begin();                  //  Initialise AeroShield board
  startAngle = AeroShield.calibration(AeroShield.getRawAngle());   //  Calibrate AeroShield board + store the 0° value of the pendulum
  lastAngle=startAngle+1024;                                    //  Callculation of second angle needed for map function
  Sampling.period(Ts*1000);              // Set sampling period in milliseconds
  PIDAbs.setKp(KP);                      // Set Proportional constant
  PIDAbs.setTi(TI);                      // Set Integral constant
  PIDAbs.setTd(TD);                      // Set Derivative constant
  PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID
  Sampling.interrupt(stepEnable);        // Set interrupt function
}

void loop() {
      if(pendulumAngle>120){
      AeroShield.actuatorWrite(0);
      while(1);
      } 
      if (nextStep) {               // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {                                    // ISR
    if(nextStep == true) {                             // If previous sample still running
        realTimeViolation = true;                      // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
analogWrite(5,0);                                      // Turn off the motor
        while(1);                                      // Stop program execution
    }
    nextStep = true;                                   // Enable step flag
}

void step() {                              // Define step function

#if MANUAL                                 // If Manual mode is active
    r = AeroShield.referenceRead();        // Read reference from potentiometer
#else                                      // If Automatic mode is active
    if(i>(sizeof(R)/sizeof(R[0]))) {       // If at end of trajectory
        analogWrite(5,0);                  // Turn off the motor
        while(1);                          // Stop program execution
    } else if (k % (T*i) == 0) {           // If at the end of section
        r = R[i];                          // Progress in trajectory
        i++;                               // Increment section counter
    }
#endif
    y= AutomationShield.mapFloat(AeroShield.getRawAngle(),startAngle,lastAngle,0.00,100.00);                  //  mapping the pendulum angle into % value
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID
    AeroShield.actuatorWrite(u);          // Actuate

    Serial.print(r);           // Print reference
    Serial.print(", ");
    Serial.print(y);           // Print output
    Serial.print(", ");
    Serial.println(u);         // Print input

    k++;                       // Increment index
}
