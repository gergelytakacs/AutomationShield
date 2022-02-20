/*
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by 
  1ast update: 07.02.2022.
*/

#include "AeroShield.h"               // Include main library  
#include <Sampling.h>                 // Include sampling library

#define MANUAL 0                     // Choose manual reference using potentiometer (2)  automatic reference trajectory (1) or sinusoid (0)

AeroShield aeroshield;               //  AS5600 addresses initialization

unsigned long Ts = 3;                 // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

float startangle=0;           //  Variable for storing 0° angle in raw format
float lastangle=0;            //  Variable needed for mapping of the pendulum's angle
float pendulumAngle;          //  Variable used for storing the actual angle of the pendulum in degrees
float pendulumAnglePercent;   //  Variable used for storing the actual angle of the pendulum in %

int i=i;                      // Section counter
int T=1000;                    // Section length in ms
float R[]={45.0,23.0,75.0,32.0,58.0,10.0,35.0,19.0,9.0,43.0,23.0,65.0,15.0,80.0};   // Reference trajectory
//float R[]={5.0,3.0,7.0,12.0,8.0,10.0,15.0,19.0,9.0,};
float r=0.0;                  // Reference (Wanted pendulum angle)
float y = 0.0;                // Output (Current pendulum angle)
float u = 0.0;                // Input (motor power)

  #define KP 1.7           // PID Kp constant
  #define TI 3.8           // PID Ti constant
  #define TD 0.25            // PID Td constant

void setup() {                                                  //  Setup - runs only once
  Serial.begin(250000);                                         //  Begin serial communication
  pinMode(5,OUTPUT);                                            //  Initialise pin 5 as output for the motor 
  aeroshield.begin(aeroshield.detectMagnet());                  //  Initialise AeroShield board
  startangle = aeroshield.calibration(aeroshield.getRawAngle());   //  Calibrate AeroShield board + store the 0° value of the pendulum
  lastangle=startangle+1024;                                    //  Callculation of second angle needed for map function
    Sampling.period(Ts*1000);              // Set sampling period in microseconds
    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                      // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID
    
#if MANUAL                                      // If Manual mode is active
        r = aeroshield.referenceRead();         // Read reference from potentiometer
#else                                           // If Automatic mode is active
        r = R[0];                               // Reference set to first point in trajectory
        
#endif
        pendulumAngle= AutomationShield.mapFloat(aeroshield.getRawAngle(),startangle,lastangle,0.00,90.00);   //  mapping the pendulum angle 
        y= AutomationShield.mapFloat(pendulumAngle,0.00,61.0,0.00,100.00);      //  mapping the pendulum angle to % value  
        u = PIDAbs.compute(r-y,0,100,0,100);    // PID
        
        aeroshield.actuatorWrite(u);            // Actuate

        delay(Ts);                              // Wait before repeating the loop
    
    Sampling.interrupt(stepEnable);             // Set interrupt function
}

void loop() {

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
    r = aeroshield.referenceRead();        // Read reference from potentiometer
#else                                      // If Automatic mode is active
    if(i>(sizeof(R)/sizeof(R[0]))) {       // If at end of trajectory
        analogWrite(5,0);                  // Turn off the motor
        while(1);                          // Stop program execution
    } else if (k % (T*i) == 0) {           // If at the end of section
        r = R[i];                          // Progress in trajectory
        i++;                               // Increment section counter
    }
#endif
    pendulumAngle= AutomationShield.mapFloat(aeroshield.getRawAngle(),startangle,lastangle,0.00,90.00); //  mapping the pendulum angle
    y= AutomationShield.mapFloat(pendulumAngle,0.00,61.0,0.00,100.00);                  //  mapping the pendulum angle into % value
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID
    aeroshield.actuatorWrite(u);          // Actuate


    Serial.print(r);           // Print reference
    Serial.print(", ");
    Serial.print(y);           // Print output
    Serial.print(", ");
    Serial.println(u);         // Print input

    k++;                       // Increment index
}
