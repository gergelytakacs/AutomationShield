/*
  LinkShield identification example.

  Example used to acquire data for LinkShield system identification.

  The LinkShield implements an a flexible rotational link experiment
  on an Arduino shield. This example initialises the sampling
  subsystem from the AutomationShield library and allows user

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Vríčan.
  Last update: 3.12.2022.
*/

#include <LinkShield.h>            	// Include header for hardware API


unsigned long Ts = 5;               // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool nextStep = false;              // Flag for sampling
bool realTimeViolation = false;     // Flag for real-time sampling violation
bool endExperiment = false;         // Boolean flag to end the experiment

float y = 0.0;
float y1 = 0.0;                     // Output variable
float y2 = 0.0;                     // Output variable
float r = 0.0;                        	// Input (open-loop), initialized to zero
float u = 0.0;
float R[]={0.00, PI/4, -PI/4, 0.00};
int T = 1000;                         	// Section length (appr. '/.+2 s)
unsigned int i = 0;                            // Section counter

// PID Tuning
#define KP 80                        // PID Kp
#define KI 20.5
#define KD 0.0001

#define TI 0.1                        // PID Ti
#define TD 0.15                     // PID Td


void setup() {

  Serial.begin(2000000);               // Initialize serial

  // Initialize linkshield hardware
  LinkShield.begin();                  // Define hardware pins
  //LinkShield.calibrate();              // Remove sensor bias

  // Initialize sampling function
  Sampling.period(Ts * 1000);          // Sampling init.
  Sampling.interrupt(stepEnable);      // Interrupt fcn.

  // Set the PID constants
  PIDAbs.setKp(KP); // Proportional
  PIDAbs.setKi(KI); // Integral
  PIDAbs.setKd(KD); // Derivative
  
  //PIDAbs.setTi(TI); // Integral
  //PIDAbs.setTd(TD); // Derivative
  PIDAbs.setTs(Sampling.samplingPeriod); // Sampling
}

// Main loop launches a single step at each enable time
void loop() {
  if (nextStep) {                     // If ISR enables
    step();                           // Algorithm step
    nextStep = false;                 // Then disable
  }
}

void stepEnable() {                                    // ISR
  if (endExperiment == true) {                         // If the experiment is over
  LinkShield.actuatorWriteVoltage(0.00);
    while (1);                                         // Do nothing
  }
  if (nextStep == true) {                              // If previous sample still running
    realTimeViolation = true;                      // Real-time has been violated
    Serial.println("Real-time samples violated."); // Print error message
    while (1);                                     // Stop program execution
  }
  nextStep = true;                                     // Change flag
}

// A single algorithm step
void step() {
	
	// Switching between experiment sections
	
  if (i > (sizeof(R) / sizeof(R[0]))) { // If at end of trajectory
		endExperiment = true;         // Stop program execution at next ISR
	
	} 
	else if (k % (T * i) == 0) {    // If at the end of section
    r = R[i];                     // Progress in trajectory
    i++;                          // Increment section counter
  }

	y1 = LinkShield.encoderRead();
	//y2 = LinkShield.flexRead();          // Read sensor
  
  y  = y1 + y2;
  u = PIDAbs.compute((r - y), -5, 5, -100, 100); // Compute constrained absolute-form PID
  
  	if(y>HALF_PI){u = AutomationShield.constrainFloat(u, -5.0,0.0);}
	if(y<-HALF_PI){u = AutomationShield.constrainFloat(u,0.0,5.0);}
	u = AutomationShield.constrainFloat(u,-5.0,5.0);
  
  LinkShield.actuatorWriteVoltage(u);         // [V] actuate

  // Print to serial port
  Serial.print(r);                        // Print reference
  Serial.print(", ");
  Serial.print(y);                        // Print output
  Serial.print(", ");
  Serial.println(u);                      // Print input
  k++;                                    // Increment time-step k
}
