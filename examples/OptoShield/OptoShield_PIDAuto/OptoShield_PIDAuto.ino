/*
  OptoShield manual PID response example

  PID feedback control of brightness on the OptoShield.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and starts a 
  predetermined reference trajectory for LED brightness. 
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Tibor Konkoly. 
  Last update: 28.05.2019.
*/

#include <OptoShield.h>     	 // Include hardware API
#include <Sampling.h>            // Include sampling

unsigned long Ts = 5;          // Sampling in milliseconds
unsigned long k = 0;           // Sample index
bool enable=false;             // Flag for sampling 

float r = 0.0;						    // Reference
float R[]={50.0,75.0,25.0,50.0,85.0,45.0};    // Reference trajectory
int T = 350;      			  		// Section length
int i = i;					    	  	// Section counter
float y = 0.0;						    // Output
float u = 0.0;						    // Input					

#define KP 0.06						    // PID Kp
#define TI 0.01               // PID Ti
#define TD 0.00001           // PID Td

void setup() {
  Serial.begin(250000);       // Initialize serial
  
  // Initialize and calibrate board
  OptoShield.begin();               // Define hardware pins
  OptoShield.calibration();         // Calibration percentages
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

 // Set the PID constants
 PIDAbs.setKp(KP); // Proportional
 PIDAbs.setTi(TI); // Integral
 PIDAbs.setTd(TD); // Derivative
 PIDAbs.setTs(Sampling.samplingPeriod); // Sampling
 
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) { 							// If ISR enables
    step();									// Algorithm step
    enable=false;  							// Then disable
  }  
}

void stepEnable(){  						// ISR 
  enable=true;							    // Change flag
}

// A signle algoritm step

void step(){ 

if (k % (T*i) == 0){				
  r = R[i]; 							 // Set reference
  i++;
}
  							  
y = OptoShield.sensorRead();           // Read sensor 
u = PIDAbs.compute(r-y,0,100,0,100);   // PID
OptoShield.actuatorWrite(u);           // Actuate

Serial.print(r);						// Print reference
Serial.print(", ");						 
Serial.print(y);						// Print output  
Serial.print(", ");
Serial.println(u);						// Print input
k++;									// Increment k

}
