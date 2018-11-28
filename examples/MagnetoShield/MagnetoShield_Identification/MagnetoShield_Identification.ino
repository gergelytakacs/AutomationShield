/*
  MagnetoShield closed-loop PID response example

  PID feedback control for magnet levitation.
  
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
  Last update: 28.11.2018.
*/

#include <MagnetoShield.h>     // Include header for hardware API

#define KP 2.2            // PID Kp
#define TI 0.1            // PID Ti
#define TD 0.04         // PID Td

//#define KP 2.2            // PID Kp
//#define TI 0.1            // PID Ti
//#define TD 0.03         // PID Td

unsigned long Ts = 4000;               // Sampling in microseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 
bool realTimeViolation=false;       // Flag for real-time violations

float r = 0.0;            // Reference
float R[]={15.0,14.5,15.0,15.5,15.0};    // Reference trajectory
int T = 1000;           // Section length (steps) 
int i = i;              // Section counter
float y = 0.0;            // Output
float u = 0.0;            // Input          

void setup() {
  Serial.begin(2000000);               // Initialize serial
  // Initialize and calibrate board
  MagnetoShield.begin();               // Define hardware pins
  
  // Initialize sampling function
  Sampling.interruptInitialize(Ts);   // Sampling init.
  Sampling.setInterruptCallback(stepEnable); // Interrupt fcn.

 // Set the PID constants
 PIDAbs.setKp(KP);
 PIDAbs.setTi(TI);
 PIDAbs.setTd(TD); 
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {               // If ISR enables
    step();                 // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  if(enable){
  realTimeViolation=true;
  Serial.println("Real-time samples violated.");
  while(1);
  }
  enable=true;                  // Change flag
}

// A signle algoritm step

void step(){ 

if (i>sizeof(R)/sizeof(R[0])){
  MagnetoShield.actuatorWrite(0);
  while(1);
}
else if (k % (T*i) == 0){        
  r = R[i];                // Set reference
  i++;
}
float w=(float)random(0,10)/10.0;         // [V] Input noise
y = MagnetoShield.sensorRead();           // [mm] Sensor Read 
u = PIDAbs.compute(y-r,0,12,0,20);        // [V] PID
MagnetoShield.actuatorWrite(u+w);         // Actuate

// For PID tuning
//Serial.print(r);            // Print reference
//Serial.print(", ");            
//Serial.println(y);            // Print output  

// For measurement
Serial.print(u);            // Print input only
Serial.print(", ");
Serial.print(w);            // Print noise only
Serial.print(", ");
Serial.println(y);            // Print output
k++;                  // Increment k
}