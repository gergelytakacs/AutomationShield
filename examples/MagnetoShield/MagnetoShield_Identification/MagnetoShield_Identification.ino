/*
  MagnetoShield closed-loop identification experiment

  Runs a closed-loop experiment to gather data for system
  identification.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and starts a 
  predetermined reference trajectory. Noise is injected to 
  this input trajectory to create a rich signal. Upload the
  code to your board, then use a serial terminal software 
  or Matlab to aquire the dataset for later processing. 

  Tested on: Arduino Uno, Arduino Due
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 11.02.2018.
*/

#include <MagnetoShield.h>                // Include header for hardware API

// Experiment settings
float R[]={15.0,16.0,15.5,14.5,16.0};    // Reference trajectory (pre-set)
float wPP=1.5;                           // [V] Peak-to-peak noise level

// Flags and parameters

// PID Tuning parameters
// Less-aggressive tuning introduces less saturation
#define KP 2.1                            // PID Kp
#define TI 0.1                            // PID Ti
#define TD 0.02                           // PID Td
  
#ifdef ARDUINO_ARCH_AVR
  unsigned long Ts = 4000;                // Sampling in microseconds
  int T = 1500;                           // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAMD
  unsigned long Ts = 5000;                 // Sampling in microseconds
  int T = 1500;                            // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAM
  unsigned long Ts = 1500;                 // Sampling in microseconds
  int T = 3000;                            // Experiment section length (steps) 
#endif  

unsigned long k = 0;                      // Sample index
bool enable=false;                        // Flag for sampling 
bool realTimeViolation=false;             // Flag for real-time violations
float r = 0.0;                            // Reference
int   i = 0;                              // Experiment section counter
float y = 0.0;                            // [mm] Output
float u = 0.0;                            // [V] Input          
float I = 0.0;                            // [mA] Current
float w = 0.0;                            // [V] Noise
float wBias=wPP/2.0;                      // [V] Noise bias
int   wP=(int)wPP*100;                    // For (pseudo)-random generator

void setup() {
 
#ifdef ARDUINO_ARCH_SAM
    Serial.begin(250000);                  // Initialize serial, maximum for Due
#elif ARDUINO_ARCH_AVR
    Serial.begin(2000000);                 // Initialize serial, maximum for AVR given by hardware
#elif ARDUINO_ARCH_SAMD
    Serial.begin(2000000);                  // Initialize serial
#endif 

  // Initialize and calibrate board
  MagnetoShield.begin();                  // Define hardware pins
  MagnetoShield.calibration();            // Calibrate for distance
  
  // Initialize sampling function
  Sampling.period(Ts);       // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

  // Set the PID constants
  PIDAbs.setKp(KP);                       // Proportional gain
  PIDAbs.setTi(TI);                       // Integration time constant
  PIDAbs.setTd(TD);                       // Derivative time constant
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {                           // If ISR enables
    step();                               // Algorithm step
    enable=false;                         // Then disable until next interrupt
  }  
}

void stepEnable(){                        // This is the ISR 
  if(enable){                             // If step still running
  realTimeViolation=true;                 // RT has been violated
  Serial.print("Real time sampling violated.");
  while(1);                               // Stop
  }                                       // Else 
  enable=true;                            // change flag and run step
}

// A single algoritm step
void step(){                             

// Experiment control
if (i>sizeof(R)/sizeof(R[0])){            // If finished
  MagnetoShield.actuatorWrite(0);         // Turn off magnet
  while(1);                               // and do nothing
}                                        
else if (k % (T*i) == 0){                 // else for each section
  r = R[i];                               // set reference
  i++;                                    // increment section counter
}

w=wBias-(float)random(0,wP)/100.0;        // [V] Input noise
y = MagnetoShield.sensorRead();           // [mm] Sensor Read 
I = MagnetoShield.auxReadCurrent();       // [mA] Current read
u = PIDAbs.compute(r-y,0,12,0,20)+w;      // [V] PID + noise
u = AutomationShield.constrainFloat(u,0,12);
MagnetoShield.actuatorWrite(u);           // Actuate

// For data aquisition
Serial.print(y);                          // Print input only
Serial.print(", ");
Serial.print(u);                          // Print output
Serial.print(", ");
Serial.println(I);                        // Print output
k++;                                      // Increment k
}