/*
  PID feedback control for magnet levitation.
  
  This example provides PID levitation experiment.
  Based on the data from this experiment has been
  comparison of two existing models for MagnetoShield
  performed.
  Experiment is designed for use with an Arduino DUE.

  Created by Jakub Mihalik.
  Last update: 29.4.2021
*/
#include <MagnetoShield.h>            // Include header for hardware API
#include <Sampling.h>            // Include sampling

// PID Tuning
#define KP 3.5                        // PID Kp
#define TI 0.6                        // PID Ti
#define TD 0.025                      // PID Td

float R[]={15.5,14.5,13.0,15.0,16.0,14.0}; // Reference trajectory (pre-set)
  
unsigned long k = 0;                  // Sample index
bool enable=false;                    // Flag for sampling 
bool realTimeViolation=false;         // Flag for real-time violations
float r = 0.0;                        // Reference

int j = 0;
float i = 0.0;                          // [mA] current
float y = 0.0;                        // [mm] Output
float u = 0.0;                        // [V] Input          

  unsigned long Ts = 1500;                 // Sampling in microseconds, lower limit 1.3 ms
  int T = 3000;                            // Experiment section length (steps) 

void setup() { 
  Serial.begin(250000);                  // Initialize serial, maximum for Due (baud mismatch issues)

  
  // Initialize and calibrate board
  MagnetoShield.begin();               // Define hardware pins
  MagnetoShield.calibration();         // Calibrates shield 
     
  // Initialize sampling function
  Sampling.period(Ts);    // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

 // Set the PID constants
 PIDAbs.setKp(KP); // Proportional
 PIDAbs.setTi(TI); // Integral
 PIDAbs.setTd(TD); // Derivative
 PIDAbs.setTs(Sampling.samplingPeriod); // Sampling
}

// Main loop launches a single step at each enable time
void loop(){
  if (enable){                         // If ISR enables
    step();                             // Algorithm step
    enable=false;                       // Then disable
  }  
}

void stepEnable(){                      // ISR 
  if(enable){                           // If previous still running
    realTimeViolation=true;             // RT violated
    Serial.println("Real-time samples violated.");
    while(1);                           // Stop execution
  }                                     // else
  enable=true;                          // Change flag
}

// A single algorithm step
void step(){ 

// Reference source
  if (j>sizeof(R)/sizeof(R[0])){        // If experiment finished
    realTimeViolation=false;            // Not a real-time violation
    MagnetoShield.actuatorWrite(0);     // then turn off magnet
    while(1);                           // and stop
  }
  else if (k % (T*j) == 0){             // else for each section
    r = R[j];                           // set reference
    j++;                                // and increase section counter
  }


// Control algorithm
y = MagnetoShield.sensorRead();         // [mm] sensor read
i = MagnetoShield.auxReadCurrent();
u = PIDAbs.compute(-(r-y),0,MagnetoShield.getVoltageRef(),-10,10);      // Compute constrained absolute-form PID
MagnetoShield.actuatorWrite(u);         // [V] actuate

// Print to serial port
Serial.print(r);                        // [mm] Print reference
Serial.print(", ");            
Serial.print(y);                        // [mm ]Print output  
Serial.print(", ");
Serial.print(i);                        // [mm ]Print output  
Serial.print(", ");
Serial.println(u);                      // [V] Print input
k++;                                    // Increment time-step k
}
