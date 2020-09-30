/*
  MagnetoShield closed-loop PID response example

  PID feedback control for magnet levitation.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library. You may
  select wheter the reference is given by the potentiometer
  or you want to test a predetermined reference trajectory. 
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. 
  
  Tested with Arduino Uno, Arduino Due.
  Has not been functional with Arduino Zero.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 16.01.2019
*/
#include <MagnetoShield.h>            // Include header for hardware API
#include <Sampling.h>            // Include sampling

// Manual or automatic reference?
#define MANUAL 0                      // Reference by pot (1) or automatically (0)?

// PID Tuning
#define KP 3.5                        // PID Kp
#define TI 0.6                        // PID Ti
#define TD 0.025                      // PID Td

//Alternate PID Tuning at Ts = 3250
//#define KP 4.0                        // PID Kp
//#define TI 0.2                        // PID Ti
//#define TD 0.03                       // PID Td

float R[]={14.0,13.0,14.0,15.0,14.0}; // Reference trajectory (pre-set)
  
  
#if MANUAL                             // If it is manual reference
  Ts=5000;                             // Slightly faster for manual
#endif
unsigned long k = 0;                  // Sample index
bool enable=false;                    // Flag for sampling 
bool realTimeViolation=false;         // Flag for real-time violations
float r = 0.0;                        // Reference


int   i = 0;                          // Experiment section counter
float y = 0.0;                        // [mm] Output
float u = 0.0;                        // [V] Input          

#ifdef ARDUINO_ARCH_AVR
  unsigned long Ts = 5000;                // Sampling in microseconds, lower limit near 3.2 ms
  int T = 1000;                           // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAMD
  unsigned long Ts = 5000;                 // Sampling in microseconds, lower limit 
  int T = 1000;                            // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAM
  unsigned long Ts = 5000;                 // Sampling in microseconds, lower limit 1.3 ms
  int T = 1000;                            // Experiment section length (steps) 
#endif  

void setup() {
    
#if ARDUINO_ARCH_AVR || ARDUINO_ARCH_SAMD
    Serial.begin(2000000);                 // Initialize serial, maximum for AVR given by hardware    
#elif ARDUINO_ARCH_SAM
    Serial.begin(250000);                  // Initialize serial, maximum for Due (baud mismatch issues)
#endif 
  
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
void loop() {
  if (enable) {                         // If ISR enables
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

// A single algoritm step
void step(){ 

// Reference source
#if MANUAL                              // If reference from pot
  r=AutomationShield.mapFloat(MagnetoShield.referenceRead(),0.0,100.0,12.0,17.0);
#else                                   // If pre-set experiment
  if (i>sizeof(R)/sizeof(R[0])){        // If experiment finished
    realTimeViolation=false; 
    MagnetoShield.actuatorWrite(0);     // then turn off magnet
    while(1);                           // and stop
  }
  else if (k % (T*i) == 0){             // else for each section
    r = R[i];                           // set reference
    i++;                                // and increase section counter
  }
#endif

// Control algorithm
y = MagnetoShield.sensorRead();         // [mm] sensor read
u = PIDAbs.compute(-(r-y),0,MagnetoShield.getVoltageRef(),-10,10);      // Compute constrained absolute-form PID
MagnetoShield.actuatorWrite(u);         // [V] actuate

// Print to serial port
Serial.print(r);                        // [mm] Print reference
Serial.print(", ");            
Serial.print(y);                        // [mm ]Print output  
Serial.print(", ");
Serial.println(u);                      // [V] Print input
k++;                                    // Increment time-step k
}
