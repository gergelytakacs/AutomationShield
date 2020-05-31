#include "BlowShield.h"         // Include the library
#include <Sampling.h>           // Include sampling

unsigned long Ts = 1;           // Sampling in milliseconds
bool enable = false;            // Flag for sampling

#define Kp  10                  // PID Kp
#define Ti  3                   // PID Ti
#define Td  0.1                 // PID Td

void setup() {
  
Serial.begin(9600);             // Initialize serial
BlowShield.begin();             // Initialization
BlowShield.calibration();       // Calibration

// Initialize sampling function
Sampling.period(Ts * 10);       // Sampling init.
Sampling.interrupt(stepEnable); // Interrupt fcn.

PIDAbs.setKp(Kp);               // Proportional
PIDAbs.setTi(Ti);               // Integral
PIDAbs.setTd(Td);               // Derivative
//PIDAbs.setTs(Sampling.samplingPeriod); // Sampling

}

void loop() {
  if (enable) {                 // If ISR enables
    step();                     // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}

void step(){

  float r = BlowShield.referenceRead();        // Read reference
  float y = BlowShield.sensorRead();           // Read Sensor
  float u = PIDAbs.compute(r-y,0,100,0,100);   // PID
  BlowShield.actuatorWrite(u);                 // Actuate
  
  Serial.print(BlowShield.referenceRead());    // Print reference
  Serial.print(",");
  Serial.println(BlowShield.sensorRead());     // Print output
}
