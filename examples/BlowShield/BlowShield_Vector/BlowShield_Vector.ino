#include "BlowShield.h"         // Include the library
#include <Sampling.h>           // Include sampling

unsigned long Ts = 1;           // Sampling in milliseconds
bool enable = false;            // Flag for sampling
float r = 0.0;  
float R[] = {50.0, 40.0, 60.0, 30.0, 45.0};
int T = 300;
unsigned long k = 0;    
int i;

#define Kp  20                  // PID Kp
#define Ti  2                   // PID Ti
#define Td  0.01                 // PID Td

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
PIDAbs.setTs(Sampling.samplingPeriod); // Sampling

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

if (k % (T*i) == 0){        
  if (i==5) {
    i=0;
    k=0;
  }
  r = R[i];                // Set reference
  i++;
}

  float y = BlowShield.sensorRead();           // Read Sensor
  float u = PIDAbs.compute(r-y,0,100,0,100);   // PID
  BlowShield.actuatorWrite(u);                 // Actuate
  
  Serial.print(r);    // Print reference
  Serial.print(",");
  Serial.println(y);     // Print output
  k++;
}
