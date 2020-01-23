#include "LinkShield.h"               // Include the library
#include <SamplingServo.h>            // Include sampling

unsigned long Ts = 10;                 // Sampling in milliseconds
unsigned long k = 0;                  // Sample index
bool enable=false;                    // Flag for sampling 

float y = 0.0;                        // Output variable
float u = 0;            // Input (open-loop), initialized to zero
float U[]={135.0, 45.0, 135.0, 45.0};  // Input trajectory
int T = 500;              // Section length (appr. '/.+2 s)
int i = 0;              // Section counter

void setup() {
 Serial.begin(2000000);                // Initialize serial
 LinkShield.begin();                   // Define hardware pins
 LinkShield.calibrate();



 
 // Initialize sampling function
 Sampling.period(Ts *1000);         // Sampling init.
 Sampling.interrupt(stepEnable);     // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {                      // If ISR enables
    step();                          // Algorithm step
    enable=false;                    // Then disable
  }  
}

void stepEnable(){                   // ISR 
  enable=true;                       // Change flag
}

// A signle algoritm step
void step(){ 

// Switching between experiment sections

if(i>(sizeof(U)/sizeof(U[0]))) {        // If at end of trajectory
        while(1);                           // Stop program execution
    } else if (k % (T*i) == 0) {            // If at the end of section
        u = U[i];                           // Progress in trajectory
        i++;                                // Increment section counter
    }
                 
y = LinkShield.sensorRead();           // Read sensor 
LinkShield.actuatorWrite(u);           // Actuate
   
Serial.print(y);                       // Print output  
Serial.print(", ");
Serial.println(u);                            // Print input

k++;                                          // Sample counter
}
