/*
  BoBShield Identification example.

  Example used to acquire data for BoBShield system identification.

*/
#include <BOBShield.h>                // Include the library
#include <SamplingServo.h>            // Include sampling
/*#include "prbsU.h"                    // Include PRBS sequence from .h file
int prbs;                             // Variable for storing PRBS signal
*/

//#include "aprbsU.h"                    // Include PRBS sequence from .h file -30 to 30
//int aprbs;                             // Variable for storing APRBS signal

#include "aprbs2U.h"                    // Include PRBS sequence from .h file -10 to 10
int aprbs2;                             // Variable for storing APRBS signal

//#include "aprbs3U.h"                    // Include PRBS sequence from .h file -30 to 30, higher frequency
//int aprbs3;                             // Variable for storing APRBS signal

//#include "aprbs4U.h"                    // Include PRBS sequence from .h file -5 to 5, higher frequency
//int aprbs4;                             // Variable for storing APRBS signal


unsigned long Ts = 14;            // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 

float y = 0.0;            // Output
int u = 0;            // Input (open-loop)
int T = 1;              // Section length (appr. '/.+2 s)
int i = 0;              // Section counter

void setup() {

 Serial.begin(115200);                // Initialize serial
 BOBShield.begin();                   // Define hardware pins
 BOBShield.initialize();              // Check if Adafruit VL6180 sensor is available
 BOBShield.calibration();              // Initialize sampling function
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

// A single algorithm step
void step(){ 

 if(k>7200) {                     // If at end of trajectory
        BOBShield.actuatorWrite(0.0);      // Turn off
        while(1);                            // Stop program execution
    } else {                                 // Otherwise
        
        //u=pgm_read_word(&aprbsU[k]); // Progress in trajectory from .h file -30 to 30
        u=pgm_read_word(&aprbs2U[k]); // Progress in trajectory from .h file -10 to 10
        //u=pgm_read_word(&aprbs3U[k]); // Progress in trajectory from .h file -30 to 30, higher frequency
       //u=2*pgm_read_word(&aprbs4U[k]); // Progress in trajectory from .h file -5 to 5, higher frequency

    }

y = BOBShield.sensorRead();           // Read sensor 
BOBShield.actuatorWrite(u);           // Actuate
        
Serial.print(y);            // Print output  
Serial.print(", ");
Serial.println(u);            // Print input

k++;

}
