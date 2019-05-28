#include <BOBShield.h>     // Include the library
#include <Sampling.h>            // Include sampling

unsigned long Ts = 2000;            // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 

float y = 0.0;            // Output
float u = 0.0;            // Input (open-loop)
float U[]={25.0,-85.0,50.0,15.0,-10.0,0.0};  // Input trajectory
int T = 1;              // Section length (T*Ts = section length)
int i = 0;              // Section counter

void setup() {
  Serial.begin(115200);               // Initialize serial
  
  // Initialize and calibrate board
  BOBShield.begin();                    // Define hardware pins
  BOBShield.initialize();               // Initialization
  BOBShield.calibration();              // Calibration
  
  // Initialize sampling function
  Sampling.period(Ts * 1000);   // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {               // If ISR enables
    step();                 // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}

// A signle algoritm step
void step(){ 

if (k % (T*i) == 0){        
  u = U[i];                // Set reference
  i++;
}
                  
y = BOBShield.sensorRead();           // Read sensor 
BOBShield.actuatorWrite(u);           // Actuate
   
Serial.print(y);            // Print output  
Serial.print(", ");
Serial.println(u);            // Print input
k++;                  // Increment k
}
