#include <BOBShield.h>                // Include the library
#include <SamplingServo.h>            // Include sampling

unsigned long Ts = 14;            // Sampling in milliseconds
unsigned long k = 0;                // Sample index
bool enable=false;                  // Flag for sampling 

float y = 0.0;            // Output
float u = 0.0;            // Input (open-loop)
float U[]={-30.0,-29.0,-28.0,-27.0,-26.0,-25.0,-24.0,-23.0,-22.0,-21.0,-20.0,-19.0,-18.0,-17.0,-16.0,-15.0,-14.0,-13.0,-12.0,-11.0,-10.0,-9.0,-8.0,-7.0,-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0};  // Input trajectory
int T = 1;              // Section length (appr. '/.+2 s)
int i = 0;              // Section counter
int a =0;               //flag for BoB tube shifting right / left

void setup() {
 Serial.begin(115200);                // Initialize serial
 BOBShield.begin();                   // Define hardware pins
 BOBShield.initialize();              // Check if Adafruit VL6180 sensor is available

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

// A single algorithm step
void step(){ 
if (a==0){                           // if tube is shifting right
if (k % (T*i) == 0){        
  u = U[i];                          // Set reference by modulo math operation
  i++;
}
k++;                                 //increment K index
}
if (a==1){                           // if tube is shifting left
if (k % (T*i) == 0){        
  u = U[i];                          // Set reference by modulo math operation
  i--;
}
k--;                                 //decrement K index
}                  
y = BOBShield.sensorRead();           // Read sensor 
BOBShield.actuatorWrite(u);           // Actuate
   
Serial.print(y);                     // Print output  
Serial.print(", ");
Serial.println(u);                   // Print input


if(i==60){                           // if tube was shifting right, switch direction of shifting to the left
  a=1;
}
if(i==0){                            // if tube was shifting left, switch direction of shifting to the right
  a=0;
}
}
