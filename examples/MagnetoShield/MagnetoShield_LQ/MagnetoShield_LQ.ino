#include <MagnetoShield.h>
#include <Sampling.h>
#include <BasicLinearAlgebra.h>                                   // Include template function for calculating K gain for LQ control applications

#define MANUAL 0

float R[]={14.0,13.0,14.0,15.0,14.0}; // Reference trajectory (pre-set)
  
  
#if MANUAL                            // If it is manual reference
  Ts=3250;                             // Slightly slower for manual
#endif
unsigned long k = 0;                  // Sample index
bool enable=false;                    // Flag for sampling 
bool realTimeViolation=false;         // Flag for real-time violations


float Ref;
float y = 0.0;                      // [mm] Output (Current object height)
float yp= 0.0;
float u = 0.0;                      // [V] Input (Magnet voltage)
float I = 0.0;                      // [mA] Input (Magnet current)

float y0= 14.3;                     // [mm] Linearization point based on the experimental identification
float I0= 21.9;                     // [mA] Linearization point based on the experimental identification
float u0= 4.6234;                   // [V] Linearization point based on the experimental identification

int i = 0;                          // Section counter

#if ARDUINO_ARCH_AVR
float Ts = 5;                // Sampling in microseconds, lower limit near 5 ms
  int T = 1500;                           // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAMD
float Ts = 4.200;                 // Sampling in microseconds, lower limit 
  int T = 1500;                            // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAM
 float Ts = 1.300;                 // Sampling in microseconds, lower limit 1.3 ms
  int T = 3000;                            // Experiment section length (steps) 
#endif

// LQ gain with integrator
BLA::Matrix<1, 4> K = {115.458, -4666.8, -101.15, 37.033};
BLA::Matrix<4, 1> X = {0, 0, 0, 0};                                 // Estimated state vector
BLA::Matrix<4, 1> Xr = {0, 0, 0, 0}; 

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
  Sampling.period(Ts*1000);    // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

}

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

void step(){ 

// Reference source
#if MANUAL                              // If reference from pot
  Xr[0] = AutomationShield.mapFloat(MagnetoShield.referenceRead(),0.0,100.0,12.0,17.0);
#else                                   // If pre-set experiment
 if (i>sizeof(R)/sizeof(R[0])){        // If experiment finished
    realTimeViolation=false; 
    MagnetoShield.actuatorWrite(0);     // then turn off magnet
    while(1);                           // and stop
  }
  else if (k % (T * i) == 0){             // else for each section
    Xr(1) = (R[i]-y0)/1000.0;            // set reference
    Ref = R[i];
    i++;                                // and increase section counter
  }
#endif

// Control algorithm

y = MagnetoShield.sensorRead();
float I = MagnetoShield.auxReadCurrent();
X(0) = X(0) + (Xr(1)- X(1)); 
X(1) = (y-y0)/1000.0;
X(2) = (y - yp)/(1000.0*(float(Ts)/1000.0));
X(3) = (I-I0)/1000.0;  // [mA] Current read)// [mm] sensor read

yp = y;
u = -(K*X)(0) + u0;
u = AutomationShield.constrainFloat(u,0,10);  
MagnetoShield.actuatorWrite(u); // [V] actuate


Serial.print(Ref);       // Print reference
Serial.print(" ");
Serial.print(y);           // Print output
Serial.print(" ");
Serial.print(X(0));           // Print output
Serial.print(" ");
Serial.println(u); // [V] Print input
k++;                                    // Increment time-step k

}
