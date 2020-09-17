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



int   i = 0;                          // Experiment section counter
float y = 0.0;                        // [mm] Output
float u = 0.0;                        // [V] Input          
uint8_t x1_p;
float y0= 14.3;                                                       
float i0= 0.0219;                                                     
float u0= 4.6234; 
#ifdef ARDUINO_ARCH_AVR
  unsigned long Ts = 3250;                // Sampling in microseconds, lower limit near 3.2 ms
  int T = 1500;                           // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAMD
  unsigned long Ts = 4200;                 // Sampling in microseconds, lower limit 
  int T = 1500;                            // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAM
  unsigned long Ts = 1300;                 // Sampling in microseconds, lower limit 1.3 ms
  int T = 3000;                            // Experiment section length (steps) 
#endif

// Linear, discrete state-space matrices for FloatShield model
BLA::Matrix<3, 3> A = {1.0133, 0.00327, -0.00109, 8.1875, 1.0168, -0.54516, -0.02724, -0.00542, 0.22523};
BLA::Matrix<3, 1> B = {-0, -0.00245, 0.00377};
BLA::Matrix<1, 3> C = {1, 0, 0};
// Kalman process and measurement error covariances
BLA::Matrix<3, 3> Q_Kalman = {0.0001, 0, 0, 0, 100, 0, 0, 0, 1};
BLA::Matrix<1, 1> R_Kalman = {0.001};
// LQ gain with integrator
BLA::Matrix<1, 4> K = {163.99, -7799.5, -214.59, 88.944};
BLA::Matrix<4, 1> X = {0, 0.001, 0, 0};                                 // Estimated state vector
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
  Sampling.period(Ts);    // Sampling init.
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
    //while(1);                           // Stop execution
  }                                     // else
  enable=true;                          // Change flag
}

void step(){ 

// Reference source
#if MANUAL                              // If reference from pot
  Xr(0) = AutomationShield.mapFloat(MagnetoShield.referenceRead(),0.0,100.0,12.0,17.0);
#else                                   // If pre-set experiment
 if (i>sizeof(R)/sizeof(R[0])){        // If experiment finished
    realTimeViolation=false; 
    MagnetoShield.actuatorWrite(0);     // then turn off magnet
    while(1);                           // and stop
  }
  else if (k % (T * i) == 0){             // else for each section
    Xr(0) = R[i];                           // set reference
    i++;                                // and increase section counter
  }
#endif

// Control algorithm
X(1) = (float)((MagnetoShield.sensorRead()-y0)/1000.0f);
X(3) = (float)(MagnetoShield.auxReadCurrent()/1000.0f - i0);  // [mA] Current read)// [mm] sensor read
float xr = (float)((Xr(0)-y0)/1000.0f);
X(2) = (X(1) - x1_p)/0.003250;

u = -(K(0)*X(0) + K(1)*X(1) + K(2)*X(2) + K(3)*X(3)) + u0;
X(0) = X(0) + (xr - X(1)); 
x1_p = X(1);

Serial.println(X(2),5);

if(u > 12)
  u=12;
else if (u < 0)
  u = 0;   
MagnetoShield.actuatorWrite(u); // [V] actuate
#ifdef USE_KALMAN_FILTER
  MagnetoShield.getKalmanEstimate(X, u, y/1000, A, B, C, Q_Kalman, R_Kalman); 
#else
  
#endif
 
// Print to serial port
/*Serial.print(X(0),5);                        // [mm] Print reference
Serial.print(", ");            
Serial.print(x1,5);                        // [mm ]Print output  
Serial.print(", ");
Serial.print(x2,5);                        // [mm ]Print output  
Serial.print(", ");
Serial.print(x3,5);                        // [mm ]Print output  
Serial.print(", ");
Serial.println(u);*/ // [V] Print input
k++;                                    // Increment time-step k

}
