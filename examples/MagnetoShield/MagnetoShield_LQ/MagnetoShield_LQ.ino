#include <MagnetoShield.h>
#include <Sampling.h>
                              // Include template function for calculating K gain for LQ control applications

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
BLA::Matrix<3, 3> A = {1.0315, 0.0051, -0.0021, 12.6889, 1.0384, -0.6398, -0.0533, -0.0064, 0.1027}; // State matrix A
BLA::Matrix<3, 1> B = {0.00000922, -0.0048, 0.0044};                              // Input matrix B
BLA::Matrix<1, 3> C = {1, 0, 0};                                                // Output matrix C

// Kalman process and measurement error covariances
BLA::Matrix<4, 4> Q_Kalman = {100, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 1};     // Process noise covariance matrix
BLA::Matrix<1, 1> R_Kalman = {0.0001};                                  // Measurement noise covariance matrix

// LQ gain with integrator
BLA::Matrix<1, 4> K = {387.74,-7546.3 , -163.98, 99.65};         // Pre-calculated LQ gain K
BLA::Matrix<4, 1> X = {0, 0, 0};                                 // Estimated state vector
BLA::Matrix<4, 1> Xr = {0, 0, 0}; 
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
    while(1);                           // Stop execution
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
y = MagnetoShield.sensorRead();         // [mm] sensor read
u = -(K * (X - Xr))(0);     
MagnetoShield.actuatorWrite(u);         // [V] actuate
MagnetoShield.getKalmanEstimate(X, u, y, A, B, C, Q_Kalman, R_Kalman); 
// Print to serial port
Serial.print(Xr(0));                        // [mm] Print reference
Serial.print(", ");            
Serial.print(y);                        // [mm ]Print output  
Serial.print(", ");
Serial.println(u);                      // [V] Print input
k++;                                    // Increment time-step k
}