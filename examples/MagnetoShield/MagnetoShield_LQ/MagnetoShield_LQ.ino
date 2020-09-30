//   MAGNETOSHIELD LQ EXAMPLE
//
//   This example reads uses the linearized model of the MagnetoShield
//   and the LQ gain calculated in the MagnetoShield_LQ_Simulation.m
//   example in MATLAB. The state measurement can be performed directly
//   by reading the position and current values, then differentiating
//   for the velocity, or states may be observed via the Kalman filter.
//   Due to the fast dynamics of the plant, AVR architecture-based
//   Arduinos cannot use the Kalman filter, however, the Arduino Due
//   will handle it well.
//
//   This code is part of the AutomationShield hardware and software
//   ecosystem. Visit http://www.automationshield.com for more
//   details. This code is licensed under a Creative Commons
//   Attribution-NonCommercial 4.0 International License.
//
//   If you have found any use of this code, please cite our work in your
//   academic publications, such as theses, conference articles or journal
//   papers. A list of publications connected to the AutomationShield
//   project is available at: 
//   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
//
//   Created by:       Gergely Takacs and Gabor penzinger. 
//   Created on:       23.9.2020
//   Last updated by:  Gergely Takacs
//   Last update on:   24.9.2020


#include <MagnetoShield.h>
#include <Sampling.h>

#define MANUAL 0
#define USE_KALMAN_FILTER 0           // Use Kalman filter. Kalman filtering only works if ARDUINO_ARCH_SAM = 1 (e.g. Arduino DUE)
#define USE_DIFFERENTIATION 1         // Use differentiation to claculate speed
float R[]={14.0,13.0,14.0,15.0,14.0}; // Reference trajectory (pre-set)
  
  
unsigned long k = 0;                  // Sample index
bool enable=false;                    // Flag for sampling 
bool realTimeViolation=false;         // Flag for real-time violations


float Ref;                          // [mm] Reference
float y = 0.0;                      // [mm] Output (Current object height)
float yp= 0.0;                      // [mm] Stores previous position value
float u = 0.0;                      // [V] Input (Magnet voltage)
float I = 0.0;                      // [mA] Input (Magnet current)

float y_0 = 14.3;                   // [mm] Linearization point based on the experimental identification
float I0  = 21.9;                   // [mA] Linearization point based on the experimental identification
float u0  =  4.6234;                // [V] Linearization point based on the experimental identification

int i = 0;                          // Section counter

#if ARDUINO_ARCH_AVR
float Ts = 5;                       // Sampling in microseconds, lower limit near 5 ms
  int T = 1500;                     // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAMD
float Ts = 5;                       // Sampling in microseconds
  int T = 1500;                     // Experiment section length (steps) 
#elif ARDUINO_ARCH_SAM
 float Ts = 4;                      // Sampling in microseconds, lower limit 1.3 ms
  int T = 3000;                     // Experiment section length (steps) 
#endif

#if USE_KALMAN_FILTER && ARDUINO_ARCH_SAM
  // Linear, discrete state-space matrices for MagnetoShield model 
BLA::Matrix<3, 3> A = {1.0172, 0.00405, -0.001, 8.6344, 1.034, -0.36779, -0.14718, -0.02536, 0.09199};
BLA::Matrix<3, 1> B = {-0, -0.00284, 0.00425};
BLA::Matrix<2, 3> C = {1, 0, 0, 0, 0, 1};
BLA::Matrix<3, 3> Q_Kalman = {0.0001, 0, 0, 0, 100, 0, 0, 0, 100};    // process noise  covariance matrix
BLA::Matrix<2, 2> R_Kalman = {0.001, 0, 0, 0.001};                    // measurement noise covariance matrix
BLA::Matrix<3, 1> xIC = {1e-3, 0, 0};                                 // Kalman filter initial conditions
#endif


BLA::Matrix<1, 4> K = {52.25, -4548, -122.19, 44.731};              // LQ gain with integrator, see MATLAB example
BLA::Matrix<4, 1> X = {0, 0, 0, 0};                                 // Initial state vector
BLA::Matrix<4, 1> Xr = {0, 0, 0, 0};                                // Initial state reference

void setup() {
#if ARDUINO_ARCH_AVR || ARDUINO_ARCH_SAMD
    Serial.begin(2000000);  // Initialize serial, maximum for AVR given by hardware    
    #if USE_KALMAN_FILTER
      AutomationShield.print("Kalman filter cannot be used with this architecture! Set USE_DIFFERENTIATION to 1  and USE_KALMAN_FILTER to 0 or use Aduino DUE!");
      return 0;
    #endif 
#elif ARDUINO_ARCH_SAM
    Serial.begin(250000);                  // Initialize serial, maximum for Due (baud mismatch issues)
#endif 
  
  // Initialize and calibrate board
  MagnetoShield.begin();                  // Define hardware pins
  MagnetoShield.calibration();            // Calibrates shield 
     
  // Initialize sampling function
  Sampling.period(Ts*1000);               // Sampling init.
  Sampling.interrupt(stepEnable);         // Interrupt fcn.

}

void loop() {
  if (enable) {                           // If ISR enables
    step();                               // Algorithm step
    enable=false;                         // Then disable
  }  
}

void stepEnable(){                      // ISR 
  if(enable){                           // If previous still running
    realTimeViolation=true;             // RT violated
    Serial.print("Real-time samples violated.");
    while(1);                           // Stop execution
  }                                     // else
  enable=true;                          // Change flag
}

void step(){ 

// Reference source
#if MANUAL                              // If reference from pot
  Xr[0] = AutomationShield.mapFloat(MagnetoShield.referenceRead(),0.0,100.0,12.0,17.0);
#else                                   // If pre-set experiment
 if (i>sizeof(R)/sizeof(R[0])){         // If experiment finished
    realTimeViolation=false; 
    MagnetoShield.actuatorWrite(0);     // then turn off magnet
    while(1);                           // and stop
  }
  else if (k % (T * i) == 0){           // else for each section
    Xr(1) = (R[i]-y_0)/1000.0;          // set reference
    Ref = R[i];
    i++;                                // and increase section counter
  }
#endif


//Measurements
y = MagnetoShield.sensorRead();              // [mm] sensor read
float I = MagnetoShield.auxReadCurrent();    // [mA] Current read

#if USE_DIFFERENTIATION
X(0) = X(0) + (Xr(1)- X(1));                 // integrator 
X(1) = (y-y_0)/1000.0;                       // position calculated from measurement, compensated for linearization point, converted to [m] 
X(2) = (y - yp)/(1000.0*(float(Ts)/1000.0)); // speed calculated using differentiation  
X(3) = (I-I0)/1000.0;                        // current compensated for linearization point and converted to [A] 
yp = y;                                      // at the end of the calculation current value becomes previous value for the next iteration
#endif

u = -(K*X)(0) + u0;                          // LQ control algorithm

#if USE_KALMAN_FILTER && ARDUINO_ARCH_SAM
  BLA::Matrix<2, 1> Y = {(float)((y-y_0)/1000.0f), ((I-I0)/1000.0f)};            // measured states are stored in this vector
  BLA::Matrix<3, 1> Xk = {X(1), X(2), X(3)};                                     // Kalman filter returns the results to this vector
  MagnetoShield.getKalmanEstimate(Xk, u, Y, A, B, C, Q_Kalman, R_Kalman, xIC);   // State estimation using Kalman filter
  X(1) = Xk(0);
  X(2) = Xk(1);
  X(3) = Xk(2);
  X(0) = X(0) + (Xr(1)- X(1));                                                  
#endif

u = AutomationShield.constrainFloat(u,0,10);    // actuator constraints
MagnetoShield.actuatorWrite(u);                 // [V] actuate

Serial.print(Ref);               // Print reference
Serial.print(", ");
Serial.print((X(1)*1000)+y_0);   // Print output
Serial.print(", ");
Serial.println(u);               // Print input         

k++;                             // Increment time-step k

}
