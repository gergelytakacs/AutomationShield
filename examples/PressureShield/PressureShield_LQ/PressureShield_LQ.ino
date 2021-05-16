#include <PressureShield.h>           
#include <Sampling.h>                 // Include interrupt-based sampling framework

unsigned long Ts = 10;                // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

float R[] = {50.0, 70.0, 40.0, 60.0, 80.0, 60.0, 30.0, 70.0, 60.0};  
float y = 0.0;                                                                        
float u = 0.0;                                                                        

int T = 1000;             // Section length
int i = 0;                // Section counter for pre-set reference

bool MS;

BLA::Matrix<2, 2> A = {0.993, 0.01526, -0.003223, 0.4206};      // State matrix A
BLA::Matrix<2, 1> B = {0.008751, -0.1501};                      // Input matrix B
BLA::Matrix<1, 2> C = {1.74, 0.8755};                           // Output matrix C

// Kalman process and measurement error covariances
BLA::Matrix<2, 2> Q_Kalman = {0.05, 0, 0, 0.01};   // Process noise covariance matrix
BLA::Matrix<1, 1> R_Kalman = {1};                  // Measurement noise covariance matrix

// LQ gain with integrator
BLA::Matrix<1, 3> K = {1.25, 0.05, -0.01};                   // Pre-calculated LQ gain K
BLA::Matrix<3, 1> X = {0, 0, 0};                             // Estimated state vector
BLA::Matrix<3, 1> Xr = {0, 0, 0};                            // Reference state vector

void setup() {                     // Setup - runs only once
  Serial.begin(2000000);           // Begin serial communication

  PressureShield.begin();          // Initialise PressureShield board
  PressureShield.calibration();    // Calibrate PressureShield board

  Sampling.period(Ts * 1000);         // Set sampling period in microseconds (Ts in ms)
  Sampling.interrupt(stepEnable);     // Set interrupt function
}

void loop() {                       // Loop - runs indefinitely
  if (nextStep) {                   // If ISR enables step flag
    step();                         // Run step function
    nextStep = false;               // Disable step flag
  }
}

void stepEnable() {                                // ISR
  nextStep = true;                                 // Enable step flag
}

void step() {                                   // Define step function
MS = digitalRead(PRESSURE_MSPIN);
if (MS==0) {                                    // If Manual mode is active
  Xr(0)  = PressureShield.referenceRead();      // Read reference from potentiometer
}
if (MS==1) {                                    // If Automatic mode is active
  if (i > (sizeof(R) / sizeof(R[0]))) {         // If at end of reference
    PressureShield.actuatorWrite(0.0);          // Turn off the pump
    while (1);                                  // Stop program execution
  } else if (k % (T * i) == 0) {                // If at the end of section
    Xr(0)  = R[i];                              // Progress in reference
    i++;                                        // Increment section counter
  }
}

  y = PressureShield.sensorRead();                                           // Read sensor
  PressureShield.getKalmanEstimate(X, u, y, A, B, C, Q_Kalman, R_Kalman);    // Estimate internal states X
  X(2) = X(2) + (Xr(0) - X(0) - Xr(0)*0.35);
  u = -(K * (X - Xr))(0);                                                    // Calculate LQ system input
  if (u <= 0){
    u = 0;
  }
  PressureShield.actuatorWrite(u);                                           // Actuate
  
 Serial.print(Xr(0));       // Print reference
 Serial.print(", ");
 Serial.print(y);           // Print output
 Serial.print(", ");
 Serial.println(u);

  k++;                      // Increment index
}
