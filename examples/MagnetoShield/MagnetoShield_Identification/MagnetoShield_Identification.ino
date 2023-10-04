/*
  MagnetoShield closed-loop identification experiment

  Runs a closed-loop experiment to gather data for system
  identification.

  This example initializes the sampling and PID control
  subsystems from the AutomationShield library and starts a
  predetermined reference trajectory. Noise is injected to
  this input trajectory to create a rich signal. Upload the
  code to your board, then use a serial terminal software
  or Matlab to acquire the dataset for later processing.

  Tested with Arduino Uno, Arduino Due.
  Has not been functional with Arduino Zero.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács.
  Last update: 26.08.2020.
*/

#include <MagnetoShield.h>            // Include header for hardware API
#include <Sampling.h>            // Include sampling


float R[]={14.0}; // Reference trajectory (pre-set)
int T = 10000;                           // Experiment section length (steps)
float wPP = 6.0;                         // [V] Injected input noise amplitude (peak-to-peak)

// PID Tuning parameters
// Less-aggressive tuning introduces less saturation
// Negative part of gain built in the error computation
  #define KP 2.3                        // PID Kp
  #define TI 0.1                        // PID Ti
  #define TD 0.03                       // PID Td

unsigned long k = 0;                  // Sample index
bool enable = false;                  // Flag for sampling
bool realTimeViolation = false;       // Flag for real-time violations
float r = 0.0;                        // Reference
int   i = 0;                          // Experiment section counter
float y = 0.0;                        // [mm] Output
float u = 0.0;                        // [V] Input
float I = 0.0;                        // [mA] Current at sample (k)
float w = 0.0;                        // [V] Noise at sample (k)
float wBias=wPP/2.0;                  // [V] Noise bias
int   wP=(int)wPP*100;                // For (pseudo)-random generator

#ifdef ARDUINO_ARCH_AVR
  unsigned long Ts = 3250;              // Sampling in microseconds, lower limit 3.2 ms
#elif ARDUINO_ARCH_SAMD
  unsigned long Ts = 5000;              // Sampling in microseconds                
#elif ARDUINO_ARCH_SAM
  unsigned long Ts = 3250;              // Sampling in microseconds
#elif ARDUINO_ARCH_RENESAS_UNO
  unsigned long Ts = 5000;              // Sampling in microseconds
#endif  

void setup() {

#if ARDUINO_ARCH_AVR || ARDUINO_ARCH_SAMD
  Serial.begin(2000000);                 // Initialize serial, maximum for AVR given by hardware
#elif ARDUINO_ARCH_SAM
  Serial.begin(250000);                  // Initialize serial, maximum for Due (baud mismatch issues)
#elif ARDUINO_ARCH_RENESAS_UNO
      Serial.begin(115200);              // Initialize serial, maximum for UNO R4 (serial comunication limitations)
#endif

  // Initialize and calibrate board
  MagnetoShield.begin();               // Define hardware pins
  MagnetoShield.calibration();         // Calibrates shield

  // Initialize sampling function
  Sampling.period(Ts);    // Sampling init.
  Sampling.interrupt(stepEnable); // Interrupt fcn.

  // Set the PID constants
  PIDAbs.setKp(KP); // Proportional
  PIDAbs.setTi(TI); // Integral
  PIDAbs.setTd(TD); // Derivative
  PIDAbs.setTs(Sampling.samplingPeriod); // Sampling
}

// Main loop launches a single step at each enable time
void loop() {
  if (enable) {                         // If ISR enables
    step();                             // Algorithm step
    enable = false;                     // Then disable
  }
}

void stepEnable() {                     // ISR
  if (enable) {                         // If previous still running
    realTimeViolation = true;           // RT violated
    Serial.println("Real-time samples violated.");
    while (1);                          // Stop execution
  }                                     // else
  enable = true;                        // Change flag
}

// A single algorithm step
void step() {


  if (i > sizeof(R) / sizeof(R[0])) {   // If experiment finished
    MagnetoShield.actuatorWrite(0);     // then turn off magnet
    while (1);                          // and stop
  }
  else if (k % (T * i) == 0) {          // else for each section
    r = R[i];                           // set reference
    i++;                                // and increase section counter
  }

  // Control algorithm
  w=wBias-(float)random(0,wP)/100.0;        // [V] Input noise
  y = MagnetoShield.sensorRead();             // [mm] sensor read
  u = PIDAbs.compute(-(r - y), 0, MagnetoShield.getVoltageRef(), -10, 10) + w; // Compute constrained absolute-form PID and inject noise
  u = AutomationShield.constrainFloat(u,0,MagnetoShield.getVoltageRef());
  MagnetoShield.actuatorWrite(u);             // [V] actuate
  I = MagnetoShield.auxReadCurrent();         // [mA] Current read

  // Print to serial port

  Serial.print(y);                        // [mm ]Print output
  Serial.print(", ");
  Serial.print(u);                        // [V] Print input
  Serial.print(", ");
  Serial.println(I);                      // [mA] Print current

  k++;                                    // Increment time-step k
}
