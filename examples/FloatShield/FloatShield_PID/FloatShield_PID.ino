/*
  FloatShield closed-loop PID response example

  PID feedback control of ball altitude in the FloatShield.

  This example initialises the sampling and PID control
  subsystems from the AutomationShield library and allows user
  to select whether the reference is given by the potentiometer
  or by a predetermined reference trajectory. Upload the code to 
  your board and open the Serial Plotter tool in Arduino IDE.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 28.2.2020.
*/

#include <FloatShield.h>              // Include main library  
#include <Sampling.h>                 // Include sampling library

#define MANUAL 0                      // Choose manual reference using potentiometer (1) or automatic reference trajectory (0)


unsigned long Ts = 25;                // Sampling period in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep = false;                // Flag for step function
bool realTimeViolation = false;       // Flag for real-time sampling violation

float r = 0.0;                                                    // Reference (Wanted ball altitude)
float R[] = {65.0,50.0,35.0,45.0,60.0,75.0,55.0,40.0,20.0,30.0};  // Reference trajectory
float y = 0.0;                                                    // Output (Current ball altitude)
float u = 0.0;                                                    // Input (Fan power)
float uv= 0.0;                                                    // Input (Fan Voltage)

int T = 2400;             // Section length
int i = 0;                // Section counter

#if SHIELDRELEASE == 1
  #define KP 0.25           // PID Kp constant
  #define TI 5              // PID Ti constant
  #define TD 0.01           // PID Td constant
#elif SHIELDRELEASE == 2
  #define KP 0.01           // PID Kp constant
  #define TI 2              // PID Ti constant
  #define TD 0.01           // PID Td constant
  #elif SHIELDRELEASE == 4
  #define KP 0.15           // PID Kp constant
  #define TI 3              // PID Ti constant
  #define TD 0.1           // PID Td constant
#endif

void setup() {                         // Setup - runs only once
    Serial.begin(250000);              // Begin serial communication

    FloatShield.begin();               // Initialise FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board

    Sampling.period(Ts*1000);          // Set sampling period in microseconds

    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                      // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod); // Set sampling period for PID

    while(1) {                                  // Wait for ball to lift off from ground
#if MANUAL                                      // If Manual mode is active
        r = FloatShield.referenceRead();        // Read reference from potentiometer
#else                                           // If Automatic mode is active
        r = R[0];                               // Reference set to first point in trajectory
#endif
        y = FloatShield.sensorRead();           // Read sensor
        #if SHIELDRELEASE == 1
          u = PIDAbs.compute(r-y,30,100,30,100);  // PID
        #elif SHIELDRELEASE == 2
          u = PIDAbs.compute(r-y,40,100,40,100);  // PID
        #elif SHIELDRELEASE == 4
          u = PIDAbs.compute(r-y,10,100,10,100);  // PID
        #endif
        FloatShield.actuatorWrite(u);           // Actuate
        if(y >= r*2/3) {                        // If the ball is getting close to reference
            break;                              // Continue program
        }
        delay(Ts);                              // Wait before repeating the loop
    }
    Sampling.interrupt(stepEnable);             // Set interrupt function
}

void loop() {                       // Loop - runs indefinitely
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {                                    // ISR
    if(nextStep == true) {                             // If previous sample still running
        realTimeViolation = true;                      // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
        FloatShield.actuatorWrite(0.0);                // Turn off the fan
        while(1);                                      // Stop program execution
    }
    nextStep = true;                                   // Enable step flag
}

void step() {                               // Define step function
#if MANUAL                                  // If Manual mode is active
    r = FloatShield.referenceRead();        // Read reference from potentiometer
#else                                       // If Automatic mode is active
    if(i>(sizeof(R)/sizeof(R[0]))) {        // If at end of trajectory
        FloatShield.actuatorWrite(0.0);     // Turn off the fan
        while(1);                           // Stop program execution
    } else if (k % (T*i) == 0) {            // If at the end of section
        r = R[i];                           // Progress in trajectory
        i++;                                // Increment section counter
    }
#endif
    y = FloatShield.sensorRead();         // Read sensor
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID
    FloatShield.actuatorWrite(u);         // Actuate
    uv = FloatShield.actuatorReadVoltage(); // Read actuator voltage

    Serial.print(r);           // Print reference
    Serial.print(", ");
    Serial.print(y);           // Print output
    Serial.print(", ");
    Serial.print(u);         // Print input
    Serial.print(", ");
    Serial.println(uv);         // Print actuator voltage
    k++;                       // Increment index
}
