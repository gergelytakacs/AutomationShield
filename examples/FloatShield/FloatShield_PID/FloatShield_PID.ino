#include <FloatShield.h>              // Include main library  
#include <Sampling.h>                 // Include sampling library

unsigned long Ts = 5000;              // Sampling period in microseconds
bool nextStep = false;                // Flag for step function

float r = 0.0;            // Reference
float y = 0.0;            // Output
float u = 0.0;            // Input

#define KP 0.01                // PID Kp
#define TI 0.001               // PID Ti
#define TD 0.00001             // PID Td


void setup() {
    Serial.begin(250000);              // Begin serial communication

    FloatShield.begin();               // Initialize FloatShield board
    FloatShield.calibrate();           // Calibrate FloatShield board

    Sampling.period(Ts);               // Set sampling period
    Sampling.interrupt(stepEnable);    // Set interrupt function

    PIDAbs.setKp(KP);                      // Set Proportional constant
    PIDAbs.setTi(TI);                      // Set Integral constant
    PIDAbs.setTd(TD);                      // Set Derivative constant
    PIDAbs.setTs(Sampling.samplingPeriod); // Sampling for PID
}


void loop() {
    if (nextStep) {                 // If ISR enables step flag
        step();                     // Run step function
        nextStep = false;           // Disable step flag
    }
}

void stepEnable() {               // ISR
    nextStep = true;              // Enable step flag
}

void step() {                             // Define step function
    r = FloatShield.referenceRead();      // Read reference
    y = FloatShield.sensorRead();         // Read sensor
    u = PIDAbs.compute(r-y,0,100,0,100);  // PID
    FloatShield.actuatorWrite(u);         // Actuate

    Serial.print(r);           // Print reference
    Serial.print(", ");
    Serial.print(y);           // Print output
    Serial.print(", ");
    Serial.println(u);         // Print input
}
