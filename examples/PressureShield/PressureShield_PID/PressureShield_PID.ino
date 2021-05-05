#include <PressureShield.h>         // Include the library
#include <Sampling.h>           // Include sampling

unsigned long Ts = 1;           // Sampling in milliseconds
bool enable = false;            // Flag for sampling
float r = 0.0;  
float u = 0.0;
float y = 0.0;
float R[] = {80.0, 20.0, 70.0, 30.0, 60.0};
int T = 300;
unsigned long k = 0;    
int i;
bool MS;                        // Manual switch value

#define Kp  10                 // PID Kp potom Kp=3
#define Ti  1                  // PID Ti
#define Td  0.0001             // PID Td

void setup() {
  
Serial.begin(2000000);             // Initialize serial
PressureShield.begin();             // Initialization
PressureShield.calibration();       // Calibration

// Initialize sampling function
Sampling.period(Ts * 10);       // Sampling init.
Sampling.interrupt(stepEnable); // Interrupt fcn.

PIDAbs.setKp(Kp);               // Proportional
PIDAbs.setTi(Ti);               // Integral
PIDAbs.setTd(Td);               // Derivative
PIDAbs.setTs(Sampling.samplingPeriod); // Sampling

}

void loop() {
  if (enable) {                 // If ISR enables
    step();                     // Algorithm step
    enable=false;               // Then disable
  }  
}

void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}

void step(){

MS = digitalRead(PRESSURE_MSPIN);   // Manual switch monitoring

if (MS == 1) {                  // Manual switch set to Automatic mode 
  
if (k % (T*i) == 0){        
  if (i==5) {
    i=0;
    k=0;
  }
  r = R[i];                     // Set reference
  i++;
}

  y = PressureShield.sensorRead();           // Read Sensor
  u = PIDAbs.compute(r-y,0,100,0,100);   // PID
  PressureShield.actuatorWrite(u);                 // Actuate

  /*Serial.print(r);                              // Print reference
  Serial.print(", ");
  Serial.print(y);                                // Print output  
  Serial.print(", ");
  Serial.println(u);                    
*/
  k++;
}

if (MS == 0) {                                 // Manual switch set to Manual mode
  
  r = PressureShield.referenceRead();        // Read reference
  y = PressureShield.sensorRead();           // Read Sensor
  u = PIDAbs.compute(r-y,0,100,0,100);   // PID
  PressureShield.actuatorWrite(u);                 // Actuate
}
  Serial.print(r);    // Print reference
  Serial.print(",");
  Serial.println(PressureShield.sensorRead());     // Print output
  //Serial.print(",");
  //Serial.println(u);


}
