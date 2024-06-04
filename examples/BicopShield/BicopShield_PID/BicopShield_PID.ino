/*
  BicopShield PID control example.
    The following code is an example of BicopShield API in use with
    two modes of reference value setting.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Nemček.
  Last update: 30.4.2024
*/


#include "BicopShield.h"                // Include BicopShield header
#include <Sampling.h>                   // Include Sampling header
#define MANUAL 0

#define KP 1.5                        // PID Kp constant
#define TI 2.0                        // PID Ti constant
#define TD 0.5                        // PID Td constant


const float Ts=10;                      // Sampling period
float R1[]={45.0, 55.0, 20.0, 65.0, 55.0, 20.0, 45.0, 70.0, 10.0, 50.0};      // Reference matrix
unsigned long k = 0;                    // Cycle counter
unsigned long i;                        // Counter for going through trajectory values
float u;                                // Variable for PID output (system input)
float BaseU=50.0;                       // Base input for both motors
float u1=0.0;                           // Input value 1
float u2=0.0;                           // Input value 2
float e;                                // Error
float y;                                // Output
float r;                                // Variable for actual value of reference


void setup() {
Serial.begin(115200);                      // Serial initialization
BicopShield.begin();                       // BicopShield initialization
delay(1000);
BicopShield.calibrate();                   // Sensor calibration
Sampling.period(Ts * 1000);                // Set sampling period (need to be in microseconds)
  PIDAbs.setKp(KP);                        // Set Proportional constant
  PIDAbs.setTi(TI);                        // Set Integral constant
  PIDAbs.setTd(TD);                        // Set Derivative constant
  PIDAbs.setTs(Sampling.samplingPeriod);   // Set sampling period for PID
  Serial.print('y, r, u1, u2, u');
}

void loop() {
  if (k % 2000 == 0){           //  Moving through trajectory values           
   r= R1[i];                    //  Choosing reference point
      i++;}                     //  Change reference value after defined amount of samples
    k++;                        //  Add 1 to cycle counter

y=BicopShield.sensorRead();     // Read actual angle value            
u=PIDAbs.compute(r-y,-(100-BaseU),(100-BaseU),-100,100);  //PID computing
   if(i>12){                    // Set i> count of reference points in R1 matrix
      u1=0;                     // Turn off motor1
      u2=0;                     // Turn off motor2
      BicopShield.actuatorWrite(u1,u2);   // Write turn off values
      while(1){}                          // Stop executing program
    }
  u1=BaseU+(u/2);                         // Compute final motor1 value
  u2=BaseU-(u/2);                         // Compute final motor2 value
BicopShield.actuatorWrite(u1,u2);     // Write computed values on motors

Serial.print(y);                      // Print actual angle
Serial.print(", ");                     
Serial.print(r);                      // Print reference
Serial.print(", ");
Serial.print(u1);                     // Print value for motor1
Serial.print(", ");
Serial.print(u2);                     // Print value for motor2
Serial.print(", ");
Serial.println(u);                    // Print computed PID value
}
