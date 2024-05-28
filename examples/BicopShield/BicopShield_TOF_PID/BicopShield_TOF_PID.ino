/*
  BicopShield PID control example with BoB module.
    The following code is an example of BicopShield API in use with
    Ball-on-Beam module.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Nemček, Ján Boldocký.
  Last update: 30.4.2024
*/

#include "BicopShield.h"                // Include BicopShield header
#include <Sampling.h>                   // Include Sampling header
#define MANUAL 0
//Parameters for primary PID controller
#define KP 1.0                        // PID Kp constant
#define TI 0.001                        // PID Ti constant
#define TD 0.001                      // PID Td constant
//Parameters for secondary PID controller
#define KP2 2.5                        // PID Kp constant
#define TI2 2.5                        // PID Ti constant
#define TD2 0.5                      // PID Td constant



const float Ts=10;                      // Sampling period
//float R1[]={45.0, 55.0, 30.0, 45.0, 35.0};      // Reference matrix
float R_TOF=40.0;
unsigned long k = 0;                    // Cycle counter
unsigned long i;                        // Counter for going through trajectory values
float u;                                // Variable for PID output (system input)
float U;                                // Variable for PID output (system input)
float BaseU=50.0;                       // Base input for both motors
float u1=0.0;                           // Input value 1
float u2=0.0;                           // Input value 2
float e;                                // Error
float y;                                // Output
float r;                                // Variable for actual value of reference
float TOF;

PIDAbsClass PIDAbs2;


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);                      // Serial initialization
BicopShield.begin();                       // BicopShield initialization
delay(1000);
BicopShield.TOFinitialize();
BicopShield.TOFcalibration();
delay(1000);
BicopShield.calibrate();                   // Sensor calibration
delay(1000);

Sampling.period(Ts * 1000);                // Set sampling period (need to be in microseconds)
  PIDAbs.setKp(KP);                        // Set Proportional constant
  PIDAbs.setTi(TI);                        // Set Integral constant
  PIDAbs.setTd(TD);                        // Set Derivative constant
  PIDAbs.setTs(Sampling.samplingPeriod);   // Set sampling period for PID
  PIDAbs2.setKp(KP2);                        // Set Proportional constant 2
  PIDAbs2.setTi(TI2);                        // Set Integral constant 2
  PIDAbs2.setTd(TD2);                        // Set derivative constat 2
  PIDAbs2.setTs(Sampling.samplingPeriod);   // Set sampling period for PID 2
  Serial.print('y, r, u1, u2, u');
}

void loop() {
// put your main code here, to run repeatedly:
//if (k % 5000 == 0){           //  Moving through trajectory values           
 //r= R1[i];                    //  Choosing reference point
  // i++;}                     //  Change reference value after defined amount of samples
 //k++;                        //  Add 1 to cycle counter
TOF=BicopShield.TOFsensorRead();
y=BicopShield.sensorRead();     // Read actual angle value     

U=PIDAbs.compute(R_TOF-TOF,0,80,-60,60);  //PID computing
U=map(U,0,80,50,35);
u=PIDAbs2.compute(U-y,-(60),60,-100,100);  //PID computing
  u1=BaseU+(u/2);                         // Compute final motor1 value
  u2=BaseU-(u/2);                         // Compute final motor2 value
BicopShield.actuatorWrite(u1,u2);     // Write computed values on motors



Serial.print(y);                      // Print actual angle
Serial.print(", ");                     
Serial.print(U);                      // Print reference
Serial.print(", ");
Serial.print(u1);                     // Print value for motor1
Serial.print(", ");
Serial.print(u2);                     // Print value for motor2
Serial.print(", ");
Serial.println(TOF);                    // Print computed PID value
}
