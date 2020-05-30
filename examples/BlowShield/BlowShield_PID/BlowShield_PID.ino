#include "BlowShield.h"
#include <Sampling.h>

#define Kp  30
#define Ti  3
#define Td  1
unsigned long Ts = 5;  
bool enable = false;
float r = 0.0;  
float R[] = {35.0, 50.0, 75.0, 60.0, 40.0};
int T = 300;
unsigned long k = 0;    
int i;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
BlowShield.begin();
BlowShield.calibration();

// Initialize sampling function
Sampling.period(Ts * 1000);   // Sampling init.
Sampling.interrupt(stepEnable); // Interrupt fcn.

PIDAbs.setKp(Kp); 
PIDAbs.setTi(Ti); 
PIDAbs.setTd(Td);  
PIDAbs.setTs(Sampling.samplingPeriod); // Sampling

}

void loop() {
  if (enable) {               // If ISR enables
    step();                 // Algorithm step
   enable=false;               // Then disable
  }  
}
  // put your main code here, to run repeatedly:


void stepEnable(){              // ISR 
  enable=true;                  // Change flag
}

void step(){

if (k % (T*i) == 0){        
  if (i==5) {
    i=0;
    k=0;
  }
  r = R[i];                // Set reference
  i++;
}

  //float r = BlowShield.referenceRead();
  float y = BlowShield.sensorRead();
  float u = PIDAbs.compute(r-y,0,100,0,100);   // PID
  BlowShield.actuatorWrite(u);        // Actuate
  
Serial.print(r);            // Print reference
Serial.print(", ");            
Serial.println(y);            // Print output  
//Serial.print(", ");
//Serial.println(u);            // Print input
k++;  
}
