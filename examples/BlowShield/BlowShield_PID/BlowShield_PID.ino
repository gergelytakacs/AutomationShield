#include "BlowShield.h"
//#include "Sampling.h"

#define Kp  30
#define Ti  3
#define Td  1
unsigned long Ts = 4000;  
bool enable = false;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
BlowShield.begin();
BlowShield.calibration();

// Initialize sampling function
//Sampling.period(Ts * 1000);   // Sampling init.
//Sampling.interrupt(stepEnable); // Interrupt fcn.

PIDAbs.setKp(Kp); 
PIDAbs.setTi(Ti); 
PIDAbs.setTd(Td);  
//PIDAbs.setTs(Sampling.samplingPeriod); // Sampling

}

void loop() {
 // if (enable) {               // If ISR enables
    Step();                 // Algorithm step
  //  enable=false;               // Then disable
  //}  
//}
  // put your main code here, to run repeatedly:


//void stepEnable(){              // ISR 
//  enable=true;                  // Change flag
}

void Step(){

  float r = BlowShield.referenceRead();
  float y = BlowShield.sensorRead();
  float u = PIDAbs.compute(r-y,0,100,0,100);   // PID
  BlowShield.actuatorWrite(u);        // Actuate
  
  Serial.print(BlowShield.referenceRead());
  Serial.print(",");
  Serial.println(BlowShield.sensorRead());
}
