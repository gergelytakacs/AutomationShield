// PID example fot rhe OptoShield

#include "AutomationShield.h"

unsigned long Ts = 10; // sampling time in milliseconds

bool enable=false; // flag for the sampling function

// variables for the PID

float r = 0.00;
float y = 0.00;
float u = 0.00;
float error = 0.00;


void setup() {
  
  Serial.begin(9600);
  
  OptoShield.begin(); // defining hardware pins
  OptoShield.calibration(); // calibration function for accurate measurements
  
  Sampling.interruptInitialize(Ts * 1000);  // initialize the sampling function, input is the sampling time in microseconds
  Sampling.setInterruptCallback(stepEnable); // setting the interrupts, the input is the ISR function

 // setting the PID constants
 PIDAbs.setKp(0.1);
 PIDAbs.setTi(0.015);
 PIDAbs.setTd(0.0000008);

}// end of the setup

void loop() {

  if (enable) {
    step();
    enable=false;
    
  }  

} // end of the loop


void stepEnable(){  // ISR
  enable=true;
}

void step(){ // we have to put our code here

r = OptoShield.referenceRead();  // reading the reference value of the potentiometer
y = OptoShield.sensorRead();    // reading the sensor value (LDR in the tube)

error = r - y; 

 u = PIDAbs.compute(error,0,100,0,100);

OptoShield.actuatorWrite(u);

//Serial.print(r);
//Serial.print(" ");
Serial.print(u);
Serial.print(" ");
Serial.println(y);
  

}
