// PID example fot rhe MotoShield

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
  
  MotoShield.begin(); // defining hardware pins
  MotoShield.setDirection(true); // setting the motor direction
  
  MotoShield.setMotorSpeed(100); // calibration
  delay(1000);
  
  Sampling.period(Ts * 1000);  // initialize the sampling function, input is the sampling time in microseconds
  Sampling.interrupt(stepEnable); // setting the interrupts, the input is the ISR function

 // setting the PID constants
 PIDAbs.setKp(0.007);
 PIDAbs.setTi(0.015);
 PIDAbs.setTd(0.0002);

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
  
  

r = MotoShield.referenceRead();  // reading the reference value of the potentiometer
variable = MotoShield.readRevolutions(50);    // reading RPM of the motor

y = AutomationShield.mapFloat(variable,0.00,25.00,0.00,100.00); //converting the RPM of the motor into %

error = r - y; 

 u = PIDAbs.compute(error,0,100,0,100);

MotoShield.setMotorSpeed(u);

Serial.print(r);
Serial.print(" ");
Serial.print(y);
Serial.print(" ");
Serial.println(u);
}
