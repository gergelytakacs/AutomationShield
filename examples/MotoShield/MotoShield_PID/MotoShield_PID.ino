#include <MotoShield.h>     //--Include API
#include <PIDAbs.h>        //--Include PID control lib

int Ts = 40;               //--Sampling period in milliseconds
float r = 0.0;            //--Reference
float y = 0.0;           //--Output
float u = 0.0;          //--Input          

#define KP 0.000001          //--PID Kp constant
#define TI 0.0003           //--PID Ti constant
#define TD 0.001           //--PID Td constant


void setup() {
  Serial.begin(2000000);               //--Initialize serial communication # 2 Mbaud
  MotoShield.begin(Ts);               //--Initialize MotoShield
  MotoShield.calibration();          //--Calibration
 PIDAbs.setKp(KP);    //--PID constants
 PIDAbs.setTi(TI); 
 PIDAbs.setTd(TD); 
 PIDAbs.setTs(Ts); //--Defining sampling period
}

void loop() {
  if (MotoShield.stepEnable) {      //--Running the algorithm once every sample
    step();               
    MotoShield.stepEnable=false;  //--Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}


void step(){ //--Algorith ran once per sample
  r = MotoShield.referenceRead();          //--Sensing Pot reference
  y = MotoShield.sensorReadRPMPerc();     //--Sensing angular velocity in percent
  u = PIDAbs.compute(r-y,0,100,0,100);   //--PID computation
  MotoShield.actuatorWrite(u);          //--Actuation

Serial.print(r);            //--Printing reference
Serial.print(" ");            
Serial.println(y);        //--Printing output
}
