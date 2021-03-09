/*
  MotoShield PID control example.

    The following code is an example of MotoShield API in use with
    two modes of reference value setting. For PID computation, 
    PID control library has been used.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 23.4.2020.
*/
#include <MotoShield.h>     //--Include API
#include <PIDAbs.h>        //--Include PID control lib

#define TS 40.0            //--Defining Sample period in milliseconds
#define AUTO 0           //--Defining reference Mode # MANUAL / AUTO

#define KP 0.000001          //--PID Kp constant
#define TI 0.0003           //--PID Ti constant
#define TD 0.001           //--PID Td constant

float r = 0.0;            //--Reference
float R[]={40.0,70.0,50.0,85.0,35.0,60.0};  //--Input trajectory
float y = 0.0;           //--Output
float u = 0.0;          //--Input      

unsigned int k = 0;                //--Sample index
int T = 80;              //--Section length
int i = 0;               //--Section counter


void setup() {
 Serial.begin(2000000);               //--Initialize serial communication # 2 Mbaud
 MotoShield.begin(TS);               //--Initialize MotoShield
 MotoShield.calibration();          //--Calibration
 PIDAbs.setKp(KP);    //--PID constants
 PIDAbs.setTi(TI); 
 PIDAbs.setTd(TD); 
 PIDAbs.setTs(TS); //--Defining sampling period
 Serial.println("r, y, u");
}

void loop() {
  if (MotoShield.stepEnable) {      //--Running the algorithm once every sample
    step();               
    MotoShield.stepEnable=false;  //--Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}
void step(){ //--Algorith ran once per sample
#if !AUTO
  r = MotoShield.referenceRead();          //--Sensing Pot reference
#else AUTO
  if(i >= sizeof(R)/sizeof(float)){ //--If trajectory ended
    MotoShield.actuatorWrite(0.0); //--Stop the Motor
    while(true); //--End of program execution
  }
  if (k % (T*i) == 0){ //--Moving through trajectory values    
   r = R[i];        
    i++;             //--Change input value after defined amount of samples
  }
  k++;                              //--Increment
#endif
y = MotoShield.sensorReadRPMPerc();     //--Sensing angular velocity in percent
u = PIDAbs.compute(r-y,0,100,0,100);   //--PID computation
MotoShield.actuatorWrite(u);          //--Actuation
Serial.print(r);            //--Printing reference
Serial.print(", ");            
Serial.print(y);        //--Printing output
Serial.print(", ");
Serial.println(u);
}
