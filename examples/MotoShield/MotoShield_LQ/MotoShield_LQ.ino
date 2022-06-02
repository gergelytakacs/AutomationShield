
/*
  MotoShield LQ control example.


    The following code is an example of MotoShield API in use with
    two modes of reference value setting. For LQ computation, 
    LQ and Kalman Estimate library has been used.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 27.5.2022.
*/
#include <MotoShield.h>     //--Include API

#define TS 20.0            //--Defining Sample period in milliseconds
#define AUTO 1           //--Defining reference Mode # MANUAL / AUTO

float r = 0.0;            //--Reference
float R[]={1200.0,800.0,1200.0,1000.0,700.0,1000.0,1200.0,600.0,1100.0,500.0,0.0};  //--Input trajectory
float y = 0.0;           //--Output
float u = 0.0;          //--Input      

unsigned int k = 0;                //--Sample index
int T = 100;              //--Section length
int i = 0;               //--Section counter

BLA::Matrix<2, 2> A = {0.0013, 0.4064, 0, -0.0002};
BLA::Matrix<2, 1> B = {255.1968, 0.0038};
BLA::Matrix<2, 2> C = {1, 0, 0, 1};
BLA::Matrix<2, 2> Q_Kalman = {100, 0, 0, 0.01};    //--process noise  covariance matrix
BLA::Matrix<2, 2> R_Kalman = {1,0,0,10};                    //--measurement noise covariance matrix
BLA::Matrix<2, 1> xIC = {0, 0};                                //--Kalman filter initial conditions

BLA::Matrix<3, 1> X = {0, 0, 0};                                 //--Estimated state vector
BLA::Matrix<1, 3> K = {0.0079, 0.0014, -0.0008};              //--LQ gain with integrator, see MATLAB example
BLA::Matrix<3, 1> Xr = {0, 0, 0};                                //--Initial state reference

 
void setup() {
 Serial.begin(2000000);               //--Initialize serial communication # 2 Mbaud
 MotoShield.begin(TS);               //--Initialize MotoShield
 Serial.println("r, y, u"); //--Print header
}

void loop(){
  if (MotoShield.stepEnable) {      //--Running the algorithm once every sample
    step();               
    MotoShield.stepEnable=false;  //--Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}
void step(){ //--Algorith ran once per sample
#if !AUTO
  r = MotoShield.referenceRead();          //--Sensing Pot reference
#elif AUTO
  if(i >= sizeof(R)/sizeof(float)){ //--If trajectory ended
    MotoShield.actuatorWrite(0.0); //--Stop the Motor
    while(true); //--End of program execution
  }
  if (k % (T*i) == 0){ //--Moving through trajectory values    
   //r = R[i];        
   Xr(0) = R[i];
    i++;             //--Change input value after defined amount of samples
  }
  k++;                              //--Increment
#endif
u = -(K*(X-Xr))(0);
u = constrain(u,0,5);


//u=MotoShield.referenceRead()*5.0/100.0;
MotoShield.actuatorWriteVolt(u);          //--Actuation
BLA::Matrix<2, 1> Y = {MotoShield.sensorReadRadian(), MotoShield.sensorReadCurrent()};
MotoShield.getKalmanEstimate(X, u, Y, A, B, C, Q_Kalman, R_Kalman, xIC);   //--State estimation using Kalman filter
X(2) = X(2) + (Xr(0) - X(0));  


Serial.print(Xr(0),5);            //--Printing reference
Serial.print(", ");            
Serial.print(Y(0),5);        //--Printing output
Serial.print(", ");
Serial.println(u);
}
