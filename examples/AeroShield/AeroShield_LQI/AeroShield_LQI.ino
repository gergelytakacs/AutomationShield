
/*
  AeroShield LQI control example.


    The following code is an example of MotoShield API in use with
    two modes of reference value setting. For LQ computation, 
    LQ and Kalman Estimate library has been used.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 16.1.2023.
*/
#include <AeroShield.h>     //--Include API
#include <Sampling.h>


#define TS 3.0            //--Defining Sample period in milliseconds
#define AUTO 1           //--Defining reference Mode # MANUAL / AUTO

float r = 0.0;            //--Reference
float R[]={0.8,1.0,0.35,1.3,1.0,0.35,0.8,1.2,0.0};  //--Input trajectory
float y = 0.0;           //--Output
float u = 0.0;          //--Input
float yprev = 0;

bool realTimeViolation = false;
bool nextStep = false;
unsigned int k = 0;                //--Sample index
int T = 2000;              //--Section length
int i = 0;               //--Section counter

BLA::Matrix<3, 3> A = {-1, 0, 1, 0.9998, 0.003, 0, -0.1355, 0.9896, 0};
BLA::Matrix<3, 1> B = {0, 0.00014989, 0.0997};
BLA::Matrix<1, 3> C = {0, 1, 0};
BLA::Matrix<3, 3> Q_Kalman = {1, 0, 0, 0, 1, 0 ,0, 0, 0};    //--process noise  covariance matrix
BLA::Matrix<3, 3> R_Kalman = {1,0,0,1};                    //--measurement noise covariance matrix
BLA::Matrix<3, 1> xIC = {0, 0, 0};                                //--Kalman filter initial conditions

BLA::Matrix<3, 1> X = {0, 0, 0};                                 //--Estimated state vector
BLA::Matrix<1, 3> K = {-0.0181, 1.0465, 0.2224};              //--LQ gain with integrator, see MATLAB example
BLA::Matrix<3, 1> Xr = {0, 0, 0};                                //--Initial state reference

 
void setup() {
 Serial.begin(250000);               //--Initialize serial communication # 2 Mbaud
 AeroShield.begin();               //--Initialize MotoShield
 AeroShield.calibrate();              //  Calibrate AeroShield board + store the 0° value of the pendulum
 Serial.println("r, y, u"); //--Print header
 Sampling.period(TS*1000.0);
 Sampling.interrupt(stepEnable);
 //delay(1000);
}

void loop(){
  if (y > PI) {    // If pendulum agle too big
    AeroShield.actuatorWrite(0);  // Turn off motor
    while (1);                    // Stop program
  }
  if (nextStep) {      //--Running the algorithm once every sample
    step();               

    nextStep = false;  //--Setting the flag to false # built-in ISR sets flag to true at the end of each sample
  }  
}

void stepEnable() {                                    // ISR
    if(nextStep) {                                     // If previous sample still running
        realTimeViolation = true;                      // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
        AeroShield.actuatorWrite(0.0);                 // Turn off the fan
        while(1);                                      // Stop program execution
    }
    nextStep = true;                                   // Enable step flag
}


void step(){ //--Algorith ran once per sample
  #if !AUTO
    r = MotoShield.referenceRead();          //--Sensing Pot reference
  #elif AUTO
    if(i >= sizeof(R)/sizeof(float)){ //--If trajectory ended
      AeroShield.actuatorWriteVolt(0.0); //--Stop the Motor
      while(true); //--End of program execution
    }
    if (k % (T*i) == 0){ //--Moving through trajectory values    
    //r = R[i];        
    Xr(1) = R[i];
      i++;             //--Change input value after defined amount of samples
    }
    k++;                              //--Increment
  #endif
  u = -(K*(X-Xr))(0);
  u = constrain(u,0,3.7);

  y = AeroShield.sensorReadRadian(); // Angle in radians
  //u=MotoShield.referenceRead()*5.0/100.0;
  AeroShield.actuatorWriteVolt(u);          //--Actuation
  X(1) = y;
  X(2) = (y-yprev)/(TS/1000.0);
  BLA::Matrix<2, 1> Y = {X(0), X(1)};
  //AeroShield.getKalmanEstimate(X, u, Y, A, B, C, Q_Kalman, R_Kalman, xIC);   //--State estimation using Kalman filter
  X(0) = X(0) + (Xr(1) - X(1));  // Integracna zlozka
  yprev=y;

  Serial.print(Xr(1),3);            //--Printing reference
  Serial.print(" ");            
  Serial.print(X(1),3);        //--Printing output
  Serial.print(" ");
  Serial.println(u,3);
}
