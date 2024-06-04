
/*
  BicopShield LQI control example.


    The following code is an example of BicopShield API in use with
    two modes of reference value setting.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Martin Nemček, Ján Boldocký.
  Last update: 30.4.2024
*/
#include <BicopShield.h>     //--Include API
#include <Sampling.h>


#define TS 10.0            //--Defining Sample period in milliseconds
#define AUTO 1           //--Defining reference Mode # MANUAL / AUTO
#define USE_KALMAN 0      // Use kalman filter for state estimation 

float r = 0.0;            //--Reference
          
float R[]={0.8,1.0,0.35,1.3,1.0,0.35,0.8,1.2,0.0};  //--Input trajectory
//float R[]={0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8};  //--Input trajectory
float y = 0.0;           //--Output
float u1 = 0.0;          //--Input
float u2 = 0.0;          //--Input
float yprev = 0;

bool realTimeViolation = false;
bool nextStep = false;
unsigned int k = 0;                //--Sample index
int T = 1000;              //--Section length
int i = 0;               //--Section counter
float BaseU = 50.0;      //--Base setpoint of motor inputs
float y_out = 0.0; 
float counter =0.0;


BLA::Matrix<3, 1> X = {0, 0, 0};                                 //--Estimated state vector
BLA::Matrix<2, 3> K = {47.01, 15.25, -67.85, -58.08, -18.84, 83.83};              //--LQ gain with integrator, see MATLAB example
//BLA::Matrix<2, 3> K = {100.65, 21.78, -161.71, -124.34, -26.91, 199.77};        //--LQ gain with integrator, see MATLAB example
BLA::Matrix<3, 1> Xr = {0, 0, 0};                                //--Initial state reference
BLA::Matrix<2, 1> U = {0, 0}; 
 
void setup() {
 Serial.begin(115200);               //--Initialize serial communication # 2 Mbaud
 BicopShield.begin();               //--Initialize MotoShield
 delay(1000);
 BicopShield.calibrate();              //  Calibrate BicopShield board + store the 0° value of the pendulum
 Serial.println("r, y, u"); //--Print header
 Sampling.period(TS*1000.0);
 Sampling.interrupt(stepEnable);
 //delay(1000);
}

void loop(){
  if (y > M_PI) {    // If pendulum agle too big
    BicopShield.actuatorWrite(0.0,0.0);  // Turn off motor
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
        BicopShield.actuatorWrite(0.0,0.0);                 // Turn off the fan
        while(1);                                      // Stop program execution
    }
    nextStep = true;                                   // Enable step flag
}


void step(){ //--Algorithm ran once per sample
  #if !AUTO
    r =  AutomationShield.mapFloat(BicopShield.referenceRead(),0,100,0,M_PI_2);          //--Sensing Pot reference
  #elif AUTO
    if(i >= sizeof(R)/sizeof(float)){ //--If trajectory ended
      BicopShield.actuatorWrite(0.0,0.0); //--Stop the Motor
      while(true); //--End of program execution
    }
    if (k % (T*i) == 0){ //--Moving through trajectory values    
    //r = R[i];        
    Xr(0) = R[i];
      i++;             //--Change input value after defined amount of samples
    }
    k++;                              //--Increment
  #endif

BLA::Matrix<2,1> U = -(K*(X-Xr));     // LQ computation
 
u1=U(0);
u2=U(1);
u1=constrain(u1,-30.0,30.0);          // Saturation
u2=constrain(u2,-30.0,30.0);          
u1=BaseU+u1;                          // Final Values
u2=BaseU+u2;
y = BicopShield.sensorReadRadian(); // Angle in radians
 
BicopShield.actuatorWrite(u1,u2);
X(0) = y;
X(1) = (y-yprev)/(TS/1000.0);

X(2) = X(2) + 0.01*(Xr(0) - X(0));  // Integracna zlozka
yprev=y;
y_out=BicopShield.sensorRead();

  Serial.print(Xr(0),3);            //--Printing reference
  Serial.print(", ");            
  Serial.print(y,3);        //--Printing output
  Serial.print(", ");
   Serial.print(u1,3);            // Printing motor1 input
  Serial.print(", ");
  Serial.println(u2,3);           // Printing motor2 input
}
