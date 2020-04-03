#include <MotoShield.h>     //--Including API
#define TS 40.0            //--Defining Sample period in milliseconds

unsigned int k = 0;                //--Sample index
float y = 0.0;                    //--Output
float u = 0.0;                   //--Input
float U[]={40.0,70.0,50.0,85.0,35.0,60.0};  //--Input trajectory
int T = 80;              //--Section length
int i = i;               //--Section counter

void setup() {
  Serial.begin(9600);               //--Initialize serial communication # 9.6 Kbaud
  MotoShield.begin(TS);            //--Initialize MotoShield # input argument -> Sample period
  MotoShield.calibration();       //--Calibration
}

void loop() { 
  if (MotoShield.stepEnable) {    //--Running the algorithm once every sample     
    step();                
    MotoShield.stepEnable=false; //--Setting the flag to false # built-in ISR sets flag to true at the end of each sample    
  }  
}

void step(){          //--Algorith ran once per sample
  if (k % (T*i) == 0){ //--Moving through trajectory values    
    u = U[i];         
    i++;             //--Change input value after defined amount of samples
  }
                  
  y = MotoShield.sensorReadRPMPerc();   //--Sensing angular velocity in percent
  MotoShield.actuatorWrite(u);         //--Actuation
  Serial.print(y);                    //--Printing output
  Serial.print(" ");
  Serial.println(u);                 //--Printing input
  k++;                              //--Increment
}
