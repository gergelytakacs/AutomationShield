#include <AutomationShield.h>
#include <BOBShield.h>
#include <FloatShield.h>
#include <HeatShield.h>
#include <MagnetoShield.h>
#include <MotoShield.h>
#include <OptoShield.h>
#include <PID.h>
#include <PIDAbs.h>
#include <PIDInc.h>
#include <Sampling.h>
#include <_Model.h>


//TODO values for Kp,Ki,Kd
//TODO determine values 25,55 ????


#include <ServoTimer2.h>


ServoTimer2 myservo;

int dist;
unsigned long int Ts = 100; //sampling time in ms
bool enable = false;   //flag 
int y;                 //output
float u;               //input
int r;                 //reference
float Ti,Td,Kp;        // PID constants
float error;           //error

void setup() {
  Serial.begin(115200); //start serial communication
  BOBShield.initialize(); //BOBShield initialization
    myservo.attach(9);

  Sampling.period(Ts*1000); //Sampling initialization in microseconds
  Sampling.interrupt(StepEnable); // seting the interrupt functon
  
  //Setting the PID constants
  PIDAbs.setKp(0.86);           //TODO values for Kp,Ki,Kd
  PIDAbs.setKi(1.729);
  PIDAbs.setKd(0.1081);
  //Getting constants needed for calculating PID in absolute form
  Ti = PIDAbs.getTi(); // integral time constant
  Td = PIDAbs.getTd(); // derivative time constant
  Kp = PIDAbs.getKp(); 

  }

void loop() 
{
  if(enable)
  {
    Step();
    enable = false; //enable flag becomes false after the execution of the Step() function
  }
}
  

void StepEnable(void) // interrupt function
{
  enable = true; // when the interrupt function is called, the enable is true
}

void Step (void)
{
  r = BOBShield.referenceRead(); //reading the reference value
  y = BOBShield.sensorRead(); //reading the  sensor value
  

  error = r - y; 
  u = PIDAbs.compute(error,25,55,0,100); // TODO determine values 25,55 ????
  myservo.write(u);
  Serial.print(r);
  Serial.print(", ");
  Serial.print(u);
  Serial.print(", ");
  Serial.println(y);
}
