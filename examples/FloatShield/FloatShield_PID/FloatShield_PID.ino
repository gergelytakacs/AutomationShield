
#include <AutomationShield.h>
#include <FloatShield.h>

int dist;
unsigned long int Ts = 100; //sampling time in ms
bool enable = false; //flag 
int y;  //output
float u; //input
int r; //reference
float Ti,Td,Kp; // PID constants
float error; //error
void setup() {
  Serial.begin(115200); //start serial communication
  FloatShield.initialize(); //FloatShield initialization
  FloatShield.calibrate(); //FloatShield calibration for more accurate measurements
  Sampling.interruptInitialize(Ts*1000); //Sampling initialization in microseconds
  Sampling.setInterruptCallback(StepEnable); // seting the interrupt functon
  //Setting the PID constants
  PIDAbs.setKp(0.86);
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
  y = FloatShield.positionPercent(); //reading the  sensor value
  r = FloatShield.referencePercent(); //reading the reference value

  error = r - y; 
  u = PIDAbs.compute(error,25,55,0,100); // absolute PID function with anti-wind up and saturation
  FloatShield.ventInPercent(u);
  Serial.print(r);
  Serial.print(", ");
  Serial.print(u);
  Serial.print(", ");
  Serial.println(y);
}

