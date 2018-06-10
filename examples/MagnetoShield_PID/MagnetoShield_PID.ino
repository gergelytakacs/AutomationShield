#include <AutomationShield.h>


float input, error, Setpoint, output;          //declaration of global vriables for .ino program
int Minimum;
int Maximum;

unsigned long Ts=688;                         //time of samplings in microseconds
/* 
 * 1) for flying without serial comunication = 688 microseconds
 * 2) flying with comunication = 1536 microseconds
 * 3) flying with comunication and converting to % = 1476 microseconds
 * there is possibility to use 1) sampling for all types but some data can be lost - doesn't have big influance 
 */
 
bool next=false;                              //variable which enable step() function

void setup() {
  Serial.begin(230400);                       //speed of serial comunication in bauds [bits/s]
  
   MagnetoShield.begin();
   MagnetoShield.calibration();
   Setpoint = MagnetoShield.setHeight(50.00); //returns hidden global value inside the function, so for flying without serial comunication, 
                                              //variable Setpoint not needed, function can be written like void function
   

   Minimum=MagnetoShield.getMin();            //getting borders for flying
   Maximum=MagnetoShield.getMax();
   
   Sampling.interruptInitialize(Ts);          //periaod of sampling
   Sampling.setInterruptCallback(stepEnable); //what happens when interrupt is called

   PIDAbs.setKp(12);                         //setting PID constants 
   PIDAbs.setTi(1.5);
   PIDAbs.setTd(0.02);

}

void loop() {                                 //execute program step() if next=true
   if(next){  
   step();
   next=false;
   }
}

void stepEnable(){                            //execute interrupt function -> enable step()
  next=true;
}

void step(){
// 1) Flying without Serial comunication - smoothiest regulation
    error = MagnetoShield.error();                           //diference between desired and real position of flying
    input=PIDAbs.compute(error,155,255,-65000,65000);        //PID regulation - values 155 and 255 depends on used MOSFET and his "permeability"
                                                             //possibility use 0 and 255 if these numbers are unknown
    MagnetoShield.setVoltage(input);                         //writes input into the system

// 2) Serial comunication shows inputs and outputs with theirs real values 
/*    output = MagnetoShield.readHeight();                    //output fom system - position of magnet  
    error = Setpoint-output;                                //diference between desired and real position of flying
    input=PIDAbs.compute(error,155,255,-65000,65000);       //create input into the system
    MagnetoShield.setVoltage(input);                        //writes input into the system
    Serial.print(output);                                   //shows inputs and outputs in Serial windows
    Serial.print(" ");
    Serial.println(input);
*/
// 3) Serial comunication shows inputs and outputs in %  
/*    output = MagnetoShield.readHeight();                                              //output fom system - position of magnet
    error = Setpoint-output;                                                          //diference between desired and real position of flying
    input=PIDAbs.compute(error,155,255,-65000,65000);                                 //create input into the system
    MagnetoShield.setVoltage(input);                                                  //writes input into the system
    float outputPer = AutomationShield.mapFloat(output,Minimum,Maximum,0.00,100.00);  //converts output into %
    float inputPer = AutomationShield.mapFloat(input,155.00,255.00,0.00,100.00);      //converts input into %
    Serial.print(outputPer);                                                          //shows inputs and outputs in Serial windows as %
    Serial.print(" ");
    Serial.println(inputPer);
*/    
}
