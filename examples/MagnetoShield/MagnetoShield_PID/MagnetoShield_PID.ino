/*
  MagnetoShield closed-loop PID response example

  PID feedback control for magnet levitation.
  
  This example initializes the sampling and PID control 
  subsystems from the AutomationShield library and starts a 
  predetermined reference trajectory for the heating block
  temperature. 
  
  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Jakub Mihalík. 
  Last update: 14.11.2018.
*/

#include <MagnetoShield.h>     // Include header for hardware API

float u;	                    // Input
float e; 	                    // Error
float r; 	                    // Setpoint
float y;                      // Output
int Minimum;
int Maximum;
unsigned long Ts=1000;        // Sampling in microseconds
bool next=false;              // step() enable toggle flag

void setup() {
   MagnetoShield.begin();
   MagnetoShield.calibration();
   r = MagnetoShield.setHeight(50.00); //returns hidden global value inside the function, so for flying without serial comunication, 
                                              //variable Setpoint not needed, function can be written like void function
   

   Minimum=MagnetoShield.getMin();            //getting borders for flying
   Maximum=MagnetoShield.getMax();
   
   Sampling.interruptInitialize(Ts);          //periaod of sampling
   Sampling.setInterruptCallback(stepEnable); //what happens when interrupt is called

   PIDAbs.setKp(10);                         //setting PID constants 
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
    e = MagnetoShield.error();                           //diference between desired and real position of flying
    u=PIDAbs.compute(e,155,255,-65000,65000);        //PID regulation - values 155 and 255 depends on used MOSFET and his "permeability"
                                                             //possibility use 0 and 255 if these numbers are unknown
    MagnetoShield.setVoltage(u);                         //writes input into the system
}