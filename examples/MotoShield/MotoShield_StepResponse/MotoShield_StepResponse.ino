#include "MotoShield.h"
#include <Sampling.h>            // Include sampling
   
   float Setpoint = 100.00;        // reference
   
   bool enable=false;              // variables for sampling
   unsigned long int curTime=0;
   unsigned long int prevTime=0;
   
   unsigned long Ts = 0.1;         // sampling time
   
   
   float senzor = 0.00;            // variables
   float converted = 0.00;
   
                                   // boundaries of converting
   float maximum = 24.36;          
   float minimum = 10.27;
   
   void setup() {
   
   Serial.begin(9600);
   
   Sampling.period(Ts * 1000);
   Sampling.interrupt(stepEnable);
                                   // sampling
   
   MotoShield.begin();             // board initialization   
   MotoShield.setDirection(true);  // starting the motor
   MotoShield.setMotorSpeed(Setpoint);
   }
   
   void loop() {
   
   curTime=millis();
   if (enable) {
   step();
   enable=false;
   
   } // end of the if statement  
   } // end of the loop
   
   void stepEnable(){
   enable=true;
   }
   
   void step(){
   senzor = MotoShield.readRevolutions(50);  
                                   // measuring rpm
   
   converted = AutomationShield.mapFloat(senzor,minimum,maximum,0.00,100.00); 
                                   // converting rpm into %
   
   Serial.print(Setpoint);
   Serial.print(",");
   Serial.println(converted); 
   }
