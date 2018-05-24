#include "AutomationShield.h"

float Setpoint = 30.00;


float senzor = 0.00; // variables
float converted = 0.00;

float maximum = 24.56; // determining the boundaries of the mapFloat() function
float minimum = 10.27;

void setup() {
  
Serial.begin(9600);
MotoShield.begin(); // initializing pins

MotoShield.setDirection(true);       // starting the motor
MotoShield.setMotorSpeed(Setpoint);
}

void loop() {

senzor = MotoShield.readRevolutions(50);  // measuring the RPM
converted = AutomationShield.mapFloat(senzor,minimum,maximum,0.00,100.00); // converting the RPM
   
  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.println(converted); 

}


