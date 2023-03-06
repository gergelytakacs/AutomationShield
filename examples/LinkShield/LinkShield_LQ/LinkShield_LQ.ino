/*
  LinkShield identification example.

  Example used to acquire data for LinkShield system identification.
  
  The LinkShield implements an a flexible rotational link experiment
  on an Arduino shield. This example initialises the sampling 
  subsystem from the AutomationShield library and allows user
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Martin Vríčan. 
  Last update: 23.1.2020.
*/

#include <LinkShield.h>               // Include the library
#include <BasicLinearAlgebra.h>


unsigned long Ts = 5;                 // Sampling in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep=false;                  // Flag for sampling 
bool realTimeViolation = false;       // Flag for real-time sampling violation
bool endExperiment = false;           // Boolean flag to end the experiment

float y1 = 0.0;                        // Output variable
float y2 = 0.0;                        // Output variable
float y1prev = 0.0;
float y2prev = 0.0;


float u = 0.0;                        // Input (open-loop), initialized to zero
float R[]={0.00, PI/4, -PI/4, 0.00};   // Input trajectory
int T = 500;                         // Section length (appr. '/.+2 s)
unsigned long int i = 0;                            // Section counter


BLA::Matrix<6, 6> A = {1.0000, -0.0061, 0.0049, 0.0006, 0, 0, 0, 0.9711, -0.0001, 0.0049, 0, 0, 0, -2.8489, 0.9400, 0.2229, 0, 0, 0, -11.4728, -0.0465, 0.9662, 0, 0, -1.0000, 0, 0, 0, 1.0000, 0, 0, -1.0000, 0, 0, 0, 1.0000};
BLA::Matrix<6, 1> B = {0.0001, -0.0001, 0.0572, -0.0224, 0, 0};
BLA::Matrix<2, 4> C = {1, 0, 0, 0, 0, 1, 0, 0};

//BLA::Matrix<4, 4> Q_Kalman = {};
//BLA::Matrix<1, 1> R_Kalman = {};

BLA::Matrix<1, 6> K = {34.0048, -76.0541, 1.7924, -0.7264, -0.9367, -0.0000};
BLA::Matrix<6, 1> X = {0, 0, 0, 0, 0, 0};
BLA::Matrix<6, 1> Xr = {0, 0, 0, 0, 0, 0};



void setup() {
 Serial.begin(2000000);               // Initialize serial

 // Initialize linkshield hardware
 LinkShield.begin();                  // Define hardware pins
 LinkShield.calibrate();              // Remove sensor bias
 
 // Initialize sampling function
 Sampling.period(Ts *1000);           // Sampling init.
 Sampling.interrupt(stepEnable);      // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (nextStep) {                     // If ISR enables
    step();                           // Algorithm step
    nextStep=false;                   // Then disable
  }  
}

void stepEnable(){                                     // ISR 
  if(endExperiment == true){                           // If the experiment is over
	LinkShield.actuatorWritePercent(0.00);
	
	

	//digitalWrite(9,LOW);
    while(1);      								// Do nothing
  }
  if(nextStep) {                               // If previous sample still running
        realTimeViolation = true;                      // Real-time has been violated
        Serial.println("Real-time samples violated."); // Print error message
        while(1);                                      // Stop program execution
  }   
  nextStep=true;                                       // Change flag
}

// A single algorithm step
void step(){ 

// Switching between experiment sections

if (i > sizeof(R) / sizeof(R[0])) {      // If at end of trajectory
        endExperiment=true;           // Stop program execution at next ISR		
    }
	else if (k % (T*i) == 0) {      // If at the end of section
        Xr(0) = R[i];                     // Progress in trajectory
        i++;                          // Increment section counter
    }				  
        
y1 = LinkShield.encoderRead();          // Read sensor 
//y2 = LinkShield.flexRead();

X(0) = y1;
X(1) = y2;
X(2) = (y1 - y1prev)/Ts;
X(3) = (y2 - y2prev)/Ts;
X(4) = X(4) + (Xr(0)-X(0));
X(5) = X(5) + (Xr(1)-X(1));

u = -(K * (X - Xr))(0);


	if(y1>HALF_PI)
	{
		u = AutomationShield.constrainFloat(u,0.0,5.0);	
	}
else if(y1<-HALF_PI)
	{
		u = AutomationShield.constrainFloat(u,-5.0,0.0);	
	}
else 
	{
		u = AutomationShield.constrainFloat(u,-5.0,5.0);// Calculate LQ system input
	}
y1prev = y1;
y2prev = y2;



//LinkShield.actuatorWritePWM(u);          // Actuate
//LinkShield.actuatorWritePercent(u);
LinkShield.actuatorWriteVoltage(u);
   

Serial.print(y1,4);                      // Print output  
Serial.print(", ");
//Serial.print(y2);
//Serial.print(", ");
Serial.println(u);                    // Print input

k++;                                  // Sample counter
}



    
