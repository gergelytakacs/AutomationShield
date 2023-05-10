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

  Created by Martin Vríčan.
  Last update: 20.3.2023.
*/

#include <LinkShield.h>               // Include the library

 

#define USE_DIFFERENTIATION 0
#define USE_KALMAN 1

unsigned long Ts = 5;                 // Sampling in milliseconds
unsigned long k = 0;                  // Sample index
bool nextStep=false;                  // Flag for sampling
bool realTimeViolation = false;       // Flag for real-time sampling violation
bool endExperiment = false;           // Boolean flag to end the experiment

float y_1 = 0.0;                        // Output variable
float y_2 = 0.0;                        // Output variable
float y_1prev = 0.0;
float y_2prev = 0.0;


float u = 0.0;                        // Input (open-loop), initialized to zero
float R[]={0.00, PI/6, -PI/4, 0.00};   // Input trajectory
int T = 1000;                         // Section length (appr. '/.+2 s)
unsigned int i = 0;                            // Section counter




BLA::Matrix<4, 1> X = {0, 0, 0, 0};
BLA::Matrix<4, 1> Xr = {0, 0, 0, 0};
BLA::Matrix<4, 1> U = {0, 0, 0, 0};
BLA::Matrix<2, 1> Y = {0, 0};
BLA::Matrix<4, 1> Xk = {0, 0, 0, 0};
BLA::Matrix<4, 1> xIC = {0, 0, 0, 0};


BLA::Matrix<1, 4> K = {6.0679*4, -15.234/10, 0.40104, -3.6504/5};
BLA::Matrix<4, 4> A = {1, 0.03235, 0.00475, 5e-05, 0, 0.9043, 6e-05, 0.00484, 0, 12.508, 0.90345, 0.03235, 0, -37.604, 0.02245, 0.9043};
BLA::Matrix<4, 1> B = {0.00068, -0.0004, 0.26726, -0.15689};
BLA::Matrix<2, 4> C = {1, 0, 0, 0, 0, 1, 0, 0};
BLA::Matrix<4, 4> Q_Kalman = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
BLA::Matrix<2, 2> R_Kalman = {0.05, 0, 0.015, 0};





void setup() {
 Serial.begin(250000);               // Initialize serial

 // Initialize linkshield hardware
 LinkShield.begin();                  // Define hardware pins
 //LinkShield.calibrate();              // Remove sensor bias

 // Initialize sampling function
 Sampling.period(Ts *1000);           // Sampling init.
 Sampling.interrupt(stepEnable);      // Interrupt fcn.
}

// Main loop launches a single step at each enable time
void loop() {
  if (nextStep) {                     // If ISR enables
    step();                           // Algorithm step
    nextStep = false;                   // Then disable
  }
}

void stepEnable(){                                     // ISR
  if(endExperiment == true){                           // If the experiment is over
	LinkShield.actuatorWrite(0.00);
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
		endExperiment = true;           // Stop program execution at next ISR
    }
	else if (k % (T*i) == 0) {      // If at the end of section
        Xr(0) = R[i];                     // Progress in trajectory
        i++;                          // Increment section counter
    }

y_1 = LinkShield.servoPotRead();          // Read sensor
y_2 = LinkShield.flexRead();

Y(0) = y_1;
Y(1) = y_2;

#if USE_DIFFERENTIATION
X(0) = y_1;
X(1) = y_2;
X(2) = (y_1 - y_1prev)/Ts;
X(3) = (y_2 - y_2prev)/Ts;
#endif

#if USE_KALMAN
LinkShield.getKalmanEstimate(Xk, u, Y, A, B, C, Q_Kalman, R_Kalman, xIC);
X(0) = Xk(0);
X(1) = Xk(1);
X(2) = Xk(2);
X(3) = Xk(3);
#endif


U = Xr - X;
u = (K*U)(0);


  	if		(y_1<-HALF_PI)	{u = AutomationShield.constrainFloat(u, 0.0,5.0);}
	else if	(y_1>HALF_PI)	{u = AutomationShield.constrainFloat(u,-5.0,0.0);}
	else					{u = AutomationShield.constrainFloat(u,-5.0,5.0);}

#if USE_DIFFERENTIATION
y_1prev = y_1;
y_2prev = y_2;
#endif


//LinkShield.actuatorWritePWM(u);          // Actuate
//LinkShield.actuatorWritePercent(u);
LinkShield.actuatorWrite(u);


Serial.print(y_1,4);                      // Print output
Serial.print(", ");
//Serial.print(X(0),4);
//Serial.print(", "); */
Serial.print(y_2,4);
//Serial.print(", ");
//Serial.println(X(1),4);
Serial.print(", ");
Serial.print(Xr(0),4);
Serial.print(", ");
Serial.println(u);                  // Print input

k++;                                  // Sample counter
}




