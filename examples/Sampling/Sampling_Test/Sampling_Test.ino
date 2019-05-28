/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR, SAMD and SAM-based Arduino 
  prototyping boards with the R3 pinout. This module should be 
  compatible with the Uno, Mega 2560, Arduino Zero and Arduino
  Due. There should be no timer conflicts when using the Servo library.

  This example initializes the Sampling functionality and toggles a
  digital signal each sampling period. The example can serve as a
  template to create your own feedback control framework with precise
  real-time sampling. The example can output the estimated sample
  durations by uncommenting the SERIAL_OUT macro define. Please note
  that this will somewhat slow down the execution and you are 
  unlikely to achive short samples.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács
  Last update: 28.5.2019.
*/
//#include <Servo.h>
#include <SamplingCore.h>           // Core classes for sampling, included in AutomationShield.h
#include <Sampling.h>               // Use for normal cases w/o Servo
//#include <SamplingServo.h>          // Use w Servo motors

//#define SERIAL_OUT                // Output on Serial, comment for oscilloscope only
#define PIN 12                      // For oscilloscope test

unsigned long int Ts = 2000000;     // Sampling in microseconds
bool enable=false;                 // Wheter the step should be launched

// Just for measuring the timing
unsigned long int curTime=0;      // Current time
unsigned long int prevTime=0;     // Previous time
bool state = 0;                   // For toggling the pin state

void setup() {
  pinMode(PIN,OUTPUT);            // Set the pin for digital output (e.g. an oscilloscope)

  #ifdef SERIAL_OUT               // If serial output is required
  Serial.begin(2000000);        // Maximal speed on AVR
  Serial.println("Sampling test begin:");
  #endif
  Sampling.period(Ts);            // Set period in microseconds
  Sampling.interrupt(stepEnable); // The interrupt will launch this function

  #ifdef SERIAL_OUT               // If serial output is required
  Serial.print("Period set to: ");
  Serial.print(Sampling.samplingPeriod);
  Serial.println(" s");
  Serial.println("---Actual samples---");
  #endif

}

void loop() {                     // Infinite loop
 curTime=micros();                // Measures current time
  if (enable) {                   // If the enable flag is set in the interrupt
    step();                       // Launch the algorithm step 
    enable=false;                 // then re-set the flag to false
  }  
}

void stepEnable(){                // Launched by the ISR periodically,
 enable=true;                     // changing the flag to true.
}
  
void step(){                      // The actual step
  
// ***Here comes your control application***

// This is just for testing
  #ifdef SERIAL_OUT               // If Serial is desired, print time
    Serial.print((float)(curTime-prevTime)/1000.0);
    Serial.println(" ms");
  #endif
    prevTime=curTime;             // Store previous time
  if(state == true) {             // if state
    digitalWrite(PIN,HIGH);       // High pin 
   } else {                       // else
     digitalWrite(PIN,LOW);       // Low pin
   }
    state = !state;               // Toggle state
}
