/*
  ISR for Interrupt-driven sampling for real-time control.
  Not to be used with the Servo library!
  
  The file implements the two classes necessary for configuring
  an interrupt-driven system for deploying digital control systems
  on AVR, SAMD and SAM-based Arduino prototyping boards with the 
  R3 pinout. The module should be compatible with the Arduino Uno, 
  Mega 2560, Zero, Due, Uno R4 and Adafruit Metro M4 Express.
  There should be no timer conflicts when using the Servo library.
    
  A "Sampling" object is created first from the namespace referring
  to the case that does not assume the use of the Servo library. 
  ISR are implemented based on various architectures. The 
  implementation of the classes setting up the timers of the 
  hardware are located elsewhere.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons

  AVR Timer: 	 Gergely Takacs, Richard Koplinger, Matus Biro,
                 Lukas Vadovic 2018-2019
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM51 Timer: 	 Gergely Takacs, 2019 (Metro M4)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  RA4M1 Timer:   Erik Mikuláš,   2023 (UNO R4)
  Last update: 3.10.2023 by Erik Mikuláš
*/ 

#ifndef SAMPLING_H                     //Include guard
#define SAMPLING_H


SamplingNoServo::SamplingClass Sampling; 

#ifdef ARDUINO_AVR_UNO
	#define UNO_ISR_VECT TIMER1_COMPA_vect
	#include "sampling/SamplingUNO_ISR.h"

#elif ARDUINO_AVR_MEGA2560
  #define MEGA_ISR_VECT TIMER5_COMPA_vect
	#include "sampling/SamplingMEGA_ISR.h"
    
#elif (defined(ARDUINO_SAMD_ZERO) || defined(ADAFRUIT_METRO_M4_EXPRESS))
	#include "sampling/SamplingSAMD_ISR.h"

#elif ARDUINO_ARCH_SAM
void TC5_Handler(void){
  TC_GetStatus(TC1, 2);
 (Sampling.getInterruptCallback())();
}

#elif ARDUINO_ARCH_RENESAS_UNO
  #include "sampling/SamplingUNO_R4_ISR.h"

#else
  #error "Architecture not supported."
#endif

#endif