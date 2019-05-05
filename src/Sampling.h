/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR, SAMD and SAM-based Arduino 
  prototyping boards. This module should be compatible with
  all AVR boards (Uno, Mega 2560 etc.), Arduino Zero and Arduino
  Due. Take care of timer conflicts when using the Servo library.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Koplinger 2018-2019
  AVR Timer: 	 Richard Koplinger, 2018
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 11.02.2019.
*/

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include "Arduino.h"

typedef void (*p_to_void_func)(); /*define a term p_to_void_func for pointer to function, which 
								  has a return type void and has no input parameters*/

class SamplingClass{

  public:

    SamplingClass();
    void period(unsigned long microseconds); 
    void interrupt(p_to_void_func interruptCallback);
    p_to_void_func getInterruptCallback ();
    float getSamplingPeriod();  
         
  private:
	
    static void defaultInterrupt();
    p_to_void_func interruptCallback;    

    /*#ifdef ARDUINO_ARCH_AVR
		// Default: Timer1
		const unsigned long timerResolution = 65536; 					// AVR Timer 1 is 16bit            
		const unsigned char cpuFrequency = 16; 						// CPU frequency in micro Hertz*/
	
    #ifdef ARDUINO_AVR_UNO 	//#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)	
		   // Default: Timer2
		const unsigned long timerResolution = 256; 					// AVR Timer 2 is 8bit            
		const unsigned char cpuFrequency = 16; 							// CPU frequency in micro Hertz	
    #elif ARDUINO_AVR_MEGA2560
        // Default: Timer5
	    const unsigned long timerResolution = 65536; 					// AVR Timer 5 is 16bit            
		const unsigned char cpuFrequency = 16; 							// CPU frequency in micro Hertz*/
    #elif ARDUINO_ARCH_SAMD
		// Default TC5 (randomly selected, take care of Servo!)
		const unsigned long timerResolution = 65536;     				// Configured to 16bit  
		const unsigned char cpuFrequency = VARIANT_MCK/1000000; 		// CPU frequency in micro Hertz (48 for Zero)
    #elif ARDUINO_ARCH_SAM
		// Default TC3 (randomly selected, take care of Servo!)
		const unsigned char cpuFrequency = VARIANT_MCK/1000000; 		// CPU frequency in micro Hertz (84 for Due)
    #else
		#error "Architecture not supported."
    #endif

    float samplingPeriod;        		 	// Sampling period in seconds  
    bool setSamplingPeriod(unsigned long microseconds);
};
extern SamplingClass Sampling;  
#endif
