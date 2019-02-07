#ifndef SAMPLING_H_
#define SAMPLING_H_

#include "Arduino.h"

typedef void (*p_to_void_func)(); /*define a term p_to_void_func for pointer to function, which 
								  has a return type void and has no input parameters*/

class SamplingClass{

  public:

    SamplingClass();
    void interruptInitialize(unsigned long microseconds); 
    void setInterruptCallback(p_to_void_func interruptCallback);
    p_to_void_func getInterruptCallback ();
    float getSamplingPeriod();  
         
  private:
	
    static void defaultInterrupt();
    p_to_void_func interruptCallback;    
    #ifdef ARDUINO_ARCH_AVR
   	 const unsigned long timer1Resolution = 65536; 		// AVR Timer 1 is 16bit            
    	 const unsigned char cpuFrequence = 16; 		// AVR Arduino CPU frequency in microseconds 
    #elif ARDUINO_ARCH_SAMD
  	// Not developed yet.
    #else
   	#error "Architecture not supported."
    #endif

    float samplingPeriod;        		 	// Sampling period in seconds  
    bool setSamplingPeriod(unsigned long microseconds);
};
extern SamplingClass Sampling;  
#endif
