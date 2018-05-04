#ifndef SAMPLING_H_
#define SAMPLING_H_

#include "Arduino.h"

typedef void (*p_to_void_func)();

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
    const unsigned long timer1Resolution = 65536; // timer1 is 16bit            
    const unsigned char cpuFrequence = 16; // cpu frequence in microseconds 
    float samplingPeriod;          
    bool setSamplingPeriod(unsigned long microseconds);
};
extern SamplingClass Sampling;  
#endif
