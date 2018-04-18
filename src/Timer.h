#ifndef TIMER_H_
#define TIMER_H_

#include "Arduino.h"

typedef void (*p_to_void_func)();

class TimerClass{

  public:

    TimerClass(); 
    void interruptInitialize(unsigned long microseconds);
    void setInterruptCallback(p_to_void_func isr);
    p_to_void_func getInterruptCallback ();
    float getSamplingPeriod();  
         
  private:
  
    p_to_void_func interruptCallback;    
    const unsigned long timer1Resolution = 65536; // timer1 is 16bit            
    const unsigned char cpuFrequence = 16; // cpu frequence in microseconds 
    float samplingPeriod;          
    bool setSamplingPeriod(unsigned long microseconds);
};
extern TimerClass Timer;  
#endif
