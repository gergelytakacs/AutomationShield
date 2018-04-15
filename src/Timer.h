// Timer.h

  #include "Arduino.h"

  typedef void (*p_to_void_func)();

  class Timer{

    public:
    
      void interruptInitialize(unsigned long microseconds);

      void setInterruptCallback(p_to_void_func isr);

      p_to_void_func getInterruptCallback ();

      unsigned long getSamplingPeriod();
          
    private:

      unsigned long samplingPeriod;
      
      p_to_void_func interruptCallback;

      const unsigned long timer1Resolution = 65536; // timer1 is 16bit
            
      const unsigned char cpuFrequence = 16; // cpu frequence in microseconds      
    
      bool setSamplingPeriod(unsigned long microseconds);
  };

  extern Timer timer;
