#ifndef LOWPOWER_H_                     //Include guard
#define LOWPOWER_H_

#include <Arduino.h>

 class LowPowerClass{   
  public:    
          void begin(void);                         // Initializes a power down mode, enables it
          void powerDown(void);                     // A full power down command
          void setWatchDog(void);                   // Sets a watchdog timer wake up.
          void slowDown(byte);                      // Prescale 

   private: 
          void disableBOD(void);                    // Disables BOD. 
          void disableADC(void);                    // Disables ADC for sleep
          void powerDownExecute(void);              // Executes the sleep assembler command
          void powerDownInitialize(void);           // Initializes power down sequence
          void setPinsToOutputs(void);              // Sets GPIO to outputs
 }; // end of the class


extern LowPowerClass LowPower; // Declare external instance

#endif
