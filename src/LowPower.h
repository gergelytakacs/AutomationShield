/*
 * API implementing low-power mode (LPM) functionality and sleep modes.
 * 
 * This file implements low-power mode (LPM) functionality and sleep
 * modes and has been only tested on the ATmega 328p MCU, e.g. works
 * only on the Arduino UNO, may be possibly suitable for other AVR
 * chips. The public methods a) cycle through GPIO to reduce active
 * current consumption b) power down the MCU with minimal sleeping 
 * current, yet enabling wake-up from  interrupts D2/D3 and c) set
 * the watchdog timer to wake the system.
 * 
 * This code is part of the AutomationShield hardware and software
 * ecosystem. Visit http://www.automationshield.com for more
 * details. This code is licensed under a Creative Commons
 * Attribution-NonCommercial 4.0 International License.
 *
 * Created by Gergely Takács
 * Last update: 12.2.2020.
 *
 * Based on the tutorials of 
 * Kevin Darrah https://www.youtube.com/watch?v=urLSDi7SD8M (power down)
 * Julian Ilett https://www.youtube.com/watch?v=iMC6eG24S9g (slowdown)
*/


#ifndef LOWPOWER_H_                     //Include guard
#define LOWPOWER_H_

#include <Arduino.h>
#include <AutomationShield.h>

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
