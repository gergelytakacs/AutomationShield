/*
  ISR for Interrupt-driven sampling for real-time control.
  Not to be used with the Servo library!
  
  The file implements the two classes necessary for configuring
  an interrupt-driven system for deploying digital control systems
  on AVR, SAMD and SAM-based Arduino prototyping boards with the 
  R3 pinout. The module should be compatible with the Uno, Mega 
  2560, Arduino Zero and Arduino Due. There should be no timer 
  conflicts when using the Servo library.
  
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
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 28.5.2019.
*/

SamplingNoServo::SamplingClass Sampling;

#ifdef ARDUINO_AVR_UNO
ISR(TIMER1_COMPA_vect)
{
 if (!Sampling.fireFlag){                   // If not over the maximal resolution of the counter
  (Sampling.getInterruptCallback())();      // Start the interrupt callback
 }                                          
 else if(Sampling.fireFlag){                // else, if it is over the resolution of the counter
   Sampling.fireCount++;                    // start counting
   if (Sampling.fireCount>=Sampling.getSamplingMicroseconds()/Sampling.fireResolution){ // If done with counting
      Sampling.fireCount=0;                 // make the counter zero again
      (Sampling.getInterruptCallback())();  // and start the interrupt callback
  } 
 }
}

#elif ARDUINO_AVR_MEGA2560
ISR(TIMER5_COMPA_vect)
{
 if (!Sampling.fireFlag){                   // If not over the maximal resolution of the counter
  (Sampling.getInterruptCallback())();      // Start the interrupt callback
 }                                          
 else if(Sampling.fireFlag){                // else, if it is over the resolution of the counter
   Sampling.fireCount++;                    // start counting
   if (Sampling.fireCount>=Sampling.getSamplingMicroseconds()/Sampling.fireResolution){ // If done with counting
      Sampling.fireCount=0;                 // make the counter zero again
      (Sampling.getInterruptCallback())();  // and start the interrupt callback
  } 
 }
}
    
#elif ARDUINO_ARCH_SAMD
void TC5_Handler (void) {
 //Interrupt can fire before step is done
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;    //Clear the interrupt
 (Sampling.getInterruptCallback())();
}

#elif ARDUINO_ARCH_SAM
void TC5_Handler(void){
  TC_GetStatus(TC1, 2);
 (Sampling.getInterruptCallback())();
}

#else
  #error "Architecture not supported."
#endif
