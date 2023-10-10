/*
  ISR for handling the interrupt-driven sampling for 
  real-time control on the Arduino Zero and the
  Adafruit Metro M4 Express, e.g. SAMD device-based boards.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons

  Gergely Takacs, 2019
  Last update: 3.6.2019.
*/

#ifndef SAMPLINGSAMD_ISR_H
#define SAMPLINGSAMD_ISR_H

void TC5_Handler (void) {
 if (!Sampling.fireFlag){                   // If not over the maximal resolution of the counter
   //Interrupt can fire before step is done!!!
   TC5->COUNT16.INTFLAG.bit.MC0 = 1;    	// Clear the interrupt
   (Sampling.getInterruptCallback())();	    // Launch interrupt handler
 }                                          
 else if(Sampling.fireFlag){                // Else, if period is over the resolution of the counter
    //Interrupt can fire before step is done!!!
   Sampling.fireCount++;                    // Start counting
   if (Sampling.fireCount==Sampling.getSamplingMicroseconds()/Sampling.fireResolution){ // If done with counting
	Sampling.fireCount=0;                   // Make the counter zero again 
	(Sampling.getInterruptCallback())();    // Launch interrupt handler
   } 
   TC5->COUNT16.INTFLAG.bit.MC0 = 1;        //Clear the interrupt 
 }
}
#endif