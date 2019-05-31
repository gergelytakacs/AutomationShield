void TC5_Handler (void) {
 if (!Sampling.fireFlag){                   // If not over the maximal resolution of the counter
   //Interrupt can fire before step is done!!!
   TC5->COUNT16.INTFLAG.bit.MC0 = 1;    	// Clear the interrupt
   (Sampling.getInterruptCallback())();	    // Launch interrupt handler
 }                                          
 else if(Sampling.fireFlag){                // Else, if periodis over the resolution of the counter
    //Interrupt can fire before step is done!!!
   Sampling.fireCount++;                    // Start counting
   if (Sampling.fireCount==Sampling.getSamplingMicroseconds()/Sampling.fireResolution){ // If done with counting
	Sampling.fireCount=0;                   // Make the counter zero again 
	(Sampling.getInterruptCallback())();    // Launch interrupt handler
   } 
   TC5->COUNT16.INTFLAG.bit.MC0 = 1;        //Clear the interrupt 
 }
}