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