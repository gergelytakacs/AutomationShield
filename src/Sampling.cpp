/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR, SAMD and SAM-based Arduino 
  prototyping boards with the R3 pinout. This module should be 
  compatible with the Uno, Mega 2560, Arduino Zero and Arduino
  Due. There should be no timer conflicts when using the Servo library.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Richard Koplinger 2018-2019
  AVR Timer:     Gergely Takacs, Richard Koplinger, Matus Biro, Lukas Vadovic 2018-2019
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 6.5.2019.
*/

#include "Sampling.h"

void SamplingClass::defaultInterrupt()
{
}

SamplingClass::SamplingClass(){
  interruptCallback=defaultInterrupt;
  samplingPeriod = 0.0;
}

void SamplingClass::period(unsigned long microseconds){
  #ifdef ARDUINO_AVR_UNO
 	 // for Arduino Uno use Timer2 to stay compatible with the Servo library  
    noInterrupts();                             // disable all interrupts
	
    TCCR2A = 0;                                 // clear register
    TCCR2B = 0;                                 // clear register
    TCNT2  = 0;                                 // clear register
      
    TCCR2A |= (1 << WGM21);                     // CTC mode       
    TIMSK2 |= (1 << OCIE2A);                    // enable timer compare interrupt    
     
    setSamplingPeriod(microseconds);            // Set prescalers
	  
	  interrupts();             		              // enable all interrupts
	
	#elif ARDUINO_AVR_MEGA2560 
 	 // for Arduino Mega2560 use Timer5

    noInterrupts();                             // disable all interrupts
		TCCR5A = 0;                                 // clear register
    TCCR5B = 0;                                 // clear register
    TCNT5  = 0;                                 // clear register
       
    TCCR5B |= (1 << WGM52);                     // CTC mode       
    TIMSK5 |= (1 << OCIE5A);                    // enable timer compare interrupt

    setSamplingPeriod(microseconds);
   		
		interrupts();             		   // enable all interrupts
    	
  #elif ARDUINO_ARCH_SAMD
  // For SAMD21G boards, e.g. Zero
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
     GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;        // Set Timer counter Mode to 16 bits
	   TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;        // Set TC5 mode as match frequency
      
      setSamplingPeriod(microseconds);
      #ifdef ECHO_TO_SERIAL
     	if(!setSamplingPeriod(microseconds)) 
     	  Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
      #endif
      
	 NVIC_EnableIRQ(TC5_IRQn);                               // Enable interrupt for TC5
     TC5->COUNT16.INTENSET.bit.MC0 = 1;                      // Enable the TC5 interrupt request
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;              // Enable timer
  #elif ARDUINO_ARCH_SAM
  // Use TC3 - randomly selected, what will it do with servo?
  // TC3 = TC Group 1, channel 0
     pmc_set_writeprotect(false);                            // Power management controller (PMC) disables the write protection of the timer and counter registers
     pmc_enable_periph_clk((uint32_t)TC3_IRQn);              // Power management controller (PMC) enables peripheral clock for TC3

     setSamplingPeriod(microseconds);
     #ifdef ECHO_TO_SERIAL
	   if(!setSamplingPeriod(microseconds)) 
         Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
     #endif
	
     TC_Start(TC1, 0);                                 //  Start clock, for the TC1 block, channel 0 = TC3 
     TC1->TC_CHANNEL[0].TC_IER=TC_IER_CPCS;            //  enable interrupt for the TC1 block, channel 0 = TC3 
     TC1->TC_CHANNEL[0].TC_IDR=~TC_IER_CPCS;           //  remove disable interrupt for the TC1 block, channel 0 = TC3 
     NVIC_EnableIRQ(TC3_IRQn);                         //  ISR for the interrupt 
  #else
      #error "Architecture not supported."
  #endif                 
}

bool SamplingClass::setSamplingPeriod(unsigned long microseconds){
  const unsigned long cycles = microseconds * cpuFrequency;

  #ifdef ARDUINO_AVR_UNO    
	  // For AVR-based boards, e.g. Uno   
	  if (cycles < timerResolution){		                   // max. 16 us, error 62.5 ns 
		  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20); // no prescaling
		  OCR2A = cycles-1;                                  // compare match register
	  }
	  else if(cycles < timerResolution * 8){		   	       // max. 128 us, error 0.5 us
		  TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20); // 8 prescaler 
		  OCR2A = (cycles/8)-1;                              // compare match register
	  }
	  else if(cycles < timerResolution * 32){              // max. 512 us, error, 2 us
		  TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20); // 32 prescaler 
		  OCR2A = (cycles/32)-1;                             // compare match register
	  }
    else if(cycles < timerResolution * 64){              // max. 1024 us, error, 4 us
      TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20); // 64 prescaler 
      OCR2A = (cycles/64)-1;                             // compare match register
    }
    else if(cycles < timerResolution * 128){             // max. 2048 us, error, 8 us
      TCCR2B |= (1 << CS22) | (0 << CS21) | (1 << CS20); // 128 prescaler 
      OCR2A = (cycles/128)-1;                            // compare match register
    }
	  else if(cycles < timerResolution * 256){             // max. 4096 us, 16 us 
		  TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // 256 prescaler 
		  OCR2A = (cycles/256)-1;                            // compare match register
	  }
	  else if(cycles < timerResolution * 1024){	           // max. 16384 us, 64 us 
		  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler 
		  OCR2A = (cycles/1024)-1;                           // compare match register
	  }
    // Use 0.1 ms precision up to 100 ms
    else if(cycles < CYCLES_100MS){                      // max. 100 ms, error 0.1 ms
      TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20); // 8 prescaler 
      OCR2A = (COMPARE_100US)-1;                         // compare match register
      fireFlag = 1;                                      // repeat firing
      fireResolution = 100;                              // resolution in us
    }
    // Use 1 ms precision up to 1 s
    else if(cycles < CYCLES_1S){                         // max. 1 s, error 1 ms
      TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20); // 64 prescaler 
      OCR2A = (COMPARE_1MS)-1;                           // compare match register
      fireFlag = 1;                                      // repeat firing
      fireResolution = 1000;                             // resolution in us
    }    
    // Use ~10 ms precision over 1 s
	  else if(cycles >= CYCLES_1S){
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler 
      OCR2A = (COMPARE_10MS)-1;                          // compare match register
      fireFlag = 1;                                      // repeat firing
      fireResolution = 10000;                            // resolution in us
		}	
    
 #elif ARDUINO_AVR_MEGA2560 
	  // For AVR-based board - Mega, run on timer 5
  	if (cycles < timerResolution){                        // max. 4096 us, 62.5 ns 
		  TCCR5B |= (0 << CS52) | (0 << CS51) | (1 << CS50); // no prescaling
		  OCR5A = cycles-1;                                   // compare match register
	  }
	  else if(cycles < timerResolution * 8){                // max. 32768 us, 0.5 us 
		  TCCR5B |= (0 << CS52) | (1 << CS51) | (0 << CS50);  // 8 prescaler 
		  OCR5A = (cycles/8)-1;                               // compare match register
	  }
	  else if(cycles < timerResolution * 64){               // max. 262.144 ms, 4 us 
		  TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);  // 64 prescaler 
		  OCR5A = (cycles/64)-1;                              // compare match register
	  }
	  else if(cycles < timerResolution * 256){              // max. 1.048576 s, 16 us 
		  TCCR5B |= (1 << CS52) | (0 << CS51) | (0 << CS50);  // 256 prescaler 
		  OCR5A = (cycles/256)-1;                             // compare match register
	  }
	  else if(cycles < timerResolution * 1024){             // max. 4.194304 s, 64 us 
		  TCCR5B |= (1 << CS52) | (0 << CS51) | (1 << CS50); // 1024 prescaler 
		  OCR5A = (cycles/1024)-1;                            // compare match register
	  }
    
	  else if(cycles >= timerResolution * 1024){
      TCCR5B |= (0 << CS52) | (1 << CS51) | (0 << CS50);  // 8 prescaler 
      OCR5A = (COMPARE_10MS)-1;                           // compare match register to 10 ms
      fireFlag = 1;                                       // repeat firing
      fireResolution = 10000;                             // resolution in us
	  }
   
	
 #elif ARDUINO_ARCH_SAMD								  // For SAMD21G boards, e.g. Zero
    if (cycles < timerResolution){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;  // no prescaling
      TC5->COUNT16.CC[0].reg = (uint32_t)cycles-1;                  // compare match register
	 }
    else if(cycles < timerResolution * 2){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;  // 2 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/2)-1;              // compare match register
    }
    else if(cycles < timerResolution * 4){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4;  // 4 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/4)-1;              // compare match register
    }
    else if(cycles < timerResolution * 8){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;  //  8 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/8)-1;              // compare match register
    }
    else if(cycles < timerResolution * 16){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;  //  16 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/16)-1;              // compare match register
    }
    else if(cycles < timerResolution * 64){
        TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64;  //  64 prescaler
        TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/64)-1;              // compare match register
    }
	else if(cycles < timerResolution * 256){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;  //  256 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/256)-1;              // compare match register
    }
    else if(cycles < timerResolution * 1024){
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;  //  1024 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles/1024)-1;              // compare match register
    }
	else {
		return false;
	}
 
 #elif ARDUINO_ARCH_SAM 										// For SAM boards, e.g. DUE
    TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);        
    TC_SetRC(TC1, 0, (uint32_t)(cycles/2)-1);          // Prescaler 2       	
	
 #else
      #error "Architecture not supported."
 #endif 
  
  samplingPeriod=microseconds/1000000.0;               // in seconds
  samplingMicroseconds=microseconds;                    //in microseconds
  return true;               
}

float SamplingClass::getSamplingPeriod(){
  return samplingPeriod;
}

unsigned long int SamplingClass::getSamplingMicroseconds(){
  return samplingMicroseconds;
}

//unsigned short int SamplingClass::getFireResolution(){
//  return fireResolution;
//}



void SamplingClass::interrupt(p_to_void_func interruptCallback){
  this->interruptCallback = interruptCallback;
}

p_to_void_func SamplingClass::getInterruptCallback (){
  return interruptCallback;
}

SamplingClass Sampling;

#ifdef ARDUINO_AVR_UNO

ISR(TIMER2_COMPA_vect){
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

void TC3_Handler(void){
  TC_GetStatus(TC1, 0);
 (Sampling.getInterruptCallback())();
}

#else
  #error "Architecture not supported."

#endif
