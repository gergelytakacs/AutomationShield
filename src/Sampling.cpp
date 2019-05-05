/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR, SAMD and SAM-based Arduino 
  prototyping boards. This module should be compatible with
  all AVR boards (Uno, Mega 2560 etc.), Arduino Zero and Arduino
  Due. Take care of timer conflicts when using the Servo library.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Richard Koplinger 2018-2019
  AVR Timer: 	 Richard Koplinger, 2018
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 11.02.2019.
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
  #ifdef ARDUINO_ARCH_AVR
  	// For AVR-based boards, e.g. Uno  
    noInterrupts();                             // disable all interrupts
    TCCR2A = 0;                                 // clear register
    TCCR2B = 0;                                 // clear register
    TCNT2  = 0;                                 // clear register
       
    TCCR2B |= (1 << WGM22);                     // CTC mode       
    TIMSK2 |= (1 << OCIE2A);                    // enable timer compare interrupt
  
    if(!setSamplingPeriod(microseconds)) 
    Serial.println("Sampling period is too long.\nMax is 4194303 microseconds.");       
  
    interrupts();             // enable all interrupts
   
   #elif ARDUINO_AVR_MEGA2560 //defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
 	 // For AVR-based board - Mega, run on timmer 5
    noInterrupts();                             // disable all interrupts
    TCCR5A = 0;                                 // clear register
    TCCR5B = 0;                                 // clear register
    TCNT5  = 0;                                 // clear register
       
    TCCR5B |= (1 << WGM52);                     // CTC mode       
    TIMSK5 |= (1 << OCIE5A);                    // enable timer compare interrupt
  
    if(!setSamplingPeriod(microseconds)) 
    Serial.println("Sampling period is too long.\nMax is 4194303 microseconds.");  
#warning "running on timmer 5"
	
  #elif ARDUINO_ARCH_SAMD
  // For SAMD21G boards, e.g. Zero
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
     GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;        // Set Timer counter Mode to 16 bits
	 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;        // Set TC5 mode as match frequency
     
     if(!setSamplingPeriod(microseconds)) 
     Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
     
	 NVIC_EnableIRQ(TC5_IRQn);                               // Enable interrupt for TC5
     TC5->COUNT16.INTENSET.bit.MC0 = 1;                      // Enable the TC5 interrupt request
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;              // Enable timer
  #elif ARDUINO_ARCH_SAM
  // Use TC3 - randomly selected, what will it do with servo?
  // TC3 = TC Group 1, channel 0
     pmc_set_writeprotect(false);                            // Power management controller (PMC) disables the write protection of the timer and counter registers
     pmc_enable_periph_clk((uint32_t)TC3_IRQn);              // Power management controller (PMC) enables peripheral clock for TC3
	
	 if(!setSamplingPeriod(microseconds)) 
     Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
 
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
	  if (cycles < timerResolution){
		TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20); // no prescaling
		OCR2A = cycles-1;                                  // compare match register
	  }
	  else if(cycles < timerResolution * 8){
		TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20); // 8 prescaler 
		OCR2A = (cycles/8)-1;                              // compare match register
	  }
	  else if(cycles < timerResolution * 64){
		TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20); // 64 prescaler 
		OCR2A = (cycles/64)-1;                             // compare match register
	  }
	  else if(cycles < timerResolution * 256){
		TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20); // 256 prescaler 
		OCR2A = (cycles/256)-1;                            // compare match register
	  }
	  else if(cycles < timerResolution * 1024){
		TCCR2B |= (1 << CS22) | (0 << CS21) | (1 << CS20); // 1024 prescaler 
		OCR2A = (cycles/1024)-1;                           // compare match register
	  }
	  else{
		return false;
	  }	
	    #elif ARDUINO_AVR_MEGA2560 // defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  // For AVR-based board - Mega, run on timmer 5
  	  if (cycles < timerResolution){
		TCCR15B |= (0 << CS52) | (0 << CS51) | (1 << CS50); // no prescaling
		OCR5A = cycles-1;                                  // compare match register
	  }
	  else if(cycles < timerResolution * 8){
		TCCR5B |= (0 << CS52) | (1 << CS51) | (0 << CS50); // 8 prescaler 
		OCR5A = (cycles/8)-1;                              // compare match register
	  }
	  else if(cycles < timerResolution * 64){
		TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50); // 64 prescaler 
		OCR5A = (cycles/64)-1;                             // compare match register
	  }
	  else if(cycles < timerResolution * 256){
		TCCR5B |= (1 << CS52) | (0 << CS51) | (0 << CS50); // 256 prescaler 
		OCR5A = (cycles/256)-1;                            // compare match register
	  }
	  else if(cycles < timerResolution * 1024){
		TCCR15B |= (1 << CS52) | (0 << CS51) | (1 << CS50); // 1024 prescaler 
		OCR5A = (cycles/1024)-1;                           // compare match register
	  }
	  else{
		return false;
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
	else{
		return false;
	}
#elif ARDUINO_ARCH_SAM 										// For SAM boards, e.g. DUE
    TC_Configure(TC1, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);        
    TC_SetRC(TC1, 0, (uint32_t)(cycles/2)-1);          // Prescaler 2       	
	
#else
      #error "Architecture not supported."
#endif 
  
  samplingPeriod=microseconds/1000000.0;               // in seconds
  return true;               
}

float SamplingClass::getSamplingPeriod(){
  return samplingPeriod;
}

void SamplingClass::interrupt(p_to_void_func interruptCallback){
  this->interruptCallback = interruptCallback;
}

p_to_void_func SamplingClass::getInterruptCallback (){
  return interruptCallback;
}

SamplingClass Sampling;

#ifdef ARDUINO_ARCH_AVR

ISR(TIMER1_COMPA_vect)
{
  (Sampling.getInterruptCallback())();
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
