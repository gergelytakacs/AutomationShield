/*
  Interrupt-driven sampling for real-time control.
  
  The file implements an interrupt-driven system for deploying
  digital control systems on AVR and ATSAMD-based Arduino 
  prototyping boards.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Koplinger 2018-2019
  AVR Timer: 	 Richard Koplinger, 2018
  SAMD21G Timer: Gergely Takacs, 2019
  Last update: 07.02.2019.
*/

#include "Sampling.h"

void SamplingClass::defaultInterrupt()
{
}

SamplingClass::SamplingClass(){

  interruptCallback=defaultInterrupt;
  samplingPeriod = 0.0;
}

void SamplingClass::interruptInitialize(unsigned long microseconds){

  noInterrupts();                             // disable all interrupts
  TCCR1A = 0;                                 // clear register
  TCCR1B = 0;                                 // clear register
  TCNT1  = 0;                                 // clear register
       
  TCCR1B |= (1 << WGM12);                     // CTC mode       
  TIMSK1 |= (1 << OCIE1A);                    // enable timer compare interrupt

  if(!setSamplingPeriod(microseconds)) 
    Serial.println("Sampling period is too long.\nMax is 4194303 microseconds.");       
  
  interrupts();             // enable all interrupts                
}

bool SamplingClass::setSamplingPeriod(unsigned long microseconds){

  const unsigned long cycles = microseconds * cpuFrequency;
        
  if (cycles < timerResolution){
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // no prescaling
    OCR1A = cycles-1;                                  // compare match register
  }
  else if(cycles < timerResolution * 8){
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler 
    OCR1A = (cycles/8)-1;                              // compare match register
  }
  else if(cycles < timerResolution * 64){
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // 64 prescaler 
    OCR1A = (cycles/64)-1;                             // compare match register
  }
  else if(cycles < timerResolution * 256){
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10); // 256 prescaler 
    OCR1A = (cycles/256)-1;                            // compare match register
  }
  else if(cycles < timerResolution * 1024){
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // 1024 prescaler 
    OCR1A = (cycles/1024)-1;                           // compare match register
  }
  else{
    return false;
  }
  samplingPeriod=microseconds/1000000.0;               // in seconds
  return true;               
}

float SamplingClass::getSamplingPeriod(){
  return samplingPeriod;
}

void SamplingClass::setInterruptCallback(p_to_void_func interruptCallback){
  this->interruptCallback = interruptCallback;
}

p_to_void_func SamplingClass::getInterruptCallback (){
  return interruptCallback;
}

SamplingClass Sampling;

ISR(TIMER1_COMPA_vect)
{
  (Sampling.getInterruptCallback())();
}
