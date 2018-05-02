#include "Sampling.h"

void defaultInterrupt()
{
}

SamplingClass::SamplingClass(){

  interruptCallback=defaultInterrupt;
}

void SamplingClass::interruptInitialize(unsigned long microseconds){

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
       
  TCCR1B |= (1 << WGM12);   // CTC mode       
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  if(!setSamplingPeriod(microseconds)) 
    Serial.println("Sampling period is too long.\nMax is 4194303 microseconds.");       
  
  interrupts();             // enable all interrupts                
}

bool SamplingClass::setSamplingPeriod(unsigned long microseconds){

  const unsigned long cycles = microseconds * cpuFrequence;
        
  if (cycles < timer1Resolution){
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // no prescaling
    OCR1A = cycles-1;            // compare match register
  }
  else if(cycles < timer1Resolution * 8){
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler 
    OCR1A = (cycles/8)-1;            // compare match register
  }
  else if(cycles < timer1Resolution * 64){
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // 64 prescaler 
    OCR1A = (cycles/64)-1;            // compare match register
  }
  else if(cycles < timer1Resolution * 256){
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10); // 256 prescaler 
    OCR1A = (cycles/256)-1;            // compare match register
  }
  else if(cycles < timer1Resolution * 1024){
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // 1024 prescaler 
    OCR1A = (cycles/1024)-1;            // compare match register
  }
  else{
    return false;
  }
  samplingPeriod=microseconds/1000000.0; //in seconds
  return true;               
}

float SamplingClass::getSamplingPeriod(){
  return samplingPeriod;
}

void SamplingClass::setInterruptCallback(p_to_void_func isr){
  interruptCallback = isr;        
}

p_to_void_func SamplingClass::getInterruptCallback (){
  return interruptCallback;
}

SamplingClass Sampling;

ISR(TIMER1_COMPA_vect)
{
  (Sampling.getInterruptCallback())();
}
