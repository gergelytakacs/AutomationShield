#include "Timer.h"

Timer timer;

void Timer::interruptInitialize(unsigned long microseconds){

        noInterrupts();           // disable all interrupts
        TCCR1A = 0;
        TCCR1B = 0;
        TCNT1  = 0;
       
        TCCR1B |= (1 << WGM12);   // CTC mode       
        TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

        if(!setSamplingPeriod(microseconds)){
          Serial.println("Sampling period is too long");
        }
          
        interrupts();             // enable all interrupts
}

bool Timer::setSamplingPeriod(unsigned long microseconds){

        const unsigned long cycles = microseconds * cpuFrequence;
        
        if (cycles < timer1Resolution){
          TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // no prescaling
          OCR1A = cycles;            // compare match register
        }
        else if(cycles < timer1Resolution * 8){
          TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler 
          OCR1A = cycles/8;            // compare match register
        }
        else if(cycles < timer1Resolution * 64){
          TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // 64 prescaler 
          OCR1A = cycles/64;            // compare match register
        }
        else if(cycles < timer1Resolution * 256){
          TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10); // 256 prescaler 
          OCR1A = cycles/256;            // compare match register
        }
        else if(cycles < timer1Resolution * 1024){
          TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // 1024 prescaler 
          OCR1A = cycles/1024;            // compare match register
        }
        else{
          return false;
        }
        samplingPeriod=microseconds;
        return true;               
}

unsigned long Timer::getSamplingPeriod(){

  return samplingPeriod;
}

void Timer::setInterruptCallback(p_to_void_func isr){
        interruptCallback = isr;        
}

p_to_void_func Timer::getInterruptCallback (){
        return interruptCallback;
}

ISR(TIMER1_COMPA_vect)
{
  (timer.getInterruptCallback())();
}
