#include "Sampling.h"
#include "Arduino.h"

bool StepEnable = false;




void Sampling::samplingTime(float Ts){


float  Lt = 65535;    // the maximal value which can be stored in the Timer1 on Arduino Uno (16 bit timer,  Lt = 2^16 -1)
 
 float Tr1 = 0.0625;   // resolution at p = 1
 float Tr8 = 0.5;      // resolution at p = 8
 float Tr64 = 4;       // resolution at p = 64
 float Tr256 = 16;     // resolution at p = 256
 float Tr1024 = 64;    // resolution at p = 1024

 float resolution[] = {Tr1 , Tr8 , Tr64 , Tr256 , Tr1024}; // array which contains the available resolutions, helps to set the prescaler

 float uTs = (Ts * 1000.00); // sampling Time given by the user in microseconds (us)

for(_i = 0; _i < 5; _i++){               // the for loop determines the test value and the i, knowing the value of these varaibles is essential to choose the right prescaler
  _testValue = uTs / resolution[_i];
  if(_testValue <= Lt){
    break;
  } // end of the if statement
  } // end of the for loop


  // ************* Setting the bits of the Timer1 ***************

TCCR1A = 0;                   // resets all bits
TCCR1B = 0;
TCNT1  = 0;
cli();                        // disabling global interrupts

OCR1A = _testValue - 1;       // max. value when it has to reset

TCCR1B |= (1 << WGM12);      // CTC mode
  

// choosing the right prescaler


switch (_i){

  case 0:
  TCCR1B |= (1 << CS10);       // prescaler 1 (basic)
  
  TCCR1B |= (0 << CS11);       // setting 0 the other bits
  TCCR1B |= (0 << CS12); 
  break;  

  case 1:
  TCCR1B |= (1 << CS11);       // prescaler 8 

  TCCR1B |= (0 << CS10);
  TCCR1B |= (0 << CS12);
  break;

  case 2:
  TCCR1B |= (1 << CS10);       // prescaler 64
  TCCR1B |= (1 << CS11);

  TCCR1B |= (0 << CS12);
  break;

  case 3:
  TCCR1B |= (1 << CS12);       // prescaler 256 

  TCCR1B |= (0 << CS10);
  TCCR1B |= (0 << CS11);
  break;

  case 4:
  TCCR1B |= (1 << CS10);       // prescaler 1024
  TCCR1B |= (1 << CS12);  

  TCCR1B |= (0 << CS11);
  break;

  default:
  Serial.println("The sampling time is higher than the limit, please try smaller value");
  
} 

// when the prescaler is chosen we have to enable the interrupts

TIMSK1 |= (1 << OCIE1A);    // enable timer interrupts
interrupts();               // enable global interrupts

  
}// end of the sampling function


ISR(TIMER1_COMPA_vect){   // pre-set interrupt which checks the value of the indicator. when the Timer register fills up, it changes from false to true
  StepEnable = true;
  }


  
Sampling testSample;
