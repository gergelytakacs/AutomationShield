#include "MotoShield.h"


void MotoClass::begin(void){ // begin function initializes the pins, sets the ISR and sets initial values of some variables 
 pinMode(MOTO_UPIN,OUTPUT);
 pinMode(MOTO_IN1,OUTPUT);
 pinMode(MOTO_IN2,OUTPUT);
 pinMode(MOTO_C2,INPUT_PULLUP);
 pinMode(MOTO_C1,INPUT_PULLUP);
 pinMode(MOTO_RPIN,INPUT);
 pinMode(MOTO_Vout2,INPUT);
 pinMode(MOTO_Vout1,INPUT);
 pinMode(MOTO_U1,INPUT);
 pinMode(MOTO_U2,INPUT); 

  attachInterrupt(digitalPinToInterrupt(MOTO_C1), MotoShield.countTicks, FALLING);  // setting the interrupts, it reacts, when the edge is falling

  counter = 0;             // setting the intial value of the variables
} // end of the begin() function

void MotoClass::countTicks(){          // ISR - Interrupt Service Routine
  Bstate = digitalRead(MOTO_C2);
  counter ++ ;
}

void MotoClass::motorON(){  // sets the motor's speed to maximum (switches on the motor at full speed)
  analogWrite(MOTO_UPIN,255);
}

void MotoClass::motorOFF(){  // switches off the motor 
  digitalWrite(MOTO_UPIN,LOW);
}

void MotoClass::setMotorSpeed(float value){    // sets the speed of the motor, input value from 0-100
    convertedValue = AutomationShield.mapFloat(value,0.00,100.00,0.00,255.00);
    analogWrite(MOTO_UPIN,convertedValue);   
}

void MotoClass::setDirection(bool dir){    // sets the direction of the rotation
  if(dir){                  // if dir = true, the direction is counter-clockwise             
digitalWrite(MOTO_IN1,HIGH);
digitalWrite(MOTO_IN2,LOW);
  }
 else {        // if dir = false, the direction is clockwise
digitalWrite(MOTO_IN1,LOW);
digitalWrite(MOTO_IN2,HIGH);  
 }  
} // end of the  setDirection() function


void MotoClass::revDirection(){  // reverses the pre-set direction, use only after setDirection() function
Direction = digitalRead(MOTO_IN1);
  if(Direction){                // the direction is clockwise
digitalWrite(MOTO_IN1,LOW);
digitalWrite(MOTO_IN2,HIGH);
  }

  else{        // the direction is counter-clockwise
digitalWrite(MOTO_IN1,HIGH);
digitalWrite(MOTO_IN2,LOW);
  } 
} // end of the revDirection() function

float MotoClass::referenceRead(){           // referenceRead function returns the reference value of the potentiometer in percents, which was set by the user
   _referenceRead = analogRead(MOTO_RPIN);
   referenceValue = AutomationShield.mapFloat(_referenceRead,0.00,1023.00,0.00,100.00);
  return referenceValue;
}

float MotoClass::readVoltage(){ // reads and returns the voltage drop through the resistor R
  ADC1 = analogRead(MOTO_U1);
  ADC2 = analogRead(MOTO_U2);

  if(ADC1 > ADC2){
  ADCU = ADC1 - ADC2;
  }

  if(ADC2 > ADC1){
    ADCU = ADC2 - ADC1;
  }

  k = (5.00 / 1023.00); // constant for converting analog values to Voltage

  V = ADCU * k;

 return V;
} // end of the readVoltage() function


float MotoClass::readCurrent(){  // returns the current draw of the motor in mA 
 R = 10;
 float callVolt = MotoShield.readVoltage();
  I = (callVolt / float(R)) * 1000.00;  // *1000.00 to have the value in mA
  
  return I;
} // end of the readCurrent() function


float MotoClass::durationTime(){         // returns the duration of one revolution in ms
  rev = 2660;        // counter value of one revolution               
  count = counter;   
  cValue = count - previousCount;
  
  t = millis();   // measuring the running time since the last reset
  
  if(cValue >= rev){      // if the counter value is higher or equal than one revolution
   revTime = t;
   durTime = revTime - prevTime;
   previousCount = count;  
  }
    prevTime = revTime;
    
  return durTime;
} // end of the durationTime() function

float MotoClass::readRevolutions(){ // returns the number of revolutions per minute
 duration = MotoShield.durationTime();
 
 revolutions = (60000.00) / duration ;

return revolutions;
} // end of the readRevolutions() function


MotoClass MotoShield; // Construct instance (define)




