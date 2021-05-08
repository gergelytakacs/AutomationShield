/*
  API for the MotoShield didactic hardware.

  The file is a part of the application programmers interface for
  the MotoShield didactic device for control engineering.
  The MotoShield includes a DC Motor with Hall incremental encoder
  and current measuring periphery.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Ján Boldocký.
  Last update: 23.4.2020.
*/
#ifndef MOTOSHIELD_H //--don't define if its already defined
#define MOTOSHIELD_H
#include "Arduino.h"    //--including basic Arduino library
#include "AutomationShield.h"
#include"Sampling.h"
//--defining pins for MotoShield
#define MOTO_YPIN1      3       //--hallsensor pin channel 1
#define MOTO_YPIN2      4      //--hallsensor pin channel 2 
#define MOTO_UPIN       5     //--PWM ~pin for the Motor
#define MOTO_DIR_PIN1   6    //--direction pin1 for H-bridge
#define MOTO_DIR_PIN2   7   //--direction pin2 for H-bridge
#define MOTO_RPIN       A4  //--analog pin for potentiometer reference
#define MOTO_YV1		A0     //--analog pin for voltage before shunt resistor
#define MOTO_YV2		A1    //--analog pin for voltage after shunt resistor
#define MOTO_YAMP1		A2   //--analog pin for output from differential OpAmp
#define MOTO_YAMP2		A3  //--analog pin for output from non-inverting OpAmp

//--defining constants
#define AMP_GAIN		2.96  //--Gain of non-inverting OpAmp
#define SHUNT			10.0 //--resistance of shunt resistor

class MotoShieldClass{ //--creating a class for the MotoShield
  public:
  	void setDirection(bool);
	void begin(float);
	void calibration();
	void actuatorWrite(float);
	void actuatorWriteVolt(float);
	float referenceRead();
  	float sensorRead();//--Default sensor read method: sensorReadRPMPerc();
	float sensorReadRPM();
	float sensorReadRPMPerc();
  	float sensorReadVoltage();
	float sensorReadVoltageAmp1();
	float sensorReadVoltageAmp2();
	float sensorReadCurrent();
	uint32_t minRPM, minDuty, maxRPM; //--calibration variables
	float minVolt;
	static volatile uint16_t count; //--counting pulses of hall sensor encoder
  	static volatile uint16_t counted; //--memorizing number of pulses per sample
    	static volatile bool stepEnable; //--auxiliary variable # simplifies the creation of examples
  private:
	float _K; //--number of samples in one minute
};

volatile uint16_t MotoShieldClass::count; //--initializing static variables
volatile uint16_t MotoShieldClass::counted;
volatile bool MotoShieldClass::stepEnable; 
MotoShieldClass MotoShield; //--Creating an object in MotoShieldClass 

// Sets the direction motor - clockwise if input parameter is true
//                          - counterclockwise if input parameter is false
void MotoShieldClass::setDirection(bool direction = true) { //--default direction - clockwise
  if (direction) {   //--clockwise      
    digitalWrite(MOTO_DIR_PIN1, 0);
    digitalWrite(MOTO_DIR_PIN2, 1);
  }
  else {             //--counter-clockwise 
    digitalWrite(MOTO_DIR_PIN1, 1);
    digitalWrite(MOTO_DIR_PIN2, 0);
  }
}

void MotoShieldClass::begin(float _Ts = 50.0){ //--default sample duration Ts=50 millis
  pinMode(MOTO_UPIN, OUTPUT); 
  pinMode(MOTO_DIR_PIN1,OUTPUT);
  pinMode(MOTO_DIR_PIN2,OUTPUT); 
  pinMode(MOTO_YPIN1, INPUT);   
  pinMode(MOTO_YPIN2, INPUT); //--initialize hardware pins
  pinMode(MOTO_RPIN, INPUT);
  pinMode(MOTO_YV1, INPUT);
  pinMode(MOTO_YV2, INPUT);
  pinMode(MOTO_YAMP1, INPUT);
  pinMode(MOTO_YAMP2, INPUT);
  setDirection(true);              //--making setDirection() method non-mandatory
  Sampling.period(_Ts*1000.0); //--defining sample duration in millisec
  _K=60000.0/_Ts;                 //--minute to millisec / sample duration
  Sampling.interrupt([]{counted = count;count = 0;stepEnable = true;});//--attaching sampling ISR
  attachInterrupt(digitalPinToInterrupt(MOTO_YPIN1),[]{count++;}, CHANGE);//--attaching ISR for incremental encoder # mode-FALLING
}

float MotoShieldClass::referenceRead() {//--Reads potentiometers reference value and returns it in percentage
  return AutomationShield.mapFloat((float)analogRead(MOTO_RPIN), 0.0, 1023.0, 0.0, 100.0);
}

void MotoShieldClass::actuatorWrite(float percentValue) {//--duty cycle for the Motor, determining angular velocity
  if(percentValue < minDuty && percentValue != 0) percentValue = minDuty; //--When input is 0, motor is disabled
  analogWrite(MOTO_UPIN, AutomationShield.percToPwm(percentValue));
}

void MotoShieldClass::actuatorWriteVolt(float voltageValue){ //--Expects Root Mean Square Voltage value as input  
  if(voltageValue < minVolt != 0) voltageValue = minVolt; 
  analogWrite(MOTO_UPIN,sq(voltageValue)*255.0/sq(AREF)); //--Convert Urms -> PWM duty 8-bit
}
  
//--sensing minimal duty cycle for motor to spin
//--sensing RPM at maximal PWM and at minimal PWM
void MotoShieldClass::calibration(){
  actuatorWrite(100);
  delay(1000); //--delay for motor to reach maximal angular velocity
  maxRPM = counted; //--sensing max RPM
  actuatorWrite(0); //--disabling the motor
  delay(500); //--delay for motor to stop spinning
  int i = 15; //--initial PWM in %, presumed potentionally minimal
  do{   //--infinite loop
      actuatorWrite(i);
      delay(300); //--delay for motor to spin at least one revolution
      if(counted >= 4){ //--condition for detecting motor motion  
          delay(1000);//--delay for motor to reach minimal angular velocity
        minDuty = i; //--minimal duty cycle in %
          minRPM = counted;//--sensing minimal RPM
          minVolt = AREF * sqrt(minDuty/100.0); //--Calculate minimal voltage for Motor to spin
        actuatorWrite(0);//--disabling the motor
          break; //--end of the loop
       }
      i++; //--incrementing duty cycle variable by 1
  }while(1);
}

float MotoShieldClass::sensorReadRPMPerc(){//--Sensing RPM in %
return AutomationShield.constrainFloat(AutomationShield.mapFloat(counted, (float)minRPM, (float)maxRPM, 0.0, 100.0),0.0,100.0);
}

float MotoShieldClass::sensorRead(){//--Wrapper for default sensor read method
return MotoShield.sensorReadRPMPerc();
}

float MotoShieldClass::sensorReadRPM(){//--Sensing RPM 
  return (float)counted/14.0*_K;        //--14 is the number of ticks per one rotation
}

float MotoShieldClass::sensorReadVoltage() {//--Voltage drop after the shunt(10ohm) resistor
  return fabs((analogRead(MOTO_YV2)-analogRead(MOTO_YV1))*5.0/1023.0);
}

float MotoShieldClass::sensorReadVoltageAmp1() {//--Output from differential OpAmp in Volts
  return analogRead(MOTO_YAMP1)*5.0/1023.0; 
}

float MotoShieldClass::sensorReadVoltageAmp2() {     //--Output from non-inverting OpAmp in Volt
  return analogRead(MOTO_YAMP2)*5.0/1023.0/AMP_GAIN;//--Gain of OpAmp is 2.96 
}

float MotoShieldClass::sensorReadCurrent() {//--Current draw [mA] calculation based on non-inverting OpAmp output - Ohms law
  return MotoShield.sensorReadVoltageAmp2()/SHUNT*1000.0;//--R represents 10ohms - shunt resistor # 1000 - conversion to mA
}
#endif
