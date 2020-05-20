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
#ifndef MOTOSHIELD_H_ //--don't define if its already defined
#define MOTOSHIELD_H_

#include "Arduino.h"    //--including basic Arduino library

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
	void setDirection(bool direction = true);
	void begin(float _Ts = 50.0);
	void calibration();
	void actuatorWrite(float percentValue);
	void actuatorWriteVolt(float voltageValue);
	float referenceRead();
	float sensorReadRPM();
	float sensorReadRPMPerc();
    float sensorReadVoltage();
	float sensorReadVoltageAmp1();
	float sensorReadVoltageAmp2();
	float sensorReadCurrent();
   	static inline volatile uint16_t count; //--counting pulses of hall sensor encoder
    static inline volatile uint16_t counted; //--memorizing number of pulses per sample
	static inline volatile bool stepEnable; //--auxiliary variable # simplifies the creation of examples
	uint32_t minRPM, minDuty, maxRPM; //--calibration variables
	float minVolt;
	
  private:
    static void _InterruptServiceRoutine();
    static void _InterruptSample();
	float _K; //--number of samples in one minute
  };
extern MotoShieldClass MotoShield;
#endif
