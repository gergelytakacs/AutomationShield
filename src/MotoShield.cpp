#include "MotoShield.h"
#include "AutomationShield.h"
#include "Sampling.h"

void MotoShieldClass::_InterruptServiceRoutine(){ //--ISR for the incremental encoder
	count++; 
}

void MotoShieldClass::_InterruptSample(){ //--ISR for each sample
	counted = count;	//--memorizing number of ticks per given sample
	count = 0; 			  //--resetting count variable for next the sample
	stepEnable = true; //--auxiliary variable
}

// Sets the direction motor - clockwise if input parameter is true
//                          - counterclockwise if input parameter is false
bool MotoShieldClass::setDirection(bool direction = true) { //--default direction - clockwise
  if (direction) { 	 //--clockwise      
    digitalWrite(MOTO_DIR_PIN1, 0);
    digitalWrite(MOTO_DIR_PIN2, 1);
    return 1;			 //--method returns 1 if direction is set clockwise
  }
  else {             //--counter-clockwise 
    digitalWrite(MOTO_DIR_PIN1, 1);
    digitalWrite(MOTO_DIR_PIN2, 0);
    return 0; 		//--method returns 0 if direction is set counter-clockwise
  }
}

void MotoShieldClass::begin(float _Ts = 50.0){ //--default sample duration Ts=50 millis
  pinMode(MOTO_YPIN1, INPUT); 	//--initialize hardware pins
  pinMode(MOTO_YPIN2, INPUT);
  pinMode(MOTO_RPIN, INPUT);
  pinMode(MOTO_UPIN, OUTPUT);
  pinMode(MOTO_DIR_PIN1,OUTPUT);
  pinMode(MOTO_DIR_PIN2,OUTPUT); 
  setDirection(true); 		      	 //--making setDirection() method non-mandatory
  Sampling.period(_Ts*1000.0); //--defining sample duration in millisec
  _K=60000.0/_Ts; 				        //--minute to millisec / sample duration
  Sampling.interrupt(_InterruptSample);//--attaching sampling ISR
  attachInterrupt(digitalPinToInterrupt(MOTO_YPIN1), _InterruptServiceRoutine, CHANGE);//--attaching ISR for incremental encoder # mode-FALLING
}

float MotoShieldClass::referenceRead() {//--Reads potentiometers reference value and returns it in percentage
  return AutomationShield.mapFloat((float)analogRead(MOTO_RPIN), 0.0, 1023.0, 0.0, 100.0);
}

void MotoShieldClass::actuatorWrite(float percentValue) {//--duty cycle for the Motor, determining angular velocity
	if(percentValue < _minDuty && percentValue != 0) percentValue = _minDuty; //--When input is 0, motor is disabled
	analogWrite(MOTO_UPIN, AutomationShield.percToPwm(percentValue));
}
	
 //--sensing minimal duty cycle for motor to spin
//--sensing RPM at maximal PWM and at minimal PWM
void MotoShieldClass::calibration(){
	actuatorWrite(100);
	delay(1000); //--delay for motor to reach maximal angular velocity
	_maxRPM = sensorReadRPM(); //--sensing max RPM
	actuatorWrite(0); //--disabling the motor
	delay(500); //--delay for motor to stop spinning
	int i = 25; //--initial PWM in %, presumed potentionally minimal
	do{		//--infinite loop
		  actuatorWrite(i);
		  delay(1250); //--delay for motor to spin at least one revolution
		  if(counted >= 7){ //--condition for detecting motor motion	
		    	_minDuty = i; //--minimal duty cycle in %
		    	_minRPM = sensorReadRPM();//--sensing minimal RPM
			    actuatorWrite(0);//--disabling the motor
		    	break; //--end of loop
		   }
	    i++; //--incrementing duty cycle variable by 1
	}while(1);
}

float MotoShieldClass::sensorReadRPMPerc(){//--Sensing RPM in %
return AutomationShield.constrainFloat(AutomationShield.mapFloat(MotoShield.sensorReadRPM(), (float)_minRPM, (float)_maxRPM, 0.0, 100.0),0.0,100.0);
}

float MotoShieldClass::sensorReadRPM(){//--Sensing RPM 
	return (float)counted/PPR*_K;        //--7 is the number of ticks per one rotation
}

float MotoShieldClass::sensorReadVoltage() {//--Voltage drop after the shunt(10ohm) resistor
  return (analogRead(MOTO_YV1)-analogRead(MOTO_YV2))*5.0/1023.0;
}

float MotoShieldClass::sensorReadVoltageAmp1() {//--Output from differential OpAmp in Volts
  return analogRead(MOTO_YAMP1)*5.0/1023.0; 
}

float MotoShieldClass::sensorReadVoltageAmp2() {     //--Output from non-inverting OpAmp in Volt
  return analogRead(MOTO_YAMP2)*5.0/1023.0/AMP_GAIN;//--Gain of OpAmp is 2.96 
}

float MotoShieldClass::sensorReadCurrent() {//--Current calculation based on non-inverting OpAmp output - Ohms law
  return MotoShield.sensorReadVoltageAmp2()/RES*1000.0;//--R represents 10ohms - shunt resistor # 1000 - conversion to mA
}

MotoShieldClass MotoShield; //--Creating an object in MotoShieldClass 
