#include "MagnetoShield.h"


void MagnetoShieldClass::begin() 
	{
	pinMode(HallSenzor,INPUT);	//sets pin A3 defined in .h file asi INPUT
	Wire.begin();				//start "Wire" library
	}

uint8_t MagnetoShieldClass::setVoltageV(float voltage)
{
	if(voltage > 5.00){ 				 	//DA convertor is able to create just 0-5 V
		voltage = 5.00;}
	else if (voltage < 0.00){
		voltage = 0.00;}
	float voltageF = voltage*255.00/5.00;	// conversion % -> float
	uint8_t voltageB = (uint8_t) voltageF;	// conversion float -> 8-bit number
	Wire.beginTransmission(PCF8591);	  	// I2C transmission to DA convertor
	Wire.write(0x40);
	Wire.write(voltageB);
	Wire.endTransmission();
	return voltageB;
}

void MagnetoShieldClass::calibration()
	{
	Wire.beginTransmission(PCF8591);	// I2C transmission
	Wire.write(0x40);
	Wire.write(0);
	Wire.endTransmission();
	delay(1000);						//waiting for conditious
	Min = analogRead(HallSenzor);		//lowest position of flying from hall senzor
	
	Wire.beginTransmission(PCF8591);	// I2C transmission
	Wire.write(0x40);
	Wire.write(255);
	Wire.endTransmission();
	delay(1000);						//waiting for conditious
	Max = analogRead(HallSenzor);		//highest position of flying from hall senzor
	
	Wire.beginTransmission(PCF8591);	// Sets take-off point on ground. Also posible to start levitation from upper position
	Wire.write(0x40);
	Wire.write(0);
	Wire.endTransmission();
	delay(1000);
	}	

int MagnetoShieldClass::getMin()
{
	int variable = (int) Min;	//conversion float->int
	return variable;
}	

int MagnetoShieldClass::getMax() 
{
	int variable = (int) Max;	//conversion float->int
	return variable;
}	

float MagnetoShieldClass::readHeight()
{
	float Height = analogRead(HallSenzor);	//current position of flying from hall senzor
	return Height;
}

float MagnetoShieldClass::setHeight(float set) 
{
	setpoint = AutomationShield.mapFloat(set,0.00,100.00,Min,Max); 	//recalculate position of levitation from % to float number in interval <Min;Max>
	return setpoint;												//variabl for error() function   
}

void MagnetoShieldClass::setVoltage(float u)
{
	uint8_t u8B = (uint8_t) u;			//conversion float->uint8_u
	Wire.beginTransmission(PCF8591);	// I2C transmission on DAC
	Wire.write(0x40);
	Wire.write(u8B);
	Wire.endTransmission();
}

float MagnetoShieldClass::error()
{
	float position = analogRead(HallSenzor);	//current position of flying from hall senzor
	float err = setpoint-position;				//setpoint from setHeight() function
	return err;
}


MagnetoShieldClass MagnetoShield;	//construct instance (define)
