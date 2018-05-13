#ifndef MagnetoShield_h
#define MagnetoShield_h

#include "AutomationShield.h"

#include "Wire.h" //needed for DA convertor

#define PCF8591 (0x90 >> 1)
#define HallSenzor A3

	 


class MagnetoShieldClass
{
public:

	void begin(); 								//inicialization of pins + include "Wire" library
	uint8_t setVoltageV(float voltagePer);		//sets voltage 0-5 V on DAC and gives back as 8-bit number
	float readHeight(); 						//current position of flying
	float setHeight(float set);					//sets desired position of flying in % 0-100 and give back as 10-bit number
	void calibration();							//finds out lowest and highest possitions of magnet
	int getMin();								//function witch return 10-bit value of lowest possition of magnet
	int getMax();								//function witch return 10-bit value of highest possition of magnet
	float error();								//diference between desired and current position of flying -> for PID
	void setVoltage(float u);					//sets voltage on DAC as 8-bit number 0=>0 V; 255=>5 V
	
	
private:
	 							
	float Min;									//10-bit value of min position of flying
	float Max;									//10-bit value of max position of flying
	float setpoint;								//desired value of flying, variable for error function 
};

extern MagnetoShieldClass MagnetoShield;		//declare external instance
#endif