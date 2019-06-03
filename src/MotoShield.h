#ifndef MotoShield_h
#define MotoShield_h

#include "AutomationShield.h"

// Defining the hardware pins
#define MOTO_C1    3  // 1st channel of the Hall sensor
#define MOTO_C2    4  // 2nd channel of the Hall sensor 
#define MOTO_UPIN  5  // EN1
#define MOTO_IN1   6  // IN1
#define MOTO_IN2   7  // IN2
#define MOTO_RPIN  4  // Reference (potentiometer)
#define MOTO_Vout2 3  // output of the 2nd opamp (non-inverting)
#define MOTO_Vout1 2  // output of the 1st opamp (subtractor)
#define MOTO_U1    1  // U1 from the resistor - for measuring the voltage drop
#define MOTO_U2    0  // U2 from the resistor - for measuring the voltage drop


// Defining global variables
  static int Bstate;                   // gobal variables for ISR + volatile static 
  static unsigned long counter;



/* 
  Our hall sensor has 7 pole-pairs. I set the trigger mode as FALLING, which means, that the counter's value increases by one every time when the signal drops from high state to low. 
   One rotation of the back shaft takes 7counts. The ratio between the shafts is 380:1 , it means, that 1 rotation of the main shaft takes 380 rotations of the back shaft. During one 
   rotation of the main shaft the counter's value reaches 7*380 = 2660.
   */

   class MotoClass{
public:
	// Methods
	static void begin(void);   // it has to be static because of the attachInterrupt() - initializes the pins, sets the ISR and also the initial valus of teh counter variable
	static void countTicks();   // ISR - Interrupt Service Routine
	void motorON();    // switches on the motor at maximum speed (recommended)
	void motorOFF();   // switches off the motor
	void setMotorSpeed(float value);   // sets the speed of the motor, input value 0-100 (%)
	void setDirection(bool dir);   // sets the direction, input values: true - counter clockwise, false - clockwise
	void revDirection();   // reverses the pre-set direction
	float referenceRead();  // reads the value of the POT
	float readVoltage();  // returns the value of the voltage drop through the R, value is in Volts
	float readCurrent();  // returns the current draw ot the motor in mA
	float durationTime();  // returns the duration of one revolution in ms
	float readRevolutions(int Time);  // returns the number of revolutions per minute according to the time revolution time in ms (good accuracy)

private:
	float convertedValue;
	float _referenceRead;
	float referenceValue;
	float ADCR1;
	float ADCR2;
	float ADCU;
	float V;
	float I;
	bool  dir;
	int Direction;
	int cValue;
	int rev;
	unsigned long t;
	unsigned long revTime;
	unsigned long count;
	unsigned long durTime;
	unsigned long prevTime;
	unsigned long previousCount;
	float Revolutions;
	float percentage;
	float maxRev;
	float Compare;
	float value;
	float rValue;
	unsigned long Count;
	unsigned long prevC;
	int h;
	float constant;
	float REV;
	int Time;

	// Private constants
	 float k;
	 int   R;
}; // end of the MotoClass

extern MotoClass MotoShield; // declare external instance

#endif 
