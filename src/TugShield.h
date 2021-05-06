/*
  API for the TugShield hardware.
  
  The file is a part of the application programming interface for
  the TugShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Eva Vargová. 
  Last update: 13.5.2019.
*/

#ifndef TUGSHIELD_H			// Include guard
#define TUGSHIELD_H			

#include "Arduino.h"			// Required Arduino API in libraries
#include "AutomationShield.h"
#include "Servo.h"

// Defining pins used by the Tugshield 
#define TUG_YPIN 0  			// Flexi sensor
#define TUG_UPIN 10 			// Servo motor
#define K_MIN 1023				// Initial value for variable K_MIN
#define K_MAX 0					// Initial value for variable k_max
#define DELAY_VALUE 500			// Initial value for variable delay_value
#define SERVO_MIN 0
#define SERVO_MAX 180
#define DEFAULT_MIN 500.0
#define DEFAULT_MAX 1000.0

class TugShieldClass 						// Class for the TugShield device
{			
  public:
    void begin();							// Begin method
    void calibration();						// Calibration method
	  void actuatorWrite(int servo_angle);	// Write value to servo method
	  float sensorRead();						// Read value from flexi method
  private:   
	  bool  _wasCalibrated; 					// Initializing the variable _wasCalibrated
	  int   _sensorRead ;
	  int   k_maximal;
	  int   k_minimal;
};

Servo servo;
// Inicializovanie hardvéru
void TugShieldClass::begin()
{  												
    servo.attach(TUG_UPIN);						// Nastavenie pinu serva
	  servo.write(SERVO_MAX);						// Nastavenie pozície serva na nulu
  #ifdef ARDUINO_ARCH_AVR
	analogReference(EXTERNAL); // Set reference voltage
  #elif ARDUINO_ARCH_SAM
	//analogReadResolution(12);

  #elif ARDUINO_ARCH_SAMD
        //analogReadResolution(12);
  #endif
}

// Kalibrácia
void TugShieldClass::calibration()				
{ 
  Serial.println("Prebieha kalibrácia");		// Kalibrácia prebieha
  servo.write(SERVO_MAX);						// Nastavenie serva na pozíciu nula
  delay(DELAY_VALUE);								
  for (int ii=0; ii <= 10; ii++)				// Cyklus opakovaný 11 x	
  {
    _sensorRead=analogRead(TUG_YPIN);			// Načítanie hodnoty z flexi snímača
	if (ii == 00)								// Prvé opakovanie
	{
		if(_sensorRead>K_MAX)					// Porovnanie hodnoty z nastavenou hodnotou		
		{
		k_maximal=_sensorRead;					// Priradenie hodnoty
		}
		delay(DELAY_VALUE);
	}
	else
	{
		if(_sensorRead>k_maximal)				// Porovnanie hodnoty z nameranou hodnotou			
		{
		k_maximal=_sensorRead;					// Priradenie hodnoty
		}
		delay(DELAY_VALUE);
	}
  }
  
  servo.write(SERVO_MIN);						// Nastavenie serva na pozíciu 180 stupňov
  delay(DELAY_VALUE);								
  for (int ii=0; ii <= 10; ii++)				// Cyklus opakovaný 11 x	
  {
    _sensorRead=analogRead(TUG_YPIN);			// Načítanie hodnoty z flexi snímača
	if (ii == 00)								// Prvé opakovanie
	{
		if(_sensorRead<K_MIN)					// Porovnanie hodnoty z nastavenou hodnotou			
		{			
		k_minimal=_sensorRead;					// Priradenie hodnoty
		}
		delay(DELAY_VALUE);
	}
	else
	{
		if(_sensorRead<k_minimal)				// Porovnanie hodnoty z nameranou hodnotou				
		{
		k_minimal=_sensorRead;					// Priradenie hodnoty
		}
		delay(DELAY_VALUE);
	}
  }
  _wasCalibrated = true; 							
	Serial.println("Kalibrácia bola úspešne dokončená");		// Kalibrácia bola vykonaná
}

void TugShieldClass::actuatorWrite(int servo_angle){  
    int servo_rightangle = 180 - servo_angle;				// Prepočítanie na skutočné hodnoty
     servo.write(servo_rightangle);  		 				// Nastavenie hodnoty na servo
}

float TugShieldClass::sensorRead()
{
    _sensorRead = analogRead(TUG_YPIN);					// Načítanie hodnoty z flexi senzora
    float _sensorValue;
    if(_wasCalibrated == true)
	{                  										// V prípade, že bola kalibrácia urobená
      _sensorValue = AutomationShield.mapFloat((float)_sensorRead,(float)k_minimal,(float)k_maximal, 0.00, 100.00); 
    }
    else													// V prípade ak nebola kalibrácia urobená
	{ 
      _sensorValue = AutomationShield.mapFloat( (float)_sensorRead, DEFAULT_MIN, DEFAULT_MAX, 0.00, 100.00);
    }
    float _sensor_rightValue = 100 - _sensorValue;			
	return _sensor_rightValue;
}
TugShieldClass TugShield;

#endif
