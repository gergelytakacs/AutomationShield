/*
Dalsi pokec a'la Vlado tu. 

  API for the FloatShield didactic hardware.
  The file is a part of the application programmers interface for
  the FloatShield didactic tool for control engineering and
  mechatronics education. The FloatShield implements an air-flow
  levitation experiment on an Arduino shield.
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 11.3.2020.
*/

#include "BlowShield.h"         // Include header file

void BlowClass::begin(void) {  
#ifdef ARDUINO_ARCH_AVR  
	#if SHIELDRELEASE == 1                                            // For shield version 1
//tu chyba inicializacia I2C. predpokladam, ze sa to udeje niekde v ramci adafruit kniznic...
		
		pinMode(BLOW_UPIN,OUTPUT);
		pinMode(BLOW_RPIN,INPUT);
		digitalWrite(BLOW_UPIN,HIGH); //turn off pump
	#endif
 #endif
 if (!pressureSensor.begin(BMP280_ADRESA)) {                                     // Setting up I2C pressure sensor
    AutomationShield.error("BlowShield failed to initialise!");
//toto neviem naco je, ale bolo to u Gigiho tak zatial to nechavam
	while (1);
 }
}

void BlowClass::calibration(void) {                       // Board calibration
//toto je pracovna verzia, natvrdo nastavene min a max, ale malo by to byt
//vypocitane z niekolkych merani - toto meranie si este doladime SPOLU
_minPressure=94000.00;
_maxPressure=106000.00;
_wasCalibrated=true;
}

float BlowClass::referenceRead(void) {     
//pracovna verzia, treba poanglictit.                                                  // Reference read
int val=analogRead(BLOW_RPIN);
float percento = map((float)val, 0.0, 1023.0, 0.0, 100.0);
return percento;
}

void BlowClass::actuatorWrite(float aPercent) { 
//pracovna verzia, treba poanglictit. a neviem ci su tu spravne datove tipy
    //len zapisanie PWM signalu na Led, hodnota do funkcie vstupuje ako percento.
float inPercento=255-(percento*255/100);
analogWrite(BLOW_UPIN,inPercento);
}

float BlowClass::sensorRead(void) { 
unsigned long int val=pressureSensor.readPressure();    
//pracovna verzia, treba poanglictit. a popratat a spravne datove typy pouzit.
  float percento = map((unsigned long int)val, _minPressure,_maxPressure, 0.00, 100.00);
  percento= AutomationShield.constrainFloat(_sensorPercent, 0.0, 100.0);  
  return percento;
}  
//tu prichadza sranda s PID, ktora je totalne zla a ani nema byt definovana tu, ale mali by sme pouzivat fciu 
//z AutomationShieldu. OPRAVIT! tu mam bulharsku konstantu :( 
float BlowClass::PID(float P, float I, float D,float Ts, float Ek,float Esum,float Eback){ //funkcia PID regulatora, integacny a derivacny clen je upraveny 1000kou<--prepocet Ts z millis() na sekundy
    float u=(P*Ek)+(((P*Ts)/(100000.00*I))*Esum)+((100000.00*P*D/Ts)*(Ek-Eback));
    float uc=constrain(u,0.00,100.00);
    return uc;
}

BlowClass BlowShield;                                   // Creation of BlowClass object

