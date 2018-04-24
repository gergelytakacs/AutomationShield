#include "HeatShield.h"

void HeatShieldClass::pinInitialization(){
  
      pinMode(thermPin, INPUT);  //thermistor pin
      pinMode(pwmOutputPin, OUTPUT); //pwm output
}

void HeatShieldClass::power(float percent){
  
      analogWrite(pwmOutputPin,(255.0/100.0)*percent);
}

float HeatShieldClass::getTemperature() {

    return (1 / ((1 / referenceTemperature) + (log(getThermistorResist() / resistorResistance) / (float)beta))) - 273.15;    
}

float HeatShieldClass::getThermistorVolage() {

      return (float)analogRead(thermPin) / 1023 * powerSupply;
}

float HeatShieldClass::getThermistorResist() {

      return (getThermistorVolage() / powerSupply)*((float)resistorResistance / (1 - (getThermistorVolage() / powerSupply)));
}

HeatShieldClass HeatShield;
