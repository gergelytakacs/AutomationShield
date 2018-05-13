#include "HeatShield.h"

void HeatShieldClass::begin(){
  
      pinMode(thermPin, INPUT);  //thermistor pin
      pinMode(pwmOutputPin, OUTPUT); //pwm output
}

void HeatShieldClass::actuatorWrite(float percent){
  
      analogWrite(pwmOutputPin,(255.0/100.0)*percent);
}

float HeatShieldClass::sensorReadTemperature() {

    return (1 / ((1 / referenceTemperature) + (log(getThermistorResist() / resistorResistance) / (float)beta))) - 273.15;    
}

float HeatShieldClass::getThermistorVoltage() {

      return (float)analogRead(thermPin) / 1023 * powerSupply;
}

float HeatShieldClass::getThermistorResist() {

      return (getThermistorVoltage() / powerSupply)*((float)resistorResistance / (1 - (getThermistorVoltage() / powerSupply)));
}

HeatShieldClass HeatShield;
