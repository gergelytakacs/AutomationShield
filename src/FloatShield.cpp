#include "FloatShield.h"

void FloatShieldClass::initialize() {
    Serial.println("Adafruit VL53L0X test");
    if (!lox.begin(i2c_addr, _debug)) {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}

void FloatShieldClass::debug() {
    _debug = true;
}

int FloatShieldClass::referencePercent() {
    value = analogRead(pot);
    ref = map(value, 0, 1023, 0, 100);
    return ref;
}

int FloatShieldClass::positionPercent() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        value =measure.RangeMilliMeter;
    } else {}
    
    if (value < minimum) {value = minimum;}
    else if (value > maximum) {value = lastValue;}
    
    pos = map(value,minimum,maximum,100,0);
    
    lastValue = value;
    return pos;
}

int FloatShieldClass::positionMillimeter() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        value =measure.RangeMilliMeter;
    } else {}
    
    if (value < minimum) {value = minimum;}
    else if (value > maximum) {value = maximum;}
    
    
    pos = value;
    
    return pos;
}


void FloatShieldClass::ventInPercent(int value) {
    int u = map(value, 0, 100, 0, 255);
    analogWrite(vent, u);
}

float FloatShieldClass::manualControl() {
    value = analogRead(pot);
    in = map(value, 0, 1023, 0, 255);
    analogWrite(vent, in);
    value = map(value, 0, 1023, 0, 100);
    return value;
}

void FloatShieldClass::calibrate(void){
 int i=0;
 int temp;
  minimum = 5000;
 analogWrite(vent,255);
 delay(1000);
 for(i=0;i <= 100;i++)
 {
   VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        temp =measure.RangeMilliMeter;
    } else {}

   if(temp < minimum)
   {
    minimum = temp;
   }

   delay(10);
 }
 analogWrite(vent,0);
 delay(1000);  
 int tempm =0;
  maximum = 0;
 for(i=0;i <=100;i++)
 {
  VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        tempm =measure.RangeMilliMeter;
    } else {}
  
  if(tempm > maximum)
  {
    maximum = tempm;
  }
  
  delay(10);
 }
}
FloatShieldClass FloatShield;
