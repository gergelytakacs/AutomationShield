#include "FloatShield.h"

#ifdef ADAFRUIT_VL53L0X_H // Only implement if there is a library

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
        value =measure.RangeMilliMeter; // sensor reading in mm
    } else {}
    
    if (value < minimum) {value = minimum;}
    else if (value > maximum) {value = lastValue;}
    
    pos = map(value,minimum,maximum,100,0); // converts sensor readings in mm to percents
    
    lastValue = value;
    return pos;
}

int FloatShieldClass::positionMillimeter() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        value =measure.RangeMilliMeter; // sensor reading in mm
    } else {}
    
    if (value < minimum) {value = minimum;}
    else if (value > maximum) {value = maximum;}
    
    
    pos = value;
    
    return pos; // returns position in millimeters
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
 int temp; // temporary variable
  minimum = 5000;  // we initially set the minimum to a very big number to make sure that 
                   //the measured minimum will be smaller that this value
 analogWrite(vent,255);
 delay(1000);
 for(i=0;i <= 100;i++) //measure the minimum 100 times
 {
   VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        temp =measure.RangeMilliMeter; // variable temp stores the measured value
    } else {}

   if(temp < minimum) 
   {
    minimum = temp; //if the temporary variable is smaller than the actual minimum, 
                    //the temp. variable will be the actual minimum value.
   }

   delay(10); // wait 10 ms between measurements
 }
 analogWrite(vent,0);
 delay(1000);  
 int tempm =0;
  maximum = 0; // we initially set maximum to 0, so the measued maximum will be greater than this number
 for(i=0;i <=100;i++) // measure the maximum 100 times
 {
  VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        tempm =measure.RangeMilliMeter;
    } else {}
  
  if(tempm > maximum)
  {
    maximum = tempm; //if the temporary variable is greater than the actual maximum, 
                    //the tempm. variable will be the actual maximum value.
  }
  
  delay(10); // wait 10 ms between measurements
 }
}
FloatShieldClass FloatShield;
#endif