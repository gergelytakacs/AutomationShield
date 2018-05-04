#include "FloatShield.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
void FloatShieldClass::initialize() {

    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
    while (! Serial) {
        delay(1);
    }
    
    Serial.println("Adafruit VL53L0X test");
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        while(1);
    }
    Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
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
    
    if (value < 20) {value = 0;}
    else if (value > 370) {value = lastValue;}
    
    pos = map(value,0,370,0,100);
    
    lastValue = value;
    return pos;
}

int FloatShieldClass::positionMillimeter() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        value =measure.RangeMilliMeter;
    } else {}
    
    if (value < 20) {value = 0;}
    else if (value > 1000) {value = 1000;}
    
    
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

FloatShieldClass FloatShield;
