#include "FloatShield.h"

float FloatShieldClass::FloatShieldClass() {
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
    int _value = analogRead(pot);
    int _ref = map(_value, 0, 1023, 0, 100);
    return _ref;
}

int FloatShieldClass::positionPercent() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        int _value =measure.RangeMilliMeter;
    } else {}
    
    if (_value < 20) {_value = 0;}
    else if (_value > 370) {_value = _lastValue;}
    
    int _pos = map(_value,0,370,0,100);
    
    _lastValue = _value;
    return _pos;
}

void FloatShieldClass::ventInPercent(int _value) {
    int _u = map(_value, 0, 100, 0, 255);
    analogWrite(vent, _u);
}

float FloatShieldClass::manualControl() {
    int _value = analogRead(pot);
    int _in = map(_value, 0, 1023, 0, 255);
    analogWrite(vent, _in);
    _value = map(_value, 0, 1023, 0, 100);
    return _value;
}

FloatShieldClass FloatShield;
