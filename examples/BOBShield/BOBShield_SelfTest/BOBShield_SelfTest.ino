#include "BOBShield.h"
/*Adafruit_VL6180X sens=  Adafruit_VL6180X();*/
void setup() {
  Serial.begin(115200);               // Initialize serial
 BOBShield.begin();
 BOBShield.initialize();
 BOBShield.calibration();
 
 int x = BOBShield.sensorRead();
 Serial.println(x);
 delay(1000);
 
 BOBShield.actuatorWrite(30);
 delay(1000);
 BOBShield.actuatorWrite(0);
  delay(1000);
 BOBShield.actuatorWrite(-50);
 
// myservo.write(100);

}



void loop() {
  
}
