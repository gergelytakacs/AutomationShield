#include "AutomationShield.h"

float Setpoint = 50.00;

void setup() {
  
Serial.begin(9600);
OptoShield.begin();
OptoShield.calibration();
delay(1000);
OptoShield.actuatorWrite(Setpoint);
}

void loop() {

  float senzor = OptoShield.sensorRead();

  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.println(senzor);

}
