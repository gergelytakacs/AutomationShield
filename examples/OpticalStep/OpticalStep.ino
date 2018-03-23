#include "AutomationShield.h" //<-Change to <>

AutomationShield Optical; // Initialize the OpticalShield

void setup() {
  // Just an example
  Optical.input(25); // Actuator at 50% power.
  delay(2000);
  Optical.input(0);  // Actuator off...

  Serial.begin(9600);
  int x=Optical.output(); // Read Input pin;
  Serial.print(x);
  
}

void loop() {
  // put your main code here, to run repeatedly:
}
