#include <MagnetoShield.h>     // Include header for hardware API

void setup() {
   Serial.begin(9600);          // Starts serial communication
   MagnetoShield.begin();       // Initializes shield
  MagnetoShield.dacWrite(0);
}
void loop() {             
  int x = analogRead(0);
  int y = map(x,0,1024,0,255);
  Serial.println(y);
  MagnetoShield.dacWrite(y);                 
}