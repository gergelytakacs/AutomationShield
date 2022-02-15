#include <AS5600.h>

AS5600 encoder;
float output;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // get the angle in degrees of the encoder
  output = encoder.getAngle();
  Serial.println(output);
}
