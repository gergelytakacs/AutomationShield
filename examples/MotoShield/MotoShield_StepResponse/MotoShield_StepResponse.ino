#include <MotoShield.h>
#define TS 100.0 //--Sampling period in milliseconds
#define u  70.0 //--Step value in percent
void setup() {
  Serial.begin(9600); //--Initialize serial communication # 9600 baudrate
  MotoShield.begin(TS); //--Initialize MotoShield with sampling period 100 millis
  MotoShield.calibration(); //--Calibration method
  MotoShield.actuatorWrite(u); //--Step value in percents
}

void loop() {
  Serial.print(MotoShield.sensorReadRPMPerc()); //--Printing angular velocity in percents
  Serial.print(" ");
  Serial.println(u); //--Printing Step (reference) value
}
