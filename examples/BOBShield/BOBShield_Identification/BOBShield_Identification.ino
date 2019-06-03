#include <BOBShield.h>
#include <SamplingServo.h>

void setup() {
  Serial.begin(115200);
 BOBShield.begin();
 BOBShield.initialize();
}

void loop() {

  for (int i = -30; i <= 30; i++) {
      BOBShield.actuatorWrite(i);
      int range = BOBShield.sensorRead();
Serial.print("Range: "); Serial.print(range);
Serial.print(" ");
Serial.println(i);
    delay(16);
  }
  
for (int i = 30; i >= -30; i--) {
      BOBShield.actuatorWrite(i);
      int range = BOBShield.sensorRead();
Serial.print("Range: "); Serial.print(range);
Serial.print(" ");
Serial.println(i);
    delay(16);
  }
}
