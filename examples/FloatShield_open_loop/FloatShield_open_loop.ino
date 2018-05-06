#include <AutomationShield.h>
#include <FloatShield.h>

int dist;


void setup() {
 
  Serial.begin(115200);
  FloatShield.initialize();
  FloatShield.calibrate();
}

void loop() {
  FloatShield.manualControl();
  dist = FloatShield.positionMillimeter();
  Serial.print("Distance (mm): ");
  Serial.println(dist);
}
