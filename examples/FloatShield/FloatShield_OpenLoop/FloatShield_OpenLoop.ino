#include <AutomationShield.h>
#include <FloatShield.h>

int dist;


void setup() {
  // FloatShield.debug();     will print out in monitor sensor debug data
  Serial.begin(115200);  //start serial communication
  FloatShield.initialize(); //FloatShield initialization
  FloatShield.calibrate(); //FloatShield calibration for more accurate measurements

    if (! lox.initialize()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}
}

void loop() {
  FloatShield.manualControl(); //Calling this function will switch floatshield into manual
                               //control mode. By adjusting the potentiometer ventilator power
                               //will adjust accordingly.
  dist = FloatShield.positionMillimeter(); // read sensor 
  Serial.print("Distance (mm): ");
  Serial.println(dist);
}
