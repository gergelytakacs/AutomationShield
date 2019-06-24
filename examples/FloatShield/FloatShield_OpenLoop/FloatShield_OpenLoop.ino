#include <FloatShield.h>           // Including the FloatShield library

void setup() {                     // Setup - runs only once
    Serial.begin(9600);            // Begin serial comunication
    FloatShield.begin();           // Initialise FloatShield
    FloatShield.calibrate();       // Calibrate FloatShield
}

void loop() {                                                       // Loop - runs indefinitely
    float potentiometerReference = FloatShield.referenceRead();     // Save current percentual position of potentiometer runner to "potentiometerReference" variable
    float ballPositionInTube = FloatShield.sensorRead();            // Save current percentual position of ball to "ballPositionInTube" variable
    FloatShield.actuatorWrite(potentiometerReference);              // Set the power output of the fan based on the current potentiometer position
    Serial.print("Power output: ");
    Serial.print(potentiometerReference);                           // Write out to the Serial Monitor current potentiometer percentual position
    Serial.print(", ");
    Serial.print("Ball altitude: ");
    Serial.println(ballPositionInTube);                             // Write out to the Serial Monitor current percentual position of the ball
    delay(100);                                                     // Wait 100 miliseconds
}
