/*
  AutomationShield.h - Library for teaching control engineering.
  Authors: Tibor Konkoly, Gabor Penzinger [...], and Gergely Takacs
  2017-2018.
  Released into the public domain.
*/

#ifndef AutomationShield_h
#define AutomationShiled_h

#include "Arduino.h"

class AutomationShield
{
  public:
    void Optical(void);  // Initialize for OpticalShield
    // void Motor(void); // Initialize for MotorShield
    // ...
    void  input(float u);  // Drive the actuator
    float output(void);    // Read the sensor
    float reference(void); // Read user reference
    // ...
  private:
    //   int x;
};

void AutomationShield::Optical(void) // Initialize for OpticalShield
{
#define OPTICAL
#define UPIN 9 
#define YPIN 0 
  //const int sensorPin = 1;
  //const int referencePin = 2;
  //....
  pinMode(UPIN, OUTPUT);
}

#ifdef OPTICAL // For OpticalShield.

// Just an example
void AutomationShield::input(float u)     // Sends % power to actuator
{
  pinMode(10, OUTPUT); //delete
  digitalWrite(10, 0); //delete

  constrain(u, 0, 100);		                // Constrain input to 0-100 %
  int upwm = map((int)u, 0, 100, 0, 255); // Map percents to 8-bit PWM
  analogWrite(UPIN, upwm); 	              // Write to actuator pin
}

float AutomationShield::output(void)
{
  // Function to read outputs from the sensor
  int y = analogRead(YPIN);
  return y;
}

float AutomationShield::reference(void)
{
  // Function to read reference levels from user
}
#endif         // For OpticalShield.






// #ifdef Motor // For MotorShield.
// Analogously repeated (with the ifdef/endif) for other shields
// #ifdef Motor // For MotorShield







#endif // End of AutomationShield library.
