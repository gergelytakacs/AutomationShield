/*
  AutomationShield API

  This library serves as an API for the AutomationShield 
  ecosystem of Arduino Shields used for control engineering and
  mechatronics education.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács, Tibor Konkoly, Gábor Penzinger [...]
  Last update: 28.09.2018.
*/

#include "AutomationShield.h"
#include "Arduino.h"

float AutomationShieldClass::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) // same as Arudino map() but with floating point numbers
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // linear mapping, same as Arduino map()
}

float AutomationShieldClass::constrainFloat(float x, float min_x, float max_x){
 if (x>=max_x){x=max_x;} // if x larger than maximum, it's maximum
 if (x<=min_x){x=min_x;} // if x smaller than minimum, it's minimum
 return x; // return x
}

// Turns a floating point input in percent to 8-bit PWM
byte AutomationShieldClass::percToPwm(float perc){
	float percFloat = perc*2.55;
	return byte(percFloat);
}
 
 void AutomationShieldClass::error(const char *str) // Error handler function
 {
 #if ECHO_TO_SERIAL                      // If Serial Echo is turned on in the DEFINE (by advanced user)
 Serial.print("\nError: ");
 Serial.println(str);                    // Print error message
 #endif
 digitalWrite(ERRORPIN, HIGH);           // Enable generic error pin, should be 13 w/ the onboard LED
 while (1);                              // Stop all activity in busy cycle, so user definitely knows what's wrong.
 }

 void AutomationShieldClass::serialPrint(const char *str){   // Function for printing messages to serial monitor if its enabled
 #if ECHO_TO_SERIAL                                    // If Serial Echo is turned on in the DEFINE (by advanced user)
 Serial.print(str);                                    // Print message to serial monitor
 #endif 
 }
 
 
// Prints a single line for range measurements in an ordered form, e.g.
// Coil current 0.0 50.6  mA
// Enter the name of the range, first number, second number, then a unit
void AutomationShieldClass::printLowHigh(char *named, float low, float high, char *unit, short int precision){
  Serial.print(named);  
  Serial.print("\t");
  Serial.print(low, precision);
  Serial.print("\t");
  Serial.print(high, precision);
  Serial.print("\t");
  Serial.println(unit);
}

// Prints a line of dashes, 60 characters wide, then a new line.
void AutomationShieldClass::printSeparator(char separator){
 if (separator == '-'){
   Serial.println("------------------------------------------------------------");
 }
 else if (separator == '='){
   Serial.println("============================================================");
 }
 else if (separator == '*'){
   Serial.println("************************************************************");
 }
 else{
   Serial.println("------------------------------------------------------------"); 
 }
}

// Creates a header for displaying numeric ranges with a label and unit, e.g.
// Begins with a new line, displays TEST LOW HIGH  UNIT, then a line of dashes.
void AutomationShieldClass::printLowHighFirst(void){
   Serial.println("");
  Serial.println("TEST \t\tLOW\tHIGH\tUNIT");
  printSeparator("-");
}

// Evaluates and prints if a number fits into a range
// First come a description, then the value to be tested, the lower
// end of the range, and the higher end. 
// Returns 0 on success and 1 on failure
bool AutomationShieldClass::printTestResults(char *text,float value, float low, float high){
  Serial.print(text);
  Serial.print("\t\t");                // print tab characters
  if (value >= low && value <= high) {
    Serial.println(" Ok.");
    return 0;
  }
  else {
    Serial.println(" Fail.");
    return 1;
  }
}

// Data print of a certain number of floats for displaying and logging. 
// A common task with AutomationShield. Different number of arguments
// is handled by method overloading.
void AutomationShieldClass::print(float value1, float value2, float value3){
  Serial.print(value1);
  Serial.print(", ");
  Serial.print(value2);
  Serial.print(", ");
  Serial.println(value3);
}

float AutomationShieldClass::quality(float e, char *metric){
	// see http://facstaff.cbu.edu/rprice/lectures/tuning.html
	if (metric == "IE") { 												// Integral Square Error
		return qualityVal = (qualityVal + e);
	}
	else if (metric == "IAE") { 										// Integral Square Error
		return qualityVal = (qualityVal + abs(e));
	}
	else if (metric == "ISE") { 										// Integral Square Error
		return qualityVal = (qualityVal + pow(e,2));
	}
	else {
		return -1.0;
	}	
}

AutomationShieldClass AutomationShield; // Construct instance (define)
