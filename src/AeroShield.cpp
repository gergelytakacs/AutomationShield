<<<<<<< Updated upstream
/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by .... 
  Last update: 05.02.2022.
*/


#include "AeroShield.h"         // Include header file



// Initializes hardware pins
void AeroShieldClass::begin(){                                          // Board initialisation
      pinMode(AERO_UPIN,OUTPUT);  		                                  // Actuator pin
      Wire.begin();                                                     // Use Wire object                                      
} 

float AeroShieldClass::ams5600_initialization(bool isDetected){         // Sensor initialisation
  if(isDetected == 0 ){                                                 // If magnet not detected go on
    while(1){                                                           // Go forever until magnet detected 
        if(isDetected == 1 ){                                           // If magnet detected
            AutomationShield.serialPrint("Magnet detected \n");         // Print information then break
            break;
        }
        else{                                                           // If magnet not detected 
            AutomationShield.serialPrint("Can not detect magnet \n");   // Print information then go back to check while statement
      }
    }
  }
}

float AeroShieldClass::convertRawAngleToDegrees(word newAngle) {        // Function for converting raw angle(0-4096) to degrees(0-360°) 
  float retVal = newAngle * 0.087;                                      // 360°/4096=0.087° times the raw value
  ang = retVal;                               
  return ang;                                                           // Return angle value in degrees 
}


float AeroShieldClass::calibration(word RawAngle) {                     // Calibration 

  AutomationShield.serialPrint("Calibration running...\n");             // Print info 
  startangle=0;                                                         // Zero out Variable(precaution)
  analogWrite(AERO_UPIN,50);                                            // Power the actuator, swing the pendulum 
  delay(250);                                                           // Wait for 0.25s 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off, pendulum goes to zero position
  delay(4000);                                                          // Wait for pendulum to stop oscillating 
  
  startangle = RawAngle;                                                // Save the value of zero pozition in raw format 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off(precaution)
    for(int i=0;i<3;i++){                                               // Simple sound indication of successful calibration 3 beeps
      analogWrite(AERO_UPIN,1);                                         // Actuator powereded just a bit so the rotor doesn't turn just beep 
      delay(200);                                                       // wait 
      analogWrite(AERO_UPIN,0);                                         // Actuator powered off
      delay(200);                                                       // wait 
      }

  AutomationShield.serialPrint("Calibration done");
    return startangle;
}


  float AeroShieldClass::referenceRead(void) {                                             // Reference read
  referenceValue = (float)analogRead(AERO_RPIN);                                           // Reads the actual analog value of potentiometer runner
  referencePercent = AutomationShield.mapFloat(referenceValue, 0.0, 1024.0, 0.0, 100.0);   // Remapps the analog value from original range 0.0-1023 to percentual range 0.0-100.0
  return referencePercent;                                                                 // Returns the percentual position of potentiometer runner
}

void AeroShieldClass::actuatorWrite(float PotPercent) {                                    // Actuator write
  float mappedValue = AutomationShield.mapFloat(PotPercent, 0.0, 100.0, 0.0, 255.0);       // Takes the float type percentual value 0.0-100.0 and remapps it to range 0.0-255.0
  mappedValue = AutomationShield.constrainFloat(mappedValue, 0.0, 255.0);                  // Constrains the remapped value to fit the range 0.0-255.0 - safety precaution
  analogWrite(AERO_UPIN, (int)mappedValue);                                                // Write remapped value to actuator pin 
}

float AeroShieldClass::currentMeasure(void){                                               // Measuring current drawn by DC motor 
  for(int i=0 ; i<repeatTimes ; i++){                                                      // Function for callculating mean current value 
     voltageValue= analogRead(VOLTAGE_SENSOR_PIN);                                         // Read a value from the INA169 
     voltageValue= (voltageValue * voltageReference) / 1024;                               // Remap the ADC value into a voltage number (5V reference)
     current= current + correction1-(voltageValue / (10 * ShuntRes));                      // Equation given by the INA169 datasheet to
                                                                                           // determine the current flowing through ShuntRes. RL = 10k
     }                                                                                     // Is = (Vout x 1k) / (RS x RL)

   float currentMean= current/repeatTimes;                                                 // Callculating mean current value 
   currentMean= currentMean-correction2;                                                   // Small correction of current value(determined by ampermeter)
   if(currentMean < 0.000){                                                                // Correction for occasional bug causing the value to be negative. 
      currentMean= 0.000;                                                                  // When it so happens, zero out the value. 
      }

  current= 0;                                                                              // Zero out current value        
  voltageValue= 0;                                                                         // Zero out voltage value  
  return currentMean;                                                                      // Return mean current value 

}

AeroShieldClass AeroShield;
=======
/*
  API for the AeroShield hardware.
  
  The file is a part of the application programming interface for
  the AeroShield didactic tool for control engineering and 
  mechatronics education.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by .... 
  Last update: 05.02.2022.
*/


#include "AeroShield.h"         // Include header file



// Initializes hardware pins
void AeroShieldClass::begin(){                                          // Board initialisation
      pinMode(AERO_UPIN,OUTPUT);  		                                  // Actuator pin
      Wire.begin();                                                     // Use Wire object                                      
} 

float AeroShieldClass::ams5600_initialization(bool isDetected){         // Sensor initialisation
  if(isDetected == 0 ){                                                 // If magnet not detected go on
    while(1){                                                           // Go forever until magnet detected 
        if(isDetected == 1 ){                                           // If magnet detected
            AutomationShield.serialPrint("Magnet detected \n");         // Print information then break
            break;
        }
        else{                                                           // If magnet not detected 
            AutomationShield.serialPrint("Can not detect magnet \n");   // Print information then go back to check while statement
      }
    }
  }
}

float AeroShieldClass::convertRawAngleToDegrees(word newAngle) {        // Function for converting raw angle(0-4096) to degrees(0-360°) 
  float retVal = newAngle * 0.087;                                      // 360°/4096=0.087° times the raw value
  ang = retVal;                               
  return ang;                                                           // Return angle value in degrees 
}


float AeroShieldClass::calibration(word RawAngle) {                     // Calibration 

  AutomationShield.serialPrint("Calibration running...\n");             // Print info 
  startangle=0;                                                         // Zero out Variable(precaution)
  analogWrite(AERO_UPIN,50);                                            // Power the actuator, swing the pendulum 
  delay(250);                                                           // Wait for 0.25s 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off, pendulum goes to zero position
  delay(4000);                                                          // Wait for pendulum to stop oscillating 
  
  startangle = RawAngle;                                                // Save the value of zero pozition in raw format 
  analogWrite(AERO_UPIN,0);                                             // Actuator powered off(precaution)
    for(int i=0;i<3;i++){                                               // Simple sound indication of successful calibration 3 beeps
      analogWrite(AERO_UPIN,1);                                         // Actuator powereded just a bit so the rotor doesn't turn just beep 
      delay(200);                                                       // wait 
      analogWrite(AERO_UPIN,0);                                         // Actuator powered off
      delay(200);                                                       // wait 
      }

  AutomationShield.serialPrint("Calibration done");
    return startangle;
}


  float AeroShieldClass::referenceRead(void) {                                             // Reference read
  referenceValue = (float)analogRead(AERO_RPIN);                                           // Reads the actual analog value of potentiometer runner
  referencePercent = AutomationShield.mapFloat(referenceValue, 0.0, 1024.0, 0.0, 100.0);   // Remapps the analog value from original range 0.0-1023 to percentual range 0.0-100.0
  return referencePercent;                                                                 // Returns the percentual position of potentiometer runner
}

void AeroShieldClass::actuatorWrite(float PotPercent) {                                    // Actuator write
  float mappedValue = AutomationShield.mapFloat(PotPercent, 0.0, 100.0, 0.0, 255.0);       // Takes the float type percentual value 0.0-100.0 and remapps it to range 0.0-255.0
  mappedValue = AutomationShield.constrainFloat(mappedValue, 0.0, 255.0);                  // Constrains the remapped value to fit the range 0.0-255.0 - safety precaution
  analogWrite(AERO_UPIN, (int)mappedValue);                                                // Write remapped value to actuator pin 
}

float AeroShieldClass::currentMeasure(void){                                               // Measuring current drawn by DC motor 
  for(int i=0 ; i<repeatTimes ; i++){                                                      // Function for callculating mean current value 
     voltageValue= analogRead(VOLTAGE_SENSOR_PIN);                                         // Read a value from the INA169 
     voltageValue= (voltageValue * voltageReference) / 1024;                               // Remap the ADC value into a voltage number (5V reference)
     current= current + correction1-(voltageValue / (10 * ShuntRes));                      // Equation given by the INA169 datasheet to
                                                                                           // determine the current flowing through ShuntRes. RL = 10k
     }                                                                                     // Is = (Vout x 1k) / (RS x RL)

   float currentMean= current/repeatTimes;                                                 // Callculating mean current value 
   currentMean= currentMean-correction2;                                                   // Small correction of current value(determined by ampermeter)
   if(currentMean < 0.000){                                                                // Correction for occasional bug causing the value to be negative. 
      currentMean= 0.000;                                                                  // When it so happens, zero out the value. 
      }

  current= 0;                                                                              // Zero out current value        
  voltageValue= 0;                                                                         // Zero out voltage value  
  return currentMean;                                                                      // Return mean current value 

}

AeroShieldClass AeroShield;
>>>>>>> Stashed changes
