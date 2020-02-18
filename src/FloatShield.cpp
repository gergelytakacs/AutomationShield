/*
  API for the FloatShield didactic hardware.

  The file is a part of the application programmers interface for
  the FloatShield didactic tool for control engineering and
  mechatronics education. The FloatShield implements an air-flow
  levitation experiment on an Arduino shield.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by Gergely Takács and Peter Chmurčiak.
  Last update: 5.12.2019.
*/

#include "FloatShield.h"         // Include header file

#ifdef VL53L0X_h                 // If library for distance sensor was sucessfully included

#if SHIELDRELEASE == 2           // For shield version 2
  #ifdef ARDUINO_ARCH_AVR        // For AVR architecture boards
/*     ISR (TIMER2_COMPA_vect) {                   // Interrupt routine on compare match with register A on Timer2
      FloatShield.hundredthsOfMillisecond++;    // Increment custom time unit
    } */
  #elif ARDUINO_ARCH_SAM         // For SAM architecture boards
    void TC6_Handler() {         // Interrupt routine on compare match with register RC on Timer2 channel 0
      TC_GetStatus(TC2, 0);      // Read status of Timer2 channel 0 in order to allow the next interrupt
      FloatShield.hundredthsOfMillisecond++;    // Increment custom time unit
    }
  #elif ARDUINO_ARCH_SAMD                                                   // For SAMD architecture boards
    void TC4_Handler() {                                                    // Interrupt routine on overflow of Timer/Counter 4    
      if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF) {  // Make sure that overflow happened
      FloatShield.hundredthsOfMillisecond++;                                // Increment custom time unit
      REG_TC4_INTFLAG = TC_INTFLAG_OVF;                                     // Clear the overflow interrupt flag
      }
    }
  #endif
void hallPeriodCounter(void) {                                    // Interrupt routine on external interrupt pin 1
    FloatShield.hallPeriod = FloatShield.hundredthsOfMillisecond; // Save the period of hall signal in custom time units into variable
    FloatShield.hundredthsOfMillisecond = 0;                      // Reset custom time unit counter
    FloatShield.pulseMeasured = 1;                                // Set flag that pulse length was measured
}
#endif

void FloatClass::begin(void) {                                        // Board initialisation
    AutomationShield.serialPrint("FloatShield initialisation...");
#ifdef ARDUINO_ARCH_AVR                                               // For AVR architecture boards
    Wire.begin();                                                     // Use Wire object
  #if SHIELDRELEASE == 1                                              // For shield version 1
    analogReference(DEFAULT);                                         // Use default analog reference
  #elif SHIELDRELEASE == 2                                            // For shield version 2
    analogReference(EXTERNAL);                                        // Use external analog reference
  #endif
#elif ARDUINO_ARCH_SAM                                                // For SAM architecture boards
    Wire1.begin();                                                    // Use Wire1 object
#elif ARDUINO_ARCH_SAMD                                               // For SAMD architecture boards
    Wire.begin();                                                     // Use Wire object
#endif
    distanceSensor.setTimeout(1000);                                  // Set sensor timeout to 1 second
    if (!distanceSensor.init()) {                                     // Setting up I2C distance sensor
        AutomationShield.error("FloatShield failed to initialise!");
    }
    distanceSensor.setMeasurementTimingBudget(20000);                 // Setting high-speed mode for laser sensor
    distanceSensor.startContinuous();                                 // Setting continuous mode for laser sensor
    _minDistance = 17.0;                                              // Initializing min,max variables by approximate values so the functions can be used even without calibration but with lower precision
    _maxDistance = 341.0;
    _range = _maxDistance - _minDistance;
    _wasCalibrated = false;
    
#if SHIELDRELEASE == 2          // For shield version 2                                           
#ifdef ARDUINO_ARCH_AVR         // For AVR architecture boards
    TCCR2B = 0;                 // Clear Timer2 settings
    TCNT2 = 0;                  // Clear Counter2 value
    TCCR2A = bit(WGM21);        // Set Timer2 to CTC mode
    OCR2A  = 19;                // Set register A compare value to 20 (relative to zero)
    TIMSK2 = bit (OCIE2A);      // Enable Timer2 interrupt on compare match with register A
    GTCCR = bit (PSRASY);       // Reset prescaler on Timer/Counter 2
    TCCR2B = bit(CS21);         // Set prescaler to value 8 on Timer/Counter 2
#elif ARDUINO_ARCH_SAM                                                        // For SAM architecture boards
    pmc_set_writeprotect(false);                                              // Turn off register write protection
    pmc_enable_periph_clk(TC6_IRQn);                                          // Enable peripheral clock for Timer2 channel 0
    TC_Configure(TC2, 0, TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK5);   // Configure Timer2 channel 0 to count up to value specified in RC register and to use slow clock
    TC_SetRC(TC2, 0, 319);                                                    // Count up to value 320 (relative to zero)
    TC_Start(TC2, 0);                                                         // Start Timer2 channel 0
    TC2->TC_CHANNEL[0].TC_IER =  TC_IER_CPCS | TC_IER_CPAS;                   // Enable RC compare interrupt (enable register)
    TC2->TC_CHANNEL[0].TC_IDR = ~(TC_IER_CPCS | TC_IER_CPAS);                 // Enable RC compare interrupt (disable register)
    NVIC_EnableIRQ(TC6_IRQn);                                                 // Attach interrupt routine to compare interrupt
#elif ARDUINO_ARCH_SAMD
    REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);      // Enable Generic clock 4 and set the 48MHz clock source for Generic clock 4   
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TC4_TC5;   // Set Generic clock 4 as a Timer clock source and feed it to Timer4 and Timer5 
    REG_TC4_COUNT16_CC0 = 1874;                                                                 // Set the Timer4 CC0 register as the TOP value in match frequency mode
    NVIC_EnableIRQ(TC4_IRQn);                                                                   // Connect Timer4 to Nested Vector Interrupt Controller (NVIC)    
    REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_ENABLE;       // Set prescaler to 256 and put the timer TC4 into match frequency (MFRQ) mode 
    REG_TC4_INTENSET = TC_INTENSET_OVF;                                                         // Enable Timer4 interrupts
#endif
    //attachInterrupt(digitalPinToInterrupt(FLOAT_YPIN), hallPeriodCounter, RISING); // Attach interrupt routine to external interrupt pin 1. Trigger at rising edge of signal
#endif
    AutomationShield.serialPrint(" successful.\n");
}

void FloatClass::calibrate(void) {                         // Board calibration
    AutomationShield.serialPrint("Calibration...");
    float sum = 0.0;
    actuatorWrite(100.0);                                  // Sets fan speed to maximum
    while (sensorReadDistance() > 100.0) {                 // Waits until the ball is at least in the upper third of the tube
        delay(100);                                        // (This is because of varying time it takes for the ball to travel the first half of the tube)
    }
    delay(1000);                                           // Waits unil the ball reaches top and somewhat stabilises itself
    for (int i = 0; i < 100; i++) {                        // Measures 100 values with sampling 25 miliseconds and calculates the average reading from those values
        sum += sensorReadDistance();                       // (Simply using minimal value would be inconsistent because of shape of the ball combined with its ability to move horizontally in the tube)
        delay(25);
    }
    _minDistance = sum / 100.0;                            // Sets the minimal distance variable equal to the average reading

    sum = 0.0;
    actuatorWrite(0.0);                                    // Turns off the fan
    while (sensorReadDistance() < 300.0) {                 // Waits until the ball is at least in the lower third of the tube
        delay(100);                                        // (This is probably unecessary, because ball has no problem falling down, but for the sake of consistency)
    }
    delay(1000);
    for (int i = 0; i < 100; i++) {                        // Measures 100 values with sampling 25 miliseconds and calculates the average reading from those values
        sum += sensorReadDistance();                       // (Simply using maximal value would be inconsistent as in previous case)
        delay(25);
    }
    _maxDistance = sum / 100.0;                            // Sets the maximal distance variable equal to the average reading
    _range = _maxDistance - _minDistance;                  // Sets the range variable equal to the difference of the maximal and minimal distance

    _wasCalibrated = true;                                 // Sets the status of calibration to true
    AutomationShield.serialPrint(" sucessful.\n");
}

void FloatClass::actuatorWrite(float aPercent) {                                           // Write actuator
    float mappedValue = AutomationShield.mapFloat(aPercent, 0.0, 100.0, 0.0, 255.0);       // Takes the float type percentual value 0.0-100.0 and remapps it to range 0.0-255.0
    mappedValue = AutomationShield.constrainFloat(mappedValue, 0.0, 255.0);                // Constrains the remapped value to fit the range 0.0-255.0 - safety precaution
    analogWrite(FLOAT_UPIN, (int)mappedValue);                                             // Sets the fan speed using the constrained value
}

float FloatClass::referenceRead(void) {                                                        // Reference read
    _referenceValue = (float)analogRead(FLOAT_RPIN);                                           // Reads the actual analog value of potentiometer runner
    _referencePercent = AutomationShield.mapFloat(_referenceValue, 0.0, 1023.0, 0.0, 100.0);   // Remapps the analog value from original range 0.0-1023 to percentual range 0.0-100.0
    return _referencePercent;                                                                  // Returns the percentual position of potentiometer runner
}

float FloatClass::sensorRead(void) {                                                                         // Sensor read
    float readDistance = sensorReadDistance();                                                               // Reads the actual distance between ball and sensor in milimetres
    _sensorPercent = AutomationShield.mapFloat(readDistance, _maxDistance, _minDistance, 0.0, 100.0);        // Remapps the distance based on minimal and maximal distances from calibration to the percentual altitude of the ball
    _sensorPercent = AutomationShield.constrainFloat(_sensorPercent, 0.0, 100.0);                            // Constrains the percentual altitude - safety precaution
    return _sensorPercent;                                                                                   // Returns the percentual altitude of the ball in the tube
}

float FloatClass::sensorReadAltitude(void) {                  // Sensor read altitude
    _ballAltitude = sensorReadDistance();                     // Reads the current distance of the ball from sensor
    _ballAltitude = _maxDistance - _ballAltitude;             // Inverts the reading so the bottom position is 0
    _ballAltitude = constrain(_ballAltitude, 0, _maxDistance);// Prevents the reading to go under 0
    return _ballAltitude;                                     // Returns the current altitude of the ball in milimetres
}

float FloatClass::sensorReadDistance(void) {                                    // Sensor read distance
    _sensorValue = (float)distanceSensor.readRangeContinuousMillimeters();      // Reads the distance between sensor and the ball in milimetres
    return _sensorValue;                                                        // Returns the measured distance
}

bool FloatClass::returnCalibrated(void) {                   // Returns calibration status
    return _wasCalibrated;
}

float FloatClass::returnMinDistance(void) {                 // Returns value of minimal distance measured by sensor in milimetres
    return _minDistance;
}

float FloatClass::returnMaxDistance(void) {                 // Returns value of maximal distance measured by sensor in milimetres
    return _maxDistance;
}

float FloatClass::returnRange(void) {                       // Returns range of measured distances between minimal and maximal values in milimetres
    return _range;
}

#if SHIELDRELEASE == 2                                                                    // For shield version 2
void FloatClass::actuatorWriteRPM(float rpm) {                                            // Write actuator RPM
    float calculatedPWM = rpm / FLOAT_RPM_CONST;                                          // Takes the float type RPM value 0.0-17000.0 and recalculates it to range 0.0-255.0
    calculatedPWM = AutomationShield.constrainFloat(calculatedPWM, 0.0, 255.0);           // Constrains the calculated value to fit the range 0.0-255.0 - safety precaution
    analogWrite(FLOAT_UPIN, (int)calculatedPWM);                                          // Sets the fan RPM using the constrained value of PWM
}

int FloatClass::sensorReadRPM(void) {
    if(pulseMeasured){                                      // If pulse length was correctly measured
    _rpm = 3000000 / hallPeriod;                            // Calculate RPM out of current value of period of hall signal in custom time units
    pulseMeasured = 0;                                      // Reset flag for next pulse measuring
    return _rpm;                                            // Return RPM value
    } else                                                  // If pulse length was not correctly measured - no power going to the fan                                     
    return 0;                                               // Return 0                                              
}
#endif

FloatClass FloatShield;                                     // Creation of FloatClass object

#endif
