/*
  Class declaration for Interrupt-driven sampling for real-time control.

  The file forward declares the classes necessary for configuring
  an interrupt-driven system for deploying digital control systems
  on AVR, SAMD and SAM-based Arduino prototyping boards with the
  R3 pinout. The module should be compatible with the Arduino Uno, 
  Mega 2560, Zero, Due, Uno R4 and Adafruit Metro M4 Express.
  There should be no timer conflicts when using the Servo library.

  This file contains the forward declarations of two classes
  having an identical name (SamplingClass), that are separated
  into two namespaces. The actual interrupt service routines are
  located in a pair of outside headers in the root source directory.
  A "Sampling" object is reated by calling the correct header file
  depending whether  your system uses the Servo library or not,
  first by invoking the correct namespace, then implementing the
  interrupt routine.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons

  AVR Timer:    Gergely Takacs, Richard Koplinger, Matus Biro,
                 Lukas Vadovic 2018-2019
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM51 Timer:   Gergely Takacs, 2019 (Metro M4)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  RA4M1 Timer:   Erik Mikuláš,   2023 (UNO R4)
  Last update: 4.10.2023 by Erik Mikuláš
*/

#include "SamplingCore.h"

////////////////////////////////////// NO SERVO //////////////////////////////////////

void SamplingNoServo::SamplingClass::defaultInterrupt()
{
}

SamplingNoServo::SamplingClass::SamplingClass() {
  interruptCallback = defaultInterrupt;
  samplingPeriod = 0.0;
}

void SamplingNoServo::SamplingClass::period(unsigned long microseconds) {
  #ifdef ARDUINO_AVR_UNO
    // for Arduino Uno use Timer2 to stay compatible with the Servo library
    noInterrupts();                             // disable all interrupts
    TCCR1A = 0;                                 // clear register
    TCCR1B = 0;                                 // clear register
    TCNT1  = 0;                                 // clear register

    TCCR1B |= (1 << WGM12);                     // CTC mode
    TIMSK1 |= (1 << OCIE1A);                    // enable timer compare interrupt

    setSamplingPeriod(microseconds);

    interrupts();             // enable all interrupts

  #elif ARDUINO_AVR_MEGA2560
    // for Arduino Mega2560 use Timer5
    noInterrupts();                             // disable all interrupts
    TCCR5A = 0;                                 // clear register
    TCCR5B = 0;                                 // clear register
    TCNT5  = 0;                                 // clear register

    TCCR5B |= (1 << WGM52);                     // CTC mode
    TIMSK5 |= (1 << OCIE5A);                    // enable timer compare interrupt

    setSamplingPeriod(microseconds);

    interrupts();                              // enable all interrupts


  #elif ARDUINO_SAMD_ZERO
    // For Arduino Zero
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;        // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;        // Set TC5 mode as match frequency

    setSamplingPeriod(microseconds);
    #ifdef ECHO_TO_SERIAL
      if (!setSamplingPeriod(microseconds))
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
    #endif

    NVIC_EnableIRQ(TC5_IRQn);                               // Enable interrupt for TC5
    TC5->COUNT16.INTENSET.bit.MC0 = 1;                      // Enable the TC5 interrupt request
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;              // Enable timer

  #elif ADAFRUIT_METRO_M4_EXPRESS
    // For Adafruit M4 Express
    GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    TC5->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ; // Match mode, counter resets at match

    setSamplingPeriod(microseconds);
    #ifdef ECHO_TO_SERIAL
      if (!setSamplingPeriod(microseconds))
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
    #endif

    NVIC_EnableIRQ(TC5_IRQn);                               // Enable interrupt for TC5
    TC5->COUNT16.INTENSET.bit.MC0 = 1;                // Enable the compare interrupt
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;              // Enable timer

  #elif ARDUINO_ARCH_SAM
    // TC5 is free when no Servo is used, use TC5 like for Mega
    pmc_set_writeprotect(false);                            // Power management controller (PMC) disables the write protection of the timer and counter registers
    pmc_enable_periph_clk((uint32_t)TC5_IRQn);              // Power management controller (PMC) enables peripheral clock for TC5

    setSamplingPeriod(microseconds);              // Set sampling time and prescalers
    #ifdef ECHO_TO_SERIAL
      if (!setSamplingPeriod(microseconds))
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
    #endif

    TC_Start(TC1, 2);                                 //  Start clock, for the TC1 block, channel 2 = TC5
    TC1->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;          //  enable interrupt for the TC1 block, channel 2 = TC5
    TC1->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;         //  remove disable interrupt for the TC1 block, channel 2 = TC5
    NVIC_EnableIRQ(TC5_IRQn);                         //  ISR for the interrupt

  #elif ARDUINO_ARCH_RENESAS_UNO
    uint8_t timer_type = GPT_TIMER;
    int8_t tindex = FspTimer::get_available_timer(timer_type);
    if (tindex < 0){
      tindex = FspTimer::get_available_timer(timer_type, true);
    }
    if (tindex < 0){
      #ifdef ECHO_TO_SERIAL
        Serial.println("NO timer available");
      #endif
    }
    if(tindex < GTP32_HOWMANY) { // if you get 32bit timer set correct resolution
        /* timer a 32 BIT */
        timerResolution = 4294967296;
    }
      
    if (!setSamplingPeriod(microseconds)){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
      #endif
    }
    FspTimer::force_use_of_pwm_reserved_timer();

    if(!sampling_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, period_counts, 0.0f, prescaler, GPTimerCbk)){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: did not begin");
      #endif
    }
    if (!sampling_timer.setup_overflow_irq()){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: setup overflov irq failed");
      #endif
    }

    if (!sampling_timer.open()){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: timer did not open");
      #endif
    }
    if (!sampling_timer.start()){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: timer did not start");
      #endif
    }
  #elif ARDUINO_ARCH_STM32
    sampling_timer->setOverflow((uint32_t)microseconds, MICROSEC_FORMAT);

  #else
    #error "Architecture not supported."
  #endif
}

bool SamplingNoServo::SamplingClass::setSamplingPeriod(unsigned long microseconds) {
  const unsigned long int cycles = microseconds * cpuFrequency;

  #ifdef ARDUINO_AVR_UNO
    if (cycles < timerResolution) {
      TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // no prescaling
      OCR1A = cycles - 1;                                // compare match register
    }
    else if (cycles < timerResolution * 8) {
      TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler
      OCR1A = (cycles / 8) - 1;                          // compare match register
    }
    else if (cycles < timerResolution * 64) {
      TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); // 64 prescaler
      OCR1A = (cycles / 64) - 1;                         // compare match register
    }
    else if (cycles < timerResolution * 256) {
      TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10); // 256 prescaler
      OCR1A = (cycles / 256) - 1;                        // compare match register
    }
    else if (cycles < timerResolution * 1024) {
      TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // 1024 prescaler
      OCR1A = (cycles / 1024) - 1;                       // compare match register
    }
    else if (cycles >= timerResolution * 1024) {
      TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // 8 prescaler
      OCR1A = (COMPARE_10MS_16B) - 1;                     // compare match register to 10 ms
      fireFlag = 1;                                       // repeat firing
      fireResolution = 10000;                             // resolution in us
    }

  #elif ARDUINO_AVR_MEGA2560
    // For AVR-based board - Mega, run on timer 5
    if (cycles < timerResolution) {                       // max. 4096 us, 62.5 ns
      TCCR5B |= (0 << CS52) | (0 << CS51) | (1 << CS50); // no prescaling
      OCR5A = cycles - 1;                                 // compare match register
    }
    else if (cycles < timerResolution * 8) {              // max. 32768 us, 0.5 us
      TCCR5B |= (0 << CS52) | (1 << CS51) | (0 << CS50);  // 8 prescaler
      OCR5A = (cycles / 8) - 1;                           // compare match register
    }
    else if (cycles < timerResolution * 64) {             // max. 262.144 ms, 4 us
      TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);  // 64 prescaler
      OCR5A = (cycles / 64) - 1;                          // compare match register
    }
    else if (cycles < timerResolution * 256) {            // max. 1.048576 s, 16 us
      TCCR5B |= (1 << CS52) | (0 << CS51) | (0 << CS50);  // 256 prescaler
      OCR5A = (cycles / 256) - 1;                         // compare match register
    }
    else if (cycles < timerResolution * 1024) {           // max. 4.194304 s, 64 us
      TCCR5B |= (1 << CS52) | (0 << CS51) | (1 << CS50); // 1024 prescaler
      OCR5A = (cycles / 1024) - 1;                        // compare match register
    }

    else if (cycles >= timerResolution * 1024) {
      TCCR5B |= (0 << CS52) | (1 << CS51) | (0 << CS50);  // 8 prescaler
      OCR5A = (COMPARE_10MS_16B) - 1;                         // compare match register to 10 ms
      fireFlag = 1;                                       // repeat firing
      fireResolution = 10000;                             // resolution in us
    }


  #elif (defined(ARDUINO_SAMD_ZERO) || defined (ADAFRUIT_METRO_M4_EXPRESS))    // For SAMD21G boards, e.g. Zero
    // Up to 1.3653 ms with 20.8 ns resolution
    if (cycles < timerResolution) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;    // no prescaling
      TC5->COUNT16.CC[0].reg = (uint32_t)cycles - 1;        // compare match register
    }
    // Up to 2.7307 ms with 41.67 ns resolution
    else if (cycles < timerResolution * 2) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;    // 2 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 2) - 1;  // compare match register
    }
    // Up to 5.4613 ms with 83.3 ns resolution
    else if (cycles < timerResolution * 4) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4;    // 4 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 4) - 1;  // compare match register
    }
    // Up to 10.9227 ms with 166.67 ns resolution
    else if (cycles < timerResolution * 8) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;    //  8 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 8) - 1;  // compare match register
    }
    // Up to 21.8453 ms with 333.33 ns resolution
    else if (cycles < timerResolution * 16) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;   //  16 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 16) - 1; // compare match register
    }
    // Up to  87.3813 ms with 1.33 us resolution
    else if (cycles < timerResolution * 64) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64; //  64 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 64) - 1; // compare match register
    }
    // Up to 349.5253 ms with 5.33 us resolution
    else if (cycles < timerResolution * 256) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;  //  256 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 256) - 1; // compare match register
    }
    // Up to 1.3981 s with 21.3333 us resolution
    else if (cycles < timerResolution * 1024) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; //  1024 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 1024) - 1; // compare match register
    }
    // Over 1.3981 s with 10 ms resolution
    else if (cycles >= timerResolution * 1024) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;    // 8 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(COMPARE_10MS) - 1; // compare match register
      fireFlag = 1;                                         // repeat firing
      fireResolution = 10000;                               // resolution in us
    }

    else {
      return false;
    }

  #elif ARDUINO_ARCH_SAM                     // For SAM boards, e.g. DUE
    // 32 bit timer w/ prescaler 2 = 143 minutes max period
    TC_Configure(TC1, 2, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
    TC_SetRC(TC1, 2, (uint32_t)(cycles / 2) - 1);               // Prescaler 2

  #elif ARDUINO_ARCH_RENESAS_UNO

    if(cycles / 1 < timerResolution) {
        period_counts = (uint32_t) (cycles / 1.0);
        prescaler = TIMER_SOURCE_DIV_1;
    }
    else if(cycles / 4 < timerResolution) {
        period_counts = (uint32_t) (cycles / 4.0);
        prescaler = TIMER_SOURCE_DIV_4;
    }
    else if(cycles / 16 < timerResolution) {
        period_counts = (uint32_t) (cycles / 16.0 );
        prescaler = TIMER_SOURCE_DIV_16;
    }
    else if(cycles / 64 < timerResolution) {
        period_counts = (uint32_t) (cycles / 64.0 );
        prescaler = TIMER_SOURCE_DIV_64;
    }
    else if(cycles / 256 < timerResolution) {
        period_counts = (uint32_t) (cycles / 256.0 );
        prescaler = TIMER_SOURCE_DIV_256;
    }
    else if(cycles / 1024 < timerResolution) {
        period_counts = (uint32_t) (cycles / 1024.0 );
        prescaler = TIMER_SOURCE_DIV_1024;
    }
    else if (cycles / 1024 >= timerResolution){
        period_counts = (uint32_t) (COMPARE_10MS);
        prescaler = TIMER_SOURCE_DIV_16;
        fireFlag = 1;                                         // repeat firing
        fireResolution = 10000;                               // resolution in us
    }
    else {
        return false;
    }
  #elif ARDUINO_ARCH_STM32
  // just nothing to do....

  #else
    #error "Architecture not supported."
  #endif

  samplingPeriod = microseconds / 1000000.0;           // in seconds
  samplingMicroseconds = microseconds;                  //in microseconds
  return true;
}


unsigned long int SamplingNoServo::SamplingClass::getSamplingMicroseconds() {
  return samplingMicroseconds;
}

//unsigned short int SamplingNoServo::SamplingClass::getFireResolution(){
//  return fireResolution;
//}


void SamplingNoServo::SamplingClass::interrupt(p_to_void_func interruptCallback) {
  this->interruptCallback = interruptCallback;
  #if defined(ARDUINO_ARCH_STM32)
  sampling_timer->attachInterrupt(interruptCallback);
  sampling_timer->resume();
  #endif
}

p_to_void_func SamplingNoServo::SamplingClass::getInterruptCallback () {
  return interruptCallback;
}


////////////////////////////////////// SERVO //////////////////////////////////////


void SamplingServo::SamplingClass::defaultInterrupt()
{
}

SamplingServo::SamplingClass::SamplingClass() {
  interruptCallback = defaultInterrupt;
  samplingPeriod = 0.0;
}

void SamplingServo::SamplingClass::period(unsigned long microseconds) {
  #if (defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560))
    // for Arduino Uno use Timer2 to stay compatible with the Servo library
    noInterrupts();                             // disable all interrupts

    TCCR2A = 0;                                 // clear register
    TCCR2B = 0;                                 // clear register
    TCNT2  = 0;                                 // clear register

    TCCR2A |= (1 << WGM21);                     // CTC mode
    TIMSK2 |= (1 << OCIE2A);                    // enable timer compare interrupt

    setSamplingPeriod(microseconds);            // Set prescalers

    interrupts();                               // enable all interrupts

  #elif ARDUINO_SAMD_ZERO
    // For Arduino Zero
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;        // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;        // Set TC5 mode as match frequency

    setSamplingPeriod(microseconds);
    #ifdef ECHO_TO_SERIAL
      if (!setSamplingPeriod(microseconds))
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
    #endif

    NVIC_EnableIRQ(TC5_IRQn);                               // Enable interrupt for TC5
    TC5->COUNT16.INTENSET.bit.MC0 = 1;                      // Enable the TC5 interrupt request
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;              // Enable timer

  #elif ADAFRUIT_METRO_M4_EXPRESS
    // For Adafruit M4 Express
    GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    TC5->COUNT16.CTRLA.bit.ENABLE = 0;
    TC5->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ; // Match mode, counter resets at match

    setSamplingPeriod(microseconds);
    #ifdef ECHO_TO_SERIAL
      if (!setSamplingPeriod(microseconds))
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
    #endif

    NVIC_EnableIRQ(TC5_IRQn);                               // Enable interrupt for TC5
    TC5->COUNT16.INTENSET.bit.MC0 = 1;                // Enable the compare interrupt
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;              // Enable timer

  #elif ARDUINO_ARCH_SAM
    // Due has three 3-channel general-purpose 32-bit timers = 9 timers. TC1 serves PWM 60,61, these will be disabled
    pmc_set_writeprotect(false);                            // Power management controller (PMC) disables the write protection of the timer and counter registers
    pmc_enable_periph_clk((uint32_t)TC1_IRQn);              // Power management controller (PMC) enables peripheral clock for TC1

    setSamplingPeriod(microseconds);              // Set sampling time and prescalers
    #ifdef ECHO_TO_SERIAL
      if (!setSamplingPeriod(microseconds))
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
    #endif

    TC_Start(TC0, 1);                                 //  Start clock, for the TC0 block, channel 1 = TC1
    TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;          //  enable interrupt for the TC0 block, channel 1 = TC1
    TC0->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;         //  remove disable interrupt for the TC0 block, channel 1 = TC1
    NVIC_EnableIRQ(TC1_IRQn);                         //  ISR for the interrupt
  #elif ARDUINO_ARCH_RENESAS_UNO
    uint8_t timer_type = GPT_TIMER;
    int8_t tindex = FspTimer::get_available_timer(timer_type);
    if (tindex < 0){
      tindex = FspTimer::get_available_timer(timer_type, true);
    }
    if (tindex < 0){
      #ifdef ECHO_TO_SERIAL
        Serial.println("NO timer available");
      #endif
    }
    if(tindex < GTP32_HOWMANY) { // if you get 32bit timer set correct resolution
        /* timer a 32 BIT */
        timerResolution = 4294967296;
    }
      
    if (!setSamplingPeriod(microseconds)){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling period is too long.\nMax is xxxx microseconds.");
      #endif
    }
    FspTimer::force_use_of_pwm_reserved_timer();

    if(!sampling_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, period_counts, 0.0f, prescaler, GPTimerCbk)){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: did not begin");
      #endif
    }
    if (!sampling_timer.setup_overflow_irq()){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: setup overflov irq failed");
      #endif
    }

    if (!sampling_timer.open()){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: timer did not open");
      #endif
    }
    if (!sampling_timer.start()){
      #ifdef ECHO_TO_SERIAL
        Serial.println("Sampling error: timer did not start");
      #endif
    }

  #elif ARDUINO_ARCH_STM32
    sampling_timer->setOverflow((uint32_t)microseconds, MICROSEC_FORMAT);

  #else
    #error "Architecture not supported."
  #endif
}

bool SamplingServo::SamplingClass::setSamplingPeriod(unsigned long microseconds) {
  const unsigned long cycles = microseconds * cpuFrequency;

  #if (defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560))
    // For AVR-based boards, e.g. Uno
    if (cycles < timerResolution) {                      // max. 16 us, error 62.5 ns
      TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20); // no prescaling
      OCR2A = cycles - 1;                                // compare match register
    }
    else if (cycles < timerResolution * 8) {             // max. 128 us, error 0.5 us
      TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20); // 8 prescaler
      OCR2A = (cycles / 8) - 1;                          // compare match register
    }
    else if (cycles < timerResolution * 32) {            // max. 512 us, error, 2 us
      TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20); // 32 prescaler
      OCR2A = (cycles / 32) - 1;                         // compare match register
    }
    else if (cycles < timerResolution * 64) {            // max. 1024 us, error, 4 us
      TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20); // 64 prescaler
      OCR2A = (cycles / 64) - 1;                         // compare match register
    }
    else if (cycles < timerResolution * 128) {           // max. 2048 us, error, 8 us
      TCCR2B |= (1 << CS22) | (0 << CS21) | (1 << CS20); // 128 prescaler
      OCR2A = (cycles / 128) - 1;                        // compare match register
    }
    else if (cycles < timerResolution * 256) {           // max. 4096 us, 16 us
      TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // 256 prescaler
      OCR2A = (cycles / 256) - 1;                        // compare match register
    }
    else if (cycles < timerResolution * 1024) {          // max. 16384 us, 64 us
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
      OCR2A = (cycles / 1024) - 1;                       // compare match register
    }
    // Use 0.1 ms precision up to 100 ms
    else if (cycles < CYCLES_100MS) {                    // max. 100 ms, error 0.1 ms
      TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20); // 8 prescaler
      OCR2A = (COMPARE_100US) - 1;                       // compare match register
      fireFlag = 1;                                      // repeat firing
      fireResolution = 100;                              // resolution in us
    }
    // Use 1 ms precision up to 1 s
    else if (cycles < CYCLES_1S) {                       // max. 1 s, error 1 ms
      TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20); // 64 prescaler
      OCR2A = (COMPARE_1MS) - 1;                         // compare match register
      fireFlag = 1;                                      // repeat firing
      fireResolution = 1000;                             // resolution in us
    }
    // Use ~10 ms precision over 1 s
    else if (cycles >= CYCLES_1S) {
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024 prescaler
      OCR2A = (COMPARE_10MS) - 1;                        // compare match register
      fireFlag = 1;                                      // repeat firing
      fireResolution = 10000;                            // resolution in us
    }

  #elif (defined(ARDUINO_SAMD_ZERO) || defined (ADAFRUIT_METRO_M4_EXPRESS))   // For SAMD21G boards, e.g. Zero
    // Up to 1.3653 ms with 20.8 ns resolution
    if (cycles < timerResolution) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;    // no prescaling
      TC5->COUNT16.CC[0].reg = (uint32_t)cycles - 1;        // compare match register
    }
    // Up to 2.7307 ms with 41.67 ns resolution
    else if (cycles < timerResolution * 2) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;    // 2 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 2) - 1;  // compare match register
    }
    // Up to 5.4613 ms with 83.3 ns resolution
    else if (cycles < timerResolution * 4) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4;    // 4 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 4) - 1;  // compare match register
    }
    // Up to 10.9227 ms with 166.67 ns resolution
    else if (cycles < timerResolution * 8) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;    //  8 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 8) - 1;  // compare match register
    }
    // Up to 21.8453 ms with 333.33 ns resolution
    else if (cycles < timerResolution * 16) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;   //  16 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 16) - 1; // compare match register
    }
    // Up to  87.3813 ms with 1.33 us resolution
    else if (cycles < timerResolution * 64) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64; //  64 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 64) - 1; // compare match register
    }
    // Up to 349.5253 ms with 5.33 us resolution
    else if (cycles < timerResolution * 256) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;  //  256 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 256) - 1; // compare match register
    }
    // Up to 1.3981 s with 21.3333 us resolution
    else if (cycles < timerResolution * 1024) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; //  1024 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(cycles / 1024) - 1; // compare match register
    }
    // Over 1.3981 s with 10 ms resolution
    else if (cycles >= timerResolution * 1024) {
      TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;    // 8 prescaler
      TC5->COUNT16.CC[0].reg = (uint32_t)(COMPARE_10MS) - 1; // compare match register
      fireFlag = 1;                                         // repeat firing
      fireResolution = 10000;                               // resolution in us
    }

    else {
      return false;
    }

  #elif ARDUINO_ARCH_SAM                     // For SAM boards, e.g. DUE
    // 32 bit timer w/ prescaler 2 = 143 minutes max period
    TC_Configure(TC0, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
    TC_SetRC(TC0, 1, (uint32_t)(cycles / 2) - 1);               // Prescaler 2

  #elif ARDUINO_ARCH_RENESAS_UNO

    if(cycles / 1.0 < timerResolution) {
        period_counts = (uint32_t) (cycles / 1.0);
        prescaler = TIMER_SOURCE_DIV_1;
    }
    else if(cycles / 4.0 < timerResolution) {
        period_counts = (uint32_t) (cycles / 4.0);
        prescaler = TIMER_SOURCE_DIV_4;
    }
    else if(cycles / 16.0 < timerResolution) {
        period_counts = (uint32_t) (cycles / 16.0 );
        prescaler = TIMER_SOURCE_DIV_16;
    }
    else if(cycles / 64.0 < timerResolution) {
        period_counts = (uint32_t) (cycles / 64.0 );
        prescaler = TIMER_SOURCE_DIV_64;
    }
    else if(cycles / 256.0 < timerResolution) {
        period_counts = (uint32_t) (cycles / 256.0 );
        prescaler = TIMER_SOURCE_DIV_256;
    }
    else if(cycles / 1024.0 < timerResolution) {
        period_counts = (uint32_t) (cycles / 1024.0 );
        prescaler = TIMER_SOURCE_DIV_1024;
    }
    else if (cycles / 1024.0 >= timerResolution){
        period_counts = (uint32_t) (COMPARE_10MS);
        prescaler = TIMER_SOURCE_DIV_16;
        fireFlag = 1;                                         // repeat firing
        fireResolution = 10000;                               // resolution in us
    }
    else {
        return false;
    }

    #elif ARDUINO_ARCH_STM32
    // nothing to do...

  #else
    #error "Architecture not supported."
  #endif

  samplingPeriod = microseconds / 1000000.0;           // in seconds
  samplingMicroseconds = microseconds;                  //in microseconds
  return true;
}

float SamplingServo::SamplingClass::getSamplingPeriod() {
  return samplingPeriod;
}

unsigned long int SamplingServo::SamplingClass::getSamplingMicroseconds() {
  return samplingMicroseconds;
}

//unsigned short int SamplingServo::SamplingClass::getFireResolution(){
//  return fireResolution;
//}



void SamplingServo::SamplingClass::interrupt(p_to_void_func interruptCallback) {
  this->interruptCallback = interruptCallback;
  #if defined(ARDUINO_ARCH_STM32)
    sampling_timer->attachInterrupt(interruptCallback);
    sampling_timer->resume();
  #endif
}

p_to_void_func SamplingServo::SamplingClass::getInterruptCallback () {
  return interruptCallback;
}

