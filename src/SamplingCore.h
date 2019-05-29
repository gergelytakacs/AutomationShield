/*
  Class declaration for Interrupt-driven sampling for real-time control.
  
  The file forward declares the classes necessary for configuring
  an interrupt-driven system for deploying digital control systems
  on AVR, SAMD and SAM-based Arduino prototyping boards with the 
  R3 pinout. The module should be compatible with the Uno, Mega 
  2560, Arduino Zero and Arduino Due. There should be no timer 
  conflicts when using the Servo library.
  
  This file contains the forward declarations of two classes 
  having an identical name (SamplingClass), that are separated
  into two namespaces. The actual interrupt service routines are
  located in a pair of outside headers. A "Sampling" object is 
  created by calling the correct header file depending whethe
  your system uses the Servo library or not, first by invoking 
  the correct namespace, then implementing the interrupt routine.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons

  AVR Timer: 	 Gergely Takacs, Richard Koplinger, Matus Biro,
                 Lukas Vadovic 2018-2019
  SAMD21G Timer: Gergely Takacs, 2019 (Zero)
  SAM3X Timer:   Gergely Takacs, 2019 (Due)
  Last update: 28.5.2019.
*/

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include "Arduino.h"

typedef void (*p_to_void_func)(); /*define a term p_to_void_func for pointer to function, which 
								  has a return type void and has no input parameters*/
namespace SamplingNoServo{

class SamplingClass{
  public:
    SamplingClass();
    void period(unsigned long microseconds); 
    void interrupt(p_to_void_func interruptCallback);
    p_to_void_func getInterruptCallback ();
    float getSamplingPeriod();  
    unsigned long int getSamplingMicroseconds(); 
    float samplingPeriod; // Sampling period in seconds  
	
    /*
    In case the required period is larger than what the given
    timer resolution can handle with the highes prescaler (fireFlag=1),
    the ISR will "fire" without launching the algorithm step until
    (fireCount) the desired sampling is reached, then the conter is
    reset. The "firing" of the ISR is a portion of the full sampling
    by "fireResolution" microseconds.
    */    
    #if (defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_ARCH_SAMD))
      bool fireFlag = 0;                                     // Repeat launches of ISR
      unsigned long int  fireCount = 0;                      // Counter for repeat launches of ISR
      unsigned short int fireResolution;                     // Resolution of the timer over the maximum
    #endif
       
  private:
    static void defaultInterrupt();
    p_to_void_func interruptCallback;    

    #ifdef ARDUINO_AVR_UNO 	
	  // Default: Timer1
	  const unsigned long timerResolution = 65536; 		 // AVR Timer 5 is 16bit            
	  const unsigned char cpuFrequency = 16; 			 // CPU frequency in micro Hertz*/
      // For Timer 2 (Servo)
	  #define CYCLES_100MS   16000000                    // CPU cycles @ 16 MHz for 100 ms
      #define CYCLES_1S     160000000                    // CPU cycles @ 16 MHz for 1 s
      #define COMPARE_100US       200                    // Compare @ 16 MHz, prescaler 8, for 100 us
      #define COMPARE_1MS         250                    // Compare @ 16 MHz, prescaler 64, for 1 ms
      #define COMPARE_10MS        156                    // Compare @ 16 MHz, prescaler 1024, for 10 ms
      // For Timer 1 (No servo)
      #define COMPARE_10MS_16B    20000                  // Compare @ 16 MHz, prescaler 8, for 10 ms
    #elif ARDUINO_AVR_MEGA2560
      // Default: Timer5 in both cases (No servo and servo)
	  const unsigned long timerResolution = 65536; 		 // AVR Timer 5 is 16bit            
      const unsigned char cpuFrequency = 16; 			 // CPU frequency in micro Hertz*/
	  #define COMPARE_10MS        20000                      // Compare @ 16 MHz, prescaler 8, for 10 ms
    #elif ARDUINO_ARCH_SAMD
	  // Default: Timer5 in both cases (No servo and servo)
	  const unsigned long timerResolution = 65536;     			// Configured to 16bit  
	  const unsigned char cpuFrequency = VARIANT_MCK/1000000;	// CPU frequency in micro Hertz (48 for Zero)
	  #define COMPARE_10MS        60000                      // Compare @ 48 MHz, prescaler 8, for 10 ms
    #elif ARDUINO_ARCH_SAM
      // Default: Timer5 in both cases (No servo and servo)
	  const unsigned char cpuFrequency = VARIANT_MCK/1000000;	// CPU frequency in micro Hertz (84 for Due)
    #else
		  #error "Architecture not supported."
    #endif


    unsigned long int samplingMicroseconds; // Sampling period in microseconds 
    bool setSamplingPeriod(unsigned long microseconds);
};
} // end of namespace

namespace SamplingServo{

class SamplingClass{
  public:
    SamplingClass();
    void period(unsigned long microseconds); 
    void interrupt(p_to_void_func interruptCallback);
    p_to_void_func getInterruptCallback ();
    float getSamplingPeriod();  
    unsigned long int getSamplingMicroseconds(); 
    float samplingPeriod; // Sampling period in seconds 
	
    /*
    In case the required period is larger than what the given
    timer resolution can handle with the highes prescaler (fireFlag=1),
    the ISR will "fire" without launching the algorithm step until
    (fireCount) the desired sampling is reached, then the conter is
    reset. The "firing" of the ISR is a portion of the full sampling
    by "fireResolution" microseconds.
    */    
    #if (defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_ARCH_SAMD))
      bool fireFlag = 0;                                     // Repeat launches of ISR
      unsigned long int  fireCount = 0;                      // Counter for repeat launches of ISR
      unsigned short int fireResolution;                     // Resolution of the timer over the maximum
    #endif
       
  private:
    static void defaultInterrupt();
    p_to_void_func interruptCallback;    

    #ifdef ARDUINO_AVR_UNO 	
		  // Default: Timer2
		  const unsigned long timerResolution = 256; 						 // AVR Timer 2 is 8bit            
		  const unsigned char cpuFrequency = 16; 							   // CPU frequency in micro Hertz
      
    #elif ARDUINO_AVR_MEGA2560
      // Default: Timer5
	      const unsigned long timerResolution = 65536; 					 // AVR Timer 5 is 16bit            
    	  const unsigned char cpuFrequency = 16; 							   // CPU frequency in micro Hertz*/
     
    #elif ARDUINO_ARCH_SAMD
		  // Default TC5 (randomly selected, take care of Servo!)
		  const unsigned long timerResolution = 65536;     				// Configured to 16bit  
		  const unsigned char cpuFrequency = VARIANT_MCK/1000000;	// CPU frequency in micro Hertz (48 for Zero)
    #elif ARDUINO_ARCH_SAM
		  // Default TC3 (randomly selected, take care of Servo!)
		  const unsigned char cpuFrequency = VARIANT_MCK/1000000;	// CPU frequency in micro Hertz (84 for Due)
    #else
		  #error "Architecture not supported."
    #endif

 
    unsigned long int samplingMicroseconds; // Sampling period in microseconds 
    bool setSamplingPeriod(unsigned long microseconds);
};
} // end of namespace

#endif
