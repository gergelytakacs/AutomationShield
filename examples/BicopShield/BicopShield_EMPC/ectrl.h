/*
    MATRICES FOR EXPLICIT MPC POLYTOPES AND CORRESPONDING PWA LAWS

 	  The automatically generated set of matrices have been adapted from 
    the C code auto-generation routine of the Multi-Parametric Toolbox 3 
    MPT3) of Kvasnica et al. for  MATLAB. Please see http://www.mpt3.org 
    for more information. Please see the original "toC.m" for more details
    in the MPT3.

    The following C code has been slightly adapted to perform microcontroller
    architecutre-specific changes for AVR and ARM Cortex-A devices.

    The list of changes from the original are as follows:
    - Changed "static" modifiers to "const" so microcontrollers will
    prefer to use ROM instead of RAM.
    - Added PROGMEM functionality for the AVR architecture, and calling
    the necessary headers.
    - Removed the possibility to use PWQ, this only handles PWA.
    - Removed matrix export for the tie-breaking function.

    Usage: include alongside with the empcSequential.h header to your
    example.

    This code snippet has been altered for the needs of the
    the AutomationShield hardware and software ecosystem. 
    Visit http://www.automationshield.com for more details.


    Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) 
    Revised in 2012-2013 by Martin Herceg (herceg@control.ee.ethz.ch)    
    Adapted for MCU use by Gergely Tak�cs in 2020 (gergely.takacs@stuba.sk)
*/
#include <avr/pgmspace.h> 

#define MPT_NR 5
#define MPT_DOMAIN 3
#define MPT_RANGE 2
#define MPT_ABSTOL 1.000000e-08

const float MPT_A[] PROGMEM = {
2.9635436e-02,	-9.8775575e-01,	-1.5316764e-01,	-7.0710678e-01,	7.0710678e-01,	
0.0000000e+00,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	-1.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	-1.0000000e+00,	
1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
1.0000000e+00,	-2.9635436e-02,	9.8775575e-01,	1.5316764e-01,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
-1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
-1.0000000e+00,	1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	1.0000000e+00,	2.9635436e-02,	-9.8775575e-01,	-1.5316764e-01,	
-2.9635436e-02,	9.8775575e-01,	1.5316764e-01,	2.9635436e-02,	-9.8775575e-01,	
-1.5316764e-01,	-2.9635436e-02,	9.8775575e-01,	1.5316764e-01,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
-1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
-1.0000000e+00,	1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	1.0000000e+00,	-2.9635436e-02,	9.8775575e-01,	1.5316764e-01,	
-7.0710678e-01,	7.0710678e-01,	0.0000000e+00,	0.0000000e+00,	-9.9995024e-01,	
-9.9758962e-03,	0.0000000e+00,	-1.0000000e+00,	0.0000000e+00,	7.0710678e-01,	
-7.0710678e-01,	0.0000000e+00,	-1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	-1.0000000e+00,	1.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	1.0000000e+00,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
0.0000000e+00,	9.9995024e-01,	9.9758962e-03,	0.0000000e+00,	1.0000000e+00,	
0.0000000e+00,	-1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	-1.0000000e+00,	1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	1.0000000e+00,	2.9635436e-02,	-9.8775575e-01,	
-1.5316764e-01 };

const float MPT_B[] PROGMEM = {
1.2172140e-01,	7.0710678e+03,	7.0710678e+03,	1.0000000e+04,	1.0000000e+04,	
1.0000000e+04,	1.0000000e+04,	1.2172140e-01,	7.0710678e+03,	7.0710678e+03,	
1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	1.4834728e-01,	
-1.2172140e-01,	-1.2172140e-01,	1.4834728e-01,	7.0710678e+03,	7.0710678e+03,	
1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	-1.4834728e-01,	
7.0710678e+03,	9.9996259e+03,	1.0000000e+04,	7.0710678e+03,	1.0000000e+04,	
1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	7.0710678e+03,	7.0710678e+03,	
9.9996259e+03,	1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	1.0000000e+04,	
1.0000000e+04,	-1.4834728e-01 };

const int MPT_NC[] PROGMEM = {
8,	8,	8,	9,	9 
};


const float MPT_F[] PROGMEM = {
5.9124557e+00,	-1.9706348e+02,	-3.0557908e+01,	-7.3040818e+00,	2.4344669e+02,	
3.7750381e+01,	6.3618820e+00,	-2.1204296e+02,	-3.2880720e+01,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	6.3618820e+00,	-2.1204296e+02,	-3.2880720e+01,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00 
};


const float MPT_G[] PROGMEM = {
-3.5527137e-15,	0.0000000e+00,	-1.8459251e+00,	-3.0000000e+01,	1.8459251e+00,	
3.0000000e+01,	3.0000000e+01,	-3.0000000e+01,	-3.0000000e+01,	3.0000000e+01 
};

