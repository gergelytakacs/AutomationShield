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

#define MPT_NR 53
#define MPT_DOMAIN 3
#define MPT_RANGE 3
#define MPT_ABSTOL 1.000000e-08

const float MPT_A[] PROGMEM = {
1.1914103e-01,	9.3524161e-01,	-3.3335949e-01,	5.4304906e-02,	9.9850318e-01,	
6.5102908e-03,	-1.1914103e-01,	-9.3524161e-01,	3.3335949e-01,	-5.4304906e-02,	
-9.9850318e-01,	-6.5102908e-03,	-4.8654566e-01,	8.7310868e-01,	3.0895800e-02,	
7.3192265e-02,	9.9405074e-01,	8.0659839e-02,	6.0008674e-02,	9.9795540e-01,	
-2.1999333e-02,	-6.0008674e-02,	-9.9795540e-01,	2.1999333e-02,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	
4.3708884e-01,	-8.9611102e-01,	-7.7060958e-02,	0.0000000e+00,	0.0000000e+00,	
-1.0000000e+00,	-1.1914103e-01,	-9.3524161e-01,	3.3335949e-01,	7.0216447e-02,	
9.9213941e-01,	1.0358106e-01,	9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	
-9.9687183e-02,	-9.6869111e-01,	2.2737632e-01,	-4.8157716e-01,	8.7586630e-01,	
3.0686374e-02,	-4.1713719e-01,	9.0717882e-01,	5.4983259e-02,	-5.4304906e-02,	
-9.9850318e-01,	-6.5102908e-03,	6.8097344e-02,	9.9584033e-01,	6.0537499e-02,	
6.0008674e-02,	9.9795540e-01,	-2.1999333e-02,	-6.0008674e-02,	-9.9795540e-01,	
2.1999333e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	-4.3708884e-01,	
8.9611102e-01,	7.7060958e-02,	1.1914103e-01,	9.3524161e-01,	-3.3335949e-01,	
7.0216447e-02,	9.9213941e-01,	1.0358106e-01,	9.9687183e-02,	9.6869111e-01,	
-2.2737632e-01,	-9.9687183e-02,	-9.6869111e-01,	2.2737632e-01,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	-4.8157716e-01,	8.7586630e-01,	3.0686374e-02,	
5.6290723e-03,	-9.9950562e-01,	-3.0932761e-02,	4.1713719e-01,	-9.0717882e-01,	
-5.4983259e-02,	-1.1776592e-02,	-9.9587831e-01,	-8.9931606e-02,	-1.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	5.4304906e-02,	9.9850318e-01,	6.5102908e-03,	
6.8097344e-02,	9.9584033e-01,	6.0537499e-02,	4.9766819e-01,	-8.6648396e-01,	
-3.9139642e-02,	-3.8828427e-01,	9.2115292e-01,	2.6695012e-02,	-4.9766819e-01,	
8.6648396e-01,	3.9139642e-02,	3.8828427e-01,	-9.2115292e-01,	-2.6695012e-02,	
-7.0710678e-01,	7.0710678e-01,	0.0000000e+00,	4.8654566e-01,	-8.7310868e-01,	
-3.0895800e-02,	1.1545758e-01,	9.4874839e-01,	-2.9418708e-01,	5.0947360e-02,	
9.9867989e-01,	-6.5453920e-03,	7.3192265e-02,	9.9405074e-01,	8.0659839e-02,	
-1.1545758e-01,	-9.4874839e-01,	2.9418708e-01,	-5.0947360e-02,	-9.9867989e-01,	
6.5453920e-03,	-7.3192265e-02,	-9.9405074e-01,	-8.0659839e-02,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	
-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	0.0000000e+00,	0.0000000e+00,	-1.0000000e+00,	0.0000000e+00,	
9.9397557e-01,	-1.0960186e-01,	-9.9687183e-02,	-9.6869111e-01,	2.2737632e-01,	
-6.0008674e-02,	-9.9795540e-01,	2.1999333e-02,	6.7333052e-02,	9.9548838e-01,	
6.6851608e-02,	-7.0710678e-01,	7.0710678e-01,	0.0000000e+00,	-4.9766819e-01,	
8.6648396e-01,	3.9139642e-02,	0.0000000e+00,	-9.9985275e-01,	-1.7160566e-02,	
4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	0.0000000e+00,	-9.9133553e-01,	
-1.3135399e-01,	-1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	-1.0000000e+00,	-9.9687183e-02,	-9.6869111e-01,	2.2737632e-01,	
6.0008674e-02,	9.9795540e-01,	-2.1999333e-02,	6.7333052e-02,	9.9548838e-01,	
6.6851608e-02,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	-4.6014429e-01,	
8.8407435e-01,	8.1729888e-02,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	
4.0544890e-01,	-9.1109457e-01,	-7.4282359e-02,	3.3965995e-01,	-9.3544880e-01,	
-9.7809340e-02,	0.0000000e+00,	0.0000000e+00,	-1.0000000e+00,	-3.8701242e-01,	
9.1851009e-01,	8.0997515e-02,	-4.3708884e-01,	8.9611102e-01,	7.7060958e-02,	
5.8522982e-02,	9.9751116e-01,	-3.9326228e-02,	7.0216447e-02,	9.9213941e-01,	
1.0358106e-01,	-5.8522982e-02,	-9.9751116e-01,	3.9326228e-02,	-7.0216447e-02,	
-9.9213941e-01,	-1.0358106e-01,	4.3986250e-01,	-8.9477242e-01,	-7.6832920e-02,	
-1.1545758e-01,	-9.4874839e-01,	2.9418708e-01,	-4.9766819e-01,	8.6648396e-01,	
3.9139642e-02,	-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	-6.0008674e-02,	
-9.9795540e-01,	2.1999333e-02,	9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	
6.7333052e-02,	9.9548838e-01,	6.6851608e-02,	4.9766819e-01,	-8.6648396e-01,	
-3.9139642e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	-7.0710678e-01,	
7.0710678e-01,	0.0000000e+00,	5.1641740e-01,	-8.5616784e-01,	-1.7020409e-02,	
-4.4710140e-01,	8.9420280e-01,	2.2398347e-02,	3.8828427e-01,	-9.2115292e-01,	
-2.6695012e-02,	6.7926039e-01,	7.3049132e-01,	7.0624043e-02,	4.8157716e-01,	
-8.7586630e-01,	-3.0686374e-02,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	
-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	-5.1641740e-01,	8.5616784e-01,	
1.7020409e-02,	2.0932110e-01,	9.7604219e-01,	5.9382902e-02,	4.1713719e-01,	
-9.0717882e-01,	-5.4983259e-02,	9.8718135e-02,	9.7104096e-01,	-2.1756421e-01,	
6.8097344e-02,	9.9584033e-01,	6.0537499e-02,	-9.8718135e-02,	-9.7104096e-01,	
2.1756421e-01,	-6.8097344e-02,	-9.9584033e-01,	-6.0537499e-02,	-4.1569772e-01,	
9.0783688e-01,	5.5023742e-02,	-5.0947360e-02,	-9.9867989e-01,	6.5453920e-03,	
-7.0710678e-01,	7.0710678e-01,	0.0000000e+00,	-4.9766819e-01,	8.6648396e-01,	
3.9139642e-02,	-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	0.0000000e+00,	
-9.9397557e-01,	1.0960186e-01,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	
0.0000000e+00,	-9.9985275e-01,	-1.7160566e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	0.0000000e+00,	-9.9133553e-01,	-1.3135399e-01,	-1.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	1.0000000e+00,	
9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	6.0008674e-02,	9.9795540e-01,	
-2.1999333e-02,	6.7333052e-02,	9.9548838e-01,	6.6851608e-02,	4.6014429e-01,	
-8.8407435e-01,	-8.1729888e-02,	-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	
-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	-3.8701242e-01,	9.1851009e-01,	
8.0997515e-02,	4.3708884e-01,	-8.9611102e-01,	-7.7060958e-02,	5.8522982e-02,	
9.9751116e-01,	-3.9326228e-02,	7.0216447e-02,	9.9213941e-01,	1.0358106e-01,	
-5.8522982e-02,	-9.9751116e-01,	3.9326228e-02,	-7.0216447e-02,	-9.9213941e-01,	
-1.0358106e-01,	-4.3986250e-01,	8.9477242e-01,	7.6832920e-02,	1.1545758e-01,	
9.4874839e-01,	-2.9418708e-01,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	
-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	-7.0710678e-01,	7.0710678e-01,	
0.0000000e+00,	-3.8828427e-01,	9.2115292e-01,	2.6695012e-02,	4.8157716e-01,	
-8.7586630e-01,	-3.0686374e-02,	0.0000000e+00,	9.9985275e-01,	1.7160566e-02,	
0.0000000e+00,	-9.9985275e-01,	-1.7160566e-02,	1.6660884e-01,	-9.8516998e-01,	
-4.1007290e-02,	0.0000000e+00,	-9.9874751e-01,	-5.0034050e-02,	-5.6290723e-03,	
9.9950562e-01,	3.0932761e-02,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	
-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	5.1641740e-01,	-8.5616784e-01,	
-1.7020409e-02,	1.6660884e-01,	-9.8516998e-01,	-4.1007290e-02,	2.0932110e-01,	
9.7604219e-01,	5.9382902e-02,	-4.1713719e-01,	9.0717882e-01,	5.4983259e-02,	
0.0000000e+00,	-9.9133553e-01,	-1.3135399e-01,	0.0000000e+00,	9.9133553e-01,	
1.3135399e-01,	0.0000000e+00,	-9.9874751e-01,	-5.0034050e-02,	-1.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	1.1776592e-02,	9.9587831e-01,	8.9931606e-02,	
9.8718135e-02,	9.7104096e-01,	-2.1756421e-01,	6.8097344e-02,	9.9584033e-01,	
6.0537499e-02,	-9.8718135e-02,	-9.7104096e-01,	2.1756421e-01,	-6.8097344e-02,	
-9.9584033e-01,	-6.0537499e-02,	4.1569772e-01,	-9.0783688e-01,	-5.5023742e-02,	
5.0947360e-02,	9.9867989e-01,	-6.5453920e-03,	1.1914103e-01,	9.3524161e-01,	
-3.3335949e-01,	5.4304906e-02,	9.9850318e-01,	6.5102908e-03,	-1.1914103e-01,	
-9.3524161e-01,	3.3335949e-01,	-5.4304906e-02,	-9.9850318e-01,	-6.5102908e-03,	
4.8654566e-01,	-8.7310868e-01,	-3.0895800e-02,	-7.3192265e-02,	-9.9405074e-01,	
-8.0659839e-02,	6.7333052e-02,	9.9548838e-01,	6.6851608e-02,	-6.7333052e-02,	
-9.9548838e-01,	-6.6851608e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	
-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	0.0000000e+00,	0.0000000e+00,	-1.0000000e+00,	0.0000000e+00,	
9.9397557e-01,	-1.0960186e-01,	-9.8718135e-02,	-9.7104096e-01,	2.1756421e-01,	
-5.8522982e-02,	-9.9751116e-01,	3.9326228e-02,	6.7333052e-02,	9.9548838e-01,	
6.6851608e-02,	-6.7333052e-02,	-9.9548838e-01,	-6.6851608e-02,	4.6014429e-01,	
-8.8407435e-01,	-8.1729888e-02,	-9.8718135e-02,	-9.7104096e-01,	2.1756421e-01,	
5.8522982e-02,	9.9751116e-01,	-3.9326228e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	-3.8701242e-01,	9.1851009e-01,	8.0997515e-02,	-4.6014429e-01,	
8.8407435e-01,	8.1729888e-02,	3.8701242e-01,	-9.1851009e-01,	-8.0997515e-02,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	-4.3986250e-01,	8.9477242e-01,	
7.6832920e-02,	6.0008674e-02,	9.9795540e-01,	-2.1999333e-02,	-6.0008674e-02,	
-9.9795540e-01,	2.1999333e-02,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	
4.3708884e-01,	-8.9611102e-01,	-7.7060958e-02,	-1.1914103e-01,	-9.3524161e-01,	
3.3335949e-01,	-7.0216447e-02,	-9.9213941e-01,	-1.0358106e-01,	6.7333052e-02,	
9.9548838e-01,	6.6851608e-02,	-6.7333052e-02,	-9.9548838e-01,	-6.6851608e-02,	
-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	-5.8522982e-02,	-9.9751116e-01,	
3.9326228e-02,	9.8718135e-02,	9.7104096e-01,	-2.1756421e-01,	4.9766819e-01,	
-8.6648396e-01,	-3.9139642e-02,	6.7926039e-01,	7.3049132e-01,	7.0624043e-02,	
-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	-6.7926039e-01,	-7.3049132e-01,	
-7.0624043e-02,	5.1641740e-01,	-8.5616784e-01,	-1.7020409e-02,	-4.4710140e-01,	
8.9420280e-01,	2.2398347e-02,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	
2.0932110e-01,	9.7604219e-01,	5.9382902e-02,	-4.6014429e-01,	8.8407435e-01,	
8.1729888e-02,	-2.0932110e-01,	-9.7604219e-01,	-5.9382902e-02,	-5.1641740e-01,	
8.5616784e-01,	1.7020409e-02,	4.1569772e-01,	-9.0783688e-01,	-5.5023742e-02,	
9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	-9.9687183e-02,	-9.6869111e-01,	
2.2737632e-01,	-4.1713719e-01,	9.0717882e-01,	5.4983259e-02,	7.0710678e-01,	
-7.0710678e-01,	0.0000000e+00,	4.8157716e-01,	-8.7586630e-01,	-3.0686374e-02,	
1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	1.1776592e-02,	9.9587831e-01,	
8.9931606e-02,	-5.6290723e-03,	9.9950562e-01,	3.0932761e-02,	-5.4304906e-02,	
-9.9850318e-01,	-6.5102908e-03,	-6.8097344e-02,	-9.9584033e-01,	-6.0537499e-02,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	-4.9766819e-01,	8.6648396e-01,	
3.9139642e-02,	4.4710140e-01,	-8.9420280e-01,	-2.2398347e-02,	3.1589840e-01,	
-9.4826400e-01,	-3.1679490e-02,	-5.1641740e-01,	8.5616784e-01,	1.7020409e-02,	
6.7926039e-01,	7.3049132e-01,	7.0624043e-02,	6.7333052e-02,	9.9548838e-01,	
6.6851608e-02,	-6.7333052e-02,	-9.9548838e-01,	-6.6851608e-02,	-4.6014429e-01,	
8.8407435e-01,	8.1729888e-02,	0.0000000e+00,	-9.9397557e-01,	1.0960186e-01,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	0.0000000e+00,	0.0000000e+00,	1.0000000e+00,	9.8718135e-02,	
9.7104096e-01,	-2.1756421e-01,	5.8522982e-02,	9.9751116e-01,	-3.9326228e-02,	
4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	-3.8701242e-01,	9.1851009e-01,	
8.0997515e-02,	-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	3.8701242e-01,	
-9.1851009e-01,	-8.0997515e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	
4.3986250e-01,	-8.9477242e-01,	-7.6832920e-02,	6.0008674e-02,	9.9795540e-01,	
-2.1999333e-02,	-6.0008674e-02,	-9.9795540e-01,	2.1999333e-02,	-4.3708884e-01,	
8.9611102e-01,	7.7060958e-02,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	0.0000000e+00,	0.0000000e+00,	
1.0000000e+00,	-7.0216447e-02,	-9.9213941e-01,	-1.0358106e-01,	1.1914103e-01,	
9.3524161e-01,	-3.3335949e-01,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	
2.0932110e-01,	9.7604219e-01,	5.9382902e-02,	-4.6014429e-01,	8.8407435e-01,	
8.1729888e-02,	-2.0932110e-01,	-9.7604219e-01,	-5.9382902e-02,	5.1641740e-01,	
-8.5616784e-01,	-1.7020409e-02,	-4.1569772e-01,	9.0783688e-01,	5.5023742e-02,	
9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	-9.9687183e-02,	-9.6869111e-01,	
2.2737632e-01,	4.8157716e-01,	-8.7586630e-01,	-3.0686374e-02,	4.1713719e-01,	
-9.0717882e-01,	-5.4983259e-02,	-6.8097344e-02,	-9.9584033e-01,	-6.0537499e-02,	
5.4304906e-02,	9.9850318e-01,	6.5102908e-03,	4.9766819e-01,	-8.6648396e-01,	
-3.9139642e-02,	-3.8828427e-01,	9.2115292e-01,	2.6695012e-02,	-4.9766819e-01,	
8.6648396e-01,	3.9139642e-02,	3.8828427e-01,	-9.2115292e-01,	-2.6695012e-02,	
7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	-4.8654566e-01,	8.7310868e-01,	
3.0895800e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	-4.6014429e-01,	
8.8407435e-01,	8.1729888e-02,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	0.0000000e+00,	0.0000000e+00,	-1.0000000e+00,	1.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	9.9133553e-01,	1.3135399e-01,	
0.0000000e+00,	9.9985275e-01,	1.7160566e-02,	0.0000000e+00,	9.9397557e-01,	
-1.0960186e-01,	-9.9687183e-02,	-9.6869111e-01,	2.2737632e-01,	-6.0008674e-02,	
-9.9795540e-01,	2.1999333e-02,	-6.7333052e-02,	-9.9548838e-01,	-6.6851608e-02,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	-9.9687183e-02,	-9.6869111e-01,	2.2737632e-01,	-6.7333052e-02,	
-9.9548838e-01,	-6.6851608e-02,	6.0008674e-02,	9.9795540e-01,	-2.1999333e-02,	
4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	-4.6014429e-01,	8.8407435e-01,	
8.1729888e-02,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	3.8701242e-01,	
-9.1851009e-01,	-8.0997515e-02,	-4.3708884e-01,	8.9611102e-01,	7.7060958e-02,	
-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	7.0710678e-01,	-7.0710678e-01,	
0.0000000e+00,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	1.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	1.0000000e+00,	
0.0000000e+00,	9.9133553e-01,	1.3135399e-01,	0.0000000e+00,	9.9985275e-01,	
1.7160566e-02,	-6.0008674e-02,	-9.9795540e-01,	2.1999333e-02,	-6.7333052e-02,	
-9.9548838e-01,	-6.6851608e-02,	9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	-4.9766819e-01,	8.6648396e-01,	
3.9139642e-02,	5.1641740e-01,	-8.5616784e-01,	-1.7020409e-02,	-3.1589840e-01,	
9.4826400e-01,	3.1679490e-02,	-4.4710140e-01,	8.9420280e-01,	2.2398347e-02,	
-6.7926039e-01,	-7.3049132e-01,	-7.0624043e-02,	4.6014429e-01,	-8.8407435e-01,	
-8.1729888e-02,	-4.6014429e-01,	8.8407435e-01,	8.1729888e-02,	-5.1641740e-01,	
8.5616784e-01,	1.7020409e-02,	-1.6660884e-01,	9.8516998e-01,	4.1007290e-02,	
-2.0932110e-01,	-9.7604219e-01,	-5.9382902e-02,	4.1713719e-01,	-9.0717882e-01,	
-5.4983259e-02,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	-4.9766819e-01,	
8.6648396e-01,	3.9139642e-02,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
3.8828427e-01,	-9.2115292e-01,	-2.6695012e-02,	-4.8157716e-01,	8.7586630e-01,	
3.0686374e-02,	-1.1776592e-02,	-9.9587831e-01,	-8.9931606e-02,	0.0000000e+00,	
-9.9133553e-01,	-1.3135399e-01,	0.0000000e+00,	9.9133553e-01,	1.3135399e-01,	
1.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	9.9874751e-01,	
5.0034050e-02,	5.6290723e-03,	-9.9950562e-01,	-3.0932761e-02,	0.0000000e+00,	
9.9985275e-01,	1.7160566e-02,	0.0000000e+00,	-9.9985275e-01,	-1.7160566e-02,	
-1.6660884e-01,	9.8516998e-01,	4.1007290e-02,	0.0000000e+00,	9.9874751e-01,	
5.0034050e-02,	4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	6.7926039e-01,	
7.3049132e-01,	7.0624043e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	
-6.7926039e-01,	-7.3049132e-01,	-7.0624043e-02,	4.4710140e-01,	-8.9420280e-01,	
-2.2398347e-02,	-5.1641740e-01,	8.5616784e-01,	1.7020409e-02,	-4.6014429e-01,	
8.8407435e-01,	8.1729888e-02,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	
0.0000000e+00,	-9.9397557e-01,	1.0960186e-01,	4.9766819e-01,	-8.6648396e-01,	
-3.9139642e-02,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	0.0000000e+00,	
0.0000000e+00,	1.0000000e+00,	-6.7333052e-02,	-9.9548838e-01,	-6.6851608e-02,	
9.9687183e-02,	9.6869111e-01,	-2.2737632e-01,	6.0008674e-02,	9.9795540e-01,	
-2.1999333e-02,	4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	-4.6014429e-01,	
8.8407435e-01,	8.1729888e-02,	-4.9766819e-01,	8.6648396e-01,	3.9139642e-02,	
0.0000000e+00,	0.0000000e+00,	1.0000000e+00,	-3.3965995e-01,	9.3544880e-01,	
9.7809340e-02,	-4.0544890e-01,	9.1109457e-01,	7.4282359e-02,	3.8701242e-01,	
-9.1851009e-01,	-8.0997515e-02,	4.3708884e-01,	-8.9611102e-01,	-7.7060958e-02,	
4.6014429e-01,	-8.8407435e-01,	-8.1729888e-02,	-4.6014429e-01,	8.8407435e-01,	
8.1729888e-02,	5.1641740e-01,	-8.5616784e-01,	-1.7020409e-02,	-2.0932110e-01,	
-9.7604219e-01,	-5.9382902e-02,	-4.1713719e-01,	9.0717882e-01,	5.4983259e-02,	
4.9766819e-01,	-8.6648396e-01,	-3.9139642e-02,	-4.9766819e-01,	8.6648396e-01,	
3.9139642e-02,	7.0710678e-01,	-7.0710678e-01,	0.0000000e+00,	4.4710140e-01,	
-8.9420280e-01,	-2.2398347e-02,	-5.1641740e-01,	8.5616784e-01,	1.7020409e-02,	
-6.7926039e-01,	-7.3049132e-01,	-7.0624043e-02,	-3.8828427e-01,	9.2115292e-01,	
2.6695012e-02,	-4.8157716e-01,	8.7586630e-01,	3.0686374e-02 };

const float MPT_B[] PROGMEM = {
1.2466610e+01,	5.4744201e+00,	3.0644098e-12,	0.0000000e+00,	4.8925551e+03,	
-3.8654415e-13,	6.0877146e+00,	-1.0468882e+00,	7.0710678e+03,	4.9763935e+03,	
4.4020088e+03,	1.0000000e+04,	-1.2466610e+01,	-7.0295716e-01,	1.0363372e+01,	
-1.7809969e+00,	4.8484682e+03,	4.2621348e+03,	-5.4744201e+00,	1.4890868e+00,	
5.0408264e+00,	0.0000000e+00,	4.9766819e+03,	4.4012542e+03,	-3.0213112e-12,	
-2.3594431e-13,	8.5823754e+00,	2.1096284e-12,	7.0710678e+03,	4.8484099e+03,	
1.7854189e+03,	4.2622195e+03,	2.2599369e+03,	1.0000000e+04,	2.7853059e-13,	
-2.0586336e-13,	-4.9763935e+03,	4.0173870e+03,	4.9766819e+03,	-4.0162662e+03,	
7.0710678e+03,	-4.8925551e+03,	1.1306715e+01,	6.4346938e+00,	8.3868598e+00,	
2.7792967e-12,	0.0000000e+00,	0.0000000e+00,	7.0710678e+03,	4.9763935e+03,	
4.6003319e+03,	4.6025540e+03,	1.0000000e+04,	2.1207112e+03,	-1.0363372e+01,	
-6.0877146e+00,	1.2912588e+00,	7.0710678e+03,	4.9763935e+03,	1.6929073e+03,	
4.6022873e+03,	2.5428355e+03,	1.0000000e+04,	1.0000000e+04,	-8.5823754e+00,	
1.0468882e+00,	-1.8994280e-01,	4.6025540e+03,	-4.6022873e+03,	4.9769704e+03,	
4.2789098e+03,	4.1227778e+03,	1.0000000e+04,	-3.9756034e+03,	-4.4020088e+03,	
7.0185291e+00,	8.1654384e+00,	-1.2873965e+00,	7.0295716e-01,	4.4255658e+03,	
-1.1306715e+01,	4.9766819e+03,	4.6011763e+03,	-5.0408264e+00,	1.7809969e+00,	
1.4812016e+00,	-4.9763935e+03,	4.9766819e+03,	7.0710678e+03,	-5.1643121e+03,	
4.7177365e+03,	-4.0173870e+03,	-5.6323748e+03,	-4.8484682e+03,	-4.6003319e+03,	
4.6011763e+03,	5.1643121e+03,	-1.2663355e+03,	-4.2621348e+03,	1.0065764e+01,	
7.9411592e+00,	-1.7732794e+00,	-1.4890868e+00,	4.2497361e+03,	-6.4346938e+00,	
7.0710678e+03,	4.9766819e+03,	4.6014429e+03,	2.1184863e+03,	4.9766819e+03,	
1.6933669e+03,	4.6014429e+03,	2.5416953e+03,	1.0000000e+04,	1.0000000e+04,	
-8.5890936e-13,	8.8708649e-13,	0.0000000e+00,	-4.6011763e+03,	4.6014429e+03,	
4.9766819e+03,	3.9748444e+03,	-4.4012542e+03,	5.7311326e+00,	8.8683955e+00,	
0.0000000e+00,	-5.4498370e-13,	4.4248115e+03,	-2.7792967e-12,	-4.9763935e+03,	
4.9766819e+03,	7.0710678e+03,	4.0162662e+03,	-4.8484099e+03,	-1.6929073e+03,	
1.6933669e+03,	2.7924529e+03,	1.9396862e+03,	-1.7854189e+03,	4.6022873e+03,	
-4.6014429e+03,	5.1641740e+03,	2.7924529e+03,	1.2682239e+03,	-4.2622195e+03,	
2.5428355e+03,	-2.5416953e+03,	1.9396862e+03,	1.0000000e+04,	-2.2599369e+03,	
8.2924843e+00,	6.4520725e+00,	2.0383704e-12,	-7.9298996e-13,	4.2498157e+03,	
3.9542702e-13,	1.1531500e+01,	6.7303605e+00,	9.3511060e-01,	-1.2559404e+00,	
4.8925763e+03,	-8.3868598e+00,	7.8851580e+00,	-1.2912588e+00,	4.9763935e+03,	
4.6003319e+03,	4.6025540e+03,	1.0000000e+04,	2.1207112e+03,	-1.0065764e+01,	
-7.0185291e+00,	6.4039565e+00,	1.8994280e-01,	4.6022873e+03,	-8.2924843e+00,	
1.2873965e+00,	4.6025540e+03,	-3.9745778e+03,	-4.6022873e+03,	3.9756034e+03,	
4.9769704e+03,	-4.4255658e+03,	7.1656538e+00,	-2.1248274e+00,	4.9769704e+03,	
4.4019543e+03,	-1.1531500e+01,	-8.1654384e+00,	8.0751008e+00,	-1.4812016e+00,	
4.6011763e+03,	-5.7311326e+00,	1.7732794e+00,	-4.9763935e+03,	-5.6210377e+03,	
4.9766819e+03,	5.6323748e+03,	-5.1643121e+03,	4.7177365e+03,	-4.6003319e+03,	
-1.2582565e+03,	4.6011763e+03,	1.2663355e+03,	5.1643121e+03,	-4.2497361e+03,	
1.0128211e+01,	-1.5458356e+00,	4.2621133e+03,	7.0710678e+03,	4.8483593e+03,	
1.0000000e+04,	2.2631065e+03,	1.7872067e+03,	-6.7303605e+00,	-7.9411592e+00,	
4.9769704e+03,	-4.9766819e+03,	4.7177365e+03,	3.8334789e+03,	-5.1641740e+03,	
5.6349215e+03,	6.5938992e+00,	0.0000000e+00,	4.6014429e+03,	2.1184863e+03,	
4.9766819e+03,	4.6014429e+03,	1.0000000e+04,	-8.3055310e-13,	1.0594587e-12,	
-4.6011763e+03,	3.9758699e+03,	4.6014429e+03,	-3.9748444e+03,	4.9766819e+03,	
-4.4248115e+03,	6.1187656e+00,	-1.0779392e+00,	4.4013087e+03,	7.0710678e+03,	
4.9766819e+03,	1.0000000e+04,	-8.8683955e+00,	-9.3511060e-01,	4.6022873e+03,	
1.2763029e+03,	-4.6014429e+03,	-1.2682239e+03,	5.1641740e+03,	-4.2498157e+03,	
8.3472142e+00,	2.3516125e-01,	4.8484176e+03,	4.2622409e+03,	-6.4520725e+00,	
1.2559404e+00,	4.9769704e+03,	-4.0149058e+03,	-4.9766819e+03,	4.0160265e+03,	
7.0710678e+03,	-4.8925763e+03,	4.9763935e+03,	4.6003319e+03,	7.0710678e+03,	
4.9769704e+03,	4.6025540e+03,	1.0000000e+04,	1.0000000e+04,	2.5439143e+03,	
1.6956050e+03,	2.1207112e+03,	-1.0128211e+01,	-7.1656538e+00,	-7.8851580e+00,	
4.9769704e+03,	4.6022873e+03,	-8.3472142e+00,	-6.4039565e+00,	2.1248274e+00,	
4.6025540e+03,	-4.6022873e+03,	4.9769704e+03,	3.9745778e+03,	-4.4019543e+03,	
4.6011763e+03,	7.0710678e+03,	4.9766819e+03,	1.0000000e+04,	1.0000000e+04,	
2.5450546e+03,	1.6951454e+03,	-6.1187656e+00,	-8.0751008e+00,	1.5458356e+00,	
-4.9763935e+03,	4.9766819e+03,	-5.1643121e+03,	3.8341873e+03,	4.7177365e+03,	
5.6210377e+03,	-4.6003319e+03,	4.6011763e+03,	5.1643121e+03,	2.7935393e+03,	
1.2582565e+03,	-4.2621133e+03,	4.9769704e+03,	-4.9766819e+03,	7.0710678e+03,	
4.0149058e+03,	-4.8483593e+03,	-2.2631065e+03,	-2.5439143e+03,	2.5450546e+03,	
1.0000000e+04,	1.9419218e+03,	-1.7872067e+03,	1.6956050e+03,	-1.6951454e+03,	
2.7935393e+03,	1.9419218e+03,	4.9769704e+03,	5.6462586e+03,	-4.9766819e+03,	
-5.6349215e+03,	4.7177365e+03,	-5.1641740e+03,	4.6014429e+03,	7.0710678e+03,	
2.1184863e+03,	4.9766819e+03,	4.6014429e+03,	1.0000000e+04,	-6.5938992e+00,	
-2.3516125e-01,	1.0779392e+00,	-4.6011763e+03,	4.6014429e+03,	4.9766819e+03,	
1.0000000e+04,	4.1225908e+03,	4.2782265e+03,	-3.9758699e+03,	-4.4013087e+03,	
4.6022873e+03,	-4.6014429e+03,	5.1641740e+03,	-1.2763029e+03,	-4.2622409e+03,	
4.9769704e+03,	-4.9766819e+03,	7.0710678e+03,	4.7177365e+03,	-5.1641740e+03,	
-5.6462586e+03,	-4.0160265e+03,	-4.8484176e+03 };

const int MPT_NC[] PROGMEM = {
6,	8,	6,	6,	10,	
6,	6,	9,	10,	8,	
6,	5,	8,	5,	6,	
13,	5,	6,	5,	5,	
6,	5,	6,	6,	9,	
5,	6,	6,	5,	6,	
6,	10,	6,	9,	6,	
8,	6,	6,	6,	13,	
5,	5,	10,	6,	6,	
5,	5,	5,	6,	9,	
8,	5,	8 };


const float MPT_F[] PROGMEM = {
3.5360197e-02,	2.7757296e-01,	-9.8938689e-02,	3.6703093e-02,	6.7485901e-01,	
4.4001146e-03,	3.4694470e-18,	0.0000000e+00,	-1.3877788e-17,	5.3983200e-18,	
0.0000000e+00,	0.0000000e+00,	4.4046764e-02,	7.3250588e-01,	-1.6147656e-02,	
4.4012894e-18,	0.0000000e+00,	0.0000000e+00,	4.2976747e-02,	4.1761831e-01,	
-9.8025586e-02,	0.0000000e+00,	0.0000000e+00,	1.3877788e-17,	0.0000000e+00,	
0.0000000e+00,	1.3877788e-17,	5.3983200e-18,	0.0000000e+00,	0.0000000e+00,	
4.4046764e-02,	7.3250588e-01,	-1.6147656e-02,	4.4012894e-18,	0.0000000e+00,	
0.0000000e+00,	4.2976747e-02,	4.1761831e-01,	-9.8025586e-02,	0.0000000e+00,	
0.0000000e+00,	1.3877788e-17,	0.0000000e+00,	0.0000000e+00,	1.3877788e-17,	
6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	-1.2818331e+00,	3.0409790e+00,	
8.8127573e-02,	4.4408921e-16,	-8.8817842e-16,	0.0000000e+00,	3.7782241e-02,	
3.1046762e-01,	-9.6269531e-02,	2.9295136e-02,	5.7424886e-01,	-3.7636523e-03,	
3.2289962e-02,	4.3854170e-01,	3.5584404e-02,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	
-1.2265927e+01,	-1.1339463e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
-5.7582080e-18,	0.0000000e+00,	0.0000000e+00,	3.7782241e-02,	6.4398986e-01,	
-2.5388881e-02,	2.9295136e-02,	4.1393235e-01,	4.3215249e-02,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.1115430e+01,	
-5.0209117e-01,	0.0000000e+00,	0.0000000e+00,	-1.3877788e-17,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	2.0163036e+00,	-3.8739202e+00,	-3.5813172e-01,	
1.1102230e-16,	2.2204460e-16,	2.7755576e-17,	1.1102230e-16,	-2.2204460e-16,	
-1.3877788e-17,	4.4046764e-02,	4.3326601e-01,	-9.7074357e-02,	-8.6736174e-18,	
2.2204460e-16,	1.3877788e-17,	3.9051045e-02,	5.7107375e-01,	3.4715783e-02,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.2265927e+01,	-1.1339463e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	-5.7582080e-18,	0.0000000e+00,	
0.0000000e+00,	3.7782241e-02,	6.4398986e-01,	-2.5388881e-02,	2.9295136e-02,	
4.1393235e-01,	4.3215249e-02,	6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	
0.0000000e+00,	0.0000000e+00,	-1.3877788e-17,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	8.0486270e+00,	1.3813934e-01,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	4.4408921e-16,	1.3877788e-17,	
2.0163036e+00,	-3.8739202e+00,	-3.5813172e-01,	1.1102230e-16,	2.2204460e-16,	
2.7755576e-17,	1.1102230e-16,	-2.2204460e-16,	-1.3877788e-17,	-5.7582080e-17,	
-3.2166768e+00,	-4.2621628e-01,	1.4881914e-17,	2.2204460e-16,	5.5511151e-17,	
-5.4105377e-18,	-1.3322676e-15,	-1.5265567e-16,	4.4046764e-02,	4.3326601e-01,	
-9.7074357e-02,	-8.6736174e-18,	2.2204460e-16,	1.3877788e-17,	3.9051045e-02,	
5.7107375e-01,	3.4715783e-02,	3.5360197e-02,	2.7757296e-01,	-9.8938689e-02,	
3.6703093e-02,	6.7485901e-01,	4.4001146e-03,	3.4694470e-18,	0.0000000e+00,	
-1.3877788e-17,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	3.7782241e-02,	5.5859316e-01,	3.7512091e-02,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	3.7782241e-02,	5.5859316e-01,	3.7512091e-02,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.2265927e+01,	-1.1339463e+00,	
-1.3963113e+00,	3.3139143e+00,	2.9223285e-01,	5.3983200e-18,	0.0000000e+00,	
0.0000000e+00,	4.4046764e-02,	7.3250588e-01,	-1.6147656e-02,	4.4012894e-18,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	3.7782241e-02,	5.5859316e-01,	
3.7512091e-02,	6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	0.0000000e+00,	
0.0000000e+00,	-1.3877788e-17,	2.2168411e-01,	2.3840388e-01,	2.3048934e-02,	
2.0163036e+00,	-3.8739202e+00,	-3.5813172e-01,	1.1102230e-16,	2.2204460e-16,	
2.7755576e-17,	9.5863557e-02,	4.4700164e-01,	2.7195807e-02,	4.2976747e-02,	
4.1761831e-01,	-9.8025586e-02,	0.0000000e+00,	0.0000000e+00,	1.3877788e-17,	
0.0000000e+00,	0.0000000e+00,	1.3877788e-17,	6.3841873e+00,	-1.1115430e+01,	
-5.0209117e-01,	0.0000000e+00,	0.0000000e+00,	-1.3877788e-17,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	3.7782241e-02,	5.5859316e-01,	
3.7512091e-02,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	
-1.2265927e+01,	-1.1339463e+00,	-1.3963113e+00,	3.3139143e+00,	2.9223285e-01,	
5.7582080e-18,	0.0000000e+00,	0.0000000e+00,	4.4046764e-02,	7.3250588e-01,	
-1.6147656e-02,	2.8443370e-18,	0.0000000e+00,	0.0000000e+00,	2.0163036e+00,	
-3.8739202e+00,	-3.5813172e-01,	1.1102230e-16,	2.2204460e-16,	2.7755576e-17,	
9.5863557e-02,	4.4700164e-01,	2.7195807e-02,	4.2976747e-02,	4.1761831e-01,	
-9.8025586e-02,	-1.0408341e-17,	0.0000000e+00,	1.3877788e-17,	6.9388939e-18,	
-2.2204460e-16,	1.3877788e-17,	6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	
-1.2818331e+00,	3.0409790e+00,	8.8127573e-02,	4.4408921e-16,	-8.8817842e-16,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.2265927e+01,	-1.1339463e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	
0.0000000e+00,	0.0000000e+00,	-1.3877788e-17,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	2.0163036e+00,	-3.8739202e+00,	-3.5813172e-01,	1.1102230e-16,	
2.2204460e-16,	2.7755576e-17,	1.1102230e-16,	-2.2204460e-16,	-1.3877788e-17,	
6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	0.0000000e+00,	0.0000000e+00,	
-1.3877788e-17,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	-9.8886189e-18,	
-3.2166768e+00,	-4.2621628e-01,	-9.4175512e-18,	-8.8817842e-16,	-6.9388939e-17,	
-1.7901575e-18,	6.6613381e-16,	1.3877788e-17,	0.0000000e+00,	8.0486270e+00,	
1.3813934e-01,	0.0000000e+00,	2.2204460e-16,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	1.3877788e-17,	6.3841873e+00,	-1.1115430e+01,	-5.0209117e-01,	
0.0000000e+00,	0.0000000e+00,	-1.3877788e-17,	2.2168411e-01,	2.3840388e-01,	
2.3048934e-02,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.2265927e+01,	
-1.1339463e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	2.0163036e+00,	
-3.8739202e+00,	-3.5813172e-01,	1.1102230e-16,	2.2204460e-16,	0.0000000e+00,	
1.1102230e-16,	0.0000000e+00,	0.0000000e+00,	6.3841873e+00,	-1.1115430e+01,	
-5.0209117e-01,	0.0000000e+00,	0.0000000e+00,	-1.3877788e-17,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00 };


const float MPT_G[] PROGMEM = {
9.0949470e-13,	0.0000000e+00,	0.0000000e+00,	3.7000000e+00,	-7.6842288e-01,	
0.0000000e+00,	-7.6781637e-01,	3.7000000e+00,	9.0949470e-13,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	9.0949470e-13,	0.0000000e+00,	0.0000000e+00,	
6.3841873e+04,	-1.3258799e+04,	1.8189894e-12,	9.0949470e-13,	0.0000000e+00,	
0.0000000e+00,	3.7000000e+00,	3.7000000e+00,	0.0000000e+00,	3.7000000e+00,	
-4.5474735e-13,	0.0000000e+00,	3.7000000e+00,	-6.3853588e+04,	9.0949470e-13,	
3.7000000e+00,	-8.3113884e-01,	2.9328208e-01,	0.0000000e+00,	3.7000000e+00,	
-9.0949470e-13,	6.3841873e+04,	3.7000000e+00,	1.8189894e-12,	2.0161867e+04,	
3.7000000e+00,	9.0949470e-13,	-7.9121448e-01,	3.7000000e+00,	-8.5393044e-01,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+04,	
1.8189894e-12,	0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+04,	
1.8189894e-12,	1.8189894e-12,	1.3631286e+04,	0.0000000e+00,	0.0000000e+00,	
-2.0163036e+04,	-4.5474735e-13,	-3.6379788e-12,	-8.2472705e+03,	-9.0949470e-13,	
-5.4569682e-12,	9.0949470e-13,	0.0000000e+00,	0.0000000e+00,	2.7753408e-01,	
-8.4885327e-01,	3.7000000e+00,	3.7000000e+00,	3.7000000e+00,	-7.2455724e-01,	
3.7000000e+00,	-4.5474735e-13,	1.0658160e-01,	3.7000000e+00,	-6.3853588e+04,	
1.4343673e+04,	3.7000000e+00,	-1.5596374e+00,	3.7000000e+00,	0.0000000e+00,	
3.7000000e+00,	-8.3113884e-01,	6.3841873e+04,	3.7000000e+00,	1.8381875e+03,	
2.0161867e+04,	3.7000000e+00,	5.7994836e+02,	-6.6643458e-01,	3.7000000e+00,	
3.7000000e+00,	-6.3841873e+04,	0.0000000e+00,	1.8189894e-12,	0.0000000e+00,	
0.0000000e+00,	0.0000000e+00,	0.0000000e+00,	6.3841873e+04,	-1.4340935e+04,	
0.0000000e+00,	-7.9121448e-01,	3.7000000e+00,	-2.0163036e+04,	-9.0949470e-13,	
-5.8081316e+02,	1.0138179e-01,	-4.5474735e-13,	3.7000000e+00,	-6.3841873e+04,	
1.3258008e+04,	3.7000000e+00,	3.7000000e+00,	3.7000000e+00,	3.7000000e+00,	
3.7000000e+00,	-4.5474735e-13,	3.7000000e+00,	3.7000000e+00,	-6.3853588e+04,	
3.7000000e+00,	0.0000000e+00,	3.7000000e+00,	3.7000000e+00,	6.3841873e+04,	
3.7000000e+00,	3.7000000e+00,	2.0161867e+04,	3.7000000e+00,	3.7000000e+00,	
-6.3841873e+04,	3.7000000e+00,	3.7000000e+00,	8.2581708e+03,	3.7000000e+00,	
3.7000000e+00,	-1.3645602e+04,	3.7000000e+00,	3.7000000e+00,	-6.3841873e+04,	
0.0000000e+00,	-1.8390187e+03,	0.0000000e+00,	0.0000000e+00,	3.7000000e+00,	
0.0000000e+00,	6.3841873e+04,	3.7000000e+00,	-2.0163036e+04,	0.0000000e+00,	
3.7000000e+00,	-6.3841873e+04,	0.0000000e+00,	3.7000000e+00 };

