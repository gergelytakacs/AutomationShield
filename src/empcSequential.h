//	EXPLICIT MPC SEQUENTIAL SEARCH ALGORITHM
//
//	 The function below has been adapted from the auto-generated
//   C code of the Multi-Parametric Toolbox 3 (MPT3) of Kvasnica et al. 
//   for  MATLAB. Please see http://www.mpt3.org for more information. 
//
//   The following code has been removed from the auto-generated
//   portion to perform microcontroller architecutre-specific changes 
//   for AVR and ARM Cortex-A devices.
//
//	 The list of changes from the original are as follows:
// 	 - Region number is not needed in real-time implementation, the 
//	   return value of the function is removed.
// 	 - PROGMEM compatible array reading implemented for AVR architecture.
//   - Tie-breaking function removed to save memory, much needed for MCU
// 	   implementation.
//   - PWQ functionality is removed, can handle PWA only.
//
//   This code snippet has been altered for the needs of the 
//   the AutomationShield hardware and software ecosystem. 
//   Visit http://www.automationshield.com for more details.
//
//	 Last updated by: Gergely Takacs
//   Last updated on: 11.10.2020


/* Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) 
   Revised in 2012-2013 by Martin Herceg (herceg@control.ee.ethz.ch)    
   Adapted for MCU use by Gergely Takács in 2020 (gergely.takacs@stuba.sk)
*/ 

#ifndef EMPCSEQUENTIAL_H                     //Include guard
#define EMPCSEQUENTIAL_H

/* main evaluation function using sequential search */
void empcSequential( float *X, float *U){
    int ix, jx, ic, nc, isinside;
    unsigned long ireg, abspos, iregmin, region;
    float hx, sx;

    abspos = 0;
    region = 0;
    iregmin = 0;
	
	#ifdef ARDUINO_ARCH_AVR
    for (ireg=0; ireg<MPT_NR; ireg++) {

        isinside = 1;
        nc = pgm_read_word(&(MPT_NC[ireg]));
        for (ic=0; ic<nc; ic++) {
            hx = 0;
            for (ix=0; ix<MPT_DOMAIN; ix++) {
                hx += pgm_read_float(&(MPT_A[abspos*MPT_DOMAIN+ic*MPT_DOMAIN+ix]))*X[ix];
            }
            if ((hx - pgm_read_float(&(MPT_B[abspos+ic]))) > MPT_ABSTOL) {
                /* constraint is violated, continue with next region */
                isinside = 0;
                break;
            } 
        }
        abspos = abspos + pgm_read_word(&(MPT_NC[ireg]));
        if (isinside==1) {
                iregmin = ireg;
		break;
        } 
    }
    for (ix=0; ix<MPT_RANGE; ix++) {
        sx = 0;
        for (jx=0; jx<MPT_DOMAIN; jx++) {
            sx += pgm_read_float(&(MPT_F[iregmin*MPT_DOMAIN*MPT_RANGE + ix*MPT_DOMAIN + jx]))*X[jx];
        }
        U[ix] = sx + pgm_read_float(&(MPT_G[iregmin*MPT_RANGE + ix]));
    }
	#else
	for (ireg=0; ireg<MPT_NR; ireg++) {
        isinside = 1;
        nc = MPT_NC[ireg];
        for (ic=0; ic<nc; ic++) {
            hx = 0;
            for (ix=0; ix<MPT_DOMAIN; ix++) {
                hx += MPT_A[abspos*MPT_DOMAIN+ic*MPT_DOMAIN+ix]*X[ix];
            }
            if ((hx - MPT_B[abspos+ic]) > MPT_ABSTOL) {
                /* constraint is violated, continue with next region */
                isinside = 0;
                break;
            } 
        }
        abspos = abspos + MPT_NC[ireg];
        if (isinside==1) {
                iregmin = ireg;
		break;
        }   
    }
    for (ix=0; ix<MPT_RANGE; ix++) {
        sx = 0;
        for (jx=0; jx<MPT_DOMAIN; jx++) {
            sx += MPT_F[iregmin*MPT_DOMAIN*MPT_RANGE + ix*MPT_DOMAIN + jx]*X[jx];
        }
        U[ix] = sx + MPT_G[iregmin*MPT_RANGE + ix];
    }	
	#endif	
}

#endif