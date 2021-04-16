
/*
  Autogenerated C-code S-function for simulation of explicit controllers.
    
*/

/* Copyright (C) 2005 by Michal Kvasnica (michal.kvasnica@stuba.sk) 
   Revised in 2012-2013 by Martin Herceg, Automatic Control Laboratory,
   ETH Zurich, herceg@control.ee.ethz.ch
*/

/*  This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/ 

/* Generated on 16-Mar-2021 10:52:26 by MPT @version@ */ 

#define S_FUNCTION_NAME  ectrl_sfunc   /*Name of the S-function file*/
#define S_FUNCTION_LEVEL 2		/*Level 2 S-functions allow multi-port status*/
#define MPT_TS 0.003000		/* sample time */
#include "simstruc.h"			/*Header where different routines are defined */

#include "ectrl.c" /*inclusion of the evalution algorithm */



static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    /* no states, all computation will be done in output section */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /* one input port: state x(k) + vector of references */
    ssSetNumInputPorts(S, 1); 
    
    /* one output port: control action */
    ssSetNumOutputPorts(S, 1);

    /* width of input vector - number of states of the original problem. 
     * note that tracking includes additional states, here we do not consider them
     */
    ssSetInputPortWidth(S, 0, MPT_DOMAIN);
    
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    /* width of output - number of control actions */
    ssSetOutputPortWidth(S, 0, MPT_RANGE);
    
    ssSetNumSampleTimes(S, 1);

    /* Take care when specifying exception free code - see sfuntmpl.doc */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
    
}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    /* set sampling time */
    ssSetSampleTime(S, 0, MPT_TS);
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_INITIALIZE_CONDITIONS

static void mdlInitializeConditions(SimStruct *S)
{
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            	*u   = ssGetOutputPortRealSignal(S,0);
    InputRealPtrsType    Xin   = ssGetInputPortRealSignalPtrs(S,0);
    static double U[MPT_RANGE], X[MPT_DOMAIN];
    unsigned long int region;
    int i;

    /* prepare the input signal for passing to mpt_getInput function */
    for (i=0; i<MPT_DOMAIN; i++) {
         X[i] = *Xin[i];
    }


    /* get control law */
    region = ectrl(X, U);


    /* check if control law was found, if not, stop the simulation */
    if (region<1) {
        ssSetErrorStatus(S, "No feasible control law found!");
    }
    
    /* output control action */
    for (i=0; i<MPT_RANGE; i++) {
            u[i] = (real_T)U[i]; /* current output */
    }
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}

/*End of file necessary includes*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

