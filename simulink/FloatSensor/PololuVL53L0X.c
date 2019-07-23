/*
    Simulink implementation of Pololu arduino library for
    VL53L0X Time-of-Flight distance sensor.

    This code is part of the AutomationShield hardware and software
    ecosystem. Visit http://www.automationshield.com for more
    details. This code is licensed under a Creative Commons
    Attribution-NonCommercial 4.0 International License.

    Created by Peter Chmurciak using S-Function Builder.
    Last update: 23.7.2019.
*/

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME PololuVL53L0X

#define NUM_INPUTS            0
#define NUM_OUTPUTS           1

#define OUT_PORT_0_NAME       distance
#define OUTPUT_0_WIDTH        1
#define OUTPUT_DIMS_0_COL     1
#define OUTPUT_0_DTYPE        real_T
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUT_0_FRAME_BASED     FRAME_NO
#define OUT_0_BUS_BASED       0
#define OUT_0_BUS_NAME
#define OUT_0_DIMS            1-D
#define OUT_0_ISSIGNED        1
#define OUT_0_WORDLENGTH      8
#define OUT_0_FIXPOINTSCALING 1
#define OUT_0_FRACTIONLENGTH  3
#define OUT_0_BIAS            0
#define OUT_0_SLOPE           0.125

#define NPARAMS               0

#define SAMPLE_TIME_0         INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES       0
#define DISC_STATES_IC        [0]
#define NUM_CONT_STATES       0
#define CONT_STATES_IC        [0]

#define SFUNWIZ_GENERATE_TLC  1
#define SOURCEFILES           "__SFB__"
#define PANELINDEX            8
#define USE_SIMSTRUCT         0
#define SHOW_COMPILE_STEPS    0
#define CREATE_DEBUG_MEXFILE  0
#define SAVE_CODE_ONLY        0
#define SFUNWIZ_REVISION      3.0

#include "simstruc.h"

extern void PololuVL53L0X_Start_wrapper(void);
extern void PololuVL53L0X_Outputs_wrapper(real_T *distance);
extern void PololuVL53L0X_Terminate_wrapper(void);

static void mdlInitializeSizes(SimStruct *S) {
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    ssSetArrayLayoutForCodeGen(S, SS_COLUMN_MAJOR);
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);

    if (!ssSetNumInputPorts(S, NUM_INPUTS))
        return;

    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS))
        return;

    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    ssSetNumPWork(S, 0);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimulinkVersionGeneratedIn(S, "9.3");

    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                     SS_OPTION_WORKS_WITH_CODE_REUSE));
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType) {
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S) {
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S) {
    PololuVL53L0X_Start_wrapper();
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *distance = (real_T *) ssGetOutputPortRealSignal(S, 0);
    PololuVL53L0X_Outputs_wrapper(distance);
}

static void mdlTerminate(SimStruct *S) {
    PololuVL53L0X_Terminate_wrapper();
}

#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif