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

#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif

#ifndef MATLAB_MEX_FILE
#include "utility\twi.c"
#include "Wire.cpp"
#include "VL53L0X.cpp"
VL53L0X *sensor;
#endif

#define y_width 1

extern "C" void PololuVL53L0X_Start_wrapper(void) {
#ifndef MATLAB_MEX_FILE
    Wire.begin();
    sensor = new VL53L0X();
    sensor->init();
    sensor->setMeasurementTimingBudget(20000);
    sensor->startContinuous();
#endif
}

extern "C" void PololuVL53L0X_Outputs_wrapper(real_T *distance) {
#ifndef MATLAB_MEX_FILE
    distance[0] = (double)sensor->readRangeContinuousMillimeters();
#endif
}

extern "C" void PololuVL53L0X_Terminate_wrapper(void) {
#ifndef MATLAB_MEX_FILE
    delete sensor;
#endif
}