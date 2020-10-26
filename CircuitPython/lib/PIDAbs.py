"""
  PID control in an absolute form for CircuitPython

  This code can be used to create PID control in Python. It has
  been developed for the AutomationShield initiative and has
  been tested on hardware using CicruitPython.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  If you have found any use of this code, please cite our work in your
  academic publications, such as theses, conference articles or journal
  papers. A list of publications connected to the AutomationShield
  project is available at:
  https://github.com/gergelytakacs/AutomationShield/wiki/Publications

  Created by:       Gergely Takács
  Created on:       12.10.2020.
  Last updated by:  Gergely Takács
  Last update:      13.10.2020.
"""

import AutomationShield     # Import the AutomationShield module for constrain functions
eSum = 0.0                  # Sum of the error for integral
e = [0.0, 0.0]              # List to stores curent and previous error

Kp = 0.0
Ti = 0.0
Td = 0.0
Ts = 0.0

# These could be solved in a prettier way, but let's keep consistent with the Arduino API!
def setKp(Kpin):            # Takes the proportional gain
    global Kp               # Sets Kp as a global variable
    Kp = Kpin               # Stores the input argument in the global variable

def setTi(Ti_in):           # Takes the integral time constant
    global Ti               # Sets Ts as a global variable
    Ti = Ti_in              # Stores the input argument in the global variable

def setTd(Td_in):           # Takes the derivative time constant
    global Td               # Sets Ts as a global variable
    Td = Td_in              # Stores the input argument in the global variable

def setTs(Ts_in):           # Takes the sampling time in microseconds!
    global Ts               # Sets Ts as a global variable
    Ts = Ts_in / 1000000    # Stores the input argument in the global variable

# Computes an absolute PID input
def compute(err, saturationMin, saturationMax, antiWindupMin, antiWindupMax):           # Requres error, input saturation and intergral windup saturation
    global eSum, Kp, Ti, Td, Ts, e                                                      # Sets these as global variables
    eSum = eSum + err                                                                   # Sum of errors for the integral (summation) part
    e[1] = err                                                                          # New error in position [1] (right)
    eSum = AutomationShield.constrainFloat((Kp * Ts / Ti )* eSum, antiWindupMin, antiWindupMax) / (Kp * Ts / Ti)        # Anti-windup by clamping
    u = ( Kp * e[1]) + ((Kp * Ts / Ti) * eSum) + ((Kp * Td / Ts) * (e[1] - e[0]))       # Absolute PID, computes the unsaturated input
    usat = AutomationShield.constrainFloat(u,saturationMin,saturationMax)               # Saturates the input
    e[0] = e[1]                                                                         # Stores error in the list
    return usat                                                                         # Returns the saturated input