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
  academic publications, such as thesis, conference articles or journal
  papers. A list of publications connected to the AutomationShield
  project is available at:
  https://github.com/gergelytakacs/AutomationShield/wiki/Publications

  Created by:       Gergely Takács
  Created on:       12.10.2020.
  Last updated by:  Gergely Takács
  Last update:      26.10.2020.
"""

import AutomationShield     # Import the AutomationShield module for constrain functions

class Settings_:
# These could be solved in a prettier way, but let's keep consistent with the Arduino API!
    def __init__(self):
        self.Kp = 0.0
        self.Ti = 0.0
        self.Td = 0.0
        self.Ts = 0.0
        self.eSum = 0.0                  # Sum of the error for integral
        self.e = [0.0, 0.0]              # List to stores current and previous error

    def setKp(self,Kpin):            # Takes the proportional gain
        self.Kp = Kpin               # Stores the input argument in the global variable

    def setTi(self,Ti_in):           # Takes the integral time constant
        self.Ti = Ti_in              # Stores the input argument in the global variable

    def setTd(self,Td_in):           # Takes the derivative time constant
        self.Td = Td_in              # Stores the input argument in the global variable

    def setTs(self,Ts_in):           # Takes the sampling time in microseconds!
        self.Ts = Ts_in / 1000000    # Stores the input argument in the global variable

Settings = Settings_()

# Computes an absolute PID input
def compute(err, saturationMin, saturationMax, antiWindupMin, antiWindupMax):           # Requires error, input saturation and integral windup saturation                                                                      # Sets these as global variables
    Settings.eSum = Settings.eSum + err                                                                   # Sum of errors for the integral (summation) part
    Settings.e[1] = err                                                                          # New error in position [1] (right)
    Settings.eSum = AutomationShield.constrainFloat((Settings.Kp * Settings.Ts / Settings.Ti )* Settings.eSum, antiWindupMin, antiWindupMax) / (Settings.Kp * Settings.Ts / Settings.Ti)        # Anti-windup by clamping
    u = (Settings.Kp * Settings.e[1]) + ((Settings.Kp * Settings.Ts / Settings.Ti) * Settings.eSum) + ((Settings.Kp * Settings.Td / Settings.Ts) * (Settings.e[1] - Settings.e[0]))       # Absolute PID, computes the unsaturated input
    usat = AutomationShield.constrainFloat(u,saturationMin,saturationMax)               # Saturates the input
    Settings.e[0] = Settings.e[1]                                                                         # Stores error in the list
    return usat                                                                         # Returns the saturated input