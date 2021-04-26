"""
  MagnetoShield Power Calibration Experiment

  Upload the code to your board, then log and export the data
  using a serial terminal. The code goes through analog levels
  supplied into the DAC module, then measures true voltage and
  current. Although due to the internal hardware design the response
  should be fairly linear, this can be (and is) used to create a
  calibration curve.

  Tested with Adafruit Metro M4 Express.

  If you have found any use of this code, please cite our work in your
  academic publications, such as thesis, conference articles or journal
  papers. A list of publications connected to the AutomationShield
  project is available at:
  https://github.com/gergelytakacs/AutomationShield/wiki/Publications

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.
  Created by Gergely Takács.
  Created on:      23.10.2020
  Last updated on: 23.10.2020
"""


import MagnetoShield         # Imports the MagnetoShield module for hardware functionality
import time                  # Imports the time module for delays


# Maximum and minimum mean voltage, ADC or position (not magnetic)
MagnetoShield.begin()                 # Initializes shield


numMeas = 100                         # Number of measurements
MagnetoShield.dacWrite(0)             # Turn off magnet
MagnetoShield.dacWrite(0)             # Turn off magnet

for level in range(0, MagnetoShield.DACMAX):        # Run through all levels
    MagnetoShield.dacWrite(level)     # Set current level
    time.sleep(0.1)                   # Wait for things to settle
    usum = 0.0                        # Sum of voltage measurements
    isum = 0.0                        # Sum of current measurements
    for avgrun in range(0, numMeas):  # make averages
        usum = usum + MagnetoShield.auxReadVoltage() # Create sum of voltage measurements
        isum  = MagnetoShield.auxReadCurrent()   # Create sum of current measurements
    u = usum / numMeas                           # Calculate average
    i = isum / numMeas                           # Calculate average
    print((level, u, i))                         # Print results