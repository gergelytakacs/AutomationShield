"""
  MagnetoShield automatic distance calibration experiment

  This example initializes the MagnetoShield and measures
  the Hall sensor output with the electromagnet turned
  on and off. It returns the minimal and maximal ADC levels,
  equivalent magnetic flux in Gauss and the position as
  estimated by a power curve.

  Upload the code to your board, then open the Serial
  Plotter function in your Arduino IDE. You may change the
  reference trajectory in the code.

  Tested with Adafruit Metro M4 Express.

  If you have found any use of this code, please cite our work in your
  academic publications, such as theses, conference articles or journal
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
import AutomationShield      # Imports AutomationShield module for generic functionality

# Maximum and minimum mean voltage, ADC or position (not magnetic)

AutomationShield.printSeparator('=')
print("Calibration in progress...")   # Begin note
MagnetoShield.begin()                 # Initializes shield
MagnetoShield.calibration()           # Calibrates shield
print("Done.")                        # Done note
AutomationShield.printSeparator('=')

Minimum=MagnetoShield.minCalibrated   #  minimum ADC, maximum Gauss
Maximum=MagnetoShield.maxCalibrated   #  maximum ADC, minimum Gauss


print("Hall sensor maximum magnetic reading at: ", Minimum, \
"ADC levels when magnet at bottom, that is ",MagnetoShield.adcToGauss(Minimum),\
" G, estimated distance from electromagnet is ",MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(Maximum)),\
" mm")

AutomationShield.printSeparator('-')

print("Hall sensor minimum magnetic reading at: ", Maximum, \
"ADC levels when magnet at top, that is ",MagnetoShield.adcToGauss(Maximum),\
" G, estimated distance from electromagnet is ",MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(Minimum)),\
" mm")
