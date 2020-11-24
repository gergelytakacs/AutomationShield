"""
  MagnetoShield hardware self test routine

  Upload the code to your board, then open the Serial
  Monitor function in a serial console. Be prepared to turn the
  shaft of the potentiometer.

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


import MagnetoShield                            # Imports the MagnetoShield module for hardware functionality
import AutomationShield                         # Imports AutomationShield module for generic functionality
import time                                     # Imports the time module for delays

testFail = False                                # Flag to indicate wheter any of the tests failed

print("")                                       # Just leave vertical spaces
print("")                                       # Just leave vertical spaces
AutomationShield.printSeparator("=")            # Print separator line
print("TESTING MAGNETOSHIELD COMPONENTS")       # Header
AutomationShield.printSeparator("=")            # Print separator line

MagnetoShield.begin()                           # Lock I2C bus
MagnetoShield.dacWrite(0)                       # Turn off magnet


# Reference potentiometer test ----------------------------

print("Turn pot to 0%!", end="", flush=True)                     # Asks user to turn pot
time.sleep(5)                                                    # wait until that happens
potReferenceLow = MagnetoShield.referenceRead()                  # read pot state
if potReferenceLow  >= 0.0 and potReferenceLow  <= 1.0:          # test pot state
    print(" Ok.")                                                # if it is ok, print ok
    testFail = False                                             # the test has not failed
else:                                                            # else the test has failed
    print(" Fail.")                                              # print failed
    testFail = True                                              # turn flag on

print("Turn pot to 100%!", end="", flush=True)                   # Asks user to turn pot
time.sleep(5)                                                    # wait until that happens
potReferenceHigh = MagnetoShield.referenceRead()                 # read pot state
if potReferenceHigh  >= 99.0 and potReferenceHigh  <= 100.0:     # test pot state
    print(" Ok.")                                                # if it is ok, print ok
    testFail = testFail or False                                 # the test has not failed
else:                                                            # else the test has failed
    print(" Fail.")                                              # print failed
    testFail = testFail or True                                  # turn flag on
    testFail = testFail or True                                  # turn flag on

# Hall sensor test and magnet polarity test----------------------------

MagnetoShield.dacWrite(0)                           # Turn off magnet
hallLow = MagnetoShield.MAGNETO_YPIN.value          # Extract analog input levels
testFail = testFail or AutomationShield.printTestResults("Testing Hall sensor low...",hallLow,2500,2600)

MagnetoShield.dacWrite(MagnetoShield.DACMAX)        # Turn on magnet
time.sleep(0.2)
hallHigh = MagnetoShield.MAGNETO_YPIN.value         # Extract analog input levels
testFail = testFail or AutomationShield.printTestResults("Testing Hall sensor high...",hallHigh,40000,41000)
AutomationShield.printTestResults("Testing Hall sensor high...",hallHigh,40000,41000)

AutomationShield.printTestResults("Testing magnet polarity...",hallHigh,0,41000)
testFail = testFail or AutomationShield.printTestResults("Testing magnet polarity...",hallHigh,0,41000)

# Voltage supply test --------------------------------------
MagnetoShield.dacWrite(0)                           # Turn off magnet
coilVoltageLow = MagnetoShield.auxReadVoltage()     # Coil voltage
testFail = testFail or AutomationShield.printTestResults("Testing coil voltage (off)...",coilVoltageLow,0.0,0.5)
AutomationShield.printTestResults("Testing coil voltage (off)...",coilVoltageLow,0.0,0.5)

MagnetoShield.dacWrite(MagnetoShield.DACMAX)        # Turn on magnet
time.sleep(0.2)
coilVoltageHigh = MagnetoShield.auxReadVoltage()    # Coil voltage
testFail = testFail or AutomationShield.printTestResults("Testing coil voltage (on)...",coilVoltageHigh,9.0,11.0)
AutomationShield.printTestResults("Testing coil voltage (on)...",coilVoltageHigh,9.0,11.0)

# Current sensor test --------------------------------------
MagnetoShield.dacWrite(0)                           # Turn off magnet
coilCurrentLow = MagnetoShield.auxReadCurrent()     # Coil current
testFail = testFail or AutomationShield.printTestResults("Testing coil current (off)...",coilCurrentLow,0.0,1.0)
AutomationShield.printTestResults("Testing coil current (off)...",coilCurrentLow,0.0,1.0)

MagnetoShield.dacWrite(MagnetoShield.DACMAX)        # Turn on magnet
time.sleep(0.2)
coilCurrentHigh = MagnetoShield.auxReadCurrent()     # Coil current
testFail = testFail or AutomationShield.printTestResults("Testing coil current (on)...",coilCurrentHigh,49.0,60.0)
AutomationShield.printTestResults("Testing coil current (on)...",coilCurrentHigh,49.0,60.0)

MagnetoShield.dacWrite(0)                           # Turn off magnet
print("")                                           # New line
if testFail:
    print("SOME TESTS FAILED!")
else:
    print("ALL TESTS PASSED.")

# Summary -----------------------------
print(" ")                                      # New line
AutomationShield.printSeparator("=")            # Prints separator
print("Details:")
AutomationShield.printLowHighFirst()            # Header
AutomationShield.printLowHigh("Potentiometer",potReferenceLow,potReferenceHigh,"%",0)
AutomationShield.printLowHigh("Hall sensor",hallLow,hallHigh,"10-bit levels",0)
AutomationShield.printLowHigh("Hall sensor",MagnetoShield.adcToGauss(hallHigh),MagnetoShield.adcToGauss(hallLow),"G",0)
AutomationShield.printLowHigh("Distance",MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(hallHigh)),MagnetoShield.gaussToDistance(MagnetoShield.adcToGauss(hallLow)),"mm",1)
AutomationShield.printLowHigh("Coil voltage",coilVoltageLow,coilVoltageHigh,"V",1)
AutomationShield.printLowHigh("Coil current",coilCurrentLow,coilCurrentHigh,"mA",1)