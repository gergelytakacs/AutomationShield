"""
  AutomationShield API for CircuitPython

  This library serves as an API for the AutomationShield
  ecosystem of Arduino Shields used for control engineering and
  mechatronics education.

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
import sys                                 # Import system module
if '/AutomationShield' not in sys.path:
    sys.path.append('/AutomationShield')
#import gc                                  # Garbage Collector module
#print(gc.mem_free())                       # Reports free memory

ADCREF = 3.3                                # ADC reference voltage is only 3.3 V with Python compatible boards
ADCRES = const(65536)                       # Analog resolution of the Metro M4 is 16 bits
ARES3V3 = ADCREF / ADCRES                   # Voltage per analog resolution level

def mapFloat(x, in_min, in_max, out_min, out_max):                                      # same as Arudino map() but with floating point numbers
    return ((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min)         # linear mapping, same as Arduino map()

def baseConstrain(x, xmin, xmax):           # A simple constrain function, that can be called in constrain() and constrainFloat() to keep the similarity between the Arduino code
    if x <= xmin:                           # If input is smaller than minimum
        x = xmin                            # make input equal to minimum.
    elif x >= xmax:                         # If input is greater than maximum
        x = xmax                            # make it equal to maximum
    return x                                # Return the (modified) input

def constrain(x, xmin, xmax):               # A wrapper to keep compatibility with Arduino API
    return baseConstrain(x, xmin, xmax)     # Just calls baseConstrain()

def constrainFloat(x, xmin, xmax):          # A wrapper to keep compatibility with Arduino API
    return baseConstrain(x, xmin, xmax)     # Just calls baseConstrain()

# Prints a line of separators
def printSeparator(separator):
    length = 60
    print(separator*length)

# Evaluates and prints if a number fits into a range
# First come a description, then the value to be tested, the lower
# end of the range, and the higher end.
# Returns 0 on success and 1 on failure
def printTestResults(text, value, low, high):
    print(text, end="", flush=True)            # Print line
    if value >= low and value <= high:                           # compare
        print(" Ok.")                                            # if it is ok, print ok
        testFail = False                                         # the test has not failed
    else:                                                        # else the test has failed
        print(" Fail.")                                          # print failed
        testFail = True                                          # turn flag on
    return testFail                                                  # return state

# Creates a header for displaying numeric ranges with a label and unit, e.g.
# Begins with a new line, displays TEST LOW HIGH  UNIT, then a line of dashes.
def printLowHighFirst():
    print("")
    header = ['TEST', 'LOW', 'HIGH', 'UNIT']
    print ('{0:<20} {1:<10} {2:<10} {3:<10} '.format(*header))
    printSeparator('-')

# Prints a single line for range measurements in an ordered form, e.g.
# Coil current 0.0 50.6  mA
# Enter the name of the range, first number, second number, then a unit
def printLowHigh(name, low, high, unit, precision):
    line = [name, low, high, unit]
    print ('{0:<20} {1:<10.{prec}f} {2:<10.{prec}f} {3:<10} '.format(*line,prec=precision))