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