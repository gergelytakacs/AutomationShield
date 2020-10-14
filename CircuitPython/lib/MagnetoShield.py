"""
  CircuitPython API for the MagnetoShield didactic hardware.

  The file is a part of the application programmers interface for
  the MagnetoShield didactic tool for control engineering and
  mechatronics education. The MagnetoShield implements a magnetic
  levitation experiment on an Arduino shield. The MagnetoShield is
  compatible with CircuitPython enabled boards, such as the
  Adafruit Metro M4 Express.

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
import math                         # Imports math module for the logarithm operation in the calibration routine
import time                         # Imports the time module that is needed to wait for transients in the calibration procedure
import board                        # Imports the boars module defining pin locations and other hardware functions
import busio                        # Imports the busio module defining I2C communicaiton for the DAC chip
from analogio import AnalogIn       # Imports AnalogIn from the analogio module defining analog input for the sensors
import AutomationShield             # Imports the AutomationShield module for common functions and common constants
import microcontroller              # Imports microcontroller functionality, e.g. to tell CPU clock

calibrated = False                  # Variable storing calibration state, initialized as false

SHIELDRELEASE = const(4)            # Use number only: e.g. for R4 is 4. Should work with R3, will NOT work with R1, R2
VGAIN = 4.2256                      # Defines the voltage sensing gain (voltage divider), theoretical value 4.0
voltageRef = 10.0                   # Hardware calibrated maximum voltage on the magnet. This will be re-measured at calibration
IGAIN = 33.333333333333333          # Defines the current sensing gain mA/V

# Setup shorthands for the physical pins
MAGNETO_RPIN = AnalogIn(board.A0)   # Defines the location of reference pot
MAGNETO_VPIN = AnalogIn(board.A1)   # Defines the location of input voltage sensing
MAGNETO_IPIN = AnalogIn(board.A2)   # Defines the location of input current sensing
MAGNETO_YPIN = AnalogIn(board.A3)   # Defines the location of the Hall sensor

EMAGNET_HEIGHT = const(20)          # [mm] Location of electromagnet above ground
MAGNET_LOW = const(3)     	        # [mm] Top of the magnet from ground - distance from Hall element (low)
MAGNET_HIGH = const(8)		        # [mm] Top of the magnet from ground - distance from Hall element (high)

HALL_SENSITIVITY = const(800)       # [G/V] = 1.25 mV/G Sensitivity of the TI DRV5055Z4 Hall sensor
HALL_LSAT = const(2528)             # [16-bit ADC] Lower saturation of the Hall sensor
HALL_HSAT = const(40928)            # [16-bit ADC] Upper saturation of the Hall sensor

# Distance model based on magnetic flux density
# As it is hard to make exact measurements a two-point
# calibration of a power function seems to work best
D_P1_DEF = 3.233100   		        # Default distance function constant (f(y) = D_P1*x^D_P2) for Flux vs. distance from magnet
D_P2_DEF = 0.220571			        # Default distance function constant (f(y) = D_P1*x^D_P2) for Flux vs. distance from magnet

MCP4725 = const(0x60)               # I2C Address of the DAC module
DACMAX = const(4095)                # Maximal decimal DAC value for 12 bits
i2c = busio.I2C(board.SCL, board.SDA)   # Create an object with the I2C bus
dacBuffer = bytearray(2)           # byte array to store the DAC write value

P1 = 1.41353993     		        # Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
P2 = -15.4070873			        # Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
P3 = 389.266686			            # Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage
P4 = -11.7613432			        # Polynomial constant (f(y) =  p1*x^3 + p2*x^2 + p3*x + p4 for DAC vs. Output voltage

# Reads the reference from the reference pot in percents 0-100
def referenceRead():
    return AutomationShield.mapFloat(MAGNETO_RPIN.value, 0.0, AutomationShield.ADCRES, 0.0, 100.0)  # [%] Reads the voltage on the pot and recomputes it into %

# Reads the current through the Electromagnet in mA
def auxReadCurrent():
    return MAGNETO_IPIN.value * AutomationShield.ARES3V3 * IGAIN     # [mA] Reads the voltage from the INA169 sensor and computes it in mA

# Reads the current across the Electromagnet in V
def auxReadVoltage():
    return MAGNETO_VPIN.value * AutomationShield.ARES3V3 * VGAIN      # [V] Reads the  ADC on the magnet, computes it Volts

# Converts a 12-bit ADC reading from the TI DRV5055Z4 Hall sensor
# to Gauss. The Hall sensor picks up on the levitating magnet
def adcToGauss(adc):
    return (2.5-(adc*AutomationShield.ARES3V3))*HALL_SENSITIVITY       # [G] Zero of the hall sensor minus reading times sensor sensitivity

# Reads sensor and returns the Hall sensor reading in Gauss
def sensorReadGauss():
    return adcToGauss(MAGNETO_YPIN.value)                              # [G] Takes a direct ADC reading, then changes this to gauss

# Converts the magnetic flux reading from the Hall sensor
# to distance measured from the bottom of the magnet
def gaussToDistance(g):                                                # [mm] Takes magnetic measurement in Gauss and remaps to mm
    global calibrated, d_p1, d_p2, D_P1_DEF, D_P2_DEF                  # Take these variables as global
    if (not calibrated):                                               # If the board has not been calibrated
        d_p1 = D_P1_DEF                                                # use the default values
        d_p2 = D_P2_DEF
    return d_p1*pow(g, d_p2)                                           # Otherwise compute the power approximation of the distance

# Reads sensor and returns the Hall sensor reading in mm
def sensorReadDistance():	                                           # Wrapper function to read magnet distance in mm
	return gaussToDistance(sensorReadGauss())                          # Read the magnetic flux and re-compute to mm

# Default sensor reading method returns mm distance from magnet
def sensorRead():	                                                   # Default method (wrapper)
	return sensorReadDistance()                                        # Calls the distance function

# Reads sensor and returns percentage of voltage from Hall sensor
# effectively giving an indirect percentual distance
def sensorReadPercents():
    global HALL_HSAT, HALL_LSAT                                      # Make these accessible from global
    if not calibrated:                                               # If the calibration has been not launched
        minCalibrated = HALL_LSAT                                     # Use default values
        maxCalibrated = HALL_HSAT	
	# Recalculates measured value in interval (minCalibrated, maxCalibrated) to percents 0-100%
	return AutomationShield.mapFloat(MAGNETO_YPIN.value, minCalibrated, maxCalibrated, 0.0, 100.0)

# Write DAC levels (12-bit) to the MCP4725 chip
def dacWrite(DAClevel):                                                # Writes 12 bit levels to DAC chip
    if DAClevel < 0:                                                   # If input argument is negative
        DAClevel = 0                                                   # Constrain it to 0
    DAClevel = DAClevel & 0xFFF                                        # Modifies input to 12 bits
    dacBuffer[0] = (DAClevel >> 8) & 0xFF                              # Prepares MSB (see MCP4725 datasheet)
    dacBuffer[1] = DAClevel & 0xFF                                     # Prepares LSB (see MCP4725 datasheet)
    i2c.writeto(MCP4725, dacBuffer)                                    # Writes all to this address (as bytearray)

# An initialization method for the MagnetoShield
def begin():                                                           # uses Arduino syntax for consistency
    while not i2c.try_lock():                                          # Locks the I2C peripheral
        pass                                                           # Waits until done so

# Calibration method for MagnetoShield
def calibration():                                                     # Calibration of the height reading and voltage supply
    dacWrite(0)                                                        # Turn the magnet off
    time.sleep(0.1)                                                    # Wait for things to settle

    # Measures minimal ADC levels on hall sensor
    global minCalibrated                                               # Minimal ADC levels on Hall sensor
    minCalibrated = MAGNETO_YPIN.value                                 # Initialize by reading
    for measurement in range(100):	                                   # Perform 100 measurements
        min = MAGNETO_YPIN.value			                           # Measure
        if (min < minCalibrated): 		                               # If higher than already
            minCalibrated = min 	                                   # Save new minimum

    dacWrite(DACMAX)                                                   # Turns on magnet completely
    time.sleep(0.5)                                                    # Wait for things to settle

    # Measures maximal ADC levels on hall sensor
    global maxCalibrated                                               # Minimal ADC levels on Hall sensor
    maxCalibrated = MAGNETO_YPIN.value                                 # Initialize by reading
    for measurement in range(100):	                                   # Perform 100 measurements
        max = MAGNETO_YPIN.value			                           # Measure
        if (max > maxCalibrated): 		                               # If higher than already
            maxCalibrated = max 	                                   # Save new minimum

    # Measuring maximal supply voltage
    global voltageRef                                                  # Global variable to store voltage reference
    voltageRef = 12                                                    # Initialize this as 12
    for measurement in range(100):	                                   # Perform 100 measurements
        minV = auxReadVoltage()			                               # Measure
        if (minV < voltageRef): 		                               # If higher than already
            voltageRef = minV 	                                       # Save new minimum
    dacWrite(0)

    # Recalibrate distance based on these
    global d_p2, d_p1, calibrated                                      # Define these as global, the Gauss to mmm function and others need it
    d_p2 = math.log((EMAGNET_HEIGHT-MAGNET_LOW) / (EMAGNET_HEIGHT-MAGNET_HIGH)) / math.log(adcToGauss(minCalibrated) / adcToGauss(maxCalibrated))
    d_p1 = (EMAGNET_HEIGHT-MAGNET_HIGH) / (pow(adcToGauss(maxCalibrated), d_p2))

    time.sleep(0.5)                                                    # Wait for things to settle
    calibrated = 1                                                     # The shield has been calibrated

# Writes input to actuator as a percentage in the range of 0-100%
def actuatorWritePercents(u):                                          # Expects percentages, supplies input to magnet
    actuatorWriteVoltage(AutomationShield.mapFloat(u, 0.0, 100.0, 0.0, voltageRef))         # Writes as Voltage - needed because of the nonlinearity DAC/voltage

# Default actuator write function (just a call)
def actuatorWrite(u):                                                  # Just a wrapper
	actuatorWriteVoltage(u)                                            # Calls preferred routine

# Writes input to actuator as desired voltage on magnet
def actuatorWriteVoltage(u):                                           # Writes desiredvoltages to the magnet
	dacIn = voltageToDac(u)							                   # Re-computes DAC levels according to Voltage
	dacIn = AutomationShield.constrain(dacIn, 0 ,DACMAX)			   # Constrain input into acceptable range
	dacWrite(dacIn)  	   											   # Writes to DAC

# Computes DAC levels for equivalent magnet voltage. This is nearly linear anyways
def voltageToDac(vOut):                                                # Projects desired voltages to DAC levels
    dacOut=round((P1 * pow(vOut,3)) + (P2 * pow(vOut,2)) + (P3 * vOut) + P4)
    if dacOut < 0.0:                                                   # Prevent negative values
        dacOut = 0.0		                                           # Make it 0 if negative
    return dacOut                                                      # Returns the DAC levels needed for this voltage