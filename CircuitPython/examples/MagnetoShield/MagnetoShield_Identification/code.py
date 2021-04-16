"""
  MagnetoShield closed-loop identification experiment

  Runs a closed-loop experiment to gather data for system
  identification.

  This example initializes the sampling and PID control
  subsystems from the AutomationShield modules and starts a
  predetermined reference trajectory. Noise is injected to
  this input trajectory to create a rich signal suitable for
  system identification. Upload the code to your board along
  with the necessary module then use a serial terminal software
  or Matlab to acquire the dataset for later processing.

  WARNING: Do not use "Mu" for data aquisition.
  WARNING: This is "soft real-time", the timing is not perfect,
           yet, it is possible to gather meaningful data for
           identification.
  WARNING: It is unlikely that the Metro M0 or other less
           powerful boards handle this code properly, timing
           will be unreliable, thus, model quality will suffer.

  Tested with the
    - Adafruit Metro M4 Express (CircuitPython 5.3.1).

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
  Last updated on: 26.10.2020
"""
import AutomationShield                         # Imports the AutomationShield module
import MagnetoShield                            # Imports the MagnetoShield module for hardware functionality
import Sampling                                 # Imports the Sampling module for pseudo-real time sampling
import PIDAbs                                   # Imports the PIDAbs module for the absolute PID algorithm
import time

from random import seed                         # Import seeding functionality
from random import randint                      # Generate random integers

wPP = 4.0                                       # [V] Injected input noise amplitude (peak-to-peak)
seed(1)                                         # Seed random number generator
PLOTTING_POST = True                            # Does not supply data while the experiment is running, only does it after it is finished
                                                # this helps Mu Plotter not to be flooded. You will only see Y and U plotted. As an alternative
                                                # use an external serial program like CoolTerm.

# Sampling rate and PID Tuning
Ts = int(5000)                                  # [ms] Sampling in microseconds, lower limit unknown for the M4 Express

# PID Tuning
KP = 2.3                                        # PID Kp (proportional constant)
TI = 0.1                                        # PID Ti (integral time constant)
TD = 0.03                                       # PID Td (derivative time constant)

R = [14.0]                                      # [mm] Desired reference trajectory (pre-set)
T = int(4000)                                   # [steps] Experiment section length
r = R[0]                                        # Initial reference

wBias=wPP/2.0                                   # [V] Noise bias
wP=int(wPP)*100                                 # For (pseudo)-random generator

# Fallback for slower processors, since sampling cannot be kept up with desired speed
def fallbackSettings():                                 # Used later, comment out the function call if not needed.
    import microcontroller                              # Imports the microcontroller module so that CPU speed can be determined
    global Ts, KP, TI, TD, R, T                         # Makes these global to be settable
    if (microcontroller.cpu.frequency/1000000 == 48):   # For the Adafruit Metro M0 @48 MHz override defaults
        Ts = int(6000)                                  # [ms] Sampling in microseconds, lower limit unknown for the M0 Express
        KP = 2.0                                        # PID Kp (proportional constant)
        TI = 0.4                                        # PID Ti (integral time constant)
        TD = 0.02                                       # PID Td (derivative time constant)
        R = [14.0, 14.0]                                # [mm] Desired reference trajectory (pre-set)
        T = int(2500)                                   # [steps] Experiment section length

if PLOTTING_POST:                               # If the plotter of Mu is used, this speed will flood it, so plot it later.
    Ulog = []                                   # Empty list to store input results
    Ylog = []                                   # Empty list to store position output results
    Ilog = []                                   # Empty list to store current output results

k = int(1)                                      # Sample index
i = int(1)                                      # Experiment section counter

# Initialize and calibrate board
MagnetoShield.begin()                           # Lock I2C bus
MagnetoShield.calibration()                     # Calibrate device
fallbackSettings()                              # These are only active when CPU speed is 48 MHz. Comment if you want to use settings as above

# Set the PID settings
PIDAbs.Settings.setKp(KP)                       # Proportional
PIDAbs.Settings.setTi(TI)                       # Integral
PIDAbs.Settings.setTd(TD)                       # Derivative
PIDAbs.Settings.setTs(Ts)                       # Sampling (use Ts in microseconds)

Sampling.begin(Ts)                              # Initialize sampling subsystem (based on time.monotonic_ns())

# Algorithm step - every step that is necessary for control
def step():
    global r, R, i, k                           # Access these global variables
    if (k > (len(R) * T) - 1):                  # if the experiment is overs
        Sampling.Settings.realTimeViolation = False      # Not a real-time violation
        MagnetoShield.actuatorWrite(0.0)        # then turn off magnet
        if PLOTTING_POST:                       # In case plotting in post is enabled
            for j in enumerate(Ylog):           # for every element in the log vector of outputs
                print((Ulog[j[0]],Ylog[j[0]],Ilog[j[0]],))   # Print to serial
                time.sleep(0.05)               # Wait a bit so that Mu plotter can catch up. Uncomment this for Mu illustration
        while True:                             # then stop
            pass                                # and do nothing
    else:                                       # if the experiment is not yet over
        if (k % (T*i) == 0):                    # else for each section
            r = R[i]                            # set reference
            i += 1                              # and increase section counter for next

    y = MagnetoShield.sensorRead()              # [mm] sensor read routine
    I = MagnetoShield.auxReadCurrent()          # [mA] Read current value
    w = wBias-float(randint(0,wP))/100.0        # [V] Input noise, gaussian dist.
    u = PIDAbs.compute(-(r-y), 0.0, 10.0, -10.0, 10.0) + w #Compute constrained absolute-form PID + noise
    u = AutomationShield.constrain(u, 0.0, 10.0)  # [V] contstrain to physically realizable data
    MagnetoShield.actuatorWrite(u)              # [V] write input to actuator
    if PLOTTING_POST:                           # If we are plotting after the experiment
        Ulog.append(u)                          # append input u to input vector
        Ylog.append(y)                          # append output y to output vector
        Ilog.append(I)                          # append current output I to output vector
    k += 1                                      # Increment time-step k

# Main loop launches a single step at each enable time
while True:                                     # Infinite loop
    Sampling.stepEnable()                       # Routine to enable the algorithm step, changes the flag Sampling.enable
    if Sampling.Settings.enable:                # If time comes
        step()                                  # Algorithm step
        Sampling.Settings.enable = False        # Then disable