"""
   MAGNETOSHIELD PYTHON MINIMAL COMMUNICATION SPEED TEST

   WARNING: This example is under development and does not work as
            expected!!! Suspected cause is, that the time.monotonic
            function is not granular enough to perform the measurement.

   This example tests for the absolute minimal sampling period that is
   achievable with this shield using Python. The script just tests reading the
   analog input and sending data to the I2C bus wihtout attempting feedback
   control, as these are the most essential operations required for control.

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
   Created on:       29.10.2020
   Last updated by:  Gergely Takács
   Last update on:   29.10.2020
   Tested on:        Adafruit Metro M4 Express
"""
import AutomationShield                         # Import the AutomationShield module
import MagnetoShield                            # Imports the MagnetoShield module for hardware functionality
import time                                     # Imports the time module for delays


DATA_OUTPUT = True                              # Only experiment (False) or with data dumped to serial (True)
PLOTTING_POST = True                            # Does not supply data while the experiment is running, only does it after it is finished
                                                # this helps Mu Plotter not to be flooded. You will only see Y and U plotted. As an alternative
                                                # use an external serial program like CoolTerm.
if PLOTTING_POST:                               # If the plotter of Mu is used, this speed will flood it, so plot it later.
    tlog = []                                   # Empty list to store timing results

k = int(1)                                      # Sample index
expLen = 500                                    # Experiment length in samples
t = 0.0
u = 0.0
previous = 0.0

# Initialize and calibrate board
MagnetoShield.begin()                           # Lock I2C bus
MagnetoShield.calibration()                     # Calibrate device

# Algorithm step - every step that is necessary for control
def step():
    global u, k, t, previous                   # Access these global variables
    now = time.monotonic()                  # Time now
    if (k > expLen):                        # if the experiment is over
        MagnetoShield.actuatorWrite(0.0)    # then turn off magnet
        if DATA_OUTPUT:                     # if outputs are requested
            if PLOTTING_POST:               # In case plotting in post is enabled
                for j in enumerate(tlog):   # for every element in the log vector of outputs
                    print(("{:.10f}".format(tlog[j[0]])))     # Print to serial
            while True:                     # then stop
                pass                        # and do nothing


    # SISO read/write routine
    u += 0.01;
    y = MagnetoShield.sensorRead()              # [mm] sensor read routine
    MagnetoShield.actuatorWrite(u)              # [V] write input to actuator
    t = now - previous                      # [s] execution time
    previous = now                          # [s] Store previous sample
    if DATA_OUTPUT:
        if PLOTTING_POST:                   # If we are plotting after the experiment
            tlog.append(t)                  # append output y to output vector
        else:                               # otherwise we are plotting "real time"
            print((t))                      # send data to output and
    k += 1                                      # Increment time-step k


# Main loop launches a single step at each enable time
while True:                                     # Infinite loop
        step()                                  # Algorithm step