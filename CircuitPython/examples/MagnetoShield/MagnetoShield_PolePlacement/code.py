"""
   MangetoShield pole placement example

   This example reads uses the linearized model of the MagnetoShield
   and the pole placement gain calculated in the
   MagnetoShield_PolePlacement_Simulation.m example in MATLAB.
   The state measurement can be performed directly by reading the position
   and current values, then differentiating for the velocity.

  Tested with the following boards
  - Adafruit Metro M4 Express (1),(4)
  - Adafruit Metro M0 Express (2),(3)
  - Adafruit Metro M4 Grand Central (1),(4)

  (1) Runs as expected. Mu cannot keep up with data output, use external means to log.
  (2) Runs only without serial output.
  (3) Serious degradation visible, might stabilize only for a while, increase sampling if possible
  (4) Strict real-time not achievable

  If you have found any use of this code, please cite our work in your
  academic publications, such as theses, conference articles or journal
  papers. A list of publications connected to the AutomationShield
  project is available at:
  https://github.com/gergelytakacs/AutomationShield/wiki/Publications

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Created by:       Gergely Takács
  Created on:       26.10.2020.
  Last updated by:  Gergely Takács
  Last update:      26.10.2020.
"""
import AutomationShield                         # Import the AutomationShield module
import MagnetoShield                            # Imports the MagnetoShield module for hardware functionality
import Sampling                                 # Imports the Sampling module for pseudo-real time sampling
import time                                     # Imports the time module for delays
import sys                                      # Imports system module to tell platform

# Determine version, ulab is not available on some boards
if (sys.platform == 'MicroChip SAMD51'):         # Identify board
    import ulab as np                            # Imports ulab, but uses numpy naming
    from ulab import linalg                      # Imports linear algebra functionality from ulab


MANUAL = False                                  # Reference by pot (True) or automatically (False)?
DATA_OUTPUT = True                              # Only experiment (False) or with data dumped to serial (True)
PLOTTING_POST = False                           # Does not supply data while the experiment is running, only does it after it is finished
                                                # this helps Mu Plotter not to be flooded. You will only see Y and U plotted. As an alternative
                                                # use an external serial program like CoolTerm.

# Sampling rate
Ts = int(5000)                                  # [ms] Sampling in microseconds, lower limit unknown for the M4 Express

# Linearization points
y0 = 14.3                                       # [mm] Linearization point based on the experimental identification
I0 = 21.9                                       # [mA] Linearization point based on the experimental identification
u0 =  4.6234                                    # [V] Linearization point based on the experimental identification
yp = 0.0					# [mm] Previous output measurement

R = [14.0, 13.0, 14.0, 15.0, 14.0]              # [mm] Desired reference trajectory (pre-set)
T = int(1000)                                   # [steps] Experiment section length
r = R[0]                                        # Initial reference


# LQ gain and initial states
# Use ulab for SAMD51
if (sys.platform == 'MicroChip SAMD51'):                # Identify board
    K =  np.array([8.7355, -2773, -62.884, 22.922])     # LQ gain with integrator, see MATLAB example
    X =  np.zeros((4, 1))                               # Initial state vector
    Xr = np.zeros((4, 1))                               # Initial state reference
else:                                                   # else just use lists
    K =  [8.7355, -2773, -62.884, 22.922]               # pole placement gain with integrator, see MATLAB example
    X =  [0.0, 0.0, 0.0, 0.0]                           # Initial state vector
    Xr = [0.0, 0.0, 0.0, 0.0]                           # Initial state reference

# Fallback for slower processors, since sampling cannot be kept up with desired speed
def fallbackSettings():                                 # Used later, comment out the function call if not needed.
    import microcontroller                              # Imports the microcontroller module so that CPU speed can be determined
    global Ts, KP, TI, TD, R, T, DATA_OUTPUT            # Makes these global to be settable
    if (microcontroller.cpu.frequency/1000000 == 48):   # For the Adafruit Metro M0 @48 MHz override defaults
        Ts = int(7000)                                  # [ms] Sampling in microseconds, lower limit unknown for the M0 Express
        K = [8.7355, -2773, -62.884, 22.922]            # Less agressive pole placement gaing to make it work (at least on some level)
        R = [14.0, 14.0]                                # [mm] Desired reference trajectory (pre-set)
        T = int(2500)                                   # [steps] Experiment section length
        DATA_OUTPUT = False                             # Disable logging output

if PLOTTING_POST:                               # If the plotter of Mu is used, this speed will flood it, so plot it later.
    Ylog = []                                   # Empty list to store output results
    Ulog = []                                   # Empty list to store input results

k = int(1)                                      # Sample index
i = int(1)                                      # Experiment section counter

# Initialize and calibrate board
MagnetoShield.begin()                           # Lock I2C bus
MagnetoShield.calibration()                     # Calibrate device
fallbackSettings()                              # These are only active when CPU speed is 48 MHz. Comment if you want to use settings as above

Sampling.begin(Ts)                              # Initialize sampling subsystem (based on time.monotonic_ns())

# Algorithm step - every step that is necessary for control
def step():
    global yp, r, R, i, k, Ts                   # Access these global variables
    if MANUAL:                                  # If reference from potentiometer
        Xr[0] = AutomationShield.mapFloat(MagnetoShield.referenceRead(), 0.0, 100.0, 12.0, 17.0)
    else:                                       # if pre-set experiment
        if (k > (len(R) * T) - 1):              # if the experiment is overs
            Sampling.Settings.realTimeViolation = False  # Not a real-time violation
            MagnetoShield.actuatorWrite(0.0)    # then turn off magnet
            if DATA_OUTPUT:                     # if outputs are requested
                if PLOTTING_POST:                   # In case plotting in post is enabled
                    for j in range(0,len(Ylog)):    # for every element in the log vector of outputs
                        print((Ylog[j],Ulog[j],))   # Print to serial
                        time.sleep(0.03)            # Wait a bit so that Mu plotter can catch up
            while True:                         # then stop
                pass                            # and do nothing
        else:                                   # if the experiment is not yet over
            if (k % (T*i) == 0):                # else for each section
    		    Xr[1] = (R[i]-y0)/1000.0    # set reference
                    r = R[i]                    # set reference
                    i += 1                      # and increase section counter for next

    y = MagnetoShield.sensorRead()              # [mm] sensor read routine
    I = MagnetoShield.auxReadCurrent()	        # [mA] Current read

# Direct state measurement with differentiation for speed
    X[0] = X[0] + (Xr[1] - X[1])                 	 	    # integrator
    X[1] = (y - y0) / 1000.0                        	 	# position calculated from measurement, compensated for linearization point, converted to [m]
    X[2] = (y - yp) / (1000.0 * (float(Ts) / 1000000.0)) 	# speed calculated using differentiation
    X[3] = (I - I0) / 1000.0                        	 	# current compensated for linearization point and converted to [A]
    yp = y                                      		    # at the end of the calculation current value becomes previous value for the next iteration

# Pole placement control algorithm
    if (sys.platform == 'MicroChip SAMD51'):                # Identify board
        u = -(linalg.dot(K, X) + u0)[0]                     # Pole placement control algorithm using ulab
    else:
        u = 0.0                                             # Initialize u
        for iVec in range(0, 4):                            # For each element in K
            u = u - K[iVec]*X[iVec]                         # Nominal pole placement part
            u = u + u0                                      # Correct for linearization point

    MagnetoShield.actuatorWrite(u)              # [V] write input to actuator
    if DATA_OUTPUT:
        if PLOTTING_POST:                       # If we are plotting after the experiment
            Ylog.append(y)                      # append output y to output vector
            Ulog.append(u)                      # append input u to input vector
        else:                                   # otherwise we are plotting "real time"
            print((r, y, u))                    # send data to output and
    k += 1                                      # Increment time-step k

# Main loop launches a single step at each enable time
while True:                                     # Infinite loop
    Sampling.stepEnable()                       # Routine to enable the algorithm step, changes the flag Sampling.enable
    print(Sampling.Settings.enable)
    if Sampling.Settings.enable:                # If time comes
        step()                                  # Algorithm step
        Sampling.Settings.enable = False        # Then disable