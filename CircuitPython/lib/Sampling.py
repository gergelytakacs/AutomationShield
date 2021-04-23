"""
  Sampling subsystem for CircuitPython

  This serves as a sampling subsystem for a pseudo real-time feedback
  control using CircuitPython boards, such as the Adafruit Metro M4.
  Currently there is no way to access the internal timers of these
  Arduino R3 pinout compatible boards, so instead the monotonic
  functions are used. As (Circuit) Python is an interpreted language
  it offers less predictable performance than compiled languages
  (such as C). The real timing of your feedback task will be affected
  by garbage collection (ranging a few ms!) and background tasks like
  USB activity and file operations.

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

import time                                                 # Imports time functions

class Settings_:
    def __init__(self):
        self.enable = False                                              # Flag for sampling
        self.strictRealTime = False                                      # Not recommended. Python is an interpreted language with less than ideal timekeeping
        self.printTiming = False                                         # Sends real timing to plotter
        self.realTimeViolation = False                                   # Not recommended. Flag for real-time violations. Will not be active anyways.
        self.Tsampling = 0.0                                             # Sampling time

Settings = Settings_()

# Initialization of the sampling subsystem
def begin(Ts):                                                       # Sampling functionality is initialized with begin() to keep consistent with Arduino API
    Settings.Tsampling = Ts*1000                                     # Convert sampling time to _nano_seconds
    Settings.t_last = time.monotonic_ns()                            # Initialize "last" sample as current monotonic time.
    Settings.t_next = Settings.t_last + Settings.Tsampling           # Initialize next sample. The next sampling time is the current time + sampling

# Checking if next sample is on
def stepEnable():                                                    # Must run in an infinite loop (while True:) as it continuously checks
    t = time.monotonic_ns()                                          # Check current time
    if (t >= Settings.t_next):                                       # If time has come
        if Settings.printTiming:                                     # If plotting a real timing is required
            print((round((t - Settings.t_last)) / 1000000, ))        # True sampling in ms. For e.g. plotting.
        if Settings.strictRealTime:
            if (round((t - Settings.t_last) / 100000) * 100000 > Settings.Tsampling): # If it took longer than one sample (at 1/10 ms resolution)
                Settings.realTimeViolation = True                    # Real time constraints have been violated
                print("Real time samples violated")
                while True:                                          # Enter into infinite loop
                    pass                                             # and do nothing
        Settings.t_next += Settings.Tsampling                        # Next sample_rate
        Settings.t_last = t                                          # Exact time when this condition was true
        Settings.enable = True                                       # Enable algorithm step