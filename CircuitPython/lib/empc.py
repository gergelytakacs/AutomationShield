"""
  SEQUENTIAL SEARCH FOR ONLINE EXPLICIT MODEL PREDICTIVE CONTROL

  WARNING: This is an experimental example and it does
           not work properly.

  This module implements the the online part of explicit model
  predictive control (EMPC, Explicit MPC). The "Sequential()"
  function realizes a simple sequential search for the region
  with the current state, then computes the PWA control law
  associated with it.

  The sequential search algorithm has been re-written to Python
  ased on the original found in the Multi-Parametric Toolbox
  (MPT) of Kvasnica et. al (see. http://www.mpt3.org).

  The definition of the controller itself is auto-generated by
  MPT in MATLAB, see the empcToPython.m function. The definition
  matrices of the regions and associated laws are stored in the
  lists found in ectrl.py.

  There are several limitations to this code. CircuitPython compiles
  the *.py files at runtime and in RAM. Large problems stored in
  ectrl.py cannot be evaluated because the file size itself is too
  large. Moreover, the Python evaluation of the search function is
  much slower than the C implementation. There are a couple of
  possible but untried workarounds: (a) storing the lists as frozen
  bytcode in the firmware, and (b) using uLab functionality instead
  of lists and C-like element-by-element operations.

  If you have found any use of this code, please cite our work in your
  academic publications, such as thesis, conference articles or journal
  papers. A list of publications connected to the AutomationShield
  project is available at:
  https://github.com/gergelytakacs/AutomationShield/wiki/Publications

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons
  Attribution-NonCommercial 4.0 International License.

  Original C version by: Michal Kvasnica et al.
  Adapted to Python by:  Gergely Takács
  Created on:            11.11.2020.
  Last updated by:       Gergely Takács
  Last update:           11.11.2020.
"""

import ectrl
def Sequential(X):

    abspos = 0
    iregmin = 0

    U = [0] * (ectrl.MPT_RANGE)                                             # Initialize predicted U so later addressing does not fail

    for ireg in range(0, ectrl.MPT_NR):
        isinside = 1
        nc = ectrl.MPT_NC[ireg]
        for ic in range(0, nc):
            hx = 0
            for ix in range(0, ectrl.MPT_DOMAIN):
                hx += ectrl.MPT_A[abspos*ectrl.MPT_DOMAIN+ic*ectrl.MPT_DOMAIN+ix] * X[ix]
            if ((hx - ectrl.MPT_B[abspos+ic]) > ectrl.MPT_ABSTOL):           # constraint is violated, continue with next region
                isinside = 0
                break
        abspos = abspos + ectrl.MPT_NC[ireg]
        if isinside:
            iregmin = ireg
            break

    for ix in range(0, ectrl.MPT_RANGE):
        sx = 0
        for jx in range(0, ectrl.MPT_DOMAIN):
            sx += (ectrl.MPT_F[iregmin * ectrl.MPT_DOMAIN * ectrl.MPT_RANGE + ix * ectrl.MPT_DOMAIN + jx] * X[jx])
            U[ix] = (sx + ectrl.MPT_G[iregmin*ectrl.MPT_RANGE + ix])
    return U