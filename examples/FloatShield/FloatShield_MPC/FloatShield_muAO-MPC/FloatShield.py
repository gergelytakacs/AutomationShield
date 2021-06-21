# MPC problem definition file for muaOMPC in Python
# Visit www.automationshield.com for more information.
# ========================================================

dt = 0.025        # Sampling period
N = 2             # Prediction horizon
# Discretized state-space matrix A
Ad = [[ 1, 0.02437, 0.00061, 0],
      [ 0, 0.9502,  0.04721, 0],
      [ 0, 0,       0.89871, 0],
      [-1, 0,       0,       1]]
# Discretized state-space matrix A
Bd =[[0.00011],
     [0.01321],
     [0.51677],
     [0]]
# System input constraints
u_lb = [[0]]         # Lower constraint
u_ub = [[100]]       # Upper constraint
# States weighting matrix Q
Q = [[1, 0,        0,   0],
     [0, 1,        0,   0],
     [0, 0, 10000000,   0],
     [0, 0,        0, 100]]
# Input weighting matrix R
R = [[10000000]]
# Weighting matrix P
P = [[269008349.71, 115828228.55, 10867210.5, -1643757.68],
     [115828228.55,  50413960.28,  4735137.81, -645277.14],
     [ 10867210.5,    4735137.81, 21245328.08,  -59947.47],
     [ -1643757.68,   -645277.14,   -59947.47,   18181.58]]