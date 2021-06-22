import numpy as np
from numpy.linalg import inv
from numpy.linalg import multi_dot
from numpy import dot
from scipy import signal
from scipy import linalg
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import math

# Parameters def
Ts = 0.004  # Sampling period

R = np.array([[14.0, 13.0, 15.0, 14.5, 13.5, 13.0]])           # [mm] Reference levels for the simulation
T = 1000                                                       # [samples] Section length for each reference level

Tend = T * R.shape[1] * Ts  # End time of simulation

X0 = np.array([[1.0E-3], [0.0], [0.0]])  # initial conditions

# linearization point
y0 = 14.3     # [mm] Linearization point based on the experimental identification
i0 = 0.0219   # [A] Linearization point based on the experimental identification
u0 = 4.6234   # [V] Linearization point based on the experimental identification


ra = (R-y0)/1000                  # [mm] Adjusted reference levels for the linearization point
# t = np.arange(0, (T*R.shape[1]-1)*Ts, Ts)    # [s] Time vector for the simulation
umin = 0.0                          # [V] Lower input constraint
umax = 12.0                        # [V] Upper input constraint
r = ra[0, 0]                      # [mm] First reference to start simulation

x1_min = 12.0e-3 - y0*1.0e-3
x1_max = 17.0e-3 - y0*1.0e-3

x3_min = -i0
x3_max = 60.0e-3 - i0

print('ra')
print(ra)

# State-space model definition
A = np.array([[0, 1, 0], [2132.16759667712, 0, -243.618066600915], [0, -16.8436441136086, -618.268173743509]])
B = np.array([[0], [0], [2.83867770431028]])
C = np.array([[1, 0, 0], [0, 0, 1]])
D = np.array([[0], [0]])


Ad, Bd, Cd, Dd, _ = signal.cont2discrete((A, B, C, D), Ts, method='zoh')   # Model discretization

Cd = np.array([[1, 0, 0]])

nx = Ad.shape[0]
nu = Bd.shape[1]
ny = Cd.shape[0]

# model augmentation by integration

Ai1 = np.column_stack((np.eye(ny), -Cd))
Ai2 = np.column_stack((np.zeros((nx, ny)), Ad))   # Augmenting A by an integrator
Ai = np.vstack((Ai1, Ai2))

print('Ai')
print(Ai)

Bi = np.vstack((np.zeros((ny, nu)), Bd))   # Augmenting B by the integrator
Ci = np.column_stack((np.zeros((ny, nx)), Cd))    # Augmenting C by the integrator


# LQ Controller

Qlq = np.diagflat([50, 100, 100, 10])
Rlq = np.array([0.1])

print('Qlq')
print(Qlq)
print('Rlq')
print(Rlq)


Xlq = linalg.solve_discrete_are(Ai, Bi, Qlq, Rlq)   # solve the Riccatti equation

Klq = dot(inv(multi_dot([Bi.T, Xlq, Bi])+Rlq), multi_dot([Bi.T, Xlq, Ai]))  # compute the LQR gain

print('Klq')
print(Klq)

# Simulation

n = math.ceil(Tend/Ts)      # number of samples in simulation

X = np.zeros(shape=(Ad.shape[0], n))   # Create arrays for states and outputs
Y = np.zeros(shape=(2, n))
U = np.zeros(shape=(Bd.shape[1], n))
xI = np.zeros(shape=(1, n))
Rr = np.zeros(shape=(1,n))

X[:, [0]] = X0
Y[:, [0]] = np.array([[17], [0]])
# Y[:, [0]] = Cd.dot(X[:, [0]])

t = np.arange(0, Tend, Ts)  # Create time vector

# xI = 0                                                   # Integrator initialization
# x1p = 0                                                  # Previous state

# r = (13.0-y0) * 1e-3

i = 1

for k in range(1, n):

    Rr[0, [k]] = R[0, [i-1]]                                                 # Original reference, just for logging
    if (k % (T*i)) == 0:                                    # At each section change
        i = i + 1                                             # We move to the next index
        r = ra[0, [i-1]]                                           # and use the actual, corrected reference

    xI[:, [k]] = xI[:, [k-1]] + (r - X[0, [k-1]])
    x = np.vstack((xI[:, [k-1]], X[:, [k-1]]))
    U[:, [k]] = (dot(-Klq, x)) + u0
    U[:, [k]] = np.clip(U[:, [k]], umin, umax)

    X[:, [k]] = Ad.dot(X[:, [k-1]]) + Bd.dot(U[:, [k]] - u0)
    X[0, [k]] = np.clip(X[0, [k]], x1_min, x1_max)
    X[2, [k]] = np.clip(X[2, [k]], x3_min, x3_max)

    # Y[:, [k]] = Cd.dot(X[:, [k]])
    Y[0, [k]] = X[0, [k-1]] * 1e3 + y0
    Y[1, [k]] = (X[2, [k-1]] + i0) * 1e3


# Plot results

fig = plt.figure()

plt.subplot(3, 1, 1)
plt.plot(t, np.squeeze(Y[0, :]), 'b-', linewidth=1, label='Position')
plt.plot(t, Rr[0, :], 'k-', linewidth=0.5, label='Reference')
plt.xlim([0, max(t)])
plt.ylim([12, 17])
# plt.xlabel('Time')
plt.ylabel('Position (mm)')
plt.legend(loc='best')

plt.subplot(3, 1, 2)
plt.plot(t, np.squeeze(Y[1, :]), 'r-', linewidth=1, label='Current')
plt.xlim([0, max(t)])
# plt.xlabel('Time')
plt.ylabel('Current (mA)')
plt.legend(loc='best')

plt.subplot(3, 1, 3)
plt.plot(t, np.squeeze(U), 'g-', linewidth=1, label='Voltage')
# plt.plot(t3,y3,'r-',linewidth=1,label='ODE Integrator')
plt.xlim([0, max(t)])
plt.xlabel('Time')
plt.ylabel('Input (V)')
plt.legend(loc='best')

plt.draw()
plt.savefig('Magneto.pdf', format='pdf', transparent=True)

plt.show()
