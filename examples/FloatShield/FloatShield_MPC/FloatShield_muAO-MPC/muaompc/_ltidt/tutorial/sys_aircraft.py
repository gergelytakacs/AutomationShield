r"""
.. default-role:: math

The system description
----------------------

The system considered is the Cessna Citation 500
aircraft presented in ([M02]_, p.64).  A continuous-time
linear model is given by `\dot{x} = A_c x + B_c u, y = C x`, where

.. math::
   A_c = \left[ \begin{matrix}
   -1.2822 & 0 & 0.98 & 0 \\
   0 & 0 & 1 & 0 \\
   -5.4293 & 0 & -1.8366 & 0 \\
   -128.2 & 128.2 & 0 & 0 \\
   \end{matrix} \right], \;\;
   B_c = \left[ \begin{matrix}
   -0.3 \\
   0 \\
   -17 \\
   0 \\
   \end{matrix} \right],
   C = \left[ \begin{matrix}
   0 & 1 & 0 & 0 \\
   0 & 0 & 0 & 1 \\
   -128.2 & 128.2 & 0 & 0 \\
   \end{matrix} \right],

and the state vector is given by `x = [x_1 \; x_2 \; x_3 \; x_4]^T`, where:

* `x_1` is the angle of attack (rad),
* `x_2` is the pitch angle (rad),
* `x_3` is the pitch angle rate (rad/s), and
* `x_4` is the altitude (m).

The only input `u_1` is the elevator angle (rad).
The outputs are `y_1 = x_2`,  `y_2 = x_4`, and `y_3 = -128.2 x_1 + 128.2 x_2`
is the altitude rate (m/s)

The system is subject to the following constraints:

* input constraints `-0.262 \leq u_1 \leq 0.262`,
* slew rate constraint in the input `-0.524 \leq \dot{u}_1 \leq 0.524`
* state constraints `-0.349 \leq x_2 \leq 0.349`,
* output constraints `-30.0 \leq y3 \leq 30.0`.

To consider the slew rate constraint in the input, we introduce an additional
state `x_5`. The sampling interval is `dt = 0.5` s, and the
horizon length is `N = 10` steps.

The controller parameters
-------------------------

The book [M02]_ proposes to use identity matrices of appropriate size for
the weighting matrices `Q` and `R`. We instead select them diagonal
with values that give a similar controller performance and much lower
condition number of the Hessian of the MPC quadratic program (see
:ref:`tuning.cond`),
a desirable property for any numerical algorithm.

"""

from numpy import diag  # similar effect with: from numpy import *

dt = 0.5
N = 10
mu = 100
# discrete-time system
Ad = [[  0.23996015,   0., 0.17871287,   0., 0.],
      [ -0.37221757,   1., 0.27026411,   0., 0.],
      [ -0.99008755,   0., 0.13885973,   0., 0.],
      [-48.93540655, 64.1, 2.39923411,   1., 0.],
      [0., 0., 0., 0., 0.]]
Bd = [[-1.2346445 ],
      [-1.43828223],
      [-4.48282454],
      [-1.79989043],
      [1.]]
# Weighting matrices for a problem with a better condition number
Q = diag([1014.7, 3.2407, 5674.8, 0.3695, 471.75])
R = diag([471.65])
P = Q
# input constraints
eui = 0.262  # rad (15 degrees). Elevator angle.
u_lb = [[-eui]] 
u_ub =  [[eui]]
# mixed constraints 
ex2 = 0.349  # rad/s (20 degrees). Pitch angle constraint.
ex5 = 0.524 * dt  # rad/s * dt input slew rate constraint in discrete time
ey3 = 30.
# bounds
e_lb = [[-ex2], [-ey3], [-ex5]]
e_ub = [[ex2], [ey3], [ex5]]
# constraint matrices
Kx = [[0, 1, 0, 0, 0],
      [-128.2, 128.2, 0, 0, 0],
      [0., 0., 0., 0., -1.]]
Ku = [[0],
      [0],
      [1]]
# terminal state constraints
f_lb = e_lb
f_ub = e_ub
F = Kx
