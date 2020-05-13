r"""
The system description
----------------------

.. default-role:: math

We consider as system a DC-motor, which can be modeled
as:

.. math::
   \ddot{y} = \frac{1}{T} (Ku - \dot{y})

where `T` is the time constant in seconds, and `K` is amplification factor. The
continuous-time state-space representation `\dot{x} = A_c x + B_c u` is given
by the following matrices:

.. math::
   A_c = \left[ \begin{matrix}
   0 & 1 \\
   0 & -\frac{1}{T} \\
   \end{matrix} \right], \;\;
   B_c = \left[ \begin{matrix}
   0 \\ \frac{K}{T} \\
   \end{matrix} \right]

with the state vector `x = [x_1 \; x_2]^T`, where `x_1` and `x_2` are the rotor
position and angular speed, respectively. The input is the PWM voltage, which is
the percentage of the full-scale voltage applied to the motor. It is therefore
constrained to be between -100% and 100%. This constraint can be written as
`-100 \leq u \leq 100`.

As the continuous-time system should be controlled by the MPC
digital controller, a suitable discretization time must be chosen.
A rule of thumb is to choose the
discretization time as one-tenth of the system's time constant. In this case,
`dt = \frac{T}{10}`.

*Note: to discretize a system, SciPy needs to be installed.*

The controller parameters
-------------------------

The horizon length is specified as steps through the parameter `N`.
In this case we choose the value `N=10`.
The additional parameters for the controller are
the weighting matrices. They are usually chosen via a tuning procedure.
For this example, we set them to be
identity matrices of appropriate size, i.e. `Q = I \in \mathbb{R}^{2
\times 2}`, and `R = 1`. Additionally we set `P` as ``"auto"``, which will
compute it as a stabilizing matrix.

*Note: to use the "auto" feature, Slycot needs to be installed.*

"""

T = 0.1
K = 0.2
dt = T / 10
N = 10
# continuous time system
Ac = [[0, 1], [0, -1/T]]
Bc = [[0], [K/T]]
# input constraints
u_lb = [[-100]]
u_ub = [[100]]
# weighting matrices
Q = [[1, 0], [0, 1]]
R = [[1]]
P = "auto"

