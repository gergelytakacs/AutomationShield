"""Compute the data required by the ALM+FGM algorithm.

This module provide classes that compute the data required to
solve convex quadratic programs of model predictive controllers
using an algorithm that combines an augmented Lagrangian method (ALM)
and Nesterov's fast gradient method (FGM). 
It is basically subdivided into two main categories: 
input constrained case, and mix constrained case. 
These classes are intended for internal use by the automatic code generation.
"""

import numpy as np
from numpy import sqrt, zeros, ones, eye, matrix as mtx

ROWS, COLS = (0, 1)

class _FgmQPBase(object):

    """Base clase for different types of QP, intended to be subclassed.

    Compute the data to solve the quadratic programming problem:
    minimize (1/2) * u.T * H * u + u.T * G * x
    subject to u_lb < u < u_ub (box constraints)
    using Nesterov's gradient method (a.k.a. fast gradient method, FGM).
    """

    def __init__(self, H, G, u_lb, u_ub):
        """Store the inputs variables to self."""
        self.u_lb = mtx(u_lb)
        self.u_ub = mtx(u_ub)


class FgmInputConstrQP(_FgmQPBase):

    """Compute data to solve a input constrained MPC quadratic program.
    
    This subclass extends the _FgmQPBase.__init__ method by computing the
    appropriate Lipschitz constant and convexity parameter.
    """

    def __init__(self, H, G, u_lb, u_ub):
        """Setup the method self variables from the given inputs variables."""
        _FgmQPBase.__init__(self, H, G, u_lb, u_ub)
        self.nu, L = self._compute_extra_step_constant(H)
        self.Linv = 1. / L
        self.HoL = H * self.Linv
        self.GoL = G * self.Linv
    
    def _compute_extra_step_constant(self, H):
        maxl = max(np.linalg.eigvalsh(H))
        minl = min(np.linalg.eigvalsh(H))
        nu = (sqrt(maxl) - sqrt(minl)) / (sqrt(maxl) + sqrt(minl))
        return nu, maxl


class FgmStateConstrQP(FgmInputConstrQP):
 
    """Compute data to solve a mix constrained MPC quadratic program.
    
    This subclass extends the FgmInputConstrQP.__init__ method by computing the
    appropriate constants related to the augmented Lagrangian method.
    """
     
    def __init__(self, H, G, 
                 state_factor, u_lb, u_ub, penalty_param):
        """Setup the method self variables from the given inputs variables."""
        _FgmQPBase.__init__(self, H, G, u_lb, u_ub)       
        self.E = mtx(state_factor)
        self.mu = penalty_param

        self.nu, L = self._compute_extra_step_constant(H, self.mu, self.E)
        self.Linv = 1. / L
        self.HoL = H * self.Linv
        self.GoL = G * self.Linv
        
    def _compute_extra_step_constant(self, H, mu, E):
        """Overloaded method.
        E: 2-side state constraint prediction matrix.
        Return a tuple with the FGM extra step and Lipschitz constants.
        """
        L = max(np.linalg.eigvalsh(H + mu * (E.T * E)))
        phi = min(np.linalg.eigvalsh(H))
        nu = ((sqrt(L) - sqrt(phi)) / (sqrt(L) + sqrt(phi)))
        return (nu, L)


class FgmDualFunction(FgmInputConstrQP):
    
    """Compute the Lagrange dual function of a state constrained MPC problem.
    """
    def __init__(self, H, G, E, u_lb, u_ub):
        FgmInputConstrQP.__init__(self, H, G, u_lb, u_ub)
        self.EToL = E.T * self.Linv
        
    
    def minimize_qp(self, x, l, u_0, j_in):
        """Solve the quadratic program for the current system state.
         
        Inputs:
        x -- the current system state.
        u_0 -- the initial guess of the solution.
        j_in -- maximum number of algorithm iterations.
        """
        w = u_old = u_0; 
        GoL_x = self.GoL * x
        # the Lagrange multiplier l is given in a concise way, 
        # some values of l are "negative" due to the mixed box constraints
        # it should normally be all entries positive and twice as large
        EToL_l = self.EToL * (np.maximum(l, 0) + np.minimum(l, 0))
        goL = GoL_x + EToL_l 
        for i in range(j_in):
            gradoL = self._compute_grad_over_L(goL, w)
            u, u_old, w = self._minimize_qp_iteration(u_old, w, gradoL)
        self.u_opt = u
        return u    
    
    def _compute_grad_over_L(self, goL, w):
        return self.HoL * w + goL
