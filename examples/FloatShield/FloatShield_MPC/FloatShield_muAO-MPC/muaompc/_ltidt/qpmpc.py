"""Create parametric quadratic programs of an MPC setup.

The generated quadratic program has the form

.. math::
   \\underset{u}{\\text{minimize}} & \\;\\;  u^T H u + u^T (G x) \\\\
   \\text{subject to} & \\;\\; u\_lb < u < u\_ub  \\\\
   &  \\;\\; z\_lb(x) < E u < z\_ub(x) \\\\

"""

import numpy as np
from numpy import zeros, ones, matrix as mtx

from muaompc._ltidt.fgmqp import FgmInputConstrQP as fgmic
from muaompc._ltidt.fgmqp import FgmStateConstrQP as fgmsc

class QPError(Exception): pass

ROWS, COLS = (0, 1)

class _QPBase(object):

    """Base clase for different QP objects, intended to be subclassed."""

    def __init__(self, sys, wmx, size):
        """Store the inputs variables to self and create the QP matrices."""
        self.sys = sys
        self.wmx = wmx
        self.size = size

        self.H, self.G = self._setup_unconstrained_qp(sys, wmx, size)
        try:
            np.linalg.cholesky(self.H)
        except np.linalg.LinAlgError:
            msg = ('QP not strictly convex. '
                    'Hessian is not symmetric positive definite.')
            raise QPError(msg)
    
    def compute_value_function(self, x, uk):
        # FIXME: aren't other functions called calc_ instead of compute_?
        hess_cost = 0.5 * uk.T * self.H * uk
        grad_cost = uk.T * self.G * x
        return hess_cost + grad_cost
    
    def _concatenate_bound_N_times(self, constr, N, bound_name, 
                                   end_bound_name=None):
        bound_N = bound = constr.__dict__[bound_name]
        for j in range(N - 1):
            if (j == N - 2) and (end_bound_name is not None):
                bound = constr.__dict__[end_bound_name]
            bound_N = np.concatenate((bound_N, bound))
        return bound_N
    
    def _setup_extended_matrices(self, A, B, Q, R, P, size):
        Ah, Bh = self._setup_extended_system_matrices(A, B, size)
        Qh, Rh = self._setup_extended_weight_matrices(Q, R, P, size)
        return (Ah, Bh, Qh, Rh)

    def _setup_extended_weight_matrices(self, Q, R, P, size):
        N, m, n = size.hor, size.inputs, size.states  
        Qh = mtx(zeros((n * N, n * N)))
        Rh = mtx(zeros((m * N, m * N)))
        # alternatively, use scipy.linalg.block_diag
        for k in range(N):
            ini = k * m
            end = (k + 1) * m
            Rh[ini : end, ini : end] = R
            
            ini = k * n
            end = (k + 1) * n
            Qh[ini : end, ini : end] = Q
            if k == N - 1: # terminal cost penalty
                Qh[ini : end, ini : end] = P
            
        return (Qh, Rh)
    
    def _setup_extended_system_matrices(self, A, B, size):
        N, m, n = size.hor, size.inputs, size.states
        Ah = mtx(zeros((n * N, n)))
        Bh = mtx(zeros((n * N, m * N)))
        # alternatively, use scipy.linalg.block_diag        
        for k in range(N):
            ini = k * n
            end = (k + 1) * n
            
            Ah[ini : end, :] = A ** (k + 1)
            
            for j in range(k + 1):
                col_ini = j * m 
                col_end = (j + 1) * m          
                Bh[ini : end, col_ini : col_end] = (A ** (k - j)) * B
                
        return (Ah, Bh)            
 
    def _setup_hess_grad_matrices(self, Ah, Bh, Qh, Rh):
        H = (Bh.T * Qh * Bh) + Rh
        G = Bh.T * Qh.T * Ah
        return H, G        
 
    def _setup_unconstrained_qp(self, sys, wmx, size):
        self.Ah, self.Bh, self.Qh, self.Rh = self._setup_extended_matrices(
                                                    sys.Ad, sys.Bd, 
                                                    wmx.Q, wmx.R, 
                                                    wmx.P, size)

        H, G = self._setup_hess_grad_matrices(self.Ah, self.Bh, 
                                              self.Qh, self.Rh)
        
        return (H, G)
    

class _QPUnConstr(_QPBase):

    """Solve an unconstrained parametric quadratic program. 

    This subclass extends the base class with the following:
    Public method:
    solve_problem -- solve the unconstrained QP for a given parameter.
    """

    def __init__(self, sys, wmx, size):
        _QPBase.__init__(self, sys, wmx, size)
        
    def solve_problem(self, x):
        """Solve the unconstrained QP for a given parameter. 
        inputs:
        x -- the parameter.
        returns:
        the vector that minimizes the QP for the given parameter.
        """
        x = mtx(x).T
        Hinv_G = np.linalg.inv(self.H) * self.G
        umin = -Hinv_G * x
        return umin


class _QPInputConstr(_QPBase):

    """Create an input constrained parametric QP.
    
    This subclass extends the _QPBase.__init__ method by computing the
    input constraint lower and upper bounds.
    """

    def __init__(self, sys, constr, wmx, size):
        def setup_input_constr_vector(constr, N, constrName):
            return self._concatenate_bound_N_times(constr, N, constrName)

        _QPBase.__init__(self, sys, wmx, size)
        
        self.constr = constr 
        self.u_lb = setup_input_constr_vector(constr, size.hor, 'u_lb')
        self.u_ub = setup_input_constr_vector(constr, size.hor, 'u_ub')
        
        self.fgm = self._setup_fast_gradient_solver(self.H, self.G, 
                                               self.u_lb, 
                                               self.u_ub)

    def _setup_fast_gradient_solver(self, H, G, eu_lN, eu_rN):
        return fgmic(H, G, eu_lN, eu_rN)
        
       
class _QPInputStateConstrBase(_QPInputConstr):

    """Create an state (mixed) constrained parametric QP.
    
    This subclass extends the _QPInputConstr.__init__ method by computing the
    parametric state (mixed) constraint lower and upper bounds. 
    This is a base class, intended to be subclassed.
    """

    def __init__(self, sys, constr, wmx, size, mu):
        
        def setup_state_constraint_vector(constr, size, 
                                          constrName, end_constrName):
            return self._concatenate_bound_N_times(constr, size, 
                                                   constrName, end_constrName)
        
        def setup_affine_state_constraint_constants(sys, constr, size):
            '''Partial elimination of constraints, only state constraints. 
            Compute the reduced augmented Lagrange multipliers (alm).'''
            A, B = sys.Ad, sys.Bd
            N, m, n = size.hor, size.inputs, size.states        
            Kx, Ku = constr.Kx, constr.Ku
            qxu = Kx.shape[ROWS] # Kx and Ku have the same shape
            F = constr.F
            qf = F.shape[ROWS]
            Ex = mtx(zeros(((N) * qxu + qf, N * m)))
            Kx_Ai = mtx(zeros(((N) * qxu + qf, n)))
    
            for j in range(N+1):
                if j == N: # terminal region
                    Kx = F
                    row_end = qxu*j + qf
                else:
                    row_end = qxu*(j+1)
                    
                row_ini = qxu*j
                Kx_Ai[row_ini:row_end] = Kx * (A**j)
                
                for k in range(j+1):
                    col_ini = m * k
                    col_end = m * (k + 1)

                    if k == j:
                        if k != N:  # last row does not have Ku
                            Ex[row_ini:row_end, col_ini:col_end] = Ku
                    else:
                        Ex[row_ini:row_end, col_ini:col_end] = (Kx * 
                                                    (A ** ((j - 1) - k)) * B)

            return (Ex, Kx_Ai)  
    

        _QPInputConstr.__init__(self, sys, constr, wmx, size)
        
        self.e_lb = setup_state_constraint_vector(constr, size.hor + 1, 
                                                    'e_lb', 'f_lb')
        self.e_ub = setup_state_constraint_vector(constr, size.hor + 1, 
                                                    'e_ub', 'f_ub')
        self.Ex, self.Kx_Ai = setup_affine_state_constraint_constants(
                                                    self.sys, self.constr, 
                                                    self.size)


class _QPInputStateConstr(_QPInputStateConstrBase):
    """Create a mixed constrained parametric QP.
    
    This subclass extends the _QPInputStateConstrBase.__init__ method by 
    computing setting up all the required data. 
    """

    def __init__(self, sys, constr, wmx, size, mu):
        
        def _setup_fast_gradient_solver(H, G, pred, u_lb, u_ub, mu):
            return fgmsc(H, G, pred, u_lb, u_ub, mu)
            
        _QPInputStateConstrBase.__init__(self, sys, constr, wmx, size, mu)
        self.E = self.Ex[:]
        
        if mu is None:
            fgm = _setup_fast_gradient_solver(self.H, self.G, self.E, 
                                           self.u_lb, self.u_ub, 1.)
            mu = self._find_penalty_parameter(fgm)

        self.mu = mu
        self.fgm = _setup_fast_gradient_solver(self.H, self.G, self.E, 
                                           self.u_lb, self.u_ub, mu)


    def _find_penalty_parameter(self, fgm=None):
        """Find a penalty parameter appropriate for the problem. 
        
        Returns a penalty parameter that makes the condition number of the 
        internal problem (FGM) not much larger than the Hessian's 
        condition number.       
        Do note that the returned penalty parameter is just a suggestion.
        It can be used as starting point for a finer tuning. 
        input:
        verbose (default False): print extra information. 
        return:
        mu: a penalty parameter that is not too big and not too small.
        If the problems has no state constraints, mu=None.
        
        """
        if fgm is None:
            fgm = self.fgm

        msg = '_find_penalty_parameter '
        
        LOWER_LIMIT, UPPER_LIMIT, MAX_ITER = (2, 10, 100)
        mu = 1.
        cnH = self._compute_alg_condnum(0, fgm)
        r = self._compute_alg_condnum(mu, fgm) / cnH 
        
        for k in range(MAX_ITER):
            if r > UPPER_LIMIT:
                mu /= 2
            elif r < LOWER_LIMIT:
                mu *= 2
            else:
                break
            r = self._compute_alg_condnum(mu, fgm) / cnH
            if k == MAX_ITER - 1:
                msg += 'WARNING: reached maximum number of iterations.'
        
        return mu

    def _compute_alg_condnum(self, mu, fgm=None):
        if fgm is None:
            fgm = self.fgm
        na, Lipschitz = fgm._compute_extra_step_constant(self.H, mu, self.E)
        return Lipschitz / (min(np.linalg.eigvals(self.H))) 

