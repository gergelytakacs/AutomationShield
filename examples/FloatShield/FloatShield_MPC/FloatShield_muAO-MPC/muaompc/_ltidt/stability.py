r"""
.. default-role:: math

The ``auto`` keyword indicates that the terminal state weighting matrix `P`
is to be selected automatically, such that the terminal cost is a control
Lyapunov function for the model predictive control setup. We consider the
following special cases:

* the open-loop system is stable and there are only constraints on
  the inputs. The solution of a matrix Lyapunov equation is returned. In this
  case the regulation point of the closed-loop system is globally exponentially
  stable.

* the pair (Ad, Bd) is stabilizable and there are constraints
  on the inputs and states. The solution of a discrete algebraic Riccati
  equation is returned. The regulation point is asymptotically (exponentially)
  stable with region of attraction `X_N` (`X_S`).

The following conditions are expected on the MPC setup:

* `Q` is symmetric positive definite.

* `R` is symmetric positive definite.

* The pair `(A_d, B_d)` is stabilizable.

* If `Q` is positive semi-definite, the pair `(A_d, Q^{\frac{1}{2}})` must be
  detectable. This is not checked at the moment.

The following conditions are checked:

1. Does the problem have only input constraints?

2. Does `A_d` have all its eigenvalues strictly inside the unit circle?

If conditions 1. and 2. are both true, a discrete Lyapunov equation is solved.
If any of the conditions 1. and 2. is false, then
a discrete algebraic Ricatti equation is solved.

In this context, `X_N` is defined as the set of states for which the MPC problem
is feasible. The set `X_S` is any sublevel set of the MPC optimal value [RM09]_.

"""

import numpy as np
from numpy import linalg

from muaompc._ltidt import csf

class AutoKeyError(Exception): pass

def calc_terminal_cost_matrix(A, B, Q, R, is_stc):
    """
    
    A the system matrix (n x n)
    B the input matrix (n x m)
    Q the state weighting matrix (symmetric positive semi-definite) (n x n)
    R the input weighting matrix (symmetric positive definite) (m x m)
    is_stc is the problem state constrained (bool)
    
    return
    mtx (n x n) symmetric positive semi-definite matrix, the solution to 
    a Lyapunov or Riccati equation.
    
    msg a sentence (string) verbosely indicating which operation was done
    type_ a word (string) indicating which operation was done
    
    """
    try:
        from slycot import (sb02od as slycot_solve_dare,  # Riccati 
                            sb03md as slycot_solve_dmle)  # Lyapunov
    except(ImportError,):
        print('WARNING: No solver found for the Riccati/Lyapunov Eq.',
              'Install Slycot or manually set P in your system module.')
        print('WARNING: The terminal penalty weighting matrix P', 
              'has been set equal to the states weighting matrix Q.')
        mtx = Q
        msg = ''
        type_ = 'other'
    else:
        if _conditions_ok(A, B, Q, R):
            n, m = B.shape
            if (_is_strictly_stable(A) and (not is_stc)):
                Udummy = np.zeros((n, n))
                # Solve for P: A.T * P * A - P = c * Q
                ans = slycot_solve_dmle(n, -Q, A, Udummy, 'D')
                mtx_o_fact = ans[0]  # scaled solution matrix
                fact = ans[1]  # the scaling factor 
                mtx = fact * mtx_o_fact
                msg = ('The terminal penalty weighting matrix has been set equal' +
                       ' to the solution of a discrete-time Lyapunov equation.')
                type_ = 'lyapunov'
            else: 
                ans = slycot_solve_dare(n, m, A, B, Q, R, 'D')
                mtx = ans[0]
                msg = ('The terminal penalty weighting matrix has been set equal' +
                       ' to the solution of a discrete algebraic Riccati equation.')
                type_ = 'ricatti'
    return mtx, msg, type_

def _conditions_ok(A, B, Q, R):
    #et_trace()
    is_ok = False
    if (_is_sympd(R) and _is_sympsd(Q) and _is_stabilizable(A, B)):
        is_ok = True
    else:
        if not _is_sympd(R):
            msg = ('Cannot automatically compute P. The matrix R is not' 
                    ' symmetric positive definite.')
            raise AutoKeyError(msg)
        if not _is_sympsd(Q):
            msg = ('Cannot automatically compute P. The matrix Q is not' 
                    ' symmetric positive semidefinite.')
            raise AutoKeyError(msg)
        if not _is_stabilizable(A, B):
            msg = ('Cannot automatically compute P. The pair (A, B) is not' 
                    ' stabilizable.') 
            raise AutoKeyError(msg)

    return is_ok

def _is_symmetric(M):
    return np.allclose(M, M.T)

def _is_sympd(M):
    'Check that the matrix M is symmetric positive definite'
    try:
        linalg.cholesky(M)  # check that is a symmetric pd
        is_sympd = True
    except linalg.LinAlgError as err:
        is_sympd = False
    return is_sympd

def _is_sympsd(M):
    'Check that the matrix M is symmetric positive semi-definite'
    is_sympsd = False
    if _is_symmetric(M):
        minevM = min(linalg.eigvals(M))
        if (minevM >= 0.):
            is_sympsd = True
    return is_sympsd

def _is_strictly_stable(Ad):
    is_sstb = False
    eigAd = np.linalg.eigvals(Ad)
    abs_eigAd = np.absolute(eigAd)
    abs_eigAd_is_lt1 = (abs_eigAd < 1.)
    if abs_eigAd_is_lt1.all():  # seems strictly stable
        is_sstb = True
        for abs_eig in abs_eigAd:  # check for numerical approximation error
            delta = 1. - abs_eig  # always positive
            if delta < 1e-10:
                is_sstb = False  # marginally stable
                break

    return is_sstb

def _is_stabilizable(A, B):
    C = np.matrix(np.eye(A.shape[0]))
    A, B, C, T = csf.ucsf(A, B, C, impl='qr')
    nc = csf.controllable_states(A, B)
    n = A.shape[0]
    if n == nc:  # no uncontrollable states
        return True
    else:
        Aunctb = A[nc:,nc:]
        return _is_strictly_stable(Aunctb)

