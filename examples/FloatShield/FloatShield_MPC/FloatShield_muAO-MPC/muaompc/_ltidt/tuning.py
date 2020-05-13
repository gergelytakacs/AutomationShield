"""Help find appropriate parameters for the ALM+FGM MPC algorithm."""

from os import system

import numpy as np
# commented, buggy scipy does not allow to import
from scipy.optimize import minimize

from muaompc import ltidt 
from muaompc._ltidt import simula

ROWS, COLS = (0, 1)

class MPCTuning(object):
    
    def __init__(self, mpc):
        self._mpc = mpc
        self._nlp = _NLPSolver(self._mpc)
        if type(mpc) is ltidt._MPCInputStateConstr:
            self.find_penalty_parameter = mpc._qp._find_penalty_parameter
            self.mu = mpc._qp.mu
            
    def calc_int_cn(self, penalties=None, verbose=False):
        if penalties is None:
            penalties = [self.mu]

        samples = len(penalties)
        mu_vec = np.zeros((samples))
        kappa_vec = np.zeros((samples))
        
        for k, mu in enumerate(penalties):
            kappa_vec[k] = self._mpc._qp._compute_alg_condnum(mu) 
            mu_vec[k] = mu
        if verbose: 
            for k, mu in enumerate(penalties):
                print('For penalty:', mu_vec[k],
                      'the condition number of internal problem is:', 
                      kappa_vec[k])
            print('Strong convexity parameter:', 
                  min(np.linalg.eigvals(self._mpc._qp.H)))
        return (mu_vec, kappa_vec)

    def reduce_H_cn(self, xref, uref, stw, inw, cn_ub=100,
                          stw0=None, inw0=None):
        """Find weighting matrices that decrease the Hessian's condition number.

        :param xref: the trajectory the states should follow.
           ``xref`` has shape ``(states, points)``.
        :type xref: numpy array

        :param uref: the sequence of inputs for the given state trajectory 
           ``xref``. ``uref`` has shape ``(inputs, points)``.
        :type uref: numpy array

        :param stw: the relative weights for the states given as a vector
           of shape (states,). This vector correspond to the diagonal
           of the state weighting matrix ``Q``, i.e. ``Q = diag(stw)``.
        :type stw: numpy array

        :param inw: the relative weights for the inputs given as a vector
           of shape (inputs,). This vector correspond to the diagonal
           of the input weighting matrix ``R``, i.e. ``R = diag(inw)``.
        :type inw: numpy array

        :param scalar cn_ub: upper bound for the condition number of the Hessian.
           It must be greater than 1.

        :param stw0: initial guess of the solution for the state weights.
           If ``None``, it will be computed as an identity matrix of appropriate shape.
        :type stw0: None or numpy array

        :param inw0: initial guess of the solution for the inputs weights of shape inputs
           If ``None``, it will be computed as an identity matrix of appropriate shape.
        :type inw0: None or numpy array

        :return: a dictionary with the diagonals of the newly tuned
           weights. The keys are as follows:
           
             - "stw" is the diagonal of the weighting matrix for the states.
             - "inw" is the diagonal of the weighting matrix for the inputs.

        Given a reference state and input trajectory ``(xref, uref)``,
        the difference of a simulated trajectory for the unconstrained system
        (with initial condition the first point of the given trajectory
        ``(xref, uref)``)
        should be minimized with respect to the diagonal weigthing matrices 
        of the QP. This problem is posed as a nonlinear program (NLP).

        This NLP takes a reference trajectory as input (for the states and the
        input sequences), the reference weighting matrices that set the desired
        controller performance, and an upper bound on the condition number of the
        QP's Hessian. The NLP returns weighting matrices
        that give a similar controller performance and make :math:`\kappa_H`
        (the condition number of the Hessian) lower than
        the specified upper bound. As this NLP is in general nonconvex,
        there might be several local solutions,
        and finding a (good) solution or not might depend on the initial
        guesses for the weighting matrices.
        For details see [WAD11]_.

        The procedure to find suitable weighting matrices that reduce
        :math:`\kappa_H` is as follows:

            #. Manually tune your MPC controller, as you would normally do.
               At the moment, only diagonal matrices are accepted. These
               matrices will be used as base for the NLP.
            #. Generate a trajectory for the states and the corresponding
               input sequences that is representative of the MPC application.
            #. Optionally, select an upper bound for :math:`\kappa_H`.
            #. Optionally, select an initial guess for the weighting matrices.
            #. Repeat for different parameters if the optimization was not
               successful, or if you are not satisfied with the results.

        Take for example the system in :ref:`tutor.advanced`. We consider 
        a change in the altitude to represent the typical behaviour of the
        control system.
        A very brief example follows::

           from numpy import eye, diag
           import muaompc
           mpc = muaompc.ltidt.setup_mpc_problem('sys_aircraft')
           # xref, uref have been already generated via simulation
           # the reference weighting matrices have been manually tuned as identity
           Qref = diag(eye(5))
           Rref = diag(eye(1))
           r = mpc.tun.reduce_H_cn(xref, uref, Qref, Rref)
           Qtun = r['stw']
           Rtun = r['intw']

        """

        xref = np.matrix(xref)
        uref = np.matrix(uref)

        rstw, rinw = self._nlp.solve(xref, uref, stw, inw, cn_ub,
                          stw0, inw0)

        return dict(stw=rstw, inw=rinw)


class _NLPSolver(object):
    def __init__(self, mpc):
        self.mpc = ltidt._MPCUnConstr(mpc.sys, mpc.wmx, mpc.size, mpc.lqr)
        self.Ah, self.Bh = self.mpc._qp._setup_extended_system_matrices(
                                    self.mpc.sys.Ad, self.mpc.sys.Bd, 
                                    self.mpc.size)

        self.sim = simula.MPCSim(self.mpc)

    def solve(self, xref, uref, stw_nlp, inw_nlp, cn_ub,
                          stw_mpc, inw_mpc):
        x0 = xref[:,0]
        stw_nlp = np.array(stw_nlp).reshape((len(stw_nlp),))
        inw_nlp = np.array(inw_nlp).reshape((len(inw_nlp),))
        wnlp = np.append(stw_nlp, inw_nlp)

        if stw_mpc is None:
            stw_mpc = np.ones(xref.shape[ROWS])
        if inw_mpc is None:
            inw_mpc = np.ones(uref.shape[ROWS])
        wmpc0 = np.append(stw_mpc, inw_mpc)
        #self._define_minim_vars_constr(wvec)
        constr = self._define_constr(wmpc0, cn_ub) 
        res = minimize(self._calc_cost, 
                       wmpc0,
                       args=(x0, xref, uref, wnlp),
                       constraints=constr,
                       method='SLSQP',
                       #method='COBYLA',
                       #options=dict(maxiter=1000),
                       )
        if res.success:
            print('Optimization successful.')
        else:
            print('WARNING: optimization not successful.')
        
        rQ = res.x[:xref.shape[ROWS]]
        rR = res.x[xref.shape[ROWS]:]
        return rQ, rR
     
    def _calc_weight_matrices(self, wmpc):
        stw, inw = self._split_state_input_weight_vector(wmpc, self.mpc.size)
        Q = np.diag(stw)  # only works correctly if 1-dim array
        R = np.diag(inw)  # only works correctly if 1-dim array
        P = Q  # FIXME: End penalty matrix should be recomputed as well? 
        return self.mpc._qp._setup_extended_weight_matrices(Q, R, P, 
                                                            self.mpc.size)

    def _split_state_input_weight_vector(self, wvec, size):
        # weight vector has the state weights at the beginning (first n elem.), 
        # the input weights are the last elements. 
        # WARNING: wvec MUST be an 1-dim array with wvec.shape = (n + p,)
        n = size.states
        stw = wvec[:n]
        inw = wvec[n:]
        return stw, inw
    
    def _update_mpc_qp_matrices(self, wmpc):
        self.mpc._qp.H, self.mpc._qp.F = self._calc_unconstr_qp_matrices(wmpc) 

    def _calc_unconstr_qp_matrices(self, wmpc):
        Qh, Rh =  self._calc_weight_matrices(wmpc)
        H, F = self.mpc._qp._setup_hess_grad_matrices(self.Ah, self.Bh, Qh, Rh)
        return H, F

    def _calc_hessian_condnum_constr(self, wmpc, cn_ub):
        H, F = self._calc_unconstr_qp_matrices(wmpc)
        return np.array([cn_ub - np.linalg.cond(H)])
    
    def _sim_unconstr_closed_loop(self, x_ini, steps):
        self.sim._run_unconstr_sim(x_ini, steps)
        return self.sim.data.x, self.sim.data.u0
           
    def _calc_cost(self, wmpc, x0, xref, uref, wnlp):
        self._update_mpc_qp_matrices(wmpc)
        steps = xref.shape[COLS]
        x, u = self._sim_unconstr_closed_loop(np.array(x0), steps)
        x_diff = np.square(x - xref)
        u_diff = np.square(u - uref)
        stw, inw = self._split_state_input_weight_vector(wnlp, self.mpc.size) 
        st_cost = np.sum(stw * x_diff)
        in_cost = np.sum(inw * u_diff) 
        return (st_cost + in_cost)

    def _define_constr(self, wmpc, cn_ub):
        constrs = []
        for k in range(len(wmpc)):
            j = np.zeros(len(wmpc))
            j[k] = 1
            constr_k = dict(type = 'ineq',
                            fun = lambda x, k=k: np.array([x[k]]),
                            jac = lambda x, j=j: j 
                            )
            constrs.append(constr_k)
            
        constr_condnum = dict(type = 'ineq',
                              fun = lambda x, ub=cn_ub: (
                                    self._calc_hessian_condnum_constr(x, ub)))
        constrs.append(constr_condnum)
        return tuple(constrs)
