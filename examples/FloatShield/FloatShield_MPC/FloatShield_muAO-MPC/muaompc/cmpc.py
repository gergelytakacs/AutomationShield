"""This is the Python interface to the C external module."""
import muaompc._cmpc
import numpy

class conf(object):
    def __init__(self):
        self._data = None

    def _set_ex_iter(self, ex_iter):
        self._data['conf']['ex_iter'] = ex_iter

    def _get_ex_iter(self):
        return self._data['conf']['ex_iter']

    def _set_in_iter(self, in_iter):
        self._data['conf']['in_iter'] = in_iter

    def _get_in_iter(self):
        return self._data['conf']['in_iter']

    def _set_warmstart(self, warmstart):
        self._data['conf']['warmstart'] = warmstart

    def _get_warmstart(self):
        return self._data['conf']['warmstart']

    ex_iter = property(_get_ex_iter, _set_ex_iter)
    in_iter = property(_get_in_iter, _set_in_iter)
    warmstart = property(_get_warmstart, _set_warmstart)


class wmx(object):
    def __init__(self):
        self._data = None

    def _set_Q(self, Q):
        self._data['wmx']['Q'] = Q

    def _get_Q(self):
        return self._data['wmx']['Q']

    def _set_R(self, R):
        self._data['wmx']['R'] = R

    def _get_R(self):
        return self._data['wmx']['R']

    def _set_P(self, P):
        self._data['wmx']['P'] = P

    def _get_P(self):
        return self._data['wmx']['P']

    Q = property(_get_Q, _set_Q)
    R = property(_get_R, _set_R)
    P = property(_get_P, _set_P)


class lqr(object):
    def __init__(self):
        self._data = None

    def _set_K(self, K):
        self._data['lqr']['K'] = K

    def _get_K(self):
        return self._data['lqr']['K']

    def _set_P(self, P):
        self._data['lqr']['P'] = P

    def _get_P(self):
        return self._data['lqr']['P']

    K = property(_get_K, _set_K)
    P = property(_get_P, _set_P)


class sys(object):
    def __init__(self):
        self._data = None

    def _set_Ad(self, Ad):
        self._data['sys']['Ad'] = Ad

    def _get_Ad(self):
        return self._data['sys']['Ad']

    def _set_Bd(self, Bd):
        self._data['sys']['Bd'] = Bd

    def _get_Bd(self):
        return self._data['sys']['Bd']

    def _set_dt(self, dt):
        self._data['sys']['dt'] = dt

    def _get_dt(self):
        return self._data['sys']['dt']

    Ad = property(_get_Ad, _set_Ad)
    Bd = property(_get_Bd, _set_Bd)
    dt = property(_get_dt, _set_dt)


class qpx(object):
    def __init__(self):
        self._data = None

    def _set_HoL(self, HoL):
        self._data['qpx']['HoL'] = HoL
        
    def _get_HoL(self):
        return self._data['qpx']['HoL']

    def _set_gxoL(self, gxoL):
        self._data['qpx']['gxoL'] = gxoL
        
    def _get_gxoL(self):
        return self._data['qpx']['gxoL']

    def _set_E(self, E):
        self._data['qpx']['E'] = E

    def _get_E(self):
        return self._data['qpx']['E']

    def _set_u_lb(self, u_lb):
        self._data['qpx']['u_lb'] = u_lb

    def _get_u_lb(self):
        return self._data['qpx']['u_lb']

    def _set_u_ub(self, u_ub):
        self._data['qpx']['u_ub'] = u_ub

    def _get_u_ub(self):
        return self._data['qpx']['u_ub']

    def _set_zx_lb(self, zx_lb):
        self._data['qpx']['zx_lb'] = zx_lb

    def _get_zx_lb(self):
        return self._data['qpx']['zx_lb']

    def _set_zx_ub(self, zx_ub):
        self._data['qpx']['zx_ub'] = zx_ub

    def _get_zx_ub(self):
        return self._data['qpx']['zx_ub']

    def _set_HOR_INPUTS(self, HOR_INPUTS):
        self._data['qpx']['HOR_INPUTS'] = HOR_INPUTS

    def _get_HOR_INPUTS(self):
        return self._data['qpx']['HOR_INPUTS']

    def _set_HOR_MXCONSTRS(self, HOR_MXCONSTRS):
        self._data['qpx']['HOR_MXCONSTRS'] = HOR_MXCONSTRS

    def _get_HOR_MXCONSTRS(self):
        return self._data['qpx']['HOR_MXCONSTRS']
        
    HoL = property(_get_HoL, _set_HoL)
    gxoL = property(_get_gxoL, _set_gxoL)
    E = property(_get_E, _set_E)
    u_lb = property(_get_u_lb, _set_u_lb)
    u_ub = property(_get_u_ub, _set_u_ub)
    zx_lb = property(_get_zx_lb, _set_zx_lb)
    zx_ub = property(_get_zx_ub, _set_zx_ub)
    HOR_INPUTS = property(_get_HOR_INPUTS, _set_HOR_INPUTS)
    HOR_MXCONSTRS = property(_get_HOR_MXCONSTRS, _set_HOR_MXCONSTRS)
    
    
class ctl(object):
    def __init__(self, mpc):
        defines, mtxs, scalars, enums = mpc._generate_c_files_data()

        self._data = muaompc._cmpc.vla_get_data({
            'alm' : {
                'fgm' : {
                    'HOR_STATES' : enums.hor_states,
                    'HOR' : enums.hor,
                    'INPUTS' : enums.inputs,
                    'GoL' : mtxs.GoL.astype(numpy.double),
                    'Bh_T' : mtxs.Bh_T.copy(),
                    'u_lb' : mtxs.u_lb.astype(numpy.double),
                    'u_ub' : mtxs.u_ub.astype(numpy.double),
                    'nu' : scalars.nu
                },
                'HOR_INPUTS' : enums.hor_inputs,
                'HOR_MXCONSTRS' : enums.hor_mxconstrs,
                'MXCONSTRS' : enums.mxconstrs,
                'STATES' : enums.states,
                'E' : mtxs.E.astype(numpy.double),
                'Kx_Ai' : mtxs.Kx_Ai.astype(numpy.double),
                'e_lb' : mtxs.e_lb.astype(numpy.double),
                'e_ub' : mtxs.e_ub.astype(numpy.double),
                'mu' : scalars.mu,
                'Linv' : scalars.linv
            },
            'sys' : {
                'Ad' : mtxs.Ad.astype(numpy.double),
                'Bd' : mtxs.Bd.astype(numpy.double),
                'dt' : scalars.dt
            },
            'wmx' : {
                'Q' : mtxs.Q.astype(numpy.double),
                'R' : mtxs.R.astype(numpy.double),
                'P' : mtxs.P.astype(numpy.double)
            },
            'lqr' : {
                'K' : mtxs.K.astype(numpy.double),
                'P' : mtxs.P.astype(numpy.double)
            },
            'conf' : {
                'in_iter' : 1,
                'ex_iter' : 1,
                'warmstart' : 0
            },
            'qpx' : {
                'HoL' : mtxs.HoL
            },
            'u_ini' : numpy.zeros(mtxs.u_lb.shape),
            'l_ini' : numpy.zeros(mtxs.e_lb.shape),
            'x_ref' : numpy.zeros([enums.hor_states, 1]),
            'u_ref' : numpy.zeros([enums.hor_inputs, 1]),
        })

        self._conf = conf()
        self._wmx = wmx()
        self._lqr = lqr()
        self._sys = sys()
        self._qpx = qpx()
        self._conf._data = self._data
        self._wmx._data = self._data
        self._lqr._data = self._data
        self._sys._data = self._data
        self._qpx._data = self._data

    def solve_problem(self, x):
        '''Solve the MPC problem for the given state using the default solver.
        
        :param x: the current state. It must be of size ``states``.
        :type x: numpy array
        This method is an interface to the C code. See its documentation
        for further details. 
        
        This method relies on other fields of the ``ctl`` structure:

           #. ``conf`` configuration structure. 
           #. ``x_ref`` state reference trajectory
           #. ``u_ref`` input reference trajectory
           #. ``u_ini`` initial guess for the optimization variable
           #. ``l_ini`` initial guess for the Lagrange multipliers
           #. ``u_opt`` approximate solution to the optimization problem
           #. ``l_opt`` approximate optimal Lagrange multiplier 
           #. ``x_trj`` state trajectory under the current u_opt
            
        ``conf`` contains the 
        basic configuration parameters of the optimization algorithm. It 
        consist of the fields:
         
           * ``warmstart``: if ``True``, use a warmstart strategy
             (default: ``False``)
           * ``in_iter``: number of internal iterations (default: 1)
           * ``ex_iter``: number of external iterations. If mixed constraints
             are not present, it should be set to 1. (default: 1)
              
        ``x_ref`` is an array of shape (``hor_states``, 1), whereas
        ``u_ref`` is an array of shape (``hor_inputs``, 1).
        By default, ``x_ref`` and ``u_ref`` are zero vectors 
        of appropriate size. In other words, the default case is MPC regulation
        to the origin.
        
        ``u_ini``  is an array of shape (``hor_inputs``, 1).
        ``l_ini``  is an array of shape (``hor_mxconstrs``, 1). 
        ``l_ini`` is only of interest for problems with mixed constraints.
        By default, these are also zero. These mainly need to be set by
        the user in the case of manually coldstarting the MPC algorithm 
        (``conf.warmstart = False``). If ``conf.warmstart = True``, they are
        automatically computed based on the previous solution. 
        
        ``u_opt``  is an array of shape (``hor_inputs``, 1).
        ``l_opt``  is an array of shape (``hor_mxconstrs``, 1). 
        ``l_opt`` is only of interest for problems with mixed constraints.
        ``x_trj``  is an array of shape (``hor_states+states``, 1).
        
        Usually in an MPC scheme, only the first input vector of
        the optimal input sequence ``u_opt`` is of interest, i.e. the first ``inputs`` 
        elements of ``u_opt``.
        
        For example, assuming ``mpc`` is an MPC object with only input 
        constraints, and we want all states to be regulated to 2, starting
        at states all 3::
        
           mpc.ctl.conf.in_iter = 5  # configuration done only once
           mpc.ctl.conf.warmstart = True  # use warmstart
           
           mpc.ctl.x_ref = numpy.ones(mpc.ctl.x_ref.shape) * 2
           
           # repeat the following lines for every new state x 
           x = numpy.ones(mpc.ctl.x_ref.shape) * 3
           mpc.ctl.solve_problem(x)
           u0 = mpc.ctl.u_opt[:mpc.size.inputs]  # 1st input vector in sequence

        '''

        self._data = muaompc._cmpc.vla_solve_problem(self._data, 
                                                       x.astype(numpy.double))
        self._conf._data = self._data        
        self._wmx._data = self._data        
        self._lqr._data = self._data        
        self._sys._data = self._data        
        self._qpx._data = self._data        
        
    def form_qp(self, x):
        '''Compute the parametric quadratic program data using x as parameter.
        
        :param x: the current state. It must be of size ``states``.
        :type x: numpy array
        
        This method is an interface to the C code. See its documentation
        for further details. 
        
        This method relies on other fields of the ``ctl`` structure:

           #. ``x_ref`` state reference trajectory
           #. ``u_ref`` input reference trajectory
           #. ``qpx`` the structure with the created quadratic program data
            
        ``x_ref`` is an array of shape (``hor_states``, 1), whereas
        ``u_ref`` is an array of shape (``hor_inputs``, 1).
        By default, ``x_ref`` and ``u_ref`` are zero vectors 
        of appropriate size. In other words, the default case is MPC regulation
        to the origin.
        
        ``qpx`` contains the computed data of the :ref:`mpc.qp` 
        using the given state and references.   
        It consist of the fields:
         
           * ``HoL`` Hessian matrix
           * ``gxoL`` gradient vector for the current state and references
           * ``u_lb`` lower bound on the optimization variable
           * ``u_ub`` upper bound on the optimization variable
           * ``E`` matrix of state dependent constraints
           * ``zx_lb`` lower bound on the state dependent constraints
           * ``zx_ub`` upper bound on the state dependent constraints
           
        Refer to the MPC description for a precise definition of these fields.
        
        For example, assuming ``mpc`` is an MPC object with only input 
        constraints, and we want all states to be regulated to 2, starting
        at states all 3::
        
           mpc.ctl.x_ref = numpy.ones(mpc.ctl.x_ref.shape) * 2
           
           # repeat the following lines for every new state x 
           x = numpy.ones(mpc.ctl.x_ref.shape) * 3
           mpc.ctl.form_qp(x)
           # use mpc.ctl.qpx together with the QP solver of your preference

        '''
        
        self._data = muaompc._cmpc.vla_form_qp(self._data, 
                                                      x.astype(numpy.double))
        self._conf._data = self._data        
        self._wmx._data = self._data        
        self._lqr._data = self._data        
        self._sys._data = self._data        
        self._qpx._data = self._data        
        
    def _get_conf(self):
        return self._conf
    
    def _get_wmx(self):
        return self._wmx    

    def _get_lqr(self):
        return self._lqr    

    def _get_sys(self):
        return self._sys    

    def _get_qpx(self):
        return self._qpx

    def _get_u_opt(self):
        return self._data['u_opt']

    def _get_l_opt(self):
        return self._data['l_opt']

    def _get_x_trj(self):
        return self._data['x_trj']

    def _get_u_ini(self):
        return self._data['u_ini']

    def _set_u_ini(self, u_ini):
        self._data['u_ini'] = u_ini.astype(float)

    def _get_l_ini(self):
        return self._data['l_ini']

    def _set_l_ini(self, l_ini):
        self._data['l_ini'] = l_ini.astype(float)

    def _get_u_ref(self):
        return self._data['u_ref']

    def _set_u_ref(self, u_ref):
        self._data['u_ref'] = u_ref.astype(float)

    def _get_x_ref(self):
        return self._data['x_ref']
    
    def _set_x_ref(self, x_ref):
        self._data['x_ref'] = x_ref.astype(float)

    conf = property(_get_conf)
    wmx = property(_get_wmx)
    lqr = property(_get_lqr)
    sys = property(_get_sys)
    qpx = property(_get_qpx)
    u_opt = property(_get_u_opt)
    l_opt = property(_get_l_opt)
    x_trj = property(_get_x_trj)
    u_ini = property(_get_u_ini, _set_u_ini)
    l_ini = property(_get_l_ini, _set_l_ini)
    x_ref = property(_get_x_ref, _set_x_ref)
    u_ref = property(_get_u_ref, _set_u_ref)
