r"""
This module creates a model predictive control (MPC) controller for a
linear time-invariant (*lti*) discrete-time (*dt*)  system with input and
(optionally) state constraints, and a quadratic cost function.
The MPC problem is reformulated as a condensed convex quadratic program,
which is solved using an augmented Lagrangian method together with
Nesterov's gradient method, as described in [KZF12]_, [KF11]_.

"""

import types

import numpy as np
from numpy import zeros, matrix as mtx

from muaompc._ltidt.qpmpc import _QPUnConstr as _QPUC
from muaompc._ltidt.qpmpc import _QPInputConstr as _QPIC
from muaompc._ltidt.qpmpc import _QPInputStateConstr as _QPISC

ROWS, COLS = (0, 1)

class SetupError(Exception): pass
class ShapeError(SetupError): pass
class CodeGenError(Exception): pass

def setup_mpc_problem(system_name, verbose=False):
    '''Create an MPC object from a given system module.

    :param system_name: a string with the name of a Python module containing
       the system description, e.g. for a system.py, ``system_name="system"``,
       or ``system_name="system.py"``. 
       It can also be the name of a MATLAB mat file. In that case, the file
       name must end with the extension .mat, e.g. for a system.mat
       the parameter is given as ``system_name="system.mat"``

       Alternatively, a Python module can be given as input. For example,
       if you ``import system``, then ``system_name=system``.
    :type system_name: a string or a Python module

    :param verbose: if True, print on-screen information about the problem.
    :type verbose: bool

    :returns: an instance of the appropriate MPC class according to the
       given system module.

    For example, assuming ``system1.py`` and ``system2.mat`` both contain 
    exactly the same MPC setup, we can write::
       
       import muaompc
       mpcx = muaompc.ltidt.setup_mpc_problem("system1")  # short way
       mpcy = muaompc.ltidt.setup_mpc_problem("system1.py", verbose=True)
       mpcz = muaompc.ltidt.setup_mpc_problem("system2.mat")  # MATLAB way
       # The three objects, mpcx, mpcy, and mpcz contain the same information


    Additionally, you can quickly create setups that differ only slighlty
    between them::
       
       from muaompc.ltidt import setup_mpc_problem
       import system

       system.N = 10  # set horizon length to 10 steps
       mpcx = setup_mpc_problem(system)  # system is a module, not a string
       system.N = 5  # same setup but with a shorter horizon
       mpcy = setup_mpc_problem(system)  #  mpcx is not the same as mpcy

    '''
    
    class sys(object): pass
    class wmx(object): pass
    class constr(object): pass
    class size(object): pass
    class misc(object): pass
    class lqr(object): pass
    
    system_name_type = type(system_name)
    if system_name_type is types.ModuleType:
        system = system_name
    elif system_name_type is str:
        system = _get_system_from_str(system_name)
    else:
        msg = ('Wrong system_name type. ' + str(system_name) +
                ' is of type ' + str(system_name_type) +
                '. Check the documentation for accepted types.')
        raise SetupError(msg)

    # continuous time system matrices
    try:
        dt = system.dt
    except(AttributeError):
        msg = 'No system discretization time found.'
        raise SetupError(msg)
    else:
        dtm = mtx(dt)  # consider .mat case (dt loaded as matrix)
        dt = dtm[0, 0]
        if dt <= 0:
            msg = 'Discretization time must be a positive real number.'
            raise SetupError(msg)
    
    discrete = True
    try:
        sys.Ad = mtx(system.Ad)
        sys.Bd = mtx(system.Bd)
        sys.dt = dt
    except(AttributeError,):
        if verbose:
            print('No discrete-time system matrices found.')
        discrete = False

    if not discrete:
        try:
            from scipy import linalg
        except(ImportError,):
            msg = ('Could not import scipy.linalg.' 
                  'Continuous-time matrices Ac, Bc cannot be discretized. ' 
                  'Check that Scipy is correctly installed or declare the'
                  'discrete-time variables Ad, Bd and dt in the system module.')
            raise SetupError(msg)
        else:
            try:
                Ac = mtx(system.Ac)
                Bc = mtx(system.Bc)
            except(AttributeError,):
                msg = ('No system matrices found. Declare the '
                      'matrices Ac and Bc for a continuous time-system, or ' 
                      'Ad and Bd for a discrete-time system.')
                raise SetupError(msg)

            _check_sys_shape(Ac, Bc)
            Cc = zeros(Ac.shape)
            Dc = zeros(Bc.shape)
            sysc2d = _c2d((Ac, Bc, Cc, Dc), dt)

            sys.Ad = mtx(sysc2d[0]) 
            sys.Bd = mtx(sysc2d[1])
            sys.dt = dt

    # two-sided affine input (required) and...
    required_attr = ((constr, 'u_lb'), (constr, 'u_ub'), 
                (wmx, 'Q'), (wmx, 'R'), (misc, 'N'))
    for (attr_class, attr_name) in required_attr:
        attr = _get_attribute(system, attr_name)
        if attr is not None:
            setattr(attr_class, attr_name, mtx(attr))
        else:
            msg = ('Required attribute '+attr_name+' was not found.')
            raise SetupError(msg)
    
    (n, m) = _check_sys_shape(sys.Ad, sys.Bd)
    _check_wmx_shape(wmx, n, m)
    _check_inc_shape(constr, m)
    _check_bounds(constr.u_lb, constr.u_ub, 'input')
    _check_symmetry(wmx, 'Q')
    _check_symmetry(wmx, 'R')

    size.states = sys.Ad.shape[ROWS]
    size.inputs = sys.Bd.shape[COLS]
    size.hor = misc.N[0, 0]
    size.hor_inputs = size.hor * size.inputs
    size.hor_states = size.hor * size.states

    # ... state constraints (optional) 
    is_state_constr = False
    optional_attr = ((constr, 'e_lb'), (constr, 'e_ub'), (constr, 'Kx'))
    for (attr_class, attr_name) in optional_attr:
        attr = _get_attribute(system, attr_name)
        if attr is not None:
            setattr(attr_class, attr_name, mtx(attr))
            is_state_constr = True
            if verbose:
                print('Found state constraint attribute', attr_name)
        else:
            is_state_constr = False
            if verbose:
                print('Not found attribute required for state constraint', 
                      attr_name, 
                      '. Setting up problem without state constraints.')
            break

    P = _get_wmx_P(system, sys, wmx, is_state_constr)
    wmx.P = P['P']
    lqr.K = None
    if P['type'] is 'ricatti':
        lqr.K = _calc_lqr_mtx(sys, wmx)
        lqr.P = wmx.P
    if verbose:
        print(P['msg'])

    if is_state_constr:
        mu = _get_attribute(system, 'mu')
        if type(mu) is np.ndarray:
            mu = mu[0,0]  # matrix to scalar
        if (mu is None):
            if verbose:
                print('Penalty parameter mu not found. Computing it automatically.')
        else:
            if not (isinstance(mu, float) or isinstance(mu, int)):
                print(type(mu))
                msg = ('Wrong type for penalty parameter mu. '
                        'Accepted types are float, int or None.')
                raise SetupError(msg)
            elif mu <= 0:
                msg = ('Penalty parameter must be greater than zero')
                raise SetupError(msg)

        
        Ku = _get_attribute(system, 'Ku')
        if Ku is not None:
            constr.Ku = mtx(Ku)
        else:
            constr.Ku = mtx(zeros((constr.Kx.shape[ROWS], size.inputs)))

        f_lb = _get_attribute(system, 'f_lb')
        F = _get_attribute(system, 'F')
        f_ub = _get_attribute(system, 'f_ub')
        if (f_lb is None) or (F is None) or (f_ub is None):
            if verbose:
                print('No terminal state constraint found, setting it',  
                      'equal to the stage state constraint.')
            constr.f_lb = constr.e_lb
            constr.F = constr.Kx
            constr.f_ub = constr.e_ub
        else:
            constr.f_lb = mtx(f_lb)
            constr.f_ub = mtx(f_ub)
            constr.F = mtx(F)

        _check_stc_shape(constr, size.states, size.inputs)
        _check_bounds(constr.e_lb, constr.e_ub, 'mixed')
        _check_bounds(constr.f_lb, constr.f_ub, 'terminal state')
        
        size.mxconstrs = constr.Kx.shape[ROWS]
        size.termconstrs = constr.F.shape[ROWS]
        size.hor_mxconstrs = (size.hor * size.mxconstrs + size.termconstrs)

    if is_state_constr:
        mpc = _MPCInputStateConstr(sys, constr, wmx, size, lqr, mu)
    else:
        mpc = _MPCInputConstr(sys, constr, wmx, size, lqr)

    if verbose:
        _print_info(mpc)

    return mpc


class _MPCBase(object):
    def __init__(self, sys, wmx, size, lqr):
        self.sys = sys
        self.wmx = wmx
        self.size = size
        self.lqr = lqr
        self.dt = sys.dt
        self.N = size.hor
        
        from muaompc._ltidt.writecfiles import FLOAT32, FLOAT64, ACCUM, FIP
        self.numeric = {'float32': FLOAT32, 'float64': FLOAT64, 
                        'accum': ACCUM, 'fip': FIP}
        
        self._ctl = None
        self._tun = None
        self._sim = None

    def generate_c_files(self, numeric='float64', fracbits=None,
                         matlab=False, singledir=False):
        """Write, in the current path, C code for the current MPC problem.

        :param numeric: indicates the numeric representation of the
           C variables, valid strings are:

              * ``"float64"``: double precision floating point (64 bits)
              * ``"float32"``: single precision floating point (32 bits)
              * ``"accum"``: fixed-point data type of the C extension to
                support embedded processors (signed 32 bits).
                Your compiler must support the extension.
              * ``"fip"``: (EXPERIMENTAL) fixed-point (signed 32 bits).

        :type numeric: string
        :param fracbits: an integer greater than 0 that indicates the number of
           fractional bits of fixed-point representation. (EXPERIMENTAL)
           Only required if numeric is "fip".
        :type fracbits: integer or None
        :param matlab: if True, generate code for MATLAB/Simulink interfaces.
        :type matlab: bool
        :param singledir: if True, all generated code is saved in a single
           directory. Useful for example for Arduino microcontrollers.
        :type singledir: bool

        For example, assuming that ``system.py`` contains a valid MPC setup,
        and we want to generate code appropriate for a microcontroller to be
        programmed via Simulink::
           
           import muaompc
           mpc = muaompc.ltidt.setup_mpc_problem("system")
           mpc.generate_c_files(numeric="float32", matlab=True)
           # you will find the generated C code in the current directory
        """
        
        try:
            num = self.numeric[numeric]
        except (KeyError,):
            msg = ('Unrecognized numeric format. No files were generated.') 
            raise CodeGenError(msg)
            raise
        
        if (num is self.numeric['fip']):
            try:
                ok = fracbits > 0
                if not ok:
                    raise(TypeError)
            except (TypeError,):
                msg = ('fracbits must be an integer greater than 0.'
                      ' No files were generated.')
                raise CodeGenError(msg)
            else:
                num += fracbits

        defines, mtxs, scalars, enums = self._generate_c_files_data()
        self._generate_c_files(defines, mtxs, scalars, enums, num, matlab,
                                singledir)

    def compute_performance_index(self, x0, u_opt):
        x = np.matrix(np.zeros((self.size.states, self.size.hor + 1)))
        uj = np.matrix(np.zeros((self.size.inputs, 1)))
        stage_cost = 0
        x[:,0] = np.matrix(x0.reshape(-1)).T
        for j in range(self.size.hor):
            ini_ = j * self.size.inputs
            end_ = (j + 1) * self.size.inputs
            uj[:,0] = u_opt[ini_:end_, :]
            st_cost = x[:,j].T * self.wmx.Q * x[:,j]
            in_cost = uj.T * self.wmx.R * uj
            stage_cost += st_cost + in_cost

            x[:,j+1] = self.sys.Ad * x[:,j] + self.sys.Bd * uj

        end_cost = x[:,j].T * self.wmx.P * x[:,j]

        return 0.5 * (stage_cost + end_cost)

    def _generate_c_files(self, defines, mtxs, scalars, enums,
                                    numeric, matlab=False, singledir=False):
        import muaompc._ltidt.writecfiles as wf
        from muaompc import _ltidt
        
        prefix = _ltidt.__path__[0]  # the path to the C template files

        def float_to_fip(data, frac_bits):
            factor = float(1 << frac_bits)
            #data = mtx(data_in)
            data_int = np.int32(data)
            data_frac = np.int32((data - data_int) * factor)
            return (data_int << frac_bits) + data_frac

        def mtxflop_to_mtxfip(mtxs, frac_bits):
            for key, mtx in vars(mtxs).items():
                if ((key == '__weakref__') or (key == '__doc__') or 
                    (key == '__dict__') or (key == '__module__')):
                    pass  # do nothing for special (non matrices) keys
                else:
                    fip_mtx = float_to_fip(mtx, frac_bits)
                    setattr(mtxs, key, fip_mtx)
        
        if numeric >= self.numeric['fip']:
            fip_fracbits = numeric - self.numeric['fip']
            mtxflop_to_mtxfip(mtxs, fip_fracbits)
            mtxflop_to_mtxfip(scalars, fip_fracbits)
         
        wf.write_c_files(defines, mtxs, scalars, enums, prefix,
                                   numeric, matlab=matlab, singledir=singledir)
         
    def _warmstart_vector(self, vec, unit_size):
        return np.concatenate((vec[unit_size:], vec[-unit_size:]))

    # the get_* functions are used to build the interface to the C code
    def _get_ctl(self):
        if self._ctl is None:
            import muaompc.cmpc
            self._ctl = muaompc.cmpc.ctl(self)
        return self._ctl
    
    def _get_tun(self):
        if self._tun is None:
            from muaompc._ltidt.tuning import MPCTuning
            self._tun = MPCTuning(self)
        return self._tun

    def _get_sim(self):
        if self._sim is None:
            from muaompc._ltidt.simula import MPCSim
            self._sim = MPCSim(self)
        return self._sim


class _MPCUnConstr(_MPCBase):
    def __init__(self, sys, wmx, size, lqr):
        _MPCBase.__init__(self, sys, wmx, size, lqr)
        self._qp = _QPUC(sys, wmx, size)
        self.sys = sys

    def solve_problem(self, x):
        self.u_opt = self._qp.solve_problem(x)
        self.ctl.u_opt = np.array(self.u_opt)

    def _get_ctl(self):
        if self._ctl is None:
            class _ctl(object): pass
            _ctl.solve_problem = _MPCUnConstr.solve_problem
            _ctl.u_opt = 0
            self._ctl = _ctl
            self._ctl.sys = self.sys
            self._ctl.sys.Ad, self._ctl.sys.Bd = np.array(self.sys.Ad), np.array(self.sys.Bd)
        return self._ctl
    
    ctl = property(_get_ctl)     


class _MPCInputConstr(_MPCBase):
    def __init__(self, sys, constr, wmx, size, lqr):
        _MPCBase.__init__(self, sys, wmx, size, lqr)
        self.constr = constr
        self._qp = _QPIC(sys, constr, wmx, size)
        self.u_0 = mtx(zeros((size.hor_inputs, 1)))

    ctl = property(_MPCBase._get_ctl)
    
    tun = property(_MPCBase._get_tun)
    
    sim = property(_MPCBase._get_sim)

    def _generate_c_files_data(self):
        class mtxs(object): pass
        class scalars(object): pass
        class enums(object): pass
        
        defines = []
        
        dummy_mtx = mtx([[0]])
        
        mtxs.HoL = self._qp.fgm.HoL
        mtxs.GoL = self._qp.fgm.GoL
        mtxs.u_lb = self._qp.u_lb
        mtxs.u_ub = self._qp.u_ub
        mtxs.E = dummy_mtx 
        mtxs.e_lb = dummy_mtx
        mtxs.e_ub =  dummy_mtx
        mtxs.Kx_Ai = dummy_mtx 
        mtxs.Ad = self.sys.Ad
        mtxs.Bd = self.sys.Bd
        mtxs.Q = self.wmx.Q
        mtxs.R = self.wmx.R
        mtxs.P = self.wmx.P
        mtxs.Bh_T = self._qp.Bh.T
        # dual mode MPC
        if self.lqr.K is not None:
            mtxs.K = self.lqr.K
        else:
            mtxs.K = dummy_mtx
          
        # 0. is dummy value, only required to correctly compile
        scalars.dt = self.dt
        scalars.nu = self._qp.fgm.nu 
        scalars.mu = 0.
        scalars.linv = self._qp.fgm.Linv

        enums.hor = self.size.hor
        enums.states = self.size.states
        enums.inputs = self.size.inputs
        enums.mxconstrs = 0
        enums.hor_inputs = self.size.hor_inputs
        enums.hor_states = self.size.hor_states
        enums.hor_mxconstrs = 1  # dummy value, to correctly compile
        
        return defines, mtxs, scalars, enums        
        

class _MPCInputStateConstr(_MPCInputConstr):
    def __init__(self, sys, constr, wmx, size, lqr, mu):
        _MPCInputConstr.__init__(self, sys, constr, wmx, size, lqr)
        self._qp = _QPISC(sys, constr, wmx, size, mu)
        self.l_0 = mtx(zeros((size.hor_mxconstrs, 1)))        

    def _generate_c_files_data(self):
               
        defines, mtxs, scalars, enums = _MPCInputConstr._generate_c_files_data(self)
        defines.append(['', 'STATE_CONSTR',
                 'State constrained problem.'])
        
        mtxs.E = self._qp.E
        mtxs.Kx_Ai = self._qp.Kx_Ai
        mtxs.e_lb = self._qp.e_lb
        mtxs.e_ub = self._qp.e_ub
        
        scalars.mu = self._qp.fgm.mu
        scalars.linv = self._qp.fgm.Linv

        enums.hor_mxconstrs = self.size.hor_mxconstrs
        enums.mxconstrs = self.size.mxconstrs
        
        return defines, mtxs, scalars, enums


def _get_system_from_str(system_name):
    system_name, dot, extension = system_name.partition('.')
    if extension == 'mat':
        system = _get_system_from_mat(system_name)
    elif (extension == 'py') or (extension == ''):
        system = _get_system_from_py(system_name)
    
    return system

def _get_system_from_mat(system_name):
    try:
        from scipy import io
    except(ImportError,):
        msg = ('Attempting to open the system module from a '
              'MATLAB mat file but scipy.io could not be imported. '
              'Check that Scipy is correctly installed.')
        raise SetupError(msg)
    else:
        try:
            system_dict = io.loadmat(system_name)
        except(IOError,):
            msg = ('Could not find MATLAB system module.')   
            raise SetupError(msg)
        else:
            class system(object): pass
            for key in system_dict:
                setattr(system, key, system_dict[key])
        
    return system
    
def _get_system_from_py(system_name):
    try:
        system = __import__(system_name)
    except(ImportError,):
        msg = ('Could not find Python system module.')   
        raise SetupError(msg)
        
    return system 

def _c2d(sysc, dt):
    # We have made our own version of c2d to avoid importing the
    # scipy.signal module (it gives importing problems sometimes)
    # The following was copied from scipy.linalg.cont2discrete.py
    from scipy import linalg
    a, b, c, d = sysc
    # Build an exponential matrix
    em_upper = np.hstack((a, b)) 

    # Need to stack zeros under the a and b matrices
    em_lower = np.hstack((np.zeros((b.shape[1], a.shape[0])),
                  np.zeros((b.shape[1], b.shape[1])) ))

    em = np.vstack((em_upper, em_lower))
    ms = linalg.expm(dt * em) 

    # Dispose of the lower rows
    ms = ms[:a.shape[0], :]

    ad = ms[:, 0:a.shape[1]]
    bd = ms[:, a.shape[1]:]

    cd = c 
    dd = d 
    return ad, bd, cd, dd, dt

def _get_attribute(system, attr_name):
    try:
        attr = getattr(system, attr_name)
    except(AttributeError,):
        attr = None
    return attr

def _get_wmx_P(system, sys, wmx, stateconstr):
    msg = ""
    P = _get_attribute(system, 'P')
    type_ = None
    if P == 'auto':
        from muaompc._ltidt import stability
        P, msg, type_ = stability.calc_terminal_cost_matrix(sys.Ad, sys.Bd,
                                                   wmx.Q, wmx.R, stateconstr)
    elif (type(P) is str):
        msg = (P + ' is not a valid keyword.')
        raise SetupError(msg) 
    elif P is None:
        msg = 'Required attribute P was not found.'
        raise SetupError(msg)
    
    return dict(P=mtx(P), msg=msg, type=type_)

def _calc_lqr_mtx(sys, wmx):
    # compute linear quadratic regulator (lqr) matrix K, s.t. u = K*x
    Ad, Bd = sys.Ad, sys.Bd
    R, P = wmx.R, wmx.P
    return -np.linalg.inv(Bd.T * P * Bd + R) * Bd.T * P * Ad

def _print_info(mpc):
    qp = mpc._qp
    print('Problem summary: ')
    if type(mpc) is _MPCInputConstr:
        print('Problem with input constraints only (no mixed constraints).')
    elif type(mpc) is _MPCInputStateConstr:
        print('Problem with input and state (mixed) constraints.')

    cnH = np.linalg.cond(qp.H)
    print('The condition number of the QP Hessian is: ', cnH)
    if cnH > 1000.:
        print('This value is too high. The default solver might not work.')
    elif cnH > 100.:
        print('This value is high. It may to lead innacurate solutions.')

    if cnH > 10.:
        print('It is recommended that you try reducing it. '
                'For example by using the tuning methods in mpc.tun.')
    else:
        print('This is a nice condition number.')


def _check_symmetry(wmx, attr):
    M = getattr(wmx, attr)
    if not np.allclose(M, M.T):
        msg = ('Matrix ' + attr + ' not symmetric.') 
        raise SetupError(msg)

def _check_bounds(lb, ub, type_):
    if not np.alltrue(lb < ub):
        msg = ('Lower bound not lower than upper bound of ' +
                type_ + ' constraints.')
        raise SetupError(msg)

def _check_sys_shape(A, B):
    n, n2 = A.shape
    if n != n2:
        msg = ('The system matrix A is not a square matrix. '
                'A.shape is ' + str(A.shape) + '.')
        raise ShapeError(msg)
    
    n3, m = B.shape
    if n != n3:
        msg = ('Incompatible number of columns of matrices A and B.' + 
                'Columns of A is ' + str(n) +
                ', columns of B is ' + str(n3) + '.')
        raise ShapeError(msg)

    return (n, m)

def _check_wmx_shape(wmx, n, m):
    n2, n3 = wmx.Q.shape
    if (n2 != n) or (n3 != n):
        msg = ('The matrix Q has shape'+str(wmx.Q.shape)+ 
                '. Expected ('+str(n)+', '+str(n)+').')
        raise ShapeError(msg)
    
    m2, m3 = wmx.R.shape
    if (m2 != m) or (m3 != m):
        msg = ('The matrix R has shape'+str(wmx.R.shape)+ 
                '. Expected ('+str(m)+', '+str(m)+').')
        raise ShapeError(msg)

    return

def _check_inc_shape(constr, m):
    m0, c = constr.u_lb.shape
    if (m0 != m) or (c != 1):
        msg = ('The vector u_lb has shape'+str(constr.u_lb.shape)+ 
                '. Expected ('+str(m)+', '+str(1)+').')
        raise ShapeError(msg)
    
    m0, c = constr.u_ub.shape
    if (m0 != m) or (c != 1):
        msg = ('The vector u_ub has shape'+str(constr.u_ub.shape)+ 
                '. Expected ('+str(m)+', '+str(1)+').')
        raise ShapeError(msg)
    
    return

def _check_stc_shape(constr, n, m):
    q, n0 = constr.Kx.shape
    if (n0 != n):
        msg = ('Incompatible number of rows of matrix Kx and number of states.'
                ' Rows of Kx is ' + str(n0) +
                ', number of states is ' + str(n) + '.')
        raise ShapeError(msg)

    q0, m0 = constr.Ku.shape
    if (q0 != q) or (m0 != m):
        msg = ('The matrix Ku has shape ' + str(constr.Ku.shape) +
                '. Expected ('+str(q)+', '+str(m)+').')
        raise ShapeError(msg)

    r, n0 = constr.F.shape
    if (n0 != n):
        msg = ('Incompatible number of rows of matrix F and number of states.'
                ' Rows of F is ' + str(n0) +
                ', number of states is ' + str(n) + '.')
        raise ShapeError(msg)

    q0, c = constr.e_lb.shape
    if (q0 != q) or (c != 1):
        msg = ('The vector e_lb has shape'+str(constr.e_lb.shape)+ 
                '. Expected ('+str(q)+', '+str(1)+').')
        raise ShapeError(msg)

    q0, c = constr.e_ub.shape
    if (q0 != q) or (c != 1):
        msg = ('The vector e_ub has shape'+str(constr.e_ub.shape)+ 
                '. Expected ('+str(q)+', '+str(1)+').')
        raise ShapeError(msg)

    r0, c = constr.f_lb.shape
    if (r0 != r) or (c != 1):
        msg = ('The vector f_lb has shape'+str(constr.f_lb.shape)+ 
                '. Expected ('+str(r)+', '+str(1)+').')
        raise ShapeError(msg)

    r0, c = constr.f_ub.shape
    if (r0 != r) or (c != 1):
        msg = ('The vector f_ub has shape'+str(constr.f_ub.shape)+ 
                '. Expected ('+str(r)+', '+str(1)+').')
        raise ShapeError(msg)

