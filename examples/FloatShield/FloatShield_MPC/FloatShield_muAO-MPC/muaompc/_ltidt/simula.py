"""Provide a simple simulation class. """

import numpy as np

from muaompc import ltidt

class MPCSim(object):
    '''
    classdocs
    '''


    def __init__(self, mpc):
        '''
        Constructor
        '''
        self.mpc = mpc

    def predict_next_state(self, xk, uk):
        '''Compute the succesor state xk1 = A*xk + B*uk

        :param xk: the current system state. It must be of size ``states``.
        :type xk: numpy array
        :param uk: the input to apply to the system.
           It must be of size ``inputs``.
        :type uk: numpy array

        :return: the succesor state.
        :rtype: numpy array of size ``states``.

        For example, assuming the ``mpc`` object exist and ``x``
        represents the current system state, we can do::

           uk = mpc.ctl.u_opt[:mpc.size.inputs, 0]  # use only the first input
           xk1 = mpc.sim.predict_next_state(xk, uk)
        '''

        A = self.mpc.ctl.sys.Ad
        B = self.mpc.ctl.sys.Bd
        return A.dot(xk) + B.dot(uk)

    def regulate_ref(self, steps, x_ini):
        """Simulate MPC regulation to a reference point for a number of steps.

        :param steps: number of discret steps to simulate. The period of each
           step is the discretization time dt.
        :type steps: int

        :param x_ini: initial state where the simulation starts.
           It must be of size ``states``.
        :type x_ini: numpy array

        Starting at a point ``x_ini``, this function will apply to the
        discrete time system ``(sys.Ad, sys.Bd)`` the first input vector of the
        control sequence ``ctl.u_opt`` at each time instant for the given number
        of steps.
        The goal of the controller is to stabilize the system at the point
        specified by ``ctl.x_ref`` and ``ctl.u_ref``.
        The simulation results are stored in the structure ``sim.data``.

        For example, assuming ``mpc`` is a valid MPC object, and ``mpc.ctl`` 
        has been already configured, we can simulate
        the regulation to the origin starting at
        all states 1 (one) for :math:`T=10` steps,
        i.e. from time :math:`t_i=0`, to :math:`t_f=(T-1)*dt`::

           import numpy as np
           T = 10
           mpc.ctl.x_ref = np.zeros(mpc.size.hor_states)
           mpc.ctl.u_ref = np.zeros(mpc.size.hor_inputs)
           mpc.sim.regulate_ref(T, np.ones(mpc.size.states))
           mpc.sim.data.x  # contains the state evolution
           mpc.sim.data.u0  # contains the applied inputs
        """

        self.data = _DataPlot(self.mpc, steps)
        self.x_hat = x_ini.reshape(-1)

        for k in range(steps):
            xk = self.x_hat[:]
            self.mpc.ctl.solve_problem(xk)
            self.uk = self.mpc.ctl.u_opt[:self.mpc.size.inputs, 0]
            self.u = self.mpc.ctl.u_opt[:, 0]
            if type(self.mpc) is ltidt._MPCInputStateConstr:
                self.l = self.mpc.ctl.l_opt[:, 0]
            self._store_current_step_data(k)
            self.x_hat = self.predict_next_state(xk, self.uk)

    def _run_unconstr_sim(self, x_ini, steps):
        self.data = _DataPlot(self.mpc, steps)
        self.x_hat = x_ini.reshape(-1)
        for k in range(steps):
            xk = self.x_hat[:]
            self.mpc.solve_problem(xk)
            self.uk = self.mpc.ctl.u_opt[:self.mpc.size.inputs, 0]
            self.u = self.mpc.ctl.u_opt[:, 0]
            self._store_current_step_data(k)
            self.x_hat = self.predict_next_state(xk, self.uk)

    def _store_current_step_data(self, k):
        self.data.t[k] = self.mpc.sys.dt * k
        self.data.x[:, k] = self.x_hat
        self.data.u0[:, k] = self.uk
        self.data.u[:, k] = self.u
        if type(self.mpc) is ltidt._MPCInputStateConstr:
            self.data.l[:, k] = self.l
        self.data.J[k] = self.mpc.compute_performance_index(self.x_hat,
                self.mpc.ctl.u_opt)


class _DataPlot(object):
    """This class contains the simulation data after running ``regulate_ref``.

    The available members are described below:
    """

    def __init__(self, mpc, steps):
        #: The discrete time vector, i.e. the sampling times.
        self.t = np.zeros(steps)

        #: The system state vector at each sampling time.
        self.x = np.zeros((mpc.size.states, steps))

        #: The input vector applied to the system at each sampling time.
        self.u0 = np.zeros((mpc.size.inputs, steps))

        #: The input sequence computed by the MPC controller
        #: at each sampling time.
        self.u = np.zeros((mpc.size.hor_inputs, steps))

        #: The Lagrange multiplier vectors computed by the MPC controller
        #: at each sampling time.
        if type(mpc) is ltidt._MPCInputStateConstr:
            self.l = np.zeros((mpc.size.hor_mxconstrs, steps))

        #: The value of the cost function at each sampling time.
        self.J = np.zeros(steps)

