r"""
.. default-role:: math

The matrices describing the MPC setup are read from the *system* module.
It basically contains the matrices that describe the MPC setup.

* The system matrices can be given either in discrete or continuous time.
 
   - In the first case, they must be called ``Ac`` and ``Bc``, respectively
     (in this case *SciPy* is required). 
    
   - In the discrete-time case, they must be called ``Ad`` and ``Bd``,
     respectively. 
   
   - The zero-order hold discretization time should be specified as 
     ``dt`` in both cases. 
   
* The state and input weighting matrices `Q` and `R` must be called 
  ``Q`` and ``R``, respectively. 

* The terminal weight matrix `P` is called ``P``. It can be declared
  as ``"auto"``, in which case it will be computed as a stabilizing matrix 
  (the Python package *Slycot* is required).

* The lower and upper input constraints bounds 
  `u\_lb, u\_ub` are called ``u_lb`` and ``u_ub``, respectively.

* The MPC horizon length `N` represents steps (not time) and is 
  an integer greater than one called ``N``. 

Additionally, for a state constrained problem, the following are required:

* Mixed constraints

   - The lower and upper mixed constraints bounds 
     `e\_lb, e\_ub \in \mathbb{R}^{q}` should be called ``e_lb``, and ``e_ub``, 
     respectively. 

   - `K_x`, must be called ``Kx``.

   - `K_u`, is optional and is called ``Ku``. If not specified (or ``None``), 
     it is set to a zero matrix of appropriate size.

If *any* of ``e_lb``, ``e_ub``, or ``Kx`` is not specified (or ``None``), 
then the MPC setup is considered to be without state constraints.

* Terminal state constraints are optional:

   - The terminal state bounds `f\_lb, f\_ub \in \mathbb{R}^{q}`,
     are called ``f_lb``, and ``f_ub``, respectively.

   - `F`, the terminal state constraint matrix, must be called ``F``. 

   - If *any* of ``F``, ``f_lb``, or ``f_ub`` is not specified (or ``None``), 
     *each* of them is set to ``Kx``, ``e_lb``, and ``e_ub``, respectively.  

* The penalty parameter `\mu` of the augmented Lagrangian method is 
  optional. If specified, it must be called ``mu``,
  and must be a positive real number. If not specified (or ``None``), it is computed
  automatically (recommended).
     
All matrices are accepted as Python lists of lists, or as numpy arrays, 
or numpy matrices.  The system module can also be written as a MATLAB 
*mat* file containing the required matrices using the names above. 
   
For example, an input-constrained second-order continuous-time LTI system
could be described by the following ``system.py`` Python module::

    Ac = [[0, 1], [-1, -1]]
    Bc = [[0], [1]]
    dt = 0.5
    N = 10
    Q = [[1, 0], [0, 1]]
    R = [[1]]
    P = Q
    u_lb = [[-10]]
    u_ub = [[10]]

"""
