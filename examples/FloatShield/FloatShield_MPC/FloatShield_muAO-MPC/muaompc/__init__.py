"""
This package automatically generates self-contained C-code specific for 
a model predictive control (MPC) problem. The problem is described in a Python
module created by the user. The generated C-code is a ready-to-use fast MPC
implementation. For an extended description see [ZKF13]_.

At the moment, we consider the following types of systems:

* linear time-invariant discrete-time sytems (the ``ltidt`` module)

"""

from muaompc import ltidt 
from muaompc.version import version as __version__
