#ifndef MPC_CONST_H
#define MPC_CONST_H

#include "mpc_base.h"



enum { 
MPC_HOR = 20,  /**< MPC prediction horizon. */
MPC_STATES = 4,  /**< Number of system states. */
MPC_INPUTS = 1,  /**< Number of system inputs. */
MPC_MXCONSTRS = 0, /**< Number of mixed stage constraints. */
MPC_HOR_INPUTS = 20,  /**< Horizon times number of inputs. */
MPC_HOR_STATES = 80,  /**< Horizon times number of states. */
MPC_HOR_MXCONSTRS = 1  /**< Horizon times number of mixed constrained
plus the number of end state constraints. */
}; 

#endif /* MPC_CONST_H */
