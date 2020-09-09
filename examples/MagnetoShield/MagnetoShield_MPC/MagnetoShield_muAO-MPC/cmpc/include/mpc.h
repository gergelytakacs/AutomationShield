#ifndef MPC_H
#define MPC_H
#include "mpc_base.h"
#include "mpc_inc.h"
#include "mpc_stc.h"
#include "mpc_ref.h"
#include "mpc_const.h"

/** \file  
 * \brief The user interface to the MPC algorithm and auxiliary functions.
 *  
 */

/** Compute the MPC optimal control sequence for the current state.
 * 
 * Compute the optimal control sequence u_opt given a pointer to the MPC
 * controller ctl and the current state vector x. 
 * This function is a one-size-fits-all MPC solver. It automatically call
 * the functions appropriate for the type of problem. Four cases are
 * considered:
 *
 * - input constraints, regulation of the origin
 * - input constraints, trajectory tracking
 * - mixed and input constraints, regulation of the origin
 * - mixed and input constraints, trajectory tracking
 *
 * If the problem if either input only or with input and mixed constraints is
 * determined at the time of automatically generating the code. The user
 * cannot change this at runtime, and the algorithm being called is 
 * different for each case. Both of the reference tracking cases are specified
 * at run-time, by setting any of the ctl.x_ref and ctl.u_ref pointers to point
 * to an array of appropriate size (MPC_HOR_STATES, and MPC_HOR_INPUTS,
 * respectively). In reference tracking case, if one of those two pointers 
 * is not given or set as NULL pointer
 * (by default, ctl.x_ref und ctl.u_ref are initialized as NULL pointers),
 * the array not given is initialized to point to an array of appropriate
 * with each element set to zero (0.).
 * 
 * The computed optimal control sequence is saved in ctl.u_opt.
 */
extern void mpc_ctl_solve_problem(
 	struct mpc_ctl *ctl,  /**< contains the MPC problem information */
	const real_t x[]  /**< the current state of the system */
	);

/** Form a quadratic program for the given system state vector.
 * 
 * Form a quadratic program (QP) qpx given a pointer to the MPC 
 * controller ctl and the current state vector x.
 * This function works in the same way as \ref mpc_ctl_solve_problem,
 * except that it only forms the QP to solve, without actually solving it.
 * The resulting QP for the given state is stored in the structure ctl.qpx.
 * The data stored in this structure can then be used together with 
 * other QP solvers. 
 * See \ref struct mpc_qpx for the meaning of the fields of qpx.
 */

extern void mpc_ctl_form_qp(
		struct mpc_ctl *ctl, /**< contains the MPC problem information */
		const real_t x[] /** the current state of the system */
		);

/** Predict the state at the next sampling time.
 *
 * Predict the state at the next sampling time given a pointer 
 * to the MPC controller ctl and the current state vector x. 
 * ctl contains the discrete-time system matrices Ad, and Bd 
 * and the optimal input sequence u_opt. 
 * Ad and Bd have been setup by the automatic code generation procedure,
 * and u_opt is automatically computed by mpc_ctl_solve_problem function.
 * x is an array with MPC_STATES elements. x will be overwritten with the 
 * predicted value according to x = Ad * x + Bd * u_opt.
 * Note that only the first input vector of u_opt sequence is used, i.e.
 * only the first MPC_INPUTS elements.
 */
extern void mpc_predict_next_state(
	struct mpc_ctl *ctl,  /**< contains the MPC problem information */
	real_t x[]  /**< the current state of the system */
	);

/** Generate a state trajectory using the controller input sequence.
 *
 * Generate a sequence of states of the discrete-time system under the input
 * sequence ctl->u_opt, starting from the current state, x[0]=x,
 * using x[i+1] = Ad*x[i] + Bd*u_opt[i], and finishing at x_N.
 * given a pointer to the MPC controller ctl and the current state vector x.
 * ctl contains the discrete-time system matrices Ad, and Bd
 * and the optimal input sequence u_opt.
 * Ad and Bd have been setup by the automatic code generation procedure,
 * and u_opt is automatically computed by mpc_ctl_solve_problem function.
 * x is an array with MPC_STATES elements.
 * The predicted trajectory is stored in the array ctl->x_trj.
 * Note that x_trj has MPC_STATES * (MPC_HOR + 1) elements.
 */
extern void mpc_generate_state_trj(
	struct mpc_ctl *ctl,  /**< contains the MPC problem information */
	real_t x[]  /**< the current state of the system */
	);

#endif /* MPC_H */
