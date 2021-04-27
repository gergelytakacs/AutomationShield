/** \file
 * \brief Contains the declaration of MPC structures and type definitions.
 */
#ifndef MPC_BASE_H
#define MPC_BASE_H
#include "mc04types.h"

typedef float32_t real_t; /**< Numeric type for computations.
 * It is problem specific and defined by the automatic code generation. */


struct mpc_conf {
uint32_t in_iter;  /**< Maximum number of internal loop (FGM) iterations (j_in). */
uint32_t ex_iter;  /**< Maximum number of external loop (ALM) iterations (i_ex). */
uint32_t warmstart;  /**< If not 0, automatically warmstart next algorithm iteration. */
}; /**< Configuration parameters of the MPC algorithm.
 * Some of them are pointed at by the respective algorithm internal variable. */

struct mpc_qpx {
const real_t *HoL;  /**< The Hessian matrix. */
const real_t *gxoL;  /**< The gradient vector. */
const real_t *E;  /**< The state-dependent inequality constraints matrix. */
const real_t *u_lb;  /**< The lower bound for the state independent constraints. */
const real_t *u_ub;  /**< The upper bound for the state independent constraints. */
const real_t *zx_lb;  /**< The lower bound for the state dependent constraints. */
const real_t *zx_ub;  /**< The upper bound for the state dependent constraints.  */
const uint32_t HOR_INPUTS;  /**< The length of the state independent constraints. */
const uint32_t HOR_MXCONSTRS;  /**< The length of the state dependent constraints. */
};  /**< MPC quadratic program form for a given system state x. 
 * The quadratic program to solve has the form:
 * minimize 0.5 * u^T * HoL * u + u^T * gxoL
 * subject to u_lb <= u <= u_ub
 *            zx_lb <= E * x <= zx_ub
 *
 * The cost to minimize has been scaled by the Lipschitz constant L 
 * of the gradient, which is denoted with the suffix 'oL'.  and '^T' denotes
 * the transpose of a matrix.
 */

struct mpc_sys {
const real_t *Ad;  /**< Discrete-time system matrix. */
const real_t *Bd;  /**< Discrete-time input matrix. */
const real_t *dt;  /**< Discretization time in seconds. */
}; /**< Discrete-time system data. */

struct mpc_wmx {
const real_t *Q;  /**< State weighting matrix. */
const real_t *R;  /**< Input weighting matrix. */
const real_t *P;  /**< Terminal state weighting matrix. */
};  /**< MPC weighting matrices. */

struct mpc_lqr {
const real_t *K;  /**< Linear quadratic regulator gain matrix. */
const real_t *P;  /**< Terminal state weighting matrix. */
};  /**< Liner quadratic regulator data. */
struct mpc_fgm {
real_t *u_0;  /**< Initial guess for the optimal control sequence. */
real_t *gxoL;  /**< Gradient vector over Lipschitz for the current system state. */
real_t *groL;  /**< Non-zero reference component of gradient vector component over Lipschitz constant. */
uint32_t *j_in;  /**< Maximum number of internal loop (FGM) iterations .*/
const real_t *HoL;  /**< Hessian matrix of QP over Lipschitz constant. */
const real_t *GoL;  /**< Linear term matrix of the QP, over Lipschitz constant. */
const real_t *Bh_T;  /**< Extended input matrix (used for reference tracking). */
const real_t *u_lb;  /**< Lower bound constraint of the inputs for condensed QP. */
const real_t *u_ub;  /**< Upper bound constraint of the inputs for condensed QP. */
const real_t *nu;  /**< Fast gradient extra step constant. */
const uint32_t HOR;  /**< MPC prediction horizon. */
const uint32_t STATES;   /**< Number of system states. */
const uint32_t INPUTS;  /**< Number of system inputs. */
const uint32_t HOR_INPUTS;  /**< Horizon times number of inputs. */
const uint32_t HOR_STATES;  /**< Horizon times number of states. */
};  /**< Variables used by the fast gradient method. */

struct mpc_alm {
struct mpc_fgm *fgm;
real_t *l_0;  /**< Initial guess for the optimal multiplier. */
real_t *zx_lb;  /**< Mixed constraint lower bound as function of current state. */
real_t *zx_ub;  /**< Mixed constraint upper bound as function of current state. */
uint32_t *i_ex;  /**< Maximum number of external loop (ALM) iterations. */
const real_t *mu;  /**< Augmented Lagrange method penalty parameter. */
const real_t *E;  /**< Linear factor of 2-sided state constraint. */
const real_t *Kx_Ai;  /**< Prediction component of the mixed constraint bound. */
const real_t *e_lb;  /**< Lower bound for the mixed constraint. */
const real_t *e_ub;  /**< Upper bound for the mixed constraint. */
const real_t *Linv;  /**< Inverse of gradient Lipschitz constant (1/L). */
const uint32_t STATES;   /**< Number of system states. */
const uint32_t MXCONSTRS; /**< Number of state constraints. */
const uint32_t HOR_INPUTS;  /**< Horizon times number of inputs. */
const uint32_t HOR_MXCONSTRS;  /**< Horizon times number of states constraints. */
};  /**< Variables used by the augmented Lagrangian method. */

struct mpc_ctl {
struct mpc_conf *conf;  /**< Algorithm configuration data. */
struct mpc_qpx *qpx;  /**< State-dependent quadratic program. */
struct mpc_sys *sys;  /**< Discrete-time system data. */
struct mpc_wmx *wmx;  /**< MPC weighting matrices. */
struct mpc_lqr *lqr;  /**< Liner quadratic regulator data. */
struct mpc_alm *alm;  /**< Augmented Lagrangian method data. */
real_t *u_opt;  /**< Optimal control input sequence. */
real_t *l_opt;  /**< Optimal Lagrange multiplier. */
real_t *x_trj;  /**< State trajectory under the optimal control input sequence. */
real_t *u_ref;  /**< Reference for the input sequence. */
real_t *x_ref;  /**< Reference for the state trajectory. */
real_t *u_ini;  /**< Initial guess for the optimal control input sequence. */
real_t *l_ini;  /**< Initial guess for the optimal Lagrange multiplier. */
};  /**< The main MPC structure. Contains algorithm, system and runtime data. */

#endif /* MPC_BASE_H */
