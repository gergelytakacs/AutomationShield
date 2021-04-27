#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.7973645313945321, 0.23920285707184177, 
0.23920285707184183, 0.71763084111037, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
0.43898901841896404, 0.19137098857085583, 0.599015304978518, -0.0024111595333359405, 
0.3305902975857388, 0.14418196062134866, 0.41960642716170055, -0.0018083146005838891, 

};
const real_t Bh_T[] = { /* Extended input matrix (used for reference tracking). */
0.00011, 0.01321, 0.51677, 0.0, 0.0007471574, 0.0369488537, 0.4644263667, -0.00011, 
0.0, 0.0, 0.0, 0.0, 0.00011, 0.01321, 0.51677, 0.0, 

};
const real_t E[] = { /* Linear factor (prediction matrix) of 2-sided state constraint. */
0, 

};
const real_t Kx_Ai[] = { /* Prediction component of the state constraint bound. */
0, 

};

const real_t u_lb[] = { /* Left (lower) constraint of the inputs for condensed QP. */
0, 
0, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
100, 
100, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.16439391877075868; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 4.5567875798936754e-08; /* Inverse of gradient Lipschitz constant (1/L) */

/* state dependent variables */
real_t gxoL[MPC_HOR_INPUTS];  /* gradient vector as a function of the current state */
real_t zx_lb[MPC_HOR_MXCONSTRS];  /* mixed constraint lower bound as function of current state */
real_t zx_ub[MPC_HOR_MXCONSTRS];  /* mixed constraint upper bound as function of current state */

/* reference dependent variables */
real_t groL[MPC_HOR_INPUTS];

/* MPC system: state-space and weighting matrices */
const real_t Q[] = {  /* State weighting matrix */
1, 0, 0, 0, 
0, 1, 0, 0, 
0, 0, 10000000, 0, 
0, 0, 0, 100, 

};
const real_t R[] = {   /* Input weighting matrix */
10000000, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
269008349.71, 115828228.55, 10867210.5, -1643757.68, 
115828228.55, 50413960.28, 4735137.81, -645277.14, 
10867210.5, 4735137.81, 21245328.08, -59947.47, 
-1643757.68, -645277.14, -59947.47, 18181.58, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
0, 

};
const real_t Ad[] = {   /* Discrete-time system matrix */
1.0, 0.02437, 0.00061, 0.0, 
0.0, 0.9502, 0.04721, 0.0, 
0.0, 0.0, 0.89871, 0.0, 
-1.0, 0.0, 0.0, 1.0, 

};
const real_t Bd[] = {   /* Discrete-time input matrix */
0.00011, 
0.01321, 
0.51677, 
0.0, 

};
const real_t dt = 0.025;


/* User variables created with appropriate size */
real_t u_opt[MPC_HOR_INPUTS];  /* Optimal input */
real_t l_opt[MPC_HOR_MXCONSTRS];  /* Optimal multiplier */
real_t x_trj[MPC_HOR_STATES + MPC_STATES];  /* State trajectory for u_opt */
real_t u_ini[MPC_HOR_INPUTS];  /* Initial guess input */
real_t l_ini[MPC_HOR_MXCONSTRS];  /* Initial guess multiplier */
real_t ctl_x_ref[MPC_HOR_STATES];  /* Reference for state trajectory */
real_t ctl_u_ref[MPC_HOR_INPUTS];  /* Reference for input trajectory */

/* Always check this declarations match the structure definitions */
struct mpc_conf conf = {1, 1, 0};

struct mpc_qpx qpx = {HoL, gxoL, E, u_lb, u_ub, zx_lb, zx_ub,
		MPC_HOR_INPUTS, MPC_HOR_MXCONSTRS};

struct mpc_sys sys = {Ad, Bd, &dt};

struct mpc_wmx wmx = {Q, R, P,};

struct mpc_lqr lqr = {K, P,};

struct mpc_fgm fgm = {u_ini, gxoL, groL, &(conf.in_iter),
			HoL, GoL, Bh_T, u_lb, u_ub, &nu,
			MPC_HOR, MPC_STATES, MPC_INPUTS, MPC_HOR_INPUTS, MPC_HOR_STATES};

struct mpc_alm alm = {&fgm, l_ini, zx_lb, zx_ub, &(conf.ex_iter),
			&mu,
			E, Kx_Ai, e_lb, e_ub,
			&Linv, 
			MPC_STATES, MPC_MXCONSTRS, MPC_HOR_INPUTS, MPC_HOR_MXCONSTRS};

struct mpc_ctl ctl = {&conf, &qpx, &sys, &wmx, &lqr, &alm,
			u_opt, l_opt, x_trj, 0, 0, u_ini, l_ini};

