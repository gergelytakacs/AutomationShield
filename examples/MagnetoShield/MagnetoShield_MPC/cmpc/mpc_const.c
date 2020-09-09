#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.992807385297, 0.0841993121935, 0.00714055566675, 0.000605458262362, 5.20362913137e-05, 
0.0841993121935, 0.0071408858352, 0.000605585616733, 5.13484994407e-05, 4.41316359085e-06, 
0.00714055566675, 0.000605585616733, 5.13569460614e-05, 4.35462642007e-06, 3.742605126e-07, 
0.000605458262362, 5.13484994407e-05, 4.35462642007e-06, 3.69255713336e-07, 3.17309737821e-08, 
5.20362913137e-05, 4.41316359085e-06, 3.742605126e-07, 3.17309737821e-08, 2.74715191717e-09, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
0.000854266632021, -2232.94263076, -51.6710755041, 18.8583548345, 
7.24467296719e-05, -189.37433028, -4.38218841079, 1.59936411611, 
6.14109872579e-06, -16.059964289, -0.371633205477, 0.135634700628, 
5.18017025942e-07, -1.36174808992, -0.0315113284016, 0.0115006666015, 
4.12746179522e-08, -0.117035840946, -0.00270825049207, 0.000988428212042, 

};
const real_t Bh_T[] = { /* Extended input matrix (used for reference tracking). */
0.0, -0.00217236518, -0.129810522, 0.00785228867, 0.00217236518, -0.0286879768911, -1.4185527584, 0.0357076986749, 0.0308603420711, -0.33862910855, -16.7106382908, 0.420575851801, 0.369489450621, -3.99285332818, -197.034983701, 4.95899859609, 4.3623427788, -47.0801434507, -2323.25924914, 58.4720487991, 
0.0, 0.0, 0.0, 0.0, 0.0, -0.00217236518, -0.129810522, 0.00785228867, 0.00217236518, -0.0286879768911, -1.4185527584, 0.0357076986749, 0.0308603420711, -0.33862910855, -16.7106382908, 0.420575851801, 0.369489450621, -3.99285332818, -197.034983701, 4.95899859609, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00217236518, -0.129810522, 0.00785228867, 0.00217236518, -0.0286879768911, -1.4185527584, 0.0357076986749, 0.0308603420711, -0.33862910855, -16.7106382908, 0.420575851801, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00217236518, -0.129810522, 0.00785228867, 0.00217236518, -0.0286879768911, -1.4185527584, 0.0357076986749, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00217236518, -0.129810522, 0.00785228867, 

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
0, 
0, 
0, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
10, 
10, 
10, 
10, 
10, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.9999920425; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 6.20131007844e-09; /* Inverse of gradient Lipschitz constant (1/L) */

/* state dependent variables */
real_t gxoL[MPC_HOR_INPUTS];  /* gradient vector as a function of the current state */
real_t zx_lb[MPC_HOR_MXCONSTRS];  /* mixed constraint lower bound as function of current state */
real_t zx_ub[MPC_HOR_MXCONSTRS];  /* mixed constraint upper bound as function of current state */

/* reference dependent variables */
real_t groL[MPC_HOR_INPUTS];

/* MPC system: state-space and weighting matrices */
const real_t Q[] = {  /* State weighting matrix */
100, 0, 0, 0, 
0, 10, 0, 0, 
0, 0, 10, 0, 
0, 0, 0, 1, 

};
const real_t R[] = {   /* Input weighting matrix */
0.001, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
906.374450121, -1856.3445967, -19.7781556177, 7.0958912891, 
-1856.3445967, 13089.0094636, 231.247114439, -83.6266368721, 
-19.7781556177, 231.247114439, 14.4541859536, -1.61372912236, 
7.0958912891, -83.6266368721, -1.61372912236, 1.58469825598, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
-15.0245123665, 2205.21463626, 50.9503682196, -18.6026018681, 

};
const real_t Ad[] = {   /* Discrete-time system matrix */
1.0, -1.0, 0.0, 0.0, 
0.0, 5.54124819, 0.125500443, -0.0457298843, 
0.0, 267.630146, 6.30962057, -2.30597843, 
0.0, -6.72487489, -0.159019203, 0.0581294058, 

};
const real_t Bd[] = {   /* Discrete-time input matrix */
0.0, 
-0.00217236518, 
-0.129810522, 
0.00785228867, 

};
const real_t dt = 0.05;


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

