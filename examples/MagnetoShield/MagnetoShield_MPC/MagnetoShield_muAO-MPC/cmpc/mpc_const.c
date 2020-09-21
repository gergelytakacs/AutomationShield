#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.426286151429, 0.312161912904, 0.241787939372, 0.18667911789, 0.143406151266, 0.10926619157, 0.0820786943313, 
0.312161912904, 0.26706975147, 0.188159133454, 0.14527552974, 0.111616834671, 0.0850665239806, 0.0639254890402, 
0.241787939372, 0.188159133454, 0.170492455946, 0.112992209389, 0.0868146440941, 0.0661800835743, 0.0497537475499, 
0.18667911789, 0.14527552974, 0.112992209389, 0.111989404656, 0.0674916087735, 0.0514508373014, 0.0386958516943, 
0.143406151266, 0.111616834671, 0.0868146440941, 0.0674916087735, 0.0766014272648, 0.0399874972163, 0.0300750452629, 
0.10926619157, 0.0850665239806, 0.0661800835743, 0.0514508373014, 0.0399874972163, 0.0552247844522, 0.023373884001, 
0.0820786943313, 0.0639254890402, 0.0497537475499, 0.0386958516943, 0.0300750452629, 0.023373884001, 0.0423129279699, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
17.5433855793, -3864.79962076, -87.8389553148, 32.0297768628, 
13.182785993, -2999.6342698, -68.1872373015, 24.8618010311, 
9.78661280425, -2322.60126836, -52.8154503492, 19.2572168876, 
7.14334255309, -1792.07321372, -40.7748998058, 14.8674466764, 
5.08754578071, -1375.29988757, -31.3194004725, 11.420249633, 
3.48974709497, -1046.38019372, -23.8589823688, 8.7004694991, 
2.24796139487, -784.411223058, -17.9175755688, 6.53449514701, 

};
const real_t Bh_T[] = { /* Extended input matrix (used for reference tracking). */
0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 7.78554189e-06, -3.40614972694e-05, -0.00592025279513, 0.000347899404466, 4.18470391594e-05, -6.55966936091e-05, -0.00672142472748, 0.000184078311738, 0.000107443732769, -0.000101833211946, -0.00784156516875, 0.000202993168359, 0.000209276944714, -0.000144773409066, -0.00941852984869, 0.000241315890321, 0.00035405035378, -0.000196950054442, -0.0115554992957, 0.00029426521516, 0.000551000408222, -0.00026148275461, -0.0143864644526, 0.000364853507345, 
0.0, 0.0, 0.0, 0.0, 0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 7.78554189e-06, -3.40614972694e-05, -0.00592025279513, 0.000347899404466, 4.18470391594e-05, -6.55966936091e-05, -0.00672142472748, 0.000184078311738, 0.000107443732769, -0.000101833211946, -0.00784156516875, 0.000202993168359, 0.000209276944714, -0.000144773409066, -0.00941852984869, 0.000241315890321, 0.00035405035378, -0.000196950054442, -0.0115554992957, 0.00029426521516, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 7.78554189e-06, -3.40614972694e-05, -0.00592025279513, 0.000347899404466, 4.18470391594e-05, -6.55966936091e-05, -0.00672142472748, 0.000184078311738, 0.000107443732769, -0.000101833211946, -0.00784156516875, 0.000202993168359, 0.000209276944714, -0.000144773409066, -0.00941852984869, 0.000241315890321, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 7.78554189e-06, -3.40614972694e-05, -0.00592025279513, 0.000347899404466, 4.18470391594e-05, -6.55966936091e-05, -0.00672142472748, 0.000184078311738, 0.000107443732769, -0.000101833211946, -0.00784156516875, 0.000202993168359, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 7.78554189e-06, -3.40614972694e-05, -0.00592025279513, 0.000347899404466, 4.18470391594e-05, -6.55966936091e-05, -0.00672142472748, 0.000184078311738, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 7.78554189e-06, -3.40614972694e-05, -0.00592025279513, 0.000347899404466, 
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -7.78554189e-06, -0.00391515073, 0.00444830751, 

};
const real_t E[] = { /* Linear factor (prediction matrix) of 2-sided state constraint. */
0, 

};
const real_t Kx_Ai[] = { /* Prediction component of the state constraint bound. */
0, 

};

const real_t u_lb[] = { /* Left (lower) constraint of the inputs for condensed QP. */
-4.6234, 
-4.6234, 
-4.6234, 
-4.6234, 
-4.6234, 
-4.6234, 
-4.6234, 

};
const real_t u_ub[] = { /* Right (upper) constraint of the inputs for condensed QP. */
5.3766, 
5.3766, 
5.3766, 
5.3766, 
5.3766, 
5.3766, 
5.3766, 

};
const real_t e_lb[] = { /* Left (lower) constraint of the states for condensed QP. */
0, 

};
const real_t e_ub[] = { /* Right (upper) constraint of the states for condensed QP. */
0, 

};

const real_t nu = 0.730053805013; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 2.38855152564; /* Inverse of gradient Lipschitz constant (1/L) */

/* state dependent variables */
real_t gxoL[MPC_HOR_INPUTS];  /* gradient vector as a function of the current state */
real_t zx_lb[MPC_HOR_MXCONSTRS];  /* mixed constraint lower bound as function of current state */
real_t zx_ub[MPC_HOR_MXCONSTRS];  /* mixed constraint upper bound as function of current state */

/* reference dependent variables */
real_t groL[MPC_HOR_INPUTS];

/* MPC system: state-space and weighting matrices */
const real_t Q[] = {  /* State weighting matrix */
50, 0, 0, 0, 
0, 1000, 0, 0, 
0, 0, 10, 0, 
0, 0, 0, 10, 

};
const real_t R[] = {   /* Input weighting matrix */
0.01, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
1874.31526565, -15586.6713879, -151.658314207, 50.8112719135, 
-15586.6713879, 527884.046302, 9737.12820804, -3490.09752098, 
-151.658314207, 9737.12820804, 222.616064302, -77.0552131386, 
50.8112719135, -3490.09752098, -77.0552131386, 37.9655240126, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
-53.1270583891, 4197.58051746, 93.8252719968, -34.2567907296, 

};
const real_t Ad[] = {   /* Discrete-time system matrix */
1.0, -1.0, 0.0, 0.0, 
0.0, 1.02691221, 0.00509077766, -0.00137923634, 
0.0, 10.8561017, 1.05008671, -0.387672032, 
0.0, -0.202825614, -0.0267336835, 0.0543249081, 

};
const real_t Bd[] = {   /* Discrete-time input matrix */
0.0, 
-7.78554189e-06, 
-0.00391515073, 
0.00444830751, 

};
const real_t dt = 0.005;


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

