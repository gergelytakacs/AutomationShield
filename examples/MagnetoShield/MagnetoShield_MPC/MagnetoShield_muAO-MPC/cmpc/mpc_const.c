#include "mpc_const.h" 

const real_t HoL[] = { /* Quadratic term (Hessian) matrix of QP over Lipschitz constant. */
0.422410288622, 0.316715950074, 0.245204717319, 0.188421885724, 0.142613561608, 0.104625583411, 0.0718026896396, 
0.316715950074, 0.264265497052, 0.1929008208, 0.148057952819, 0.111920512508, 0.0820086718052, 0.0562159474976, 
0.245204717319, 0.1929008208, 0.16732783578, 0.116842424658, 0.0881632998222, 0.0644715754238, 0.0441076744396, 
0.188421885724, 0.148057952819, 0.116842424658, 0.107651549517, 0.0698483958701, 0.0509343218791, 0.0347340200297, 
0.142613561608, 0.111920512508, 0.0881632998222, 0.0698483958701, 0.0706445760607, 0.0405312286879, 0.0275165185645, 
0.104625583411, 0.0820086718052, 0.0644715754238, 0.0509343218791, 0.0405312286879, 0.0474193238631, 0.0219794912531, 
0.0718026896396, 0.0562159474976, 0.0441076744396, 0.0347340200297, 0.0275165185645, 0.0219794912531, 0.0326006704887, 
 
};
const real_t GoL[] = { /* Linear term matrix of the QP, over Lipschitz constant. */
14.0863472734, -3749.30937149, -88.3908201606, 32.3052425102, 
10.5860424539, -2924.71067143, -68.7205335235, 25.1157197874, 
7.84557475072, -2273.10766054, -53.2368084111, 19.4521867192, 
5.69537249238, -1753.35287189, -40.9369144853, 14.9543156598, 
4.0021259257, -1331.80564852, -31.0051144503, 11.3236022343, 
2.66072488584, -980.223499439, -22.7604413751, 8.31071552539, 
1.58924804354, -674.680556783, -15.6291887555, 5.70567334444, 

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

const real_t nu = 0.775316078635; /* Fast gradient extra step constant */
const real_t mu = 0.0; /* Augmented Lagrange multiplier penalty parameter. */
const real_t Linv = 1.54948306648; /* Inverse of gradient Lipschitz constant (1/L) */

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
0, 0, 100, 0, 
0, 0, 0, 10, 

};
const real_t R[] = {   /* Input weighting matrix */
0.01, 

};
const real_t P[] = {   /* Terminal state weighting matrix */
2025.84859964, -21425.0444007, -161.105978889, 51.2787415766, 
-21425.0444007, 806067.197536, 10766.8638484, -3733.03172061, 
-161.105978889, 10766.8638484, 372.798783094, -99.0424431451, 
51.2787415766, -3733.03172061, -99.0424431451, 46.0737413148, 

};
const real_t K[] = {   /* Linear quadratic regulator gain matrix */
-48.7489373598, 4252.68904061, 115.544999928, -42.4246922568, 

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

