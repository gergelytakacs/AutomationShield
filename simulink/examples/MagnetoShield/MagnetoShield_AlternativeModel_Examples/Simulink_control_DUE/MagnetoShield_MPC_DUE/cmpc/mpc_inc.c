/** Quadratic Programming online solver using Fast gradients.
 */

#include <string.h> /* sizeof */
#include "mpc_inc.h"
#include "mpc_const.h"

/* static functions declaration */
static void inc_fgm_compute_projected_grad_step(const struct mpc_fgm *fgm,
		real_t u[], const real_t w[],
		const real_t gradoL[]);


/* external functions definition */

/* Solve input constrained MPC problem for the current state */
void inc_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[]) {
	inc_ctl_form_qp(ctl, x);
	inc_fgm_minimize_qp(ctl->alm->fgm, ctl->u_opt);
	return;
}

/* Form quadratic program for current state */
void inc_ctl_form_qp(struct mpc_ctl *ctl, const real_t x[])  {
	inc_compute_gxoL(ctl->alm->fgm, x);
	return;
}

/* Warmstart input constrained MPC */
void inc_ctl_warmstart(struct mpc_ctl *ctl) {
	mpc_warmstart_vector(ctl->alm->fgm, ctl->alm->fgm->u_0, ctl->u_opt, MPC_INPUTS, MPC_HOR);
}

/* Minimize MPC quadratic program using fast gradient method */
void inc_fgm_minimize_qp(const struct mpc_fgm *fgm, real_t u[]) {
	static real_t u_old[MPC_HOR_INPUTS];
	static real_t w[MPC_HOR_INPUTS];
	static real_t gradoL[MPC_HOR_INPUTS];
	uint32_t j;
	const size_t sizeof_u = MPC_HOR_INPUTS * sizeof(*u);

	memcpy(u, fgm->u_0, sizeof_u);
	memcpy(w, u, sizeof_u);
	memcpy(u_old, u, sizeof_u);

	for (j = 0; j < *(fgm->j_in); j++) {
		inc_fgm_compute_grad_over_L(fgm, gradoL, w);
		inc_fgm_minimize_qp_iteration(fgm, u, u_old, w, gradoL);
	}

	return;
}

/* Execute one iteration of the minimization algorithm */
void inc_fgm_minimize_qp_iteration(const struct mpc_fgm *fgm, real_t u[],
		real_t u_old[], real_t w[],	const real_t gradoL[]) {
	static real_t du[MPC_HOR_INPUTS];
	static real_t nu_du[MPC_HOR_INPUTS];
	const size_t sizeof_u = MPC_HOR_INPUTS * sizeof(*u);

	inc_fgm_compute_projected_grad_step(fgm, u, w, gradoL);
	mtx_substract(du, u, u_old, MPC_HOR_INPUTS, 1);
	mtx_scale(nu_du, du, *(fgm->nu), MPC_HOR_INPUTS, 1);
	mtx_add(w, u, nu_du, MPC_HOR_INPUTS, 1);
	memcpy(u_old, u, sizeof_u);

	return;
}

/* Compute gradient divided by Lipschitz constant */
void inc_fgm_compute_grad_over_L(const struct mpc_fgm *fgm, real_t gradoL[],
								const real_t w[]) {
	real_t HoL_w[MPC_HOR_INPUTS];
	/* gradoL = (H/L) * w + (G/L) * x */
	mtx_multiply_mtx_vec(HoL_w, fgm->HoL, w, MPC_HOR_INPUTS, MPC_HOR_INPUTS);
	mtx_add(gradoL, HoL_w, fgm->gxoL, MPC_HOR_INPUTS, 1);

	return;
}

/* compute gradient vector, referenced to the origin, for the current state */
void inc_compute_gxoL(struct mpc_fgm *fgm, const real_t x[]) {
	mtx_multiply_mtx_vec(fgm->gxoL, fgm->GoL, x, MPC_HOR_INPUTS, MPC_STATES);
	return;
}

/* Copy the contents of the input vector, except for the first element, to the
 * output vector. The last element of the output vector is equal to the zero.
 */
void mpc_warmstart_vector(struct mpc_fgm *fgm, real_t outvec[], const real_t vec[],
		const uint32_t unit_size, const uint32_t hor)
{
	uint32_t i;
	/* shift input vector one horizon step backwards */
	for (i=0; i < (hor - 1) * unit_size; i++) {
		outvec[i] = vec[i + unit_size];
	}

	/* zero the las element */
	for (i=(hor -1) * unit_size; i < hor * unit_size; i++) {
		outvec[i] = 0.;
	}
	return;
}

/* static functions definition */

static void inc_fgm_compute_projected_grad_step(const struct mpc_fgm *fgm,
		real_t u[], const real_t w[],
		const real_t gradoL[]) {
	mtx_substract(u, w, gradoL, MPC_HOR_INPUTS, 1);
	mtx_saturate_vec(u, fgm->u_lb, fgm->u_ub, MPC_HOR_INPUTS);

	return;
}
