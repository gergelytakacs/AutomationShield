#include "mpc_ref.h"
#include "mpc_inc.h"
#include "mpc_const.h"

/* Declaration of static functions */
void ref_compute_Rh_u_ref(struct mpc_ctl *ctl,
		real_t Rh_u_ref[], const real_t u_ref[]);
void ref_compute_Qh_x_ref(struct mpc_ctl *ctl,
		real_t Qh_x_ref[], const real_t x_ref[]);

/* Definition of external functions */

/* Compute gradient vector over Lipschitz for the given state and reference */
void ref_compute_gxoL(struct mpc_ctl *ctl, const real_t x[], 
						const real_t x_ref[], const real_t u_ref[])
{
	static real_t GoL_x[MPC_HOR_INPUTS];

	ref_compute_groL(ctl, x_ref, u_ref);
	mtx_multiply_mtx_vec(GoL_x, ctl->alm->fgm->GoL, x, MPC_HOR_INPUTS, MPC_STATES);
	mtx_substract(ctl->alm->fgm->gxoL, GoL_x, ctl->alm->fgm->groL, 
					MPC_HOR_INPUTS, 1);
	return;
}

/* Compute gradient vector (over Lipschitz) component for the given reference */ 
void ref_compute_groL(struct mpc_ctl *ctl,
					 const real_t x_ref[], const real_t u_ref[])
{
	static real_t Rh_u_ref[MPC_HOR_INPUTS];  /* Rh * u_ref */
	static real_t Qh_x_ref[MPC_HOR_STATES];  /* Qh * x_ref */
	static real_t Bh_T_Qh_x_ref[MPC_HOR_INPUTS];  /* Bh.T * Qh * x_ref */
	static real_t gr[MPC_HOR_INPUTS];  /* gradient vector component of reference */

	ref_compute_Rh_u_ref(ctl, Rh_u_ref, u_ref);
	ref_compute_Qh_x_ref(ctl, Qh_x_ref, x_ref);
	mtx_multiply_mtx_vec(Bh_T_Qh_x_ref, ctl->alm->fgm->Bh_T, Qh_x_ref,
			MPC_HOR_INPUTS, MPC_HOR_STATES);
	mtx_add(gr, Bh_T_Qh_x_ref, Rh_u_ref, MPC_HOR_INPUTS, 1);
	mtx_scale(ctl->alm->fgm->groL, gr, *(ctl->alm->Linv), MPC_HOR_INPUTS, 1);

	return;
}

/* TODO make a "thread safe" version of ref_compute_groL, such that it allows
 * the computation of groL for fixed references, i.e. it will not be called
 * at every sampling time, only once a change of operation point is required.
 * _computing_ groL is time consuming, and should be carried out by a low
 * priority thread, however _setting_ it must be made _atomically_ in the 
 * MPC controller hard real-time thread. 
 */

/* Definition of static functions */

/* compute Rh * u_ref
 * Rh is a block diagonal matrix of R */
void ref_compute_Rh_u_ref(struct mpc_ctl *ctl, real_t Rh_u_ref[],
		const real_t u_ref[])
{
	uint32_t i, ini;

	for (i=0; i<MPC_HOR; i++) {
		ini = i * MPC_INPUTS;
		mtx_multiply_mtx_vec(Rh_u_ref + ini, ctl->wmx->R, u_ref + ini,
				MPC_INPUTS, MPC_INPUTS);
	}
}

/* compute Qh * x_ref
 * Qh is a block diagonal matrix of Q, the last block is P */
void ref_compute_Qh_x_ref(struct mpc_ctl *ctl, real_t Qh_x_ref[],
		const real_t x_ref[])
{
	uint32_t i, ini;

	for (i=0; i<MPC_HOR; i++) {
		ini = i * MPC_STATES;
		if (i < (MPC_HOR - 1)) {
			mtx_multiply_mtx_vec(Qh_x_ref + ini, ctl->wmx->Q, x_ref + ini,
					MPC_STATES, MPC_STATES);
		} else {
			mtx_multiply_mtx_vec(Qh_x_ref + ini, ctl->wmx->P, x_ref + ini,
					MPC_STATES, MPC_STATES);
		}
	}
}
