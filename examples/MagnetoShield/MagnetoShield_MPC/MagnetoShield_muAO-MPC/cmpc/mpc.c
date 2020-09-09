#include <stddef.h>  /* NULL */
#include <string.h>  /* memcpy, sizeof */
#include "mpc.h"
#include "mpc_const.h"
/* FIXME: This is a dirty fix of the HOR_MXCONSTRS zero-length array problem */
#ifdef STATE_CONSTR
#define NO_MXCONSTR 0
#else
#define NO_MXCONSTR 1
#endif

/* static functions declaration */
static void inc_ref_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[]);
static void stc_ref_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[]);
static void inc_ref_form_qp(struct mpc_ctl *ctl, const real_t x[]);
static void stc_ref_form_qp(struct mpc_ctl *ctl, const real_t x[]);

/* external functions definition */

void mpc_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[])
{
  int i;
	if ((ctl->x_ref == NULL) && (ctl->u_ref == NULL)) { /* both pointer are zero */
		if (MPC_HOR_MXCONSTRS == NO_MXCONSTR) {
			inc_ctl_solve_problem(ctl, x);
		} else {
			stc_ctl_solve_problem(ctl, x);
		}
	} else { /* any or both references are given */
    /* if using nonstatic data, both pointers need to be not NULL */
    if (ctl->x_ref == NULL) {
      extern real_t ctl_x_ref[];
ctl->x_ref = ctl_x_ref;
;  /* only for static data generation */
      for (i=0; i<MPC_HOR_STATES; i++) {
        ctl->x_ref[i] = (real_t)0.;
      }
    }
    if (ctl->u_ref == NULL) {
      extern real_t ctl_u_ref[];
ctl->u_ref = ctl_u_ref;
; /* only for static data generation */
      for (i=0; i<MPC_HOR_INPUTS; i++) {
        ctl->u_ref[i] = (real_t)0.;
      }
    }
		if (MPC_HOR_MXCONSTRS == NO_MXCONSTR) {
			inc_ref_ctl_solve_problem(ctl, x);
		} else {
			stc_ref_ctl_solve_problem(ctl, x);
		}
	}

	if (ctl->conf->warmstart) {
		if (MPC_HOR_MXCONSTRS == NO_MXCONSTR) {
			inc_ctl_warmstart(ctl);
		} else {
			stc_ctl_warmstart(ctl);
		}
	}
	
	return;
}

void mpc_ctl_form_qp(struct mpc_ctl *ctl, const real_t x[])
{
	const uint32_t in_iter = ctl->conf->in_iter;
	const uint32_t ex_iter = ctl->conf->ex_iter;

	/* Form the appropriate QP without actually solving the MPC problem */
	ctl->conf->in_iter = 0;
	ctl->conf->ex_iter = 0;
	mpc_ctl_solve_problem(ctl, x);
	
	ctl->conf->in_iter = in_iter;
	ctl->conf->ex_iter = ex_iter;
	
	return;
}

void mpc_predict_next_state(struct mpc_ctl *ctl, real_t x[])
{
	real_t Adx[MPC_STATES], Bdu[MPC_STATES];

	/* compute x = Ad * x + Bd * u_opt; it overwrites x */
	mtx_multiply_mtx_vec(Adx, ctl->sys->Ad, x, MPC_STATES, MPC_STATES);
	mtx_multiply_mtx_vec(Bdu, ctl->sys->Bd, ctl->u_opt, MPC_STATES, MPC_INPUTS);
	mtx_add(x, Adx, Bdu, MPC_STATES, 1);

	return;
}

void mpc_generate_state_trj(struct mpc_ctl *ctl, real_t x[])
{
  uint32_t k; /* loop counter */
  const size_t sizeof_x = MPC_STATES * sizeof(*x);
  real_t *x_k, *x_k1;  /* pointers to current and next state in the sequence */
  real_t *u_k;  /* pointers to current input in sequence */
	real_t Adx[MPC_STATES], Bdu[MPC_STATES];

  memcpy(ctl->x_trj, x, sizeof_x);  /* x_0 = x */

  for (k=0; k<MPC_HOR; k++) {
    /* compute x_k1 = Ad*x_k + Bd*u_k, k=0,...,N-1 */
    x_k1 = &(ctl->x_trj[(k+1)*MPC_STATES]);
    x_k = &(ctl->x_trj[k*MPC_STATES]);
    u_k = &(ctl->u_opt[k*MPC_INPUTS]);

	mtx_multiply_mtx_vec(Adx, ctl->sys->Ad, x_k, MPC_STATES, MPC_STATES);
	mtx_multiply_mtx_vec(Bdu, ctl->sys->Bd, u_k, MPC_STATES, MPC_INPUTS);
	mtx_add(x_k1, Adx, Bdu, MPC_STATES, 1);

  }

	return;
}

/* Definition of static functions */

static void inc_ref_form_qp(struct mpc_ctl *ctl, const real_t x[]) {
	ref_compute_gxoL(ctl, x, ctl->x_ref, ctl->u_ref);

	return;
}

static void stc_ref_form_qp(struct mpc_ctl *ctl, const real_t x[]) {
	ref_compute_gxoL(ctl, x, ctl->x_ref, ctl->u_ref);
	stc_compute_state_constr_bound_online(ctl->alm, ctl->alm->zx_lb, x,
			ctl->alm->e_lb);
	stc_compute_state_constr_bound_online(ctl->alm, ctl->alm->zx_ub, x,
			ctl->alm->e_ub);

	return;
}

static void inc_ref_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[])
{
	inc_ref_form_qp(ctl, x);
	inc_fgm_minimize_qp(ctl->alm->fgm, ctl->u_opt);

	return;
}

static void stc_ref_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[])
{
	stc_ref_form_qp(ctl, x);
	stc_alm_minimize_qp(ctl->alm, ctl->u_opt, ctl->l_opt);

	return;
}
