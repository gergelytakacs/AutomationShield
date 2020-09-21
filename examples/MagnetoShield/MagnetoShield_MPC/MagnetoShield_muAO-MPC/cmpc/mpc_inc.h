#ifndef MPC_INC_H
#define MPC_INC_H

#include "mpc_base.h"
#include "mtx_ops.h"

/* Quadratic Programming online solver using Fast gradients.
 * Most of these functions are intended for internal use.
 */

extern void inc_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[]);

extern void inc_ctl_form_qp(struct mpc_ctl *ctl, const real_t x[]);

extern void inc_ctl_warmstart(struct mpc_ctl *ctl);

extern void inc_fgm_minimize_qp_iteration(const struct mpc_fgm *fgm,
		real_t u[], real_t u_old[], real_t w[], const real_t gradoL[]);

extern void inc_fgm_minimize_qp(const struct mpc_fgm *fgm, real_t u[]);

extern void inc_fgm_compute_grad_over_L(const struct mpc_fgm *fgm,
		real_t gradoL[], const real_t w[]);

extern void inc_compute_gxoL(struct mpc_fgm *fgm, const real_t x[]);

extern void mpc_warmstart_vector(struct mpc_fgm *fgm, real_t outvec[],
    const real_t vec[], const uint32_t unit_size, const uint32_t hor);

#endif /* MPC_INC_H */
