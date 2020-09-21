#ifndef MPC_STC_H
#define MPC_STC_H

#include "mpc_base.h"

extern void stc_ctl_solve_problem(struct mpc_ctl *ctl, const real_t x[]);

extern void stc_ctl_form_qp(struct mpc_ctl *ctl, const real_t x[]);

extern void stc_ctl_warmstart(struct mpc_ctl *ctl);

extern void stc_alm_minimize_qp(struct mpc_alm *alm,
							real_t u_opt[], real_t l_opt[]);
extern void stc_compute_state_constr_bound_online(const struct mpc_alm *alm,
		real_t out[], const real_t x[], const real_t bound[]);

#endif /* MPC_STC_H */
