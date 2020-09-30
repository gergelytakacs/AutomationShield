#ifndef MPC_REF_H
#define MPC_REF_H

#include "mpc_base.h"

extern void ref_compute_gxoL(struct mpc_ctl *ctl, const real_t x[],
		 	 	 	const real_t x_ref[], const real_t u_ref[]);

extern void ref_compute_groL(struct mpc_ctl *ctl, 
					const real_t x_ref[], const real_t u_ref[]);

#endif /* MPC_REF_H */
