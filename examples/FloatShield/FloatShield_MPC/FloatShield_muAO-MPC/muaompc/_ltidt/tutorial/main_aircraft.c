#include <stdio.h>  /* printf */
#include "cmpc/include/mpc.h"  /* the auto-generated code */

/* This file is a test of the C routines of the ALM+FGM MPC
 * algorithm. The same routines can be used in real systems.
 */
int main(void)
{
	real_t x[MPC_STATES];  /* current state of the system */
	extern struct mpc_ctl ctl;  /* already defined */
	int i;  /* loop iterator */
	int j;  /* print state iterator */
	int s = 40;  /* number of simulation steps */

	ctl.conf->in_iter = 24;  /* iterations internal loop */
	ctl.conf->ex_iter = 2;  /* iterations external loop */
	ctl.conf->warmstart = 1;  /* warmstart each iteration */

	/* The current state */
	x[0] = 0.;
	x[1] = 0.;
	x[2] = 0.;
	x[3] = -400.;
	x[4] = 0.;

	for (i = 0; i < s; i++) {
		/* Solve and simulate MPC problem */
		mpc_ctl_solve_problem(&ctl, x);	/* solve the MPC problem */
		mpc_predict_next_state(&ctl, x);
		/* print first element of input sequence and predicted state */
		printf("\n step: %d  - ", i);
		printf("u[0] = %f; ", ctl.u_opt[0]);
		for (j = 0; j < MPC_STATES; j++) {
			printf("x[%d] = %f; ", j, x[j]);
		}
	}
	printf("\n SUCCESS! \n");

	return 0;
}
