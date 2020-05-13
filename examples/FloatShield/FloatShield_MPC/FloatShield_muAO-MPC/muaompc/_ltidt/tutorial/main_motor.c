#include <stdio.h>  /* printf */
#include "cmpc/include/mpc.h"  /* the auto-generated code */

/* This file is a test of the C routines of the ALM+FGM MPC
 * algorithm. The same routines can be used in real systems.
 */
int main(void)
{
	real_t x[MPC_STATES];  /* current state of the system */
	extern struct mpc_ctl ctl;  /* already defined */

	ctl.conf->in_iter = 10;  /* number of iterations */

	/* The current state */
	x[0] = 0.1;
	x[1] = -0.5;

	/* Solve MPC problem and print the first element of input sequence */
	mpc_ctl_solve_problem(&ctl, x);  /* solve the MPC problem */
	printf("u[0] = %f \n", ctl.u_opt[0]);
	printf("\n SUCCESS! \n");

	return 0;
}
