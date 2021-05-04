
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include "../cmpc/mpc.h"
#include "../cmpc/fip_ops.h"
#include "../cmpc/mc04types.h"
#include "../cmpc/mpc_base.h"
#include "../cmpc/mpc_const.h"
#include "../cmpc/mpc_inc.h"
#include "../cmpc/mpc_ref.h"
#include "../cmpc/mpc_stc.h"
#include "../cmpc/mpc_stdint.h"
#include "../cmpc/mtx_ops.h"

extern struct mpc_ctl ctl;
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 4
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void Magneto_MPC_Outputs_wrapper(const real32_T *u,
			real32_T *y)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */

mpc_ctl_solve_problem(&ctl, u);
*y = ctl.u_opt[0];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


