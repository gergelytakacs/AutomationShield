#ifndef MTX_OPS_H
#define MTX_OPS_H

#include "mpc_base.h"  /* typedefs */

/* matrix-vector multiplication: pout = pmtx * pvec.
 * pmtx has size (rows x columns) */
extern void mtx_multiply_mtx_vec(real_t pout[], const real_t pmtx[],
		const real_t pvec[],
		const uint32_t rows,
		const uint32_t cols);

/* element-wise matrix factorisation: pout = pmtx * factor.
 * pmtx has size (rows x columns) */
extern void mtx_scale(real_t pout[], const real_t pmtx[],
		const real_t factor, const uint32_t rows,
		const uint32_t cols);

/* matrix addition: pmtxc = pmta + pmtxb.
 * pmta, pmtb, pmtc have size (rows x columns) */
extern void mtx_add(real_t pmtxc[], const real_t pmtxa[],
		const real_t pmtxb[], const uint32_t rows,
		const uint32_t cols);

/* matrix subtraction: pmtxc = pmta - pmtxb.
 * pmta, pmtb, pmtc have size (rows x columns) */
extern void mtx_substract(real_t pmtxc[], const real_t pmtxa[],
		const real_t pmtxb[], const uint32_t rows,
		const uint32_t cols);

/* elementwise 2-sided vector saturation. 
 * lower[i] < vec[i] < upper[i]. pvec has size (rows x 1) */
extern void mtx_saturate_vec(real_t pvec[], const real_t plower[],
		const real_t pupper[],
		const uint32_t rows);

/* matrix transposition. mtxout[j,i] = mtxin[i,j].
 * mtxin has size (rows x columns) */
extern void mtx_transpose(real_t * mtxout, const real_t * mtxin,
		const uint32_t rows, const uint32_t cols);

/* elementwise vector maximum against zero
 * pmax[i] = max(pa[i], 0). pmax has size (rows x 1) */
extern void mtx_max_vec_zero(real_t pmax[],	const uint32_t rows);

/* elementwise vector minimum against zero
 *  pmin[i] = min(pa[i], 0). pmin has size (rows x 1) */
extern void mtx_min_vec_zero(real_t pmin[], const uint32_t rows);

#endif
