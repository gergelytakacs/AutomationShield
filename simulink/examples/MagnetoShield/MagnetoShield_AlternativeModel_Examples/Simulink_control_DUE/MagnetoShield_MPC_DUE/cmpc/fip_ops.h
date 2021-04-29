/** Provide 32-bit fixed-point arithmetic (experimental).
 *
 */
#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include "mc04types.h"

/* converts the input in fixed-point format into single precision float */
float32_t fip_fip2real(int32_t a);
/* converts the input in single precision float format into fixed-point */
int32_t fip_real2fip(float32_t a);
/* returns the first argument multiplied by the second in fixed-point format */
int32_t fip_mul(int32_t a, int32_t b);
/* returns the first argument plus the second in fixed-point format */
int32_t fip_add(int32_t a, int32_t b);
/* returns the first argument minus the second one in fixed-point format */
int32_t fip_sub(int32_t a, int32_t b);
/* returns the first argument divided by the second in fixed-point format */
int32_t fip_div(int32_t n, int32_t d);

#endif
