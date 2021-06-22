#include "fip_ops.h"
#include "mpc_base.h"

#ifndef FRAC_BITS  /* defined (or not) in mpc_base.h */
#define FRAC_BITS 1  /* allows compilation */
#endif

enum {
	BITS_IN_BYTE = 8,
	ARCH_BITS = sizeof(float32_t) * BITS_IN_BYTE
};
const int32_t FILTER = (0xffffffff >> (ARCH_BITS - FRAC_BITS));

float32_t fip_fip2real(int32_t a)
{
	const float32_t factor = (float32_t) (1 << FRAC_BITS);
	int32_t ci = a >> FRAC_BITS;
	float32_t cd = (float32_t) (a & FILTER) / factor;
	return ci + cd;
}

int32_t fip_real2fip(float32_t a)
{
	const float32_t factor = (float32_t) (1 << FRAC_BITS);
	int32_t ci = (int32_t) a;
	int32_t cd = (int32_t) ((a - (float32_t) ci) * factor);
	return (ci << FRAC_BITS) + cd;
}

int32_t fip_mul(int32_t a, int32_t b)
{
	int64_t c;
	c = (uint64_t) a *(uint64_t) b;
	return (int32_t) (c >> FRAC_BITS);
}

int32_t fip_add(int32_t a, int32_t b)
{
	return a + b;
}

int32_t fip_sub(int32_t a, int32_t b)
{
	return a - b;
}

int32_t fip_div(int32_t n, int32_t d)
{
	int64_t cl;
	int64_t nl = (int64_t) n;
	int64_t dl = (int64_t) d;

	cl = (nl << ARCH_BITS) / dl;
	return (int32_t) (cl >> (ARCH_BITS - FRAC_BITS));
}

