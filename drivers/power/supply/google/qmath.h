/*
 * Copyright 2018 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef QMATH_H_
#define QMATH_H_

#include <linux/types.h>

typedef s32 qnum_t;
typedef	s64 qnumd_t;
typedef	u32 qnumu_t;
typedef	s64 qnumud_t;

#define QNUM_BITS	(sizeof(qnum_t)*8)
/* integer part */
#define QNUM_IBITS	8
/* fractional part and mask */
#define QNUM_FBITS	(QNUM_BITS - QNUM_IBITS)
#define QNUM_FMASK	(((qnum_t)1 << QNUM_FBITS) - 1)

#define qnum_rconst(R) \
	((qnum_t)(((R) * ((qnumd_t)1 << QNUM_FBITS))\
		+ ((R) * ((R) >= 0 ? 1 : -1) / 2)))

#define qnum_fromint(I) ((qnumd_t)(I) << QNUM_FBITS)
/* battery raw capacity is in Q8_8 */
#define qnum_from_q8_8(Q8_8) ((qnumd_t)(Q8_8) << (QNUM_FBITS-8))

/* truncate */
#define qnum_toint(F) ((int)((F) >> QNUM_FBITS))
/* round the number */
#define qnum_roundint(F, P) \
	qnum_toint(F + qnum_rconst(P))


static inline qnum_t qnum_mul(qnum_t A, qnum_t B)
{
	return (((qnumd_t)A * (qnumd_t)B) >> QNUM_FBITS);
}

static inline qnum_t qnum_div(qnum_t A, qnum_t B)
{
	return (((qnumd_t)A << QNUM_FBITS) / (qnumd_t)B);
}


/* 1, 2, 3, and 4 digits */
#define qnum_fracpart(A) ((qnum_t)(A) & QNUM_FMASK)

#define QNUM_FRACBITS(A)  \
	(((qnum_fracpart(A) << QNUM_IBITS)) & (((qnumud_t)1 << QNUM_BITS)-1))

#define QNUM_FRACn(A, n) \
	(((QNUM_FRACBITS(A) * n) >> QNUM_BITS) % n)

#define QNUM_FRAC1(A)	QNUM_FRACn(A, 10)
#define QNUM_FRAC2(A)	QNUM_FRACn(A, 100)
#define QNUM_FRAC3(A)	QNUM_FRACn(A, 1000)
#define QNUM_FRAC4(A)	QNUM_FRACn(A, 10000)


#define _PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

/* print as %d.%0<num>d */
#define _STR_(x)	# x
#define QNUM_FDGT_NUM		2
#define QNUM_CSTR_FMT		"%d.%02d"
#define QNUM_CSTR_SZ		(4 + 1 + QNUM_FDGT_NUM + 1)
#define qnum_nfracdgt(A, n)	_PRIMITIVE_CAT(QNUM_FRAC, n)(A)
#define qnum_fracdgt(A)		((int)qnum_nfracdgt(A, QNUM_FDGT_NUM))
#endif  /* QMATH_H_ */
