/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 * Copyright (c) 2013 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common Clock Framework support for all PLL's in Samsung platforms
*/

#ifndef __AIRBRUSH_CLK_PLL_H
#define __AIRBRUSH_CLK_PLL_H

enum airbrush_pll_type {
	pll_f0816x,
	pll_f0818x,
};

#define PLL_F081XX_RATE(_rate, _m, _p, _s)			\
	{							\
		.rate	=	(_rate),			\
		.mdiv	=	(_m),				\
		.pdiv	=	(_p),				\
		.sdiv	=	(_s),				\
	}

/* NOTE: Rate table should be kept sorted in descending order. */
struct airbrush_pll_rate_table {
	unsigned int rate;
	unsigned int pdiv;
	unsigned int mdiv;
	unsigned int sdiv;
	unsigned int kdiv;
	unsigned int afc;
	unsigned int mfr;
	unsigned int mrr;
	unsigned int vsel;
};

#endif /* __AIRBRUSH_CLK_PLL_H */
