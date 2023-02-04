/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DT_BINDINGS_CLOCK_ABC_H
#define _DT_BINDINGS_CLOCK_ABC_H

/*AON*/
#define FOUT_PLL_AON			1
#define MOUT_AON_PLL_AON		2
#define DOUT_AON_PLL_AON_CLK		3
#define DOUT_AON_SHARED_DIV_AON_PLL	4
#define DOUT_AON_SHARED_DIV_MIF		5
#define AON_NR_CLK			6

/*CORE*/
#define CORE_NR_CLK			1

/*FSYS*/
#define FSYS_NR_CLK			1

/*IPU*/
#define FOUT_PLL_IPU			1
#define MOUT_IPU_PLL_IPU		2
#define DOUT_IPU_IPU_PLL_CLK_DIV_1	3
#define MOUT_IPU_AONCLK_PLLCLK1		4
#define CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU 5
#define IPU_NR_CLK			6

/*MIF*/
#define MIF_NR_CLK			1

/*TPU*/
#define FOUT_PLL_TPU			1
#define MOUT_TPU_PLL_TPU		2
#define DOUT_TPU_TPU_PLL_DIV_CLK_1	3
#define MOUT_TPU_AONCLK_PLLCLK		4
#define CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU 5
#define TPU_NR_CLK			6

#endif
