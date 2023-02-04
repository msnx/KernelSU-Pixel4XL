/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk-provider.h>
#include <linux/of.h>
#include "clk.h"
#include <dt-bindings/clock/abc-clk.h>

/*********************CMU_AON****************************/
/* Register Offset definitions for CMU_AON (0x10B10000) */
#define PLL_LOCKTIME_PLL_AON	0x0
#define PLL_CON0_PLL_AON	0x100
#define CLK_CON_DIV_SHARED_DIV_AON_PLL	0x1810

/* List of registers in CMU_AON */
static const unsigned long aon_clk_regs[] = {
	PLL_LOCKTIME_PLL_AON,
	PLL_CON0_PLL_AON,
	CLK_CON_DIV_SHARED_DIV_AON_PLL,
};

/* PLL rate table for CMU_AON */
static const struct airbrush_pll_rate_table pll_aon_tbl[] = {
	PLL_F081XX_RATE(933000000, 243, 5, 0),
	PLL_F081XX_RATE(19200000, 288, 9, 5),
};

/* List of PLL clocks in CMU_AON */
static const struct airbrush_pll_clock aon_pll_clks[] = {
	PLL(pll_f0816x, FOUT_PLL_AON, "fout_pll_aon", "fin_pll_ab",
		PLL_LOCKTIME_PLL_AON, PLL_CON0_PLL_AON, pll_aon_tbl)
};

/* list of parent clocks for muxes in cmu_aon */
PNAME(mout_aon_aon_pll_p) = { "fin_pll_ab", "fout_pll_aon" };

/* List of mux clocks in CMU_AON */
static const struct airbrush_mux_clock aon_mux_clks[] = {
	MUX(MOUT_AON_PLL_AON, "mout_aon_pll_aon",
		mout_aon_aon_pll_p, PLL_CON0_PLL_AON, 4, 1),
};

/* List of div clocks in CMU_AON */
static const struct airbrush_div_clock aon_div_clks[] = {
	DIV_F(DOUT_AON_SHARED_DIV_AON_PLL, "dout_aon_shared_div_aon_pll",
		"mout_aon_pll_aon", CLK_CON_DIV_SHARED_DIV_AON_PLL,
		0, 4, CLK_IS_CRITICAL, 0),
};

static const struct airbrush_cmu_info aon_cmu_info = {
	.pll_clks	= aon_pll_clks,
	.nr_pll_clks	= ARRAY_SIZE(aon_pll_clks),
	.mux_clks	= aon_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(aon_mux_clks),
	.div_clks	= aon_div_clks,
	.nr_div_clks	= ARRAY_SIZE(aon_div_clks),
	.nr_clk_ids	= AON_NR_CLK,
	.clk_regs	= aon_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(aon_clk_regs),
};

void abc_clk_aon_init(struct device_node *np)
{
	airbrush_cmu_register_one(np, &aon_cmu_info);
}
EXPORT_SYMBOL(abc_clk_aon_init);

/*********************** CMU_IPU ************************/
/* Register Offset definitions for CMU_IPU (0x10240000) */
#define PLL_LOCKTIME_PLL_IPU	0x0
#define PLL_CON0_PLL_IPU	0x120
#define CLK_CON_MUX_MOUT_AONCLK_PLLCLK1	0x1000
#define CLK_CON_DIV_IPU_PLL_CLK_DIV_1	0x1808
#define CLK_CON_GAT_CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU	0x202c

static const unsigned long ipu_clk_regs[] = {
	PLL_LOCKTIME_PLL_IPU,
	PLL_CON0_PLL_IPU,
	CLK_CON_MUX_MOUT_AONCLK_PLLCLK1,
	CLK_CON_DIV_IPU_PLL_CLK_DIV_1,
	CLK_CON_GAT_CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU,
};

/* PLL rate table for CMU_IPU */
static const struct airbrush_pll_rate_table pll_ipu_tbl[] = {
	PLL_F081XX_RATE(849600000, 531, 3, 2),
	PLL_F081XX_RATE(680000000, 425, 3, 2),
	PLL_F081XX_RATE(609600000, 381, 3, 2),
	PLL_F081XX_RATE(549600000, 458, 2, 3),
	PLL_F081XX_RATE(543600000, 453, 2, 3),
	PLL_F081XX_RATE(440000000, 550, 3, 3),
	PLL_F081XX_RATE(408000000, 340, 2, 3),
	PLL_F081XX_RATE(330000000, 275, 2, 3),
	PLL_F081XX_RATE(271800000, 453, 2, 4),
	PLL_F081XX_RATE(220000000, 550, 3, 4),
	PLL_F081XX_RATE(50000000, 500, 3, 6),
	PLL_F081XX_RATE(19200000, 288, 9, 5),
};

/* PLL Clocks */
static const struct airbrush_pll_clock ipu_pll_clks[] = {
	PLL(pll_f0816x, FOUT_PLL_IPU, "fout_pll_ipu", "fin_pll_ab", PLL_LOCKTIME_PLL_IPU, PLL_CON0_PLL_IPU, pll_ipu_tbl)
};

/* list of parent clocks for muxes in cmu_ipu */
PNAME(mout_ipu_ipu_pll_p) = { "fin_pll_ab", "fout_pll_ipu" };
PNAME(mout_aonclk_pllclk1_p) = { "dout_aon_shared_div_aon_pll", "dout_ipu_ipu_pll_clk_div_1" };

/* PLL MUX Clocks*/
static const struct airbrush_mux_clock ipu_mux_clks[] = {
	MUX(MOUT_IPU_AONCLK_PLLCLK1, "mout_ipu_aonclk_pllclk1", mout_aonclk_pllclk1_p, CLK_CON_MUX_MOUT_AONCLK_PLLCLK1, 0, 1),
	MUX(MOUT_IPU_PLL_IPU, "mout_ipu_pll_ipu", mout_ipu_ipu_pll_p, PLL_CON0_PLL_IPU, 4, 1),
};

/* DIV Clocks */
static const struct airbrush_div_clock ipu_div_clks[] = {
	DIV(DOUT_IPU_IPU_PLL_CLK_DIV_1, "dout_ipu_ipu_pll_clk_div_1", "mout_ipu_pll_ipu", CLK_CON_DIV_IPU_PLL_CLK_DIV_1, 0, 4),
};

/* GATE Clocks */
static const struct airbrush_gate_clock ipu_gate_clks[] = {
	GATE(CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU,
		"clk_blk_ipu_uid_ipu_ipclkport_clk_ipu",
		"mout_ipu_aonclk_pllclk1",
		CLK_CON_GAT_CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU, 21,
		CLK_IGNORE_UNUSED, 0),
};

static const struct airbrush_cmu_info ipu_cmu_info = {
	.pll_clks	= ipu_pll_clks,
	.nr_pll_clks	= ARRAY_SIZE(ipu_pll_clks),
	.mux_clks	= ipu_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(ipu_mux_clks),
	.div_clks	= ipu_div_clks,
	.nr_div_clks	= ARRAY_SIZE(ipu_div_clks),
	.gate_clks	= ipu_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(ipu_gate_clks),
	.nr_clk_ids	= IPU_NR_CLK,
	.clk_regs	= ipu_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(ipu_clk_regs),
};

void abc_clk_ipu_init(struct device_node *np)
{
	airbrush_cmu_register_one(np, &ipu_cmu_info);
}
EXPORT_SYMBOL(abc_clk_ipu_init);

/************************ CMU_TPU ***********************/
/* Register Offset definitions for CMU_TPU (0x10040000) */
#define PLL_LOCKTIME_PLL_TPU	0x0
#define PLL_CON0_PLL_TPU	0x120
#define CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK	0x1000
#define CLK_CON_DIV_TPU_PLL_DIV_CLK_1	0x1814
#define CLK_CON_GAT_CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU	0x2034

static const unsigned long tpu_clk_regs[] = {
	PLL_LOCKTIME_PLL_TPU,
	PLL_CON0_PLL_TPU,
	CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK,
	CLK_CON_DIV_TPU_PLL_DIV_CLK_1,
	CLK_CON_GAT_CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU,
};

/* PLL rate table for CMU_TPU */
static const struct airbrush_pll_rate_table pll_tpu_tbl[] = {
	PLL_F081XX_RATE(1000000000, 625, 3, 2),
	PLL_F081XX_RATE(961600000, 601, 3, 2),
	PLL_F081XX_RATE(789600000, 329, 2, 2),
	PLL_F081XX_RATE(765600000, 319, 2, 2),
	PLL_F081XX_RATE(632000000, 395, 3, 2),
	PLL_F081XX_RATE(612800000, 383, 3, 2),
	PLL_F081XX_RATE(474000000, 395, 2, 3),
	PLL_F081XX_RATE(459600000, 383, 2, 3),
	PLL_F081XX_RATE(316000000, 395, 3, 3),
	PLL_F081XX_RATE(306400000, 383, 3, 3),
	PLL_F081XX_RATE(50000000, 500, 3, 6),
	PLL_F081XX_RATE(19200000, 288, 9, 5),
};

/* PLL Clocks*/
static const struct airbrush_pll_clock tpu_pll_clks[] = {
	PLL(pll_f0816x, FOUT_PLL_TPU, "fout_pll_tpu", "fin_pll_ab", PLL_LOCKTIME_PLL_TPU, PLL_CON0_PLL_TPU, pll_tpu_tbl),
};

/* list of parent clocks for muxes in cmu_tpu */
PNAME(mout_tpu_tpu_pll_p) = { "fin_pll_ab", "fout_pll_tpu" };
PNAME(mout_tpu_aonclk_pllclk_p) = { "dout_aon_shared_div_aon_pll", "dout_tpu_tpu_pll_div_clk_1" };

/* MUX Clocks*/
static const struct airbrush_mux_clock tpu_mux_clks[] = {
	MUX(MOUT_TPU_AONCLK_PLLCLK, "mout_tpu_aonclk_pllclk", mout_tpu_aonclk_pllclk_p, CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK, 0, 1),
	MUX(MOUT_TPU_PLL_TPU, "mout_tpu_pll_tpu", mout_tpu_tpu_pll_p, PLL_CON0_PLL_TPU, 4, 1),
};

/* DIV Clocks*/
static const struct airbrush_div_clock tpu_div_clks[] = {
	DIV(DOUT_TPU_TPU_PLL_DIV_CLK_1, "dout_tpu_tpu_pll_div_clk_1", "mout_tpu_pll_tpu", CLK_CON_DIV_TPU_PLL_DIV_CLK_1, 0, 4),
};

/* GATE Clocks*/
static const struct airbrush_gate_clock tpu_gate_clks[] = {
	GATE(CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU,
			"clk_blk_tpu_uid_tpu_ipclkport_clk_tpu",
			"mout_tpu_aonclk_pllclk",
			CLK_CON_GAT_CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU, 21,
			CLK_IGNORE_UNUSED, 0),
};

static const struct airbrush_cmu_info tpu_cmu_info = {
	.pll_clks	= tpu_pll_clks,
	.nr_pll_clks	= ARRAY_SIZE(tpu_pll_clks),
	.mux_clks	= tpu_mux_clks,
	.nr_mux_clks	= ARRAY_SIZE(tpu_mux_clks),
	.div_clks	= tpu_div_clks,
	.nr_div_clks	= ARRAY_SIZE(tpu_div_clks),
	.gate_clks	= tpu_gate_clks,
	.nr_gate_clks	= ARRAY_SIZE(tpu_gate_clks),
	.nr_clk_ids	= TPU_NR_CLK,
	.clk_regs	= tpu_clk_regs,
	.nr_clk_regs	= ARRAY_SIZE(tpu_clk_regs),
};

void abc_clk_tpu_init(struct device_node *np)
{
	airbrush_cmu_register_one(np, &tpu_cmu_info);
}
EXPORT_SYMBOL(abc_clk_tpu_init);
