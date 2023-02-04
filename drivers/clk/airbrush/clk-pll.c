/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * This file is based on drivers/clk/samsung/clk-pll.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file contains the utility functions to register the pll clocks.
*/

#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clkdev.h>
#include "clk.h"
#include "clk-pll.h"

struct airbrush_clk_pll {
	struct clk_hw		hw;
	void __iomem		*lock_reg;
	void __iomem		*con_reg;
	enum airbrush_pll_type	type;
	unsigned int		rate_count;
	const struct airbrush_pll_rate_table *rate_table;
};

#define to_clk_pll(_hw) container_of(_hw, struct airbrush_clk_pll, hw)

static const struct airbrush_pll_rate_table *airbrush_get_pll_settings(
				struct airbrush_clk_pll *pll, unsigned long rate)
{
	const struct airbrush_pll_rate_table  *rate_table = pll->rate_table;
	int i;

	for (i = 0; i < pll->rate_count; i++) {
		if (rate == rate_table[i].rate)
			return &rate_table[i];
	}

	return NULL;
}

static long airbrush_pll_round_rate(struct clk_hw *hw,
			unsigned long drate, unsigned long *prate)
{
	struct airbrush_clk_pll *pll = to_clk_pll(hw);
	const struct airbrush_pll_rate_table *rate_table = pll->rate_table;
	int i;

	/* Assumming rate_table is in descending order */
	for (i = 0; i < pll->rate_count; i++) {
		if (drate >= rate_table[i].rate)
			return rate_table[i].rate;
	}

	/* return minimum supported value */
	return rate_table[i - 1].rate;
}

/*
 * PLLF081XX Clock Type
 */
/* Maximum lock time can be 270 * PDIV cycles */
#define PLLF081XX_LOCK_FACTOR		(270)

#define PLLF081XX_MDIV_MASK		(0x3FF)
#define PLLF081XX_PDIV_MASK		(0x3F)
#define PLLF081XX_SDIV_MASK		(0x7)
#define PLLF081XX_LOCK_STAT_MASK	(0x1)
#define PLLF081XX_MDIV_SHIFT		(16)
#define PLLF081XX_PDIV_SHIFT		(8)
#define PLLF081XX_SDIV_SHIFT		(0)
#define PLLF081XX_LOCK_STAT_SHIFT	(29)
#define PLLF081XX_ENABLE_SHIFT		(31)

#define PLL_TIMEOUT 1000

static unsigned long airbrush_pllf081xx_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	struct airbrush_clk_pll *pll = to_clk_pll(hw);
	u32 mdiv, pdiv, sdiv, pll_con;
	u64 fvco = parent_rate;

	if (airbrush_clk_readl(pll->con_reg, &pll_con))
		return 0;

	mdiv = (pll_con >> PLLF081XX_MDIV_SHIFT) & PLLF081XX_MDIV_MASK;
	pdiv = (pll_con >> PLLF081XX_PDIV_SHIFT) & PLLF081XX_PDIV_MASK;
	sdiv = (pll_con >> PLLF081XX_SDIV_SHIFT) & PLLF081XX_SDIV_MASK;

	fvco *= mdiv;
	do_div(fvco, (pdiv << sdiv));

	return (unsigned long)fvco;
}

static inline bool airbrush_pllf081xx_mp_change(
		const struct airbrush_pll_rate_table *rate, u32 pll_con)
{
	u32 old_mdiv, old_pdiv;

	old_mdiv = (pll_con >> PLLF081XX_MDIV_SHIFT) & PLLF081XX_MDIV_MASK;
	old_pdiv = (pll_con >> PLLF081XX_PDIV_SHIFT) & PLLF081XX_PDIV_MASK;

	return (rate->mdiv != old_mdiv || rate->pdiv != old_pdiv);
}

static int airbrush_pllf081xx_enable(struct clk_hw *hw)
{
	struct airbrush_clk_pll *pll = to_clk_pll(hw);
	u32 tmp, timeout;

	timeout = PLL_TIMEOUT;

	airbrush_clk_readl(pll->con_reg, &tmp);
	tmp |= BIT(PLLF081XX_ENABLE_SHIFT);
	airbrush_clk_writel(tmp, pll->con_reg);

	/* wait_lock_time */
	do {
		cpu_relax();
		if (airbrush_clk_readl(pll->con_reg, &tmp))
			return -ENODEV;
	} while (!(tmp &
			(PLLF081XX_LOCK_STAT_MASK << PLLF081XX_LOCK_STAT_SHIFT))
			&& --timeout);

	if (!timeout) {
		pr_err("%s: waiting for PLL stable bit timed out\n", __func__);
		return -EBUSY;
	}

	return 0;
}

static void airbrush_pllf081xx_disable(struct clk_hw *hw)
{
	struct airbrush_clk_pll *pll = to_clk_pll(hw);
	u32 tmp;

	airbrush_clk_readl(pll->con_reg, &tmp);
	tmp &= ~BIT(PLLF081XX_ENABLE_SHIFT);
	airbrush_clk_writel(tmp, pll->con_reg);
}

static int airbrush_pllf081xx_set_rate(struct clk_hw *hw, unsigned long drate,
					unsigned long prate)
{
	struct airbrush_clk_pll *pll = to_clk_pll(hw);
	const struct airbrush_pll_rate_table *rate;
	u32 tmp;

	/* Get required rate settings from table */
	rate = airbrush_get_pll_settings(pll, drate);
	if (!rate) {
		pr_err("%s: Invalid rate : %lu for pll clk %s\n", __func__,
			drate, clk_hw_get_name(hw));
		return -EINVAL;
	}

	airbrush_pllf081xx_disable(hw);

	if (airbrush_clk_readl(pll->con_reg, &tmp))
		return -ENODEV;

	if (!(airbrush_pllf081xx_mp_change(rate, tmp))) {
		/* If only s change, change just s value only*/
		tmp &= ~(PLLF081XX_SDIV_MASK << PLLF081XX_SDIV_SHIFT);
		tmp |= rate->sdiv << PLLF081XX_SDIV_SHIFT;
		if (airbrush_clk_writel(tmp, pll->con_reg))
			return -ENODEV;

		return airbrush_pllf081xx_enable(hw);
	}

	/* Set PLL lock time. */
	if (airbrush_clk_writel(rate->pdiv * PLLF081XX_LOCK_FACTOR,
						pll->lock_reg))
			return -ENODEV;

	/* Change PLL PMS values */
	tmp &= ~((PLLF081XX_MDIV_MASK << PLLF081XX_MDIV_SHIFT) |
			(PLLF081XX_PDIV_MASK << PLLF081XX_PDIV_SHIFT) |
			(PLLF081XX_SDIV_MASK << PLLF081XX_SDIV_SHIFT));
	tmp |= (rate->mdiv << PLLF081XX_MDIV_SHIFT) |
			(rate->pdiv << PLLF081XX_PDIV_SHIFT) |
			(rate->sdiv << PLLF081XX_SDIV_SHIFT);
	if (airbrush_clk_writel(tmp, pll->con_reg))
		return -ENODEV;

	return airbrush_pllf081xx_enable(hw);
}

static const struct clk_ops airbrush_pllf081xx_clk_ops = {
	.recalc_rate = airbrush_pllf081xx_recalc_rate,
	.round_rate = airbrush_pll_round_rate,
	.set_rate = airbrush_pllf081xx_set_rate,
	.enable = airbrush_pllf081xx_enable,
	.disable = airbrush_pllf081xx_disable,
};

static const struct clk_ops airbrush_pllf081xx_clk_min_ops = {
	.recalc_rate = airbrush_pllf081xx_recalc_rate,
};

static void _airbrush_clk_register_pll(struct airbrush_clk_provider *ctx,
				const struct airbrush_pll_clock *pll_clk,
				void __iomem *base)
{
	struct airbrush_clk_pll *pll;
	struct clk *clk;
	struct clk_init_data init = {};
	int ret, len;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll) {
		pr_err("%s: could not allocate pll clk %s\n",
			__func__, pll_clk->name);
		return;
	}

	init.name = pll_clk->name;
	init.flags = pll_clk->flags;
	init.parent_names = &pll_clk->parent_name;
	init.num_parents = 1;

	if (pll_clk->rate_table) {
		/* find count of rates in rate_table */
		for (len = 0; pll_clk->rate_table[len].rate != 0; )
			len++;

		pll->rate_count = len;
		pll->rate_table = kmemdup(pll_clk->rate_table,
					pll->rate_count *
					sizeof(struct airbrush_pll_rate_table),
					GFP_KERNEL);
		WARN(!pll->rate_table,
			"%s: could not allocate rate table for %s\n",
			__func__, pll_clk->name);
	}

	switch (pll_clk->type) {
	/* clk_ops for f0816x and f0818x are similar */
	case pll_f0816x:
	case pll_f0818x:
		if (!pll->rate_table)
			init.ops = &airbrush_pllf081xx_clk_min_ops;
		else
			init.ops = &airbrush_pllf081xx_clk_ops;
		break;
	default:
		pr_warn("%s: Unknown pll type for pll clk %s\n",
			__func__, pll_clk->name);
	}

	pll->hw.init = &init;
	pll->type = pll_clk->type;
	pll->lock_reg = base + pll_clk->lock_offset;
	pll->con_reg = base + pll_clk->con_offset;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register pll clock %s : %ld\n",
			__func__, pll_clk->name, PTR_ERR(clk));
		kfree(pll);
		return;
	}

	airbrush_clk_add_lookup(ctx, clk, pll_clk->id);

	if (!pll_clk->alias)
		return;

	ret = clk_register_clkdev(clk, pll_clk->alias, pll_clk->dev_name);
	if (ret)
		pr_err("%s: failed to register lookup for %s : %d",
			__func__, pll_clk->name, ret);
}

void airbrush_clk_register_pll(struct airbrush_clk_provider *ctx,
			const struct airbrush_pll_clock *pll_list,
			unsigned int nr_pll, void __iomem *base)
{
	int cnt;

	for (cnt = 0; cnt < nr_pll; cnt++)
		_airbrush_clk_register_pll(ctx, &pll_list[cnt], base);
}
