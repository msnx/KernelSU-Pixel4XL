/*
 * Copyright (C) 2011 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * Copyright (C) 2011 Richard Zhao, Linaro <richard.zhao@linaro.org>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Adjustable divider clock implementation
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/log2.h>

#include "clk.h"

/*
 * DOC: basic adjustable divider clock that cannot gate
 *
 * Traits of this clock:
 * prepare - clk_prepare only ensures that parents are prepared
 * enable - clk_enable only ensures that parents are enabled
 * rate - rate is adjustable.  clk->rate = ceiling(parent->rate / divisor)
 * parent - fixed parent.  No clk_set_parent support
 */

#define to_clk_divider(_hw) container_of(_hw, struct clk_divider, hw)

#define div_mask(width)	((1 << (width)) - 1)

static unsigned int _get_table_div(const struct clk_div_table *table,
							unsigned int val)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->val == val)
			return clkt->div;
	return 0;
}

static unsigned int _get_div(const struct clk_div_table *table,
			     unsigned int val, unsigned long flags, u8 width)
{
	if (flags & CLK_DIVIDER_ONE_BASED)
		return val;
	if (flags & CLK_DIVIDER_POWER_OF_TWO)
		return 1 << val;
	if (flags & CLK_DIVIDER_MAX_AT_ZERO)
		return val ? val : div_mask(width) + 1;
	if (table)
		return _get_table_div(table, val);
	return val + 1;
}

static unsigned long clk_divider_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int val;

	if (airbrush_clk_readl(divider->reg, &val))
		return 0;

	val = val >> divider->shift;
	val &= div_mask(divider->width);

	return divider_recalc_rate(hw, parent_rate, val, divider->table,
				   divider->flags, divider->width);
}

static long clk_divider_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	int bestdiv;

	/* if read only, just return current value */
	if (divider->flags & CLK_DIVIDER_READ_ONLY) {
		if (airbrush_clk_readl(divider->reg, &bestdiv))
			return 0;

		bestdiv = bestdiv >> divider->shift;
		bestdiv &= div_mask(divider->width);
		bestdiv = _get_div(divider->table, bestdiv, divider->flags,
			divider->width);
		return DIV_ROUND_UP_ULL((u64)*prate, bestdiv);
	}

	return divider_round_rate(hw, rate, prate, divider->table,
				  divider->width, divider->flags);
}

static int clk_divider_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_divider *divider = to_clk_divider(hw);
	unsigned int value;
	unsigned long flags = 0;
	u32 val;
	int ret;

	value = divider_get_val(rate, parent_rate, divider->table,
				divider->width, divider->flags);

	if (divider->lock)
		spin_lock_irqsave(divider->lock, flags);
	else
		__acquire(divider->lock);

	ret = airbrush_clk_readl(divider->reg, &val);
	if (ret)
		goto release_lock;

	val &= ~(div_mask(divider->width) << divider->shift);

	val |= value << divider->shift;
	ret = airbrush_clk_writel(val, divider->reg);

release_lock:
	if (divider->lock)
		spin_unlock_irqrestore(divider->lock, flags);
	else
		__release(divider->lock);

	return ret;
}

const struct clk_ops airbrush_clk_divider_ops = {
	.recalc_rate = clk_divider_recalc_rate,
	.round_rate = clk_divider_round_rate,
	.set_rate = clk_divider_set_rate,
};
EXPORT_SYMBOL_GPL(airbrush_clk_divider_ops);

const struct clk_ops airbrush_clk_divider_ro_ops = {
	.recalc_rate = clk_divider_recalc_rate,
	.round_rate = clk_divider_round_rate,
};
EXPORT_SYMBOL_GPL(airbrush_clk_divider_ro_ops);

static struct clk *_register_divider(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 shift, u8 width,
		u8 clk_divider_flags, const struct clk_div_table *table,
		spinlock_t *lock)
{
	struct clk_divider *div;
	struct clk *clk;
	struct clk_init_data init = {};

	if (clk_divider_flags & CLK_DIVIDER_HIWORD_MASK) {
		if (width + shift > 16) {
			pr_warn("divider value exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the divider */
	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	if (clk_divider_flags & CLK_DIVIDER_READ_ONLY)
		init.ops = &airbrush_clk_divider_ro_ops;
	else
		init.ops = &airbrush_clk_divider_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name: NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_divider assignments */
	div->reg = reg;
	div->shift = shift;
	div->width = width;
	div->flags = clk_divider_flags;
	div->lock = lock;
	div->hw.init = &init;
	div->table = table;

	/* register the clock */
	clk = clk_register(dev, &div->hw);

	if (IS_ERR(clk))
		kfree(div);

	return clk;
}

/**
 * airbrush_clk_register_divider - register a divider clock with the clock framework
 * @dev: device registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @reg: register address to adjust divider
 * @shift: number of bits to shift the bitfield
 * @width: width of the bitfield
 * @clk_divider_flags: divider-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk *airbrush_clk_register_divider(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 shift, u8 width,
		u8 clk_divider_flags, spinlock_t *lock)
{
	return _register_divider(dev, name, parent_name, flags, reg, shift,
			width, clk_divider_flags, NULL, lock);
}
EXPORT_SYMBOL_GPL(airbrush_clk_register_divider);

/**
 * airbrush_clk_register_divider_table - register a table based divider clock with
 * the clock framework
 * @dev: device registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @flags: framework-specific flags
 * @reg: register address to adjust divider
 * @shift: number of bits to shift the bitfield
 * @width: width of the bitfield
 * @clk_divider_flags: divider-specific flags for this clock
 * @table: array of divider/value pairs ending with a div set to 0
 * @lock: shared register lock for this clock
 */
struct clk *airbrush_clk_register_divider_table(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 shift, u8 width,
		u8 clk_divider_flags, const struct clk_div_table *table,
		spinlock_t *lock)
{
	return _register_divider(dev, name, parent_name, flags, reg, shift,
			width, clk_divider_flags, table, lock);
}
EXPORT_SYMBOL_GPL(airbrush_clk_register_divider_table);

void airbrush_clk_unregister_divider(struct clk *clk)
{
	struct clk_divider *div;
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw)
		return;

	div = to_clk_divider(hw);

	clk_unregister(clk);
	kfree(div);
}
EXPORT_SYMBOL_GPL(airbrush_clk_unregister_divider);
