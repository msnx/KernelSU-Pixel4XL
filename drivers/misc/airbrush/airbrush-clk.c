/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *
 * Author: Raman Kumar Banka <raman.k2@samsung.com>
 *
 * Clock controller for airbrush state manager
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/airbrush-sm-notifier.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/mfd/abc-pcie.h>

#include "airbrush-clk.h"
#include "airbrush-regs.h"

#define GAT_CLK_BLK_IPU_UID_IPU_IPCLKPORT_CLK_IPU	0x1024202c
#define GAT_CLK_BLK_TPU_UID_TPU_IPCLKPORT_CLK_TPU	0x10042034

#define AB_SM_93_312_MHZ	93312000
#define AB_SM_466_56_MHZ	466560000
#define AB_SM_789_6_MHZ		789600000
#define AB_SM_933_12_MHZ	933120000
#define AON_CLK_RATE_REG		0x10B10100
#define AON_CLK_RATE_19_2_MHZ	0x80F30500
#define AON_CLK_RATE_933_12_MHZ	0xA0F30510
#define TPU_CLK_RATE_REG		0x10040120
#define TPU_CLK_RATE_19_2_MHZ	0xA1490202
#define TPU_CLK_RATE_789_6_MHZ	0xA1490212

#define AB_PLL_LOCK_TIMEOUT 1000

#define AB_SM_CLK_RESET		0xFFFFFFFF
#define AB_CLK_AON_PLL_MAX_DIV	9

static void __ab_aon_clk_div_2(struct ab_clk_context *ctx);
static void __ab_aon_clk_div_2_restore(struct ab_clk_context *ctx);
static void __ab_aon_clk_div_10(struct ab_clk_context *ctx);
static void __ab_aon_clk_div_10_restore(struct ab_clk_context *ctx);

static struct ab_sm_clk_ops clk_ops;
static int ab_clk_pcie_link_listener(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct ab_clk_context *clk_ctx = container_of(nb,
			struct ab_clk_context, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		mutex_lock(&clk_ctx->pcie_link_lock);
		clk_ctx->pcie_link_ready = true;
		mutex_unlock(&clk_ctx->pcie_link_lock);
		return NOTIFY_OK;
	}

	/*
	 * Handling ABC_PCIE_LINK_ERROR is the same with handling
	 * ABC_PCIE_LINK_PRE_DISABLE at this moment.
	 *
	 * If the handling is changed to access registers in the future,
	 * handling of the 2 flags need to be split.
	 */
	if (action & (ABC_PCIE_LINK_PRE_DISABLE | ABC_PCIE_LINK_ERROR)) {
		mutex_lock(&clk_ctx->pcie_link_lock);
		clk_ctx->pcie_link_ready = false;
		mutex_unlock(&clk_ctx->pcie_link_lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;  /* Don't care */
}

#define PLL_LOCKTIME_PLL_IPU		0x00240000
#define PLL_CON0_PLL_IPU			0x00240120
#define PLL_IPU_MUX_SEL_MASK		0x00000010
#define PLL_IPU_LOCK_FAIL_MASK		0x08000000
#define PLL_IPU_STABLE_MASK			0x20000000
#define PLL_IPU_USE_LOCK_FAIL_MASK	0x40000000
#define PLL_IPU_ENABLE_MASK			0x80000000
#define PLL_IPU_LOCK_FAILURES \
	(PLL_IPU_LOCK_FAIL_MASK | PLL_IPU_USE_LOCK_FAIL_MASK)

#define PLL_IPU_50MHZ_RATE		50000000
#define PLL_IPU_271MHZ_RATE		271800000
#define PLL_IPU_408MHZ_RATE		408000000
#define PLL_IPU_543MHZ_RATE		543600000
#define PLL_IPU_680MHZ_RATE		680000000

#define PLL_IPU_PMS_MASK		0x03FF3F07
#define PLL_IPU_50MHZ_PMS		0x01F40306
#define PLL_IPU_271MHZ_PMS		0x01C50204
#define PLL_IPU_408MHZ_PMS		0x01540203
#define PLL_IPU_543MHZ_PMS		0x01C50203
#define PLL_IPU_680MHZ_PMS		0x01A90302

#define AB_PLL_LOCKTIME		0x0000032A

static uint32_t get_ipu_pms_val(struct ab_clk_context *clk_ctx,
		uint32_t last_val, u64 new_rate)
{
	switch (new_rate) {
	case PLL_IPU_50MHZ_RATE:
		return (last_val & ~PLL_IPU_PMS_MASK) | PLL_IPU_50MHZ_PMS;
	case PLL_IPU_271MHZ_RATE:
		return (last_val & ~PLL_IPU_PMS_MASK) | PLL_IPU_271MHZ_PMS;
	case PLL_IPU_408MHZ_RATE:
		return (last_val & ~PLL_IPU_PMS_MASK) | PLL_IPU_408MHZ_PMS;
	case PLL_IPU_543MHZ_RATE:
		return (last_val & ~PLL_IPU_PMS_MASK) | PLL_IPU_543MHZ_PMS;
	case PLL_IPU_680MHZ_RATE:
		return (last_val & ~PLL_IPU_PMS_MASK) | PLL_IPU_680MHZ_PMS;
	default:
		dev_err(clk_ctx->dev, "Bad clock rate, using %lu\n",
				PLL_IPU_680MHZ_RATE);
		return (last_val & ~PLL_IPU_PMS_MASK) | PLL_IPU_680MHZ_PMS;
	}
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_ipu_start_rate_change(
		struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate,
		bool *locked)
{
	uint32_t val, pms_val, last_val;

	dev_dbg(clk_ctx->dev,
		"%s: set IPU clock rate to %llu\n", __func__, new_rate);

	ab_sm_start_ts(AB_SM_TS_IPU_PRE_RC_NOTIFY);
	ab_sm_clk_notify(AB_IPU_PRE_RATE_CHANGE, old_rate, new_rate);
	ab_sm_record_ts(AB_SM_TS_IPU_PRE_RC_NOTIFY);

	/* Get current state of main IPU clk register */
	ab_sm_start_ts(AB_SM_TS_IPU_GET_CLK);
	if (clk_ctx->last_ipu_val == AB_SM_CLK_RESET)
		ABC_READ(PLL_CON0_PLL_IPU, &clk_ctx->last_ipu_val);
	last_val = clk_ctx->last_ipu_val;
	ab_sm_record_ts(AB_SM_TS_IPU_GET_CLK);

	if (new_rate == AB_SM_OSC_RATE || new_rate == 0) {
		/* Set pll_ipu clock source to OSCCLK_AON */
		ab_sm_start_ts(AB_SM_TS_IPU_SET_OSCCLK);
		val = last_val & ~PLL_IPU_MUX_SEL_MASK;
		if (new_rate == 0)
			val &= ~PLL_IPU_ENABLE_MASK;
		ABC_WRITE(PLL_CON0_PLL_IPU, val);
		last_val = val;
		ab_sm_record_ts(AB_SM_TS_IPU_SET_OSCCLK);

		ab_sm_clk_notify(AB_IPU_POST_RATE_CHANGE,
				old_rate, new_rate);
		*locked = true;
		clk_ctx->last_ipu_val = last_val;
		return new_rate;
	}

	/* If pms values aren't changing and PLL is enabled
	 * we can immediately switch to pll_ipu as parent
	 */
	ab_sm_start_ts(AB_SM_TS_IPU_SET_CLKRATE);
	pms_val = get_ipu_pms_val(clk_ctx, last_val, new_rate);
	if ((last_val & PLL_IPU_PMS_MASK) ==
			(pms_val & PLL_IPU_PMS_MASK) &&
			(last_val & PLL_IPU_ENABLE_MASK)) {
		val = last_val | PLL_IPU_MUX_SEL_MASK;
		ABC_WRITE(PLL_CON0_PLL_IPU, val);
		last_val = val;
		ab_sm_record_ts(AB_SM_TS_IPU_SET_CLKRATE);

		ab_sm_start_ts(AB_SM_TS_IPU_POST_RC_NOTIFY);
		ab_sm_clk_notify(AB_IPU_POST_RATE_CHANGE,
				old_rate, new_rate);
		ab_sm_record_ts(AB_SM_TS_IPU_POST_RC_NOTIFY);
		*locked = true;
		clk_ctx->last_ipu_val = last_val;
		return new_rate;
	}

	/* Set pll_ipu clock source to OSCCLK_AON so that intermediate
	 * changes don't propagate to children
	 * Disable pll_ipu
	 */
	val = last_val & ~(PLL_IPU_MUX_SEL_MASK | PLL_IPU_ENABLE_MASK);
	if (val != last_val) {
		ABC_WRITE(PLL_CON0_PLL_IPU, val);
		last_val = val;
	}

	ABC_WRITE(PLL_LOCKTIME_PLL_IPU, AB_PLL_LOCKTIME);

	/* Update pll_ipu pms values */
	val = get_ipu_pms_val(clk_ctx, last_val, new_rate);
	ABC_WRITE(PLL_CON0_PLL_IPU, val);
	last_val = val;

	/* Enable pll_ipu */
	val = last_val | PLL_IPU_ENABLE_MASK;
	ABC_WRITE(PLL_CON0_PLL_IPU, val);
	last_val = val;
	ab_sm_record_ts(AB_SM_TS_IPU_SET_CLKRATE);

	*locked = false;
	clk_ctx->last_ipu_val = last_val;
	return 0;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_ipu_finish_rate_change(
		struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	uint32_t val, timeout, last_val;

	/* Wait for pll_ipu pll lock*/
	ab_sm_start_ts(AB_SM_TS_IPU_CLK_LOCK);
	timeout = AB_PLL_LOCK_TIMEOUT;
	do {
		ABC_READ(PLL_CON0_PLL_IPU, &val);
	} while (!(val & PLL_IPU_STABLE_MASK) &&
			 !(val & PLL_IPU_LOCK_FAILURES) &&
			 --timeout);
	ab_sm_record_ts(AB_SM_TS_IPU_CLK_LOCK);

	if (val & (PLL_IPU_LOCK_FAILURES)) {
		dev_err(clk_ctx->dev, "ipu_pll lock failure\n");
		ab_sm_clk_notify(AB_IPU_ABORT_RATE_CHANGE, old_rate, new_rate);
		return -ETIME;
	}

	last_val = clk_ctx->last_ipu_val;

	/* Switch back to ipu_pll as parent */
	ab_sm_start_ts(AB_SM_TS_IPU_FINISH_SET_CLKRATE);
	val = last_val | PLL_IPU_MUX_SEL_MASK;
	ABC_WRITE(PLL_CON0_PLL_IPU, val);
	last_val = val;

	ab_sm_record_ts(AB_SM_TS_IPU_FINISH_SET_CLKRATE);
	ab_sm_start_ts(AB_SM_TS_IPU_POST_RC_NOTIFY);
	ab_sm_clk_notify(AB_IPU_POST_RATE_CHANGE, old_rate, new_rate);
	ab_sm_record_ts(AB_SM_TS_IPU_POST_RC_NOTIFY);
	clk_ctx->last_ipu_val = last_val;
	return new_rate;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_ipu_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	bool locked;

	dev_dbg(clk_ctx->dev,
		"%s: set IPU clock rate to %llu\n", __func__, new_rate);

	ret = __ab_clk_ipu_start_rate_change(clk_ctx,
			old_rate, new_rate, &locked);
	if (ret < 0 || locked)
		return ret;

	return __ab_clk_ipu_finish_rate_change(clk_ctx,
			old_rate, new_rate);
}

static int64_t ab_clk_ipu_set_rate_handler(void *ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_ipu_set_rate_handler(clk_ctx,
				old_rate, new_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

#define PLL_LOCKTIME_PLL_TPU		0x00040000
#define PLL_CON0_PLL_TPU			0x00040120
#define CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK1		0x00041000
#define MUX_SHARED_DIV_AON_PLL		0x00000000
#define MUX_TPU_PLL_DIV_CLK_1		0x00000001
#define PLL_TPU_MUX_SEL_MASK		0x00000010
#define PLL_TPU_LOCK_FAIL_MASK		0x08000000
#define PLL_TPU_STABLE_MASK			0x20000000
#define PLL_TPU_USE_LOCK_FAIL_MASK	0x40000000
#define PLL_TPU_ENABLE_MASK			0x80000000
#define PLL_TPU_LOCK_FAILURES \
	(PLL_TPU_LOCK_FAIL_MASK | PLL_TPU_USE_LOCK_FAIL_MASK)

#define PLL_TPU_50MHZ_RATE		50000000
#define PLL_TPU_316MHZ_RATE		316000000
#define PLL_TPU_474MHZ_RATE		474000000
#define PLL_TPU_632MHZ_RATE		632000000
#define PLL_TPU_790MHZ_RATE		789600000

#define PLL_TPU_PMS_MASK		0x03FF3F07
#define PLL_TPU_50MHZ_PMS		0x01F40306
#define PLL_TPU_316MHZ_PMS		0x018B0303
#define PLL_TPU_474MHZ_PMS		0x018B0203
#define PLL_TPU_632MHZ_PMS		0x018B0302
#define PLL_TPU_790MHZ_PMS		0x01490202
static uint32_t get_tpu_pms_val(struct ab_clk_context *clk_ctx,
		uint32_t last_val, u64 new_rate)
{
	switch (new_rate) {
	case PLL_TPU_50MHZ_RATE:
		return (last_val & ~PLL_TPU_PMS_MASK) | PLL_TPU_50MHZ_PMS;
	case PLL_TPU_316MHZ_RATE:
		return (last_val & ~PLL_TPU_PMS_MASK) | PLL_TPU_316MHZ_PMS;
	case PLL_TPU_474MHZ_RATE:
		return (last_val & ~PLL_TPU_PMS_MASK) | PLL_TPU_474MHZ_PMS;
	case PLL_TPU_632MHZ_RATE:
		return (last_val & ~PLL_TPU_PMS_MASK) | PLL_TPU_632MHZ_PMS;
	case PLL_TPU_790MHZ_RATE:
		return (last_val & ~PLL_TPU_PMS_MASK) | PLL_TPU_790MHZ_PMS;
	default:
		dev_err(clk_ctx->dev, "Bad clock rate, using %lu\n",
				PLL_TPU_790MHZ_RATE);
		return (last_val & ~PLL_TPU_PMS_MASK) | PLL_TPU_790MHZ_PMS;
	}
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_tpu_start_rate_change(
		struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate,
		bool *locked)
{
	uint32_t val, pms_val, last_val;

	ab_sm_start_ts(AB_SM_TS_TPU_PRE_RC_NOTIFY);
	ab_sm_clk_notify(AB_TPU_PRE_RATE_CHANGE, old_rate, new_rate);
	ab_sm_record_ts(AB_SM_TS_TPU_PRE_RC_NOTIFY);

	/* Get current state of main TPU clk register */
	ab_sm_start_ts(AB_SM_TS_TPU_GET_CLK);
	if (clk_ctx->last_tpu_val == AB_SM_CLK_RESET)
		ABC_READ(PLL_CON0_PLL_TPU, &clk_ctx->last_tpu_val);
	last_val = clk_ctx->last_tpu_val;
	ab_sm_record_ts(AB_SM_TS_TPU_GET_CLK);

	if (new_rate == AB_SM_OSC_RATE || new_rate == 0) {
		ab_sm_start_ts(AB_SM_TS_TPU_SET_OSCCLK);
		/* Set pll_tpu clock source to OSCCLK_AON */
		val = last_val & ~PLL_TPU_MUX_SEL_MASK;
		if (new_rate == 0)
			val &= ~PLL_TPU_ENABLE_MASK;
		ABC_WRITE(PLL_CON0_PLL_TPU, val);
		last_val = val;
		ab_sm_record_ts(AB_SM_TS_TPU_SET_OSCCLK);

		ab_sm_start_ts(AB_SM_TS_TPU_POST_RC_NOTIFY);
		ab_sm_clk_notify(AB_TPU_POST_RATE_CHANGE,
				old_rate, new_rate);
		ab_sm_record_ts(AB_SM_TS_TPU_POST_RC_NOTIFY);
		*locked = true;
		clk_ctx->last_tpu_val = last_val;
		return new_rate;
	}

	ab_sm_start_ts(AB_SM_TS_TPU_SET_CLKRATE);
	/* Switch mux parent to SHARED_DIV_AON_PLL to prevent droop */
	ABC_WRITE(CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK1, MUX_SHARED_DIV_AON_PLL);

	/* If pms values aren't changing and PLL is enabled
	 * we can immediately switch to pll_tpu as parent
	 */
	pms_val = get_tpu_pms_val(clk_ctx, last_val, new_rate);
	if ((last_val & PLL_TPU_PMS_MASK) ==
			(pms_val & PLL_TPU_PMS_MASK) &&
			(last_val & PLL_TPU_ENABLE_MASK)) {
		val = last_val | PLL_TPU_MUX_SEL_MASK;
		ABC_WRITE(PLL_CON0_PLL_TPU, val);
		last_val = val;

		/* Switch mux parent back to TPU_PLL_DIV_CLK_1 */
		ABC_WRITE(CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK1,
			MUX_TPU_PLL_DIV_CLK_1);
		ab_sm_record_ts(AB_SM_TS_TPU_SET_CLKRATE);
		ab_sm_start_ts(AB_SM_TS_TPU_POST_RC_NOTIFY);

		ab_sm_clk_notify(AB_TPU_POST_RATE_CHANGE,
				old_rate, new_rate);
		ab_sm_record_ts(AB_SM_TS_TPU_POST_RC_NOTIFY);
		*locked = true;
		clk_ctx->last_tpu_val = last_val;
		return new_rate;
	}

	/* Set pll_ipu clock source to OSCCLK_AON so that intermediate
	 * changes don't propagate to children
	 * Disable pll_tpu
	 */
	val = last_val & ~(PLL_TPU_MUX_SEL_MASK | PLL_TPU_ENABLE_MASK);
	if (val != last_val) {
		ABC_WRITE(PLL_CON0_PLL_TPU, val);
		last_val = val;
	}

	ABC_WRITE(PLL_LOCKTIME_PLL_TPU, AB_PLL_LOCKTIME);

	/* Update pll_tpu pms values */
	val = get_tpu_pms_val(clk_ctx, last_val, new_rate);
	ABC_WRITE(PLL_CON0_PLL_TPU, val);
	last_val = val;

	/* Enable pll_tpu */
	val = last_val | PLL_TPU_ENABLE_MASK;
	ABC_WRITE(PLL_CON0_PLL_TPU, val);
	last_val = val;
	ab_sm_record_ts(AB_SM_TS_TPU_SET_CLKRATE);

	*locked = false;
	clk_ctx->last_tpu_val = last_val;
	return 0;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_tpu_finish_rate_change(
		struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	uint32_t val, timeout, last_val;

	/* Wait for pll_tpu pll lock*/
	ab_sm_start_ts(AB_SM_TS_TPU_CLK_LOCK);
	timeout = AB_PLL_LOCK_TIMEOUT;
	do {
		ABC_READ(PLL_CON0_PLL_TPU, &val);
	} while (!(val & PLL_TPU_STABLE_MASK) &&
			 !(val & PLL_TPU_LOCK_FAILURES) &&
			 --timeout);
	ab_sm_record_ts(AB_SM_TS_TPU_CLK_LOCK);

	if (val & (PLL_TPU_LOCK_FAILURES)) {
		dev_err(clk_ctx->dev, "tpu_pll lock failure\n");
		ab_sm_clk_notify(AB_TPU_ABORT_RATE_CHANGE, old_rate, new_rate);
		clk_ctx->last_tpu_val = last_val;
		return -ETIME;
	}

	last_val = clk_ctx->last_tpu_val;

	ab_sm_start_ts(AB_SM_TS_TPU_FINISH_SET_CLKRATE);
	/* Switch back to ipu_pll as parent */
	val = last_val | PLL_TPU_MUX_SEL_MASK;
	ABC_WRITE(PLL_CON0_PLL_TPU, val);
	last_val = val;

	/* Switch mux parent back to TPU_PLL_DIV_CLK_1 */
	ABC_WRITE(CLK_CON_MUX_MOUT_TPU_AONCLK_PLLCLK1, MUX_TPU_PLL_DIV_CLK_1);

	ab_sm_record_ts(AB_SM_TS_TPU_FINISH_SET_CLKRATE);
	ab_sm_start_ts(AB_SM_TS_TPU_POST_RC_NOTIFY);
	ab_sm_clk_notify(AB_TPU_POST_RATE_CHANGE, old_rate, new_rate);
	ab_sm_record_ts(AB_SM_TS_TPU_POST_RC_NOTIFY);
	clk_ctx->last_tpu_val = last_val;
	return new_rate;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_tpu_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	bool locked;

	dev_dbg(clk_ctx->dev,
		"%s: set TPU clock rate to %llu\n", __func__, new_rate);

	ret = __ab_clk_tpu_start_rate_change(clk_ctx,
			old_rate, new_rate, &locked);
	if (ret < 0 || locked)
		return ret;

	return __ab_clk_tpu_finish_rate_change(clk_ctx,
			old_rate, new_rate);
}

static int64_t ab_clk_tpu_set_rate_handler(void *ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_tpu_set_rate_handler(clk_ctx,
				old_rate, new_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold clk_ctx->pcie_link_lock */
static int64_t __ab_clk_ipu_tpu_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_ipu_rate, u64 new_ipu_rate,
		u64 old_tpu_rate, u64 new_tpu_rate)
{
	int64_t ret;
	bool ipu_locked, tpu_locked;

	ret = __ab_clk_ipu_start_rate_change(clk_ctx,
			old_ipu_rate, new_ipu_rate, &ipu_locked);
	if (ret < 0)
		return ret;

	ret = __ab_clk_tpu_start_rate_change(clk_ctx,
			old_tpu_rate, new_tpu_rate, &tpu_locked);
	if (ret < 0) {
		/* Don't leave ipu in bad state */
		if (!ipu_locked)
			__ab_clk_ipu_finish_rate_change(clk_ctx,
					old_ipu_rate, new_ipu_rate);
		return ret;
	};

	if (ipu_locked && tpu_locked)
		return 0;

	if (!ipu_locked) {
		ret = __ab_clk_ipu_finish_rate_change(clk_ctx,
				old_ipu_rate, new_ipu_rate);
		if (ret) {
			/* Don't leave tpu in bad state */
			if (!tpu_locked)
				__ab_clk_tpu_finish_rate_change(clk_ctx,
						old_tpu_rate, new_tpu_rate);
			return ret;
		}
	}
	if (!tpu_locked) {
		ret = __ab_clk_tpu_finish_rate_change(clk_ctx,
					old_tpu_rate, new_tpu_rate);
		if (ret)
			return ret;
	}

	return 0;
}

static int64_t ab_clk_ipu_tpu_set_rate_handler(void *ctx,
		u64 old_ipu_rate, u64 new_ipu_rate,
		u64 old_tpu_rate, u64 new_tpu_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_ipu_tpu_set_rate_handler(clk_ctx,
				old_ipu_rate, new_ipu_rate,
				old_tpu_rate, new_tpu_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

/* Caller must hold clk_ctx->pcie_link_lock
 * This method handles the clock rate changes
 * for AON PLL
 */
static int64_t __ab_clk_aon_set_rate_handler(struct ab_clk_context *clk_ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret = 0;

	dev_dbg(clk_ctx->dev,
		"%s: set AON clock rate to %llu\n", __func__, new_rate);

	if (new_rate != AB_SM_OSC_RATE &&
			new_rate != AB_SM_93_312_MHZ &&
			new_rate != AB_SM_466_56_MHZ &&
			new_rate != AB_SM_933_12_MHZ)
		dev_warn(clk_ctx->dev,
			"Invalid AON clock rate requested, using %d instead\n",
			AB_SM_933_12_MHZ);

	if (old_rate == AB_SM_466_56_MHZ &&
			new_rate != AB_SM_466_56_MHZ)
		__ab_aon_clk_div_2_restore(clk_ctx);

	if (old_rate <= AB_SM_93_312_MHZ &&
			new_rate > AB_SM_93_312_MHZ)
		__ab_aon_clk_div_10_restore(clk_ctx);

	if (new_rate == AB_SM_OSC_RATE) {
		__ab_aon_clk_div_10(clk_ctx);
		ret |= ABC_WRITE(AON_CLK_RATE_REG, AON_CLK_RATE_19_2_MHZ);
		if (ret) {
			dev_err(clk_ctx->dev,
				"aon_pll_mux: set_parent failed(err %d)\n",
				ret);
			goto error_abort;
		}

		return new_rate;
	}

	ret |= ABC_WRITE(AON_CLK_RATE_REG, AON_CLK_RATE_933_12_MHZ);
	if (ret) {
		dev_err(clk_ctx->dev,
			"aon_pll_mux: set_parent failed(err %d)\n", ret);
		goto error_abort;
	}

	if (new_rate == AB_SM_466_56_MHZ)
		__ab_aon_clk_div_2(clk_ctx);

	if (new_rate == AB_SM_93_312_MHZ)
		__ab_aon_clk_div_10(clk_ctx);

	return new_rate;

error_abort:
	return ret;
}
static int64_t ab_clk_aon_set_rate_handler(void *ctx,
		u64 old_rate, u64 new_rate)
{
	int64_t ret;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (clk_ctx->pcie_link_ready) {
		ret = __ab_clk_aon_set_rate_handler(clk_ctx,
				old_rate, new_rate);
	} else {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		ret = -ENODEV;
	}
	mutex_unlock(&clk_ctx->pcie_link_lock);

	return ret;
}

#define CLK_CON_DIV_DIV2_PLLCLK_MIF 0x10511804
#define CLK_CON_DIV_DIV4_PLLCLK_TPU 0x10041800
#define CLK_CON_DIV_DIV4_PLLCLK_IPU 0x10241800
#define CLK_CON_DIV_DIV4_PLLCLK_FSYS 0x10711804
#define CLK_CON_DIV_DIV4_PLLCLK_CORE 0x10f11804

/* Only Bus clock are reduced by 2x */
static void __ab_aon_clk_div_2(struct ab_clk_context *ctx)
{
	dev_dbg(ctx->dev, "Reduce PLL_AON_CLK 2x\n");

	/* Multiply sub clocks by 2x to compensate /2 */
	ABC_WRITE(CLK_CON_DIV_DIV2_PLLCLK_MIF, 0x0);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_FSYS, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_CORE, 0x0);

	/* Divide PLL_AON_CLK by 2 */
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x1);
}

static void __ab_aon_clk_div_2_restore(struct ab_clk_context *ctx)
{
	dev_dbg(ctx->dev, "Restore PLL_AON_CLK\n");

	/* Restore default divider settings to undo 2x multiply */
	ABC_WRITE(CLK_CON_DIV_DIV2_PLLCLK_MIF, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_FSYS, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_CORE, 0x1);

	/* Restore default divider setting to undo 2x divide */
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x0);
}

/* All clocks derived from PLL_AON_CLK gets reduced by 5x
 * PLL_AON_CLK itself is reduced by 10x
 */
static void __ab_aon_clk_div_10(struct ab_clk_context *ctx)
{
	dev_dbg(ctx->dev, "Reduce PLL_AON_CLK 10x\n");

	/* Increase sub clocks to partially compensate /10 */
	ABC_WRITE(CLK_CON_DIV_DIV2_PLLCLK_MIF, 0x0);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_FSYS, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_CORE, 0x0);

	/* Divide PLL_AON_CLK by 10 */
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x9);
	/* Divide DIV4_PLLCLK (AON_PCLK) by 4 */
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK, 0xF);
}

static void __ab_aon_clk_div_10_restore(struct ab_clk_context *ctx)
{
	dev_dbg(ctx->dev, "Restore PLL_AON_CLK\n");

	/* Restore default divider settings to undo 10x multiply */
	ABC_WRITE(CLK_CON_DIV_DIV2_PLLCLK_MIF, 0x1);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_TPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_IPU, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_FSYS, 0x3);
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK_CORE, 0x1);

	/* Restore default divider setting to undo 10x divide */
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, 0x0);
	/* Restore DIV4_PLLCLK (AON_PCLK) to undo 4x divide */
	ABC_WRITE(CLK_CON_DIV_DIV4_PLLCLK, 0x3);

}

static int64_t ab_clk_aon_set_pll_div(void *ctx, uint32_t div)
{
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	if (div > AB_CLK_AON_PLL_MAX_DIV)
		return -EINVAL;

	mutex_lock(&clk_ctx->pcie_link_lock);
	if (!clk_ctx->pcie_link_ready) {
		dev_err(clk_ctx->dev,
				"%s: pcie link down during clk request\n",
				__func__);
		mutex_unlock(&clk_ctx->pcie_link_lock);
		return -ENODEV;
	}

	dev_dbg(clk_ctx->dev,
			"%s: set AON pll div to %llu\n", __func__, div);
	ABC_WRITE(CLK_CON_DIV_PLL_AON_CLK, div);

	mutex_unlock(&clk_ctx->pcie_link_lock);
	return 0;
}

static void ab_clk_clear_cache(void *ctx, enum block_name name)
{
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	switch (name) {
	case BLK_IPU:
		clk_ctx->last_ipu_val = AB_SM_CLK_RESET;
		break;
	case BLK_TPU:
		clk_ctx->last_tpu_val = AB_SM_CLK_RESET;
		break;
	default:
		dev_err(clk_ctx->dev,
				"Tried to clear clk cache for %d\n", name);
		WARN_ON(1);
		break;
	}
}

#define SHARED_DIV_AON_PLL_REG				0x10B11810
#define SHARED_DIV_AON_PLL_DIVRATIO_MASK	0xF
#define SHARED_DIV_AON_PLL_DIVRATIO_2		0x1

static void ab_clk_init(void *ctx)
{
	u32 val;
	struct ab_clk_context *clk_ctx = (struct ab_clk_context *)ctx;

	dev_dbg(clk_ctx->dev, "AB clock init\n");

	/* Set SHARED_DIV_AON_PLL to PLL_AON / 2 (nominally 466MHz)
	 */
	ABC_READ(SHARED_DIV_AON_PLL_REG, &val);
	val &= ~SHARED_DIV_AON_PLL_DIVRATIO_MASK;
	val |= SHARED_DIV_AON_PLL_DIVRATIO_2;
	ABC_WRITE(SHARED_DIV_AON_PLL_REG, val);

	ab_clk_clear_cache(clk_ctx, BLK_IPU);
	ab_clk_clear_cache(clk_ctx, BLK_TPU);
}

static struct ab_sm_clk_ops clk_ops = {
	.init = &ab_clk_init,
	.clear_cache = &ab_clk_clear_cache,

	.ipu_set_rate = &ab_clk_ipu_set_rate_handler,
	.tpu_set_rate = &ab_clk_tpu_set_rate_handler,
	.ipu_tpu_set_rate = &ab_clk_ipu_tpu_set_rate_handler,
	.aon_set_rate = &ab_clk_aon_set_rate_handler,
	.aon_set_pll_div = &ab_clk_aon_set_pll_div,
};

static int ab_clk_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct ab_clk_context *clk_ctx;

	clk_ctx = devm_kzalloc(dev, sizeof(struct ab_clk_context), GFP_KERNEL);
	if (!clk_ctx)
		return -ENOMEM;

	clk_ctx->dev = dev;

	mutex_init(&clk_ctx->pcie_link_lock);
	clk_ctx->pcie_link_ready = true;
	clk_ctx->pcie_link_blocking_nb.notifier_call =
			ab_clk_pcie_link_listener;
	ret = abc_register_pcie_link_blocking_event(
			&clk_ctx->pcie_link_blocking_nb);
	if (ret) {
		dev_err(dev,
				"failed to subscribe to PCIe blocking link event, ret %d\n",
				ret);
		return ret;
	}

	clk_ops.ctx = clk_ctx;
	ab_sm_register_clk_ops(&clk_ops);

	return 0;
}

static int ab_clk_remove(struct platform_device *pdev)
{
	ab_sm_unregister_clk_ops();

	return 0;
}

static const struct of_device_id ab_clk_of_match[] = {
	{ .compatible = "abc,airbrush-clk", },
	{ }
};

static struct platform_driver ab_clk_driver = {
	.probe = ab_clk_probe,
	.remove = ab_clk_remove,
	.driver = {
		.name = "ab-clk",
		.of_match_table = ab_clk_of_match,
	},
};
module_platform_driver(ab_clk_driver);
