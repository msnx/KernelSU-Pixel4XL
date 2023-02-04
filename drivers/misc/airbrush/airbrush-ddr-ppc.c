/*
 * Copyright (C) 2019 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Performance profiling driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

#define ABC_MSI_6_PPC_MIF		(6)

static bool ddr_ppc_is_event_valid(uint32_t event)
{
	switch (event) {
	case PPC_EVENT_WR_PORT0:
	case PPC_EVENT_RD_PORT0:
	case PPC_EVENT_WR_PORT1:
	case PPC_EVENT_RD_PORT1:
	case PPC_EVENT_WR_PORT2:
	case PPC_EVENT_RD_PORT2:
	case PPC_EVENT_WR_PORT3:
	case PPC_EVENT_RD_PORT3:
	case PPC_EVENT_WR2BANK0_PORT2:
	case PPC_EVENT_WR2BANK1_PORT2:
	case PPC_EVENT_WR2BANK2_PORT2:
	case PPC_EVENT_WR2BANK3_PORT2:
	case PPC_EVENT_WR_CAS:
	case PPC_EVENT_RD_CAS:
	case PPC_EVENT_RW_CAS:
	case PPC_EVENT_WR_SHED:
	case PPC_EVENT_RD_SHED:
	case PPC_EVENT_RW_SHED:
	case PPC_EVENT_WR_XFER:
	case PPC_EVENT_RD_XFER:
	case PPC_EVENT_RW_XFER:
	case PPC_EVENT_PD_CHIP0:
	case PPC_EVENT_PD_CHIP1:
	case PPC_EVENT_SREF_CHIP0:
	case PPC_EVENT_SREF_CHIP1:
	case PPC_EVENT_AXI_RD_REQ_PORT0:
	case PPC_EVENT_AXI_RD_REQ_PORT1:
	case PPC_EVENT_AXI_RD_REQ_PORT2:
	case PPC_EVENT_AXI_RD_REQ_PORT3:
	case PPC_EVENT_AXI_WR_REQ_PORT0:
	case PPC_EVENT_AXI_WR_REQ_PORT1:
	case PPC_EVENT_AXI_WR_REQ_PORT2:
	case PPC_EVENT_AXI_WR_REQ_PORT3:
		return true;
	default:
		return false;
	}
}

static int ddr_ppc_irq_callback(uint32_t irq, void *data)
{
	int i;
	uint32_t ppc_overflow_data;
	struct ddr_ppc_overflow_info *ppcinfo;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)data;

	pr_info("ddr ppc callback is called for irq %d\n", irq);

	if (!ddr_ctx) {
		pr_err("ddr ppc callback: invalid data received to callback");
		return -EINVAL;
	}

	ppcinfo = &ddr_ctx->ddr_ppc_info;
	ppc_overflow_data = ddr_reg_rd(ddr_ctx, DREX_FLAG_PCC);

	while (ppc_overflow_data) {
		if (ppc_overflow_data & PPC_CCNT_FLAG) {
			ddr_reg_set(ddr_ctx, DREX_FLAG_PCC, PPC_CCNT_FLAG);
			ppcinfo->overflow_count_cycle_cnt++;
		}

		for (i = 0; i < PPC_COUNTER_MAX; i++) {
			if (ppc_overflow_data & PPC_PMCNT_FLAG(i)) {
				ddr_reg_set(ddr_ctx,
					DREX_FLAG_PCC, PPC_PMCNT_FLAG(i));
				ppcinfo->overflow_count_cnt[i]++;
			}
		}

		ppc_overflow_data = ddr_reg_rd(ddr_ctx, DREX_FLAG_PCC);
	}

	return 0;
}

static void ddr_ppc_irq_register(struct ab_ddr_context *ddr_ctx)
{
	/* Register callback with callback function & MSI_IRQ Number */
	abc_reg_irq_callback(&ddr_ppc_irq_callback,
			     ABC_MSI_6_PPC_MIF, ddr_ctx);

	/* Enable the overflow interrupts */
	ddr_reg_wr(ddr_ctx, DREX_INTENS_PCC,
			PPC_PMCNT_INTSET(0) | PPC_PMCNT_INTSET(1) |
			PPC_PMCNT_INTSET(2) | PPC_PMCNT_INTSET(3) |
			PPC_CCNT_INTSET);
}

static void ddr_ppc_irq_deregister(struct ab_ddr_context *ddr_ctx)
{
	/* Disable the overflow interrupts */
	ddr_reg_wr(ddr_ctx, DREX_INTENC_PCC,
			PPC_PMCNT_INTCLR(0) | PPC_PMCNT_INTCLR(1) |
			PPC_PMCNT_INTCLR(2) | PPC_PMCNT_INTCLR(3) |
			PPC_CCNT_INTCLR);

	/* De-Register callback for ABC_MSI_6_PPC_MIF MSI interrupt */
	abc_reg_irq_callback(NULL, ABC_MSI_6_PPC_MIF, NULL);
}

/* Caller must hold ddr_ctx->ddr_lock */
static void ab_ddr_ppc_start(struct ab_ddr_context *ddr_ctx)
{
	int i;
	struct ddr_ppc_overflow_info *ppcinfo = &ddr_ctx->ddr_ppc_info;

	memset(ppcinfo, 0, sizeof(*ppcinfo));

	/* Performance Event Clock Enable */
	ddr_reg_set(ddr_ctx, DREX_PPCCLKCON, PEREV_CLK_EN);

	ddr_reg_wr(ddr_ctx, DREX_CNTENS_PCC, 0);
	ddr_reg_set(ddr_ctx, DREX_CNTENS_PCC, PPC_CCNT_ENABLE);

	for (i = 0; i < PPC_COUNTER_MAX; i++) {
		if (ddr_ppc_is_event_valid(ddr_ctx->ddr_ppc_events[i])) {
			ddr_reg_wr(ddr_ctx, DREX_PEREVCONFIG(i),
				   PEREVx_SEL(ddr_ctx->ddr_ppc_events[i]));
			ddr_reg_set(ddr_ctx, DREX_CNTENS_PCC,
					PPC_PMCNT_ENABLE(i));
		}
	}

	ddr_ppc_irq_register(ddr_ctx);

	ddr_reg_wr(ddr_ctx, DREX_PMNC_PCC, CYCLECNT_RESET | PPC_COUNTER_RESET);
	ddr_reg_set(ddr_ctx, DREX_PMNC_PCC, PPC_ENABLE);
}

/* Caller must hold ddr_ctx->ddr_lock */
static void ab_ddr_ppc_stop(struct ab_ddr_context *ddr_ctx)
{
	int i;
	struct ddr_ppc_overflow_info *ppcinfo = &ddr_ctx->ddr_ppc_info;

	ddr_reg_clr(ddr_ctx, DREX_PMNC_PCC, PPC_ENABLE);

	/* Print the overflow and event count information */
	pr_info("CCNT: overflow_cnt: %d, event_cnt: %u\n",
			ppcinfo->overflow_count_cycle_cnt,
			ddr_reg_rd(ddr_ctx, DREX_CCCNT_PPC));

	for (i = 0; i < PPC_COUNTER_MAX; i++) {
		pr_info("PMCNT[%d]: overflow_cnt: %d, event_cnt: %u\n",
			i, ppcinfo->overflow_count_cnt[i],
			ddr_reg_rd(ddr_ctx, DREX_PMCNT_PPC(i)));
	}

	/* Disable the count registers */
	ddr_reg_wr(ddr_ctx, DREX_CNTENC_PCC, PPC_PMCNT_DISABLE(0) |
				    PPC_PMCNT_DISABLE(1) |
				    PPC_PMCNT_DISABLE(2) |
				    PPC_PMCNT_DISABLE(3) |
				    PPC_CCNT_DISABLE);

	ddr_ppc_irq_deregister(ddr_ctx);

	ddr_reg_wr(ddr_ctx, DREX_PMNC_PCC, CYCLECNT_RESET | PPC_COUNTER_RESET);

	/* Performance Event Clock Disable */
	ddr_reg_clr(ddr_ctx, DREX_PPCCLKCON, PEREV_CLK_EN);
}

int ab_ddr_ppc_set_event(void *ctx, unsigned int counter_idx,
			 unsigned int event)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ppc set event: error!!  ddr setup is not called\n");
		return -EAGAIN;
	}

	if (counter_idx >= PPC_COUNTER_MAX) {
		pr_err("ppc set event: Invalid DDR PPC counter\n");
		return -EINVAL;
	}

	mutex_lock(&ddr_ctx->ddr_lock);
	/* set event counters */
	if (!ddr_ppc_is_event_valid(event))
		ddr_ctx->ddr_ppc_events[counter_idx] = -1;
	else
		ddr_ctx->ddr_ppc_events[counter_idx] = event;
	mutex_unlock(&ddr_ctx->ddr_lock);

	pr_info("[ddr ppc]: counter: %d, event: 0x%x\n", counter_idx, event);

	return 0;
}

void ab_ddr_ppc_ctrl(void *ctx, int ppc_start)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	static int prev_ppc_start;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr ppc ctrl: error!! ddr setup is not called\n");
		return;
	}

	/* don't allow ppc_stop without a previous ppc_start request */
	if (!ppc_start && !prev_ppc_start) {
		pr_err("ddr ppc ctrl: error!! ppc_start is not called\n");
		return;
	}

	mutex_lock(&ddr_ctx->ddr_lock);
	if (ppc_start)
		ab_ddr_ppc_start(ddr_ctx);
	else
		ab_ddr_ppc_stop(ddr_ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	prev_ppc_start = ppc_start;
}
