/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR VREF Training sequence.
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
#include "airbrush-regs.h"

#define DDR_DEFAULT_WRITE_VREF	0x5

uint32_t ddr_get_phy_vref(uint32_t idx)
{
	/* for vref voltage of PHY (13.5% ~ 46.1% of VDD2) */
	static const uint32_t phy_vref[PHY_VREF_LEVELS] = {
		0x0,  0x1,  0x2,  0x3,  0x4,  0x5,  0x6,  0x7,  0x8,  0x9,
		0xa,  0xb,  0xc,  0xd,  0xe,  0xf,  0x10, 0x11, 0x12, 0x13,
		0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
		0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
		0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
		0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
		0x3c, 0x3d, 0x3e, 0x3f
	};

	return phy_vref[idx];
}

uint32_t ddr_get_dram_vref(uint32_t idx)
{
	/* for vref voltage of DRAM (10.0% ~ 30.0%  of VDD2) */
	static const uint32_t dram_vref[DRAM_VREF_LEVELS] = {
		/* Range 0 */
		0x0,  0x1,  0x2,  0x3,  0x4,  0x5,  0x6,  0x7,  0x8,  0x9,
		0xa,  0xb,  0xc,  0xd,  0xe,  0xf,  0x10, 0x11, 0x12, 0x13,
		0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
		0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
		0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
		0x32,
		/* Range 1 */
		0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e,
		0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
		0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70,	0x71, 0x72
	};

	return dram_vref[idx];
}

void ddrphy_set_read_vref(struct ab_ddr_context *ddr_ctx,
		uint32_t vref_phy0, uint32_t vref_phy1,
				enum vref_byte_t byte)
{
	union dphy_zq_con9_t zq_con9;

	zq_con9.n = ddr_reg_rd(ddr_ctx, DPHY_ZQ_CON9);

	if (byte == VREF_BYTE0) {
		zq_con9.bits.zq_ds0_vref = vref_phy0;
	} else if (byte == VREF_BYTE1) {
		zq_con9.bits.zq_ds1_vref = vref_phy0;
	} else { /* VREF_BYTE_ALL or default */
		zq_con9.bits.zq_ds0_vref = vref_phy0;
		zq_con9.bits.zq_ds1_vref = vref_phy0;
	}

	ddr_reg_wr(ddr_ctx, DPHY_ZQ_CON9, zq_con9.n);

	zq_con9.n = ddr_reg_rd(ddr_ctx, DPHY2_ZQ_CON9);

	if (byte == VREF_BYTE0) {
		zq_con9.bits.zq_ds0_vref = vref_phy1;
	} else if (byte == VREF_BYTE1) {
		zq_con9.bits.zq_ds1_vref = vref_phy1;
	} else {    /* VREF_BYTE_ALL or default */
		zq_con9.bits.zq_ds0_vref = vref_phy1;
		zq_con9.bits.zq_ds1_vref = vref_phy1;
	}

	ddr_reg_wr(ddr_ctx, DPHY2_ZQ_CON9, zq_con9.n);
}

void ddrphy_set_write_vref(struct ab_ddr_context *ddr_ctx,
		uint32_t vref, enum vref_byte_t byte)
{
	/* issue MRW command for DRAM vref */
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, (0x8c << 9) | (vref << 2));
}

static uint32_t ddrphy_get_byte_svref(struct ab_ddr_context *ddr_ctx,
		enum vref_byte_t byte, enum phy_type_t phy)
{
	union dphy_zq_con9_t zq_con9;
	int addr_zq_con9 = phy ? DPHY2_ZQ_CON9 : DPHY_ZQ_CON9;

	zq_con9.n = ddr_reg_rd(ddr_ctx, addr_zq_con9);

	if (byte == VREF_BYTE1)
		return zq_con9.bits.zq_ds1_vref;
	else /* VREF_BYTE0 or default */
		return zq_con9.bits.zq_ds0_vref;
}

static int ddrphy_is_prbs_done(struct ab_ddr_context *ddr_ctx)
{
	return ((ddr_reg_rd(ddr_ctx, DPHY_PRBS_CON0) & 0x1) &&
		(ddr_reg_rd(ddr_ctx, DPHY2_PRBS_CON0) & 0x1));
}

static int32_t ddrphy_run_prbs_training(struct ab_ddr_context *ddr_ctx,
		enum vref_operation_t rw)
{
	union dphy_prbs_con0_t prbs_con0;
	union dphy_cal_con0_t cal_con0;
	unsigned long timeout;

	prbs_con0.n = ddr_reg_rd(ddr_ctx, DPHY_PRBS_CON0);
	if (rw == VREF_WRITE)
		prbs_con0.bits.prbs_write_start = 0x1;
	else /* VREF_READ or default */
		prbs_con0.bits.prbs_read_start = 0x1;

	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON0, prbs_con0.n);

	prbs_con0.n = ddr_reg_rd(ddr_ctx, DPHY2_PRBS_CON0);
	if (rw == VREF_WRITE)
		prbs_con0.bits.prbs_write_start = 0x1;
	else /* VREF_READ or default */
		prbs_con0.bits.prbs_read_start = 0x1;

	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON0, prbs_con0.n);

	/* wait for PRBS_DONE from both PHYs */
	timeout = jiffies + usecs_to_jiffies(VREF_PRBS_TIMEOUT_USEC);
	while (!ddrphy_is_prbs_done(ddr_ctx) && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if (!ddrphy_is_prbs_done(ddr_ctx))
		return VREF_PRBS_TIMEOUT;

	prbs_con0.n = ddr_reg_rd(ddr_ctx, DPHY_PRBS_CON0);
	if (rw == VREF_WRITE)
		prbs_con0.bits.prbs_write_start = 0x0;
	else /* VREF_READ or default */
		prbs_con0.bits.prbs_read_start = 0x0;

	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON0, prbs_con0.n);

	prbs_con0.n = ddr_reg_rd(ddr_ctx, DPHY2_PRBS_CON0);
	if (rw == VREF_WRITE)
		prbs_con0.bits.prbs_write_start = 0x0;
	else /* VREF_READ or default */
		prbs_con0.bits.prbs_read_start = 0x0;

	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON0, prbs_con0.n);

	/* wait for PRBS_DONE from both PHYs */
	timeout = jiffies + usecs_to_jiffies(VREF_PRBS_TIMEOUT_USEC);
	while (!ddrphy_is_prbs_done(ddr_ctx) && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if (!ddrphy_is_prbs_done(ddr_ctx))
		return VREF_PRBS_TIMEOUT;

	cal_con0.n = ddr_reg_rd(ddr_ctx, DPHY_CAL_CON0);
	cal_con0.bits.wrlvl_mode = 0x1;
	cal_con0.bits.ca_cal_mode = 0x1;
	ddr_reg_wr(ddr_ctx, DPHY_CAL_CON0, cal_con0.n);

	cal_con0.n = ddr_reg_rd(ddr_ctx, DPHY_CAL_CON0);
	cal_con0.bits.wrlvl_mode = 0x0;
	cal_con0.bits.ca_cal_mode = 0x0;
	ddr_reg_wr(ddr_ctx, DPHY_CAL_CON0, cal_con0.n);

	cal_con0.n = ddr_reg_rd(ddr_ctx, DPHY2_CAL_CON0);
	cal_con0.bits.wrlvl_mode = 0x1;
	cal_con0.bits.ca_cal_mode = 0x1;
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_CON0, cal_con0.n);

	cal_con0.n = ddr_reg_rd(ddr_ctx, DPHY2_CAL_CON0);
	cal_con0.bits.wrlvl_mode = 0x0;
	cal_con0.bits.ca_cal_mode = 0x0;
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_CON0, cal_con0.n);

	return VREF_PRBS_SUCCESS;
}

static uint32_t ddrphy_get_prbs_training_result(struct ab_ddr_context *ddr_ctx,
		enum vref_byte_t byte,
		enum phy_type_t phy)
{
	uint32_t byte0 = 0, byte1 = 0;
	union dphy_prbs_con6_t prbs_con6;
	union dphy_prbs_con7_t prbs_con7;
	int phy_base = phy ? DPHY2_BASE_ADDR : DPHY_BASE_ADDR;

	if (byte == VREF_BYTE0) {
		prbs_con6.n = ddr_reg_rd(ddr_ctx,
				phy_base + DPHY_OFFSET_PRBS_CON6);
		prbs_con7.n = ddr_reg_rd(ddr_ctx,
				phy_base + DPHY_OFFSET_PRBS_CON7);

		if (prbs_con6.bits.prbs_offset_left0 == 0x1ff)
			byte0 = 0;
		else
			byte0 = prbs_con6.bits.prbs_offset_left0;

		if (prbs_con7.bits.prbs_offset_right0 == 0x1ff)
			byte0 += 0;
		else
			byte0 += prbs_con7.bits.prbs_offset_right0;

		return byte0;

	} else if (byte == VREF_BYTE1) {
		prbs_con6.n = ddr_reg_rd(ddr_ctx,
				phy_base + DPHY_OFFSET_PRBS_CON6);
		prbs_con7.n = ddr_reg_rd(ddr_ctx,
				phy_base + DPHY_OFFSET_PRBS_CON7);

		if (prbs_con6.bits.prbs_offset_left1 == 0x1ff)
			byte1 = 0;
		else
			byte1 = prbs_con6.bits.prbs_offset_left1;

		if (prbs_con7.bits.prbs_offset_right1 == 0x1ff)
			byte1 += 0;
		else
			byte1 += prbs_con7.bits.prbs_offset_right1;

		return byte1;

	} else {
		prbs_con6.n = ddr_reg_rd(ddr_ctx,
				phy_base + DPHY_OFFSET_PRBS_CON6);
		prbs_con7.n = ddr_reg_rd(ddr_ctx,
				phy_base + DPHY_OFFSET_PRBS_CON7);

		if (prbs_con6.bits.prbs_offset_left0 == 0x1ff)
			byte0 = 0;
		else
			byte0 = prbs_con6.bits.prbs_offset_left0;

		if (prbs_con7.bits.prbs_offset_right0 == 0x1ff)
			byte0 += 0;
		else
			byte0 += prbs_con7.bits.prbs_offset_right0;

		if (prbs_con6.bits.prbs_offset_left1 == 0x1ff)
			byte1 = 0;
		else
			byte1 = prbs_con6.bits.prbs_offset_left1;

		if (prbs_con7.bits.prbs_offset_right1 == 0x1ff)
			byte1 += 0;
		else
			byte1 += prbs_con7.bits.prbs_offset_right1;

		if (byte0 < byte1)
			return byte0;
		else
			return byte1;
	}
}

/* Set and Clear PRBS offset value */
static void ddrphy_reset_prbs_training_result(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_wr(ddr_ctx, DPHY_CAL_CON3, 0xfc7f9840);
	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON4, 0x0);
	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON5, 0x0);
	ddr_reg_wr(ddr_ctx, DPHY_CAL_CON3, 0xfc7f9800);

	ddr_reg_wr(ddr_ctx, DPHY2_CAL_CON3, 0xfc7f9840);
	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON4, 0x0);
	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON5, 0x0);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_CON3, 0xfc7f9800);
}

static uint32_t ddrphy_sum_vref_training_prbs_result(uint32_t *vwm)
{
	uint32_t i;
	int32_t output = 0;

	for (i = 0; i < VREF_REF_NUM; i++)
		output += vwm[i];

	return output;
}

static void ddrphy_shift_prbs_result(uint32_t *vwm)
{
	uint32_t vwm_idx;

	for (vwm_idx = 1 ; vwm_idx < VREF_REF_NUM ; vwm_idx++)
		vwm[vwm_idx - 1] = vwm[vwm_idx];
}

static int32_t ddrphy_get_optimal_vref(struct ab_ddr_context *ddr_ctx,
		enum vref_operation_t rw,
		enum vref_byte_t byte)
{
	uint32_t vwm_phy0_vref[VREF_REF_NUM];
	uint32_t vwm_phy1_vref[VREF_REF_NUM];
	uint32_t vwm_sum_vref = 0;
	uint32_t vref_idx;
	uint32_t vwm_vref_idx = 0;
	int32_t  vref_at_max_sum_phy0 = -1;
	int32_t  vref_at_max_sum_phy1 = -1;
	uint32_t max_vwm_sum_phy0_vref = 0;
	uint32_t max_vwm_sum_phy1_vref = 0;
	uint32_t max_vref;
	uint32_t vref;
	int32_t optimal_vref;

	/* Initialize vwm_phy0_vref, vwm_phy1_vref to zeros */
	for (vref_idx = 0; vref_idx < VREF_REF_NUM ; vref_idx++) {
		vwm_phy0_vref[vref_idx] = 0;
		vwm_phy1_vref[vref_idx] = 0;
	}

	if (rw == VREF_WRITE)
		max_vref = DRAM_VREF_LEVELS;
	else
		max_vref = PHY_VREF_LEVELS;

	ddr_prbs_training_init(ddr_ctx);

	for (vref_idx = VREF_FROM; vref_idx < max_vref;
					vref_idx += VREF_STEP) {
		vref  = (rw == VREF_WRITE) ? ddr_get_dram_vref(vref_idx) :
					     ddr_get_phy_vref(vref_idx);
		if (rw == VREF_READ)
			ddrphy_set_read_vref(ddr_ctx, vref, vref, byte);
		else
			ddrphy_set_write_vref(ddr_ctx, vref, byte);

		/* Set and Clear PRBS offset value */
		ddrphy_reset_prbs_training_result(ddr_ctx);

		if (ddrphy_run_prbs_training(ddr_ctx, rw) != VREF_PRBS_SUCCESS)
			return VREF_ERROR;

		if (vref_idx < (VREF_FROM + VREF_REF_NUM - 1)) {
			vwm_phy0_vref[vwm_vref_idx] =
				ddrphy_get_prbs_training_result(ddr_ctx,
					byte, PHY0);
			vwm_phy1_vref[vwm_vref_idx] =
				ddrphy_get_prbs_training_result(ddr_ctx,
					byte, PHY1);
			vwm_vref_idx += 1;
		} else {
			vwm_phy0_vref[vwm_vref_idx] =
				ddrphy_get_prbs_training_result(ddr_ctx,
					byte, PHY0);
			vwm_phy1_vref[vwm_vref_idx] =
				ddrphy_get_prbs_training_result(ddr_ctx,
					byte, PHY1);
			vwm_sum_vref = ddrphy_sum_vref_training_prbs_result(
					&vwm_phy0_vref[0]);

			if (max_vwm_sum_phy0_vref < vwm_sum_vref) {
				max_vwm_sum_phy0_vref = vwm_sum_vref;
				vref_at_max_sum_phy0 = vref_idx;
			}
			ddrphy_shift_prbs_result(&vwm_phy0_vref[0]);

			vwm_sum_vref = ddrphy_sum_vref_training_prbs_result(
						&vwm_phy1_vref[0]);

			if (max_vwm_sum_phy1_vref < vwm_sum_vref) {
				max_vwm_sum_phy1_vref = vwm_sum_vref;
				vref_at_max_sum_phy1 = vref_idx;
			}

			ddrphy_shift_prbs_result(&vwm_phy1_vref[0]);
		}
	}

	if ((vref_at_max_sum_phy0 - (VREF_REF_NUM/2)) < 0)
		return VREF_ERROR;
	else if ((vref_at_max_sum_phy1 - (VREF_REF_NUM/2)) < 0)
		return VREF_ERROR;

	optimal_vref = (rw == VREF_WRITE) ?
	    ddr_get_dram_vref(vref_at_max_sum_phy0 - (VREF_REF_NUM/2)) :
	    ddr_get_phy_vref(vref_at_max_sum_phy0 - (VREF_REF_NUM/2));

	optimal_vref += ((rw == VREF_WRITE) ?
	    ddr_get_dram_vref(vref_at_max_sum_phy1 - (VREF_REF_NUM/2))  :
	    ddr_get_phy_vref(vref_at_max_sum_phy1 - (VREF_REF_NUM/2))) << 8;

	return optimal_vref;
}

int32_t ddrphy_run_vref_training(struct ab_ddr_context *ddr_ctx)
{
	int32_t optimal_vref;
	uint32_t sVref_byte0_phy0_init, sVref_byte1_phy0_init;
	uint32_t sVref_byte0_phy1_init, sVref_byte1_phy1_init;

	/* The below section of code is for Read Training of both Phy's */
	sVref_byte0_phy0_init =
		ddrphy_get_byte_svref(ddr_ctx, VREF_BYTE0, PHY0);
	sVref_byte1_phy0_init =
		ddrphy_get_byte_svref(ddr_ctx, VREF_BYTE1, PHY0);
	sVref_byte0_phy1_init =
		ddrphy_get_byte_svref(ddr_ctx, VREF_BYTE0, PHY1);
	sVref_byte1_phy1_init =
		ddrphy_get_byte_svref(ddr_ctx, VREF_BYTE1, PHY1);

	/* read vref training byte all */
	optimal_vref = ddrphy_get_optimal_vref(ddr_ctx, VREF_READ,
			VREF_BYTE_ALL);
	if (optimal_vref < 0 || ((optimal_vref >> 8) < 0)) {
		ddrphy_set_read_vref(ddr_ctx, sVref_byte0_phy0_init,
				sVref_byte0_phy1_init, VREF_BYTE0);
		ddrphy_set_read_vref(ddr_ctx, sVref_byte1_phy0_init,
				sVref_byte1_phy1_init, VREF_BYTE1);
		return VREF_ERROR;
	}
	ddrphy_set_read_vref(ddr_ctx, (optimal_vref & 0xff),
			((optimal_vref >> 8) & 0xff), VREF_BYTE_ALL);

	optimal_vref = ddrphy_get_optimal_vref(ddr_ctx, VREF_WRITE,
			VREF_BYTE_ALL);
	if (optimal_vref < 0) {
		ddrphy_set_write_vref(ddr_ctx, DDR_DEFAULT_WRITE_VREF,
				VREF_BYTE_ALL);
		return VREF_ERROR;
	}

	ddrphy_set_write_vref(ddr_ctx, (optimal_vref & 0xff), VREF_BYTE_ALL);

	ddrphy_reset_prbs_training_result(ddr_ctx);

	return VREF_SUCCESS;
}
