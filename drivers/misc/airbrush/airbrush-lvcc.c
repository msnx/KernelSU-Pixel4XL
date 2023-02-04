/*
 * Copyright (C) 2019 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *	Pankaj Kumar Dubey <pankaj.dubey@samsung.com>
 *
 * LVCC settings for airbrush
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */


#include <linux/airbrush-sm-ctrl.h>

#include "airbrush-regs.h"

#define ASV_INFO_REG_ADDR 0x8
#define ASV_TABLE_VERSION_MASK 0xFE0000
#define ASV_TABLE_VERSION_SHIFT 17
#define DVFS_DONE BIT(16)
#define DEFAULT_ASV_VERSION 0
#define MIN_ASV_VERSION 1
#define MAX_ASV_VERSION 2
#define NUM_GROUPS 16

#define IPU_RO_ADDR 0x3010
#define IPU_RO_MASK 0xFFFF
#define IPU_RO_SHIFT 0
#define IPU_SIGNOFF_VOLTAGE 750000

#define TPU_RO_ADDR 0x3018
#define TPU_RO_MASK 0xFFFF0000
#define TPU_RO_SHIFT 16
#define TPU_SIGNOFF_VOLTAGE 750000

static u32 ipu_ro_table[MAX_ASV_VERSION + 1][NUM_GROUPS + 1] = {
	[0] = { 6420, 6584, 6821, 7057, 7293,
		7529, 7765, 8002, 8238, 8420,
		   0,    0,    0,    0,    0,
		   0,    0 },
	[1] = { 6451, 6540, 6659, 6780, 6900,
		7020, 7142, 7260, 7383, 7504,
		7625, 7748, 7867, 7988, 8109,
		8230, 8532 },
	[2] = { 6451, 6540, 6659, 6780, 6900,
		7020, 7142, 7260, 7383, 7504,
		7625, 7748, 7867, 7988, 8109,
		8230, 8532 },
};

static u32 ipu_volt_table[MAX_ASV_VERSION + 1][NUM_GROUPS] = {
	[0] = { 775000, 768750, 756250, 743750, 731250,
		712500, 700000, 687500, 675000, 0,
		0,      0,      0,      0,      0,
		0 },
	[1] = { 781250, 775000, 768750, 762500, 756250,
		750000, 743750, 737500, 731250, 725000,
		712500, 706250, 700000, 693750, 687500,
		681250},
	[2] = { 781250, 775000, 768750, 762500, 756250,
		750000, 743750, 737500, 731250, 725000,
		712500, 706250, 700000, 693750, 687500,
		681250},
};

static u32 tpu_ro_table[MAX_ASV_VERSION + 1][NUM_GROUPS + 1] = {
	[0] = { 5130, 5255, 5434, 5614, 5794,
		5973, 6152, 6331, 6512, 6772,
		   0,    0,    0,    0,    0,
		   0,    0 },
	[1] = { 5062, 5150, 5250, 5350, 5450,
		5551, 5650, 5750, 5851, 5950,
		6051, 6150, 6250, 6350, 6450,
		6550, 6772 },
	[2] = { 5062, 5150, 5250, 5350, 5450,
		5551, 5650, 5750, 5851, 5950,
		6051, 6150, 6250, 6350, 6450,
		6550, 6772 },
};

static u32 tpu_volt_table[MAX_ASV_VERSION + 1][NUM_GROUPS] = {
	[0] = { 793750, 781250, 768750, 756250, 743750,
		731250, 712500, 700000, 687500, 0,
		0,      0,      0,      0,	0,
		0 },
	[1] = { 806250, 800000, 793750, 787500, 781250,
		775000, 768750, 762500, 756250, 750000,
		743750, 737500, 731250, 725000, 712500,
		706250},
	[2] = { 806250, 800000, 793750, 787500, 781250,
		775000, 768750, 762500, 756250, 750000,
		743750, 737500, 731250, 725000, 712500,
		706250},
};

u32 get_ipu_ro(void)
{
	u32 addr = ABC_BASE_OTP_WRAPPER + IPU_RO_ADDR;
	u32 val;

	ABC_READ(addr, &val);
	val = (val & IPU_RO_MASK) >> IPU_RO_SHIFT;
	return val;
}

static u32 get_ipu_volt(struct ab_asv_info *info)
{
	u32 i;
	u32 ipu_ro = info->ipu_ro;
	int asv_version = info->asv_version;

	for (i = 0; i < NUM_GROUPS; i++) {
		if (ipu_ro >= ipu_ro_table[asv_version][i] &&
				ipu_ro < ipu_ro_table[asv_version][i + 1])
			return ipu_volt_table[asv_version][i];
	}
	return IPU_SIGNOFF_VOLTAGE;
}

u32 get_tpu_ro(void)
{
	u32 addr = ABC_BASE_OTP_WRAPPER + TPU_RO_ADDR;
	u32 val;

	ABC_READ(addr, &val);
	val = (val & TPU_RO_MASK) >> TPU_RO_SHIFT;
	return val;
}

static u32 get_tpu_volt(struct ab_asv_info *info)
{
	u32 i;
	u32 tpu_ro = info->tpu_ro;
	int asv_version = info->asv_version;

	for (i = 0; i < NUM_GROUPS; i++) {
		if (tpu_ro >= tpu_ro_table[asv_version][i] &&
				tpu_ro < tpu_ro_table[asv_version][i + 1])
			return tpu_volt_table[asv_version][i];
	}
	return TPU_SIGNOFF_VOLTAGE;
}

static void find_asv_version(struct ab_asv_info *info)
{
	u32 addr = ABC_BASE_OTP_WRAPPER + ASV_INFO_REG_ADDR;
	u32 val;

	ABC_READ(addr, &val);

	info->fusing_done = val & DVFS_DONE;

	if (info->fusing_done)
		info->asv_version = (val & ASV_TABLE_VERSION_MASK) >>
			ASV_TABLE_VERSION_SHIFT;
	else
		info->asv_version = DEFAULT_ASV_VERSION;
}

/* Updates voltage based on asv_version.  Does not read OTP. */
static void update_asv_voltage(struct ab_asv_info *info)
{
	if (info->asv_version < MIN_ASV_VERSION ||
			info->asv_version > MAX_ASV_VERSION) {
		info->ipu_volt = IPU_SIGNOFF_VOLTAGE;
		info->tpu_volt = TPU_SIGNOFF_VOLTAGE;
	} else {
		info->ipu_volt = get_ipu_volt(info);
		info->tpu_volt = get_tpu_volt(info);
	}

	info->last_volt = 0;
}

/* This function reads OTP.  Caller must make sure link is up. */
void ab_lvcc_init(struct ab_asv_info *info)
{
	find_asv_version(info);
	info->ipu_ro = get_ipu_ro();
	info->tpu_ro = get_tpu_ro();
	update_asv_voltage(info);
}

void set_asv_version(struct ab_asv_info *info, int asv_version)
{
	info->asv_version = asv_version;
	update_asv_voltage(info);
}

int ab_lvcc(struct ab_state_context *sc, int chip_state)
{
	int ret;
	int smps_volt;
	struct ab_asv_info *info = &sc->asv_info;

	if (AB_SM_STATE_IN_RANGE(chip_state, CHIP_STATE_500) ||
			AB_SM_STATE_IN_RANGE(chip_state, CHIP_STATE_800))
		smps_volt = info->ipu_volt;
	else if (AB_SM_STATE_IN_RANGE(chip_state, CHIP_STATE_600) ||
		 AB_SM_STATE_IN_RANGE(chip_state, CHIP_STATE_700))
		smps_volt = info->tpu_volt;
	else
		smps_volt = max(info->ipu_volt, info->tpu_volt);

	if (smps_volt == info->last_volt)
		return 0;

	dev_info(sc->dev, "asv_ver:%d, ipu_ro:%d ipu_uv:%d, tpu_ro:%d tpu_uv:%d, final_uv:%d\n",
			info->asv_version,
			info->ipu_ro, info->ipu_volt,
			info->tpu_ro, info->tpu_volt,
			smps_volt);
	ret = regulator_set_voltage(sc->smps1, smps_volt,
				smps_volt + REGULATOR_STEP);
	if (!ret)
		info->last_volt = smps_volt;

	return ret;
}
