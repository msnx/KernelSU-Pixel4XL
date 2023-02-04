/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Initialization and Training sequence.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "ab-ddr: " fmt

#include <linux/airbrush-sm-ctrl.h>
#include <linux/airbrush-sm-notifier.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-ddr.h"
#include "airbrush-ddr-internal.h"
#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

/*
 * ----------------------------------------------------------------------------
 * NOTE: Naming conventions of functions used in this ddr driver
 * ----------------------------------------------------------------------------
 * 1] ab_ddr_* are the functions which are exposed to the other modules as
 *    interfaces or dram_ops. These functions have to acquire ddr_lock mutex
 *    before proceeding.
 * 2] __ab_ddr_* are the versions of ab_ddr_* functions, which doesn't acquire
 *    ddr_ctx->ddr_lock mutex.
 * 3] ddr_* are the internal functions of the ddr driver. These functions can
 *    be used by other files of the ddr driver but not exposed to the other
 *    modules.
 *
 * All the __ab_ddr_*() & ddr_*() functions are expected to be called only when
 * the ddr_lock is acquired by someone in the call hierarchy.
 * ----------------------------------------------------------------------------
 */

/* In case the DDR Initialization/Training OTPs are already fused to the OTP
 * array, then the below function pointer is updated with the address of the
 * function which reads OTPs from the fused OTP memory.
 *
 * In case the OTPs are not fused, this will point to the function which reads
 * the OTPs from the global array stored in the data segment.
 */
static uint32_t (*ddr_otp_rd)(uint32_t);

/* forward declarations */
static int ddr_config_post_initialization(struct ab_ddr_context *ddr_ctx);
static void ddr_refresh_control_wkqueue(struct work_struct *);

static const struct ddr_train_save_restore_t *
			get_train_save_restore_regs(unsigned int idx)
{
	static const struct ddr_train_save_restore_t
				train_save_restore_regs[] = {
		[s_DPHY_MDLL_CON1] = {
			DPHY_MDLL_CON1,
			DPHY_MDLL_CON1 },
		[s_DPHY_CA_DESKEW_CON0] = {
			DPHY_CA_DESKEW_CON0,
			DPHY_CA_DESKEW_CON0 },
		[s_DPHY_CA_DESKEW_CON1] = {
			DPHY_CA_DESKEW_CON1,
			DPHY_CA_DESKEW_CON1 },
		[s_DPHY_CA_DESKEW_CON2] = {
			DPHY_CA_DESKEW_CON2,
			DPHY_CA_DESKEW_CON2 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON_DM,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON_DM },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON0] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON0,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON0 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON1] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON1,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON1 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON2] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON2,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON2 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON3] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON3,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON3 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON4] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON4,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON4 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON5] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON5,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON5 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON6] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON6,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON6 },
		[s_DPHY_RD_DESKEW_CENTER_CS0_CON7] = {
			DPHY_RD_DESKEW_CENTER_CS0_CON7,
			DPHY_SW_RD_DESKEW_CENTER_CS0_CON7 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON_DM] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON_DM,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON_DM },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON0] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON0,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON0 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON1] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON1,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON1 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON2] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON2,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON2 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON3] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON3,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON3 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON4] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON4,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON4 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON5] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON5,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON5 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON6] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON6,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON6 },
		[s_DPHY_RD_DESKEW_CENTER_CS1_CON7] = {
			DPHY_RD_DESKEW_CENTER_CS1_CON7,
			DPHY_SW_RD_DESKEW_CENTER_CS1_CON7 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON_DM] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON_DM,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON_DM },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON0] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON0,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON0 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON1] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON1,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON1 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON2] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON2,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON2 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON3] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON3,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON3 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON4] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON4,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON4 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON5] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON5,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON5 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON6] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON6,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON6 },
		[s_DPHY_RD_DESKEW_LEFT_CS0_CON7] = {
			DPHY_RD_DESKEW_LEFT_CS0_CON7,
			DPHY_SW_RD_DESKEW_LEFT_CS0_CON7 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON_DM] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON_DM,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON_DM },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON0] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON0,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON0 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON1] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON1,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON1 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON2] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON2,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON2 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON3] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON3,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON3 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON4] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON4,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON4 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON5] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON5,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON5 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON6] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON6,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON6 },
		[s_DPHY_RD_DESKEW_LEFT_CS1_CON7] = {
			DPHY_RD_DESKEW_LEFT_CS1_CON7,
			DPHY_SW_RD_DESKEW_LEFT_CS1_CON7 },
		[s_DPHY_RD_DQS_VWMC_CS0_CON0] = {
			DPHY_RD_DQS_VWMC_CS0_CON0,
			DPHY_SW_RD_DQS_VWMC_CS0_CON0 },
		[s_DPHY_RD_DQS_VWMC_CS1_CON0] = {
			DPHY_RD_DQS_VWMC_CS1_CON0,
			DPHY_SW_RD_DQS_VWMC_CS1_CON0 },
		[s_DPHY_RD_DQS_VWML_CS0_CON0] = {
			DPHY_RD_DQS_VWML_CS0_CON0,
			DPHY_SW_RD_DQS_VWML_CS0_CON0 },
		[s_DPHY_RD_DQS_VWML_CS1_CON0] = {
			DPHY_RD_DQS_VWML_CS1_CON0,
			DPHY_SW_RD_DQS_VWML_CS1_CON0 },
		[s_DPHY_WR_DESKEWC_CS0_CON0] = {
			DPHY_WR_DESKEWC_CS0_CON0,
			DPHY_SW_WR_DESKEWC_CS0_CON0 },
		[s_DPHY_WR_DESKEWC_CS0_CON1] = {
			DPHY_WR_DESKEWC_CS0_CON1,
			DPHY_SW_WR_DESKEWC_CS0_CON1 },
		[s_DPHY_WR_DESKEWC_CS0_CON2] = {
			DPHY_WR_DESKEWC_CS0_CON2,
			DPHY_SW_WR_DESKEWC_CS0_CON2 },
		[s_DPHY_WR_DESKEWC_CS0_CON3] = {
			DPHY_WR_DESKEWC_CS0_CON3,
			DPHY_SW_WR_DESKEWC_CS0_CON3 },
		[s_DPHY_WR_DESKEWC_CS0_CON4] = {
			DPHY_WR_DESKEWC_CS0_CON4,
			DPHY_SW_WR_DESKEWC_CS0_CON4 },
		[s_DPHY_WR_DESKEWC_CS0_CON5] = {
			DPHY_WR_DESKEWC_CS0_CON5,
			DPHY_SW_WR_DESKEWC_CS0_CON5 },
		[s_DPHY_WR_DESKEWC_CS0_CON6] = {
			DPHY_WR_DESKEWC_CS0_CON6,
			DPHY_SW_WR_DESKEWC_CS0_CON6 },
		[s_DPHY_WR_DESKEWC_CS0_CON7] = {
			DPHY_WR_DESKEWC_CS0_CON7,
			DPHY_SW_WR_DESKEWC_CS0_CON7 },
		[s_DPHY_DM_DESKEWC_CS0_CON0] = {
			DPHY_DM_DESKEWC_CS0_CON0,
			DPHY_SW_DM_DESKEWC_CS0_CON0 },
		[s_DPHY_WR_DESKEWC_CS1_CON0] = {
			DPHY_WR_DESKEWC_CS1_CON0,
			DPHY_SW_WR_DESKEWC_CS1_CON0 },
		[s_DPHY_WR_DESKEWC_CS1_CON1] = {
			DPHY_WR_DESKEWC_CS1_CON1,
			DPHY_SW_WR_DESKEWC_CS1_CON1 },
		[s_DPHY_WR_DESKEWC_CS1_CON2] = {
			DPHY_WR_DESKEWC_CS1_CON2,
			DPHY_SW_WR_DESKEWC_CS1_CON2 },
		[s_DPHY_WR_DESKEWC_CS1_CON3] = {
			DPHY_WR_DESKEWC_CS1_CON3,
			DPHY_SW_WR_DESKEWC_CS1_CON3 },
		[s_DPHY_WR_DESKEWC_CS1_CON4] = {
			DPHY_WR_DESKEWC_CS1_CON4,
			DPHY_SW_WR_DESKEWC_CS1_CON4 },
		[s_DPHY_WR_DESKEWC_CS1_CON5] = {
			DPHY_WR_DESKEWC_CS1_CON5,
			DPHY_SW_WR_DESKEWC_CS1_CON5 },
		[s_DPHY_WR_DESKEWC_CS1_CON6] = {
			DPHY_WR_DESKEWC_CS1_CON6,
			DPHY_SW_WR_DESKEWC_CS1_CON6 },
		[s_DPHY_WR_DESKEWC_CS1_CON7] = {
			DPHY_WR_DESKEWC_CS1_CON7,
			DPHY_SW_WR_DESKEWC_CS1_CON7 },
		[s_DPHY_DM_DESKEWC_CS1_CON0] = {
			DPHY_DM_DESKEWC_CS1_CON0,
			DPHY_SW_DM_DESKEWC_CS1_CON0 },
		[s_DPHY_WR_DESKEWL_CS0_CON0] = {
			DPHY_WR_DESKEWL_CS0_CON0,
			DPHY_SW_WR_DESKEWL_CS0_CON0 },
		[s_DPHY_WR_DESKEWL_CS0_CON1] = {
			DPHY_WR_DESKEWL_CS0_CON1,
			DPHY_SW_WR_DESKEWL_CS0_CON1 },
		[s_DPHY_WR_DESKEWL_CS0_CON2] = {
			DPHY_WR_DESKEWL_CS0_CON2,
			DPHY_SW_WR_DESKEWL_CS0_CON2 },
		[s_DPHY_WR_DESKEWL_CS0_CON3] = {
			DPHY_WR_DESKEWL_CS0_CON3,
			DPHY_SW_WR_DESKEWL_CS0_CON3 },
		[s_DPHY_WR_DESKEWL_CS0_CON4] = {
			DPHY_WR_DESKEWL_CS0_CON4,
			DPHY_SW_WR_DESKEWL_CS0_CON4 },
		[s_DPHY_WR_DESKEWL_CS0_CON5] = {
			DPHY_WR_DESKEWL_CS0_CON5,
			DPHY_SW_WR_DESKEWL_CS0_CON5 },
		[s_DPHY_WR_DESKEWL_CS0_CON6] = {
			DPHY_WR_DESKEWL_CS0_CON6,
			DPHY_SW_WR_DESKEWL_CS0_CON6 },
		[s_DPHY_WR_DESKEWL_CS0_CON7] = {
			DPHY_WR_DESKEWL_CS0_CON7,
			DPHY_SW_WR_DESKEWL_CS0_CON7 },
		[s_DPHY_DM_DESKEWL_CS0_CON0] = {
			DPHY_DM_DESKEWL_CS0_CON0,
			DPHY_SW_DM_DESKEWL_CS0_CON0 },
		[s_DPHY_WR_DESKEWL_CS1_CON0] = {
			DPHY_WR_DESKEWL_CS1_CON0,
			DPHY_SW_WR_DESKEWL_CS1_CON0 },
		[s_DPHY_WR_DESKEWL_CS1_CON1] = {
			DPHY_WR_DESKEWL_CS1_CON1,
			DPHY_SW_WR_DESKEWL_CS1_CON1 },
		[s_DPHY_WR_DESKEWL_CS1_CON2] = {
			DPHY_WR_DESKEWL_CS1_CON2,
			DPHY_SW_WR_DESKEWL_CS1_CON2 },
		[s_DPHY_WR_DESKEWL_CS1_CON3] = {
			DPHY_WR_DESKEWL_CS1_CON3,
			DPHY_SW_WR_DESKEWL_CS1_CON3 },
		[s_DPHY_WR_DESKEWL_CS1_CON4] = {
			DPHY_WR_DESKEWL_CS1_CON4,
			DPHY_SW_WR_DESKEWL_CS1_CON4 },
		[s_DPHY_WR_DESKEWL_CS1_CON5] = {
			DPHY_WR_DESKEWL_CS1_CON5,
			DPHY_SW_WR_DESKEWL_CS1_CON5 },
		[s_DPHY_WR_DESKEWL_CS1_CON6] = {
			DPHY_WR_DESKEWL_CS1_CON6,
			DPHY_SW_WR_DESKEWL_CS1_CON6 },
		[s_DPHY_WR_DESKEWL_CS1_CON7] = {
			DPHY_WR_DESKEWL_CS1_CON7,
			DPHY_SW_WR_DESKEWL_CS1_CON7 },
		[s_DPHY_DM_DESKEWL_CS1_CON0] = {
			DPHY_DM_DESKEWL_CS1_CON0,
			DPHY_SW_DM_DESKEWL_CS1_CON0 },
		[s_DPHY_PRBS_CON2] = {
			DPHY_PRBS_CON2,
			DPHY_PRBS_CON4 },
		[s_DPHY_PRBS_CON3] = {
			DPHY_PRBS_CON3,
			DPHY_PRBS_CON5 },
		[s_DPHY_ZQ_CON9] = {
			DPHY_ZQ_CON9,
			DPHY_ZQ_CON9 },

		[s_DPHY2_MDLL_CON1] = {
			DPHY2_MDLL_CON1,
			DPHY2_MDLL_CON1 },
		[s_DPHY2_CA_DESKEW_CON0] = {
			DPHY2_CA_DESKEW_CON0,
			DPHY2_CA_DESKEW_CON0 },
		[s_DPHY2_CA_DESKEW_CON1] = {
			DPHY2_CA_DESKEW_CON1,
			DPHY2_CA_DESKEW_CON1 },
		[s_DPHY2_CA_DESKEW_CON2] = {
			DPHY2_CA_DESKEW_CON2,
			DPHY2_CA_DESKEW_CON2 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON_DM] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON_DM,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON_DM },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON0] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON0,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON0 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON1] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON1,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON1 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON2] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON2,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON2 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON3] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON3,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON3 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON4] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON4,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON4 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON5] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON5,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON5 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON6] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON6,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON6 },
		[s_DPHY2_RD_DESKEW_CENTER_CS0_CON7] = {
			DPHY2_RD_DESKEW_CENTER_CS0_CON7,
			DPHY2_SW_RD_DESKEW_CENTER_CS0_CON7 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON_DM] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON_DM,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON_DM },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON0] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON0,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON0 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON1] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON1,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON1 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON2] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON2,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON2 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON3] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON3,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON3 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON4] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON4,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON4 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON5] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON5,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON5 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON6] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON6,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON6 },
		[s_DPHY2_RD_DESKEW_CENTER_CS1_CON7] = {
			DPHY2_RD_DESKEW_CENTER_CS1_CON7,
			DPHY2_SW_RD_DESKEW_CENTER_CS1_CON7 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON_DM] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON_DM,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON_DM },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON0] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON0,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON0 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON1] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON1,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON1 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON2] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON2,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON2 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON3] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON3,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON3 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON4] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON4,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON4 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON5] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON5,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON5 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON6] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON6,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON6 },
		[s_DPHY2_RD_DESKEW_LEFT_CS0_CON7] = {
			DPHY2_RD_DESKEW_LEFT_CS0_CON7,
			DPHY2_SW_RD_DESKEW_LEFT_CS0_CON7 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON_DM] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON_DM,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON_DM },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON0] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON0,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON0 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON1] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON1,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON1 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON2] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON2,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON2 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON3] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON3,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON3 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON4] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON4,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON4 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON5] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON5,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON5 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON6] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON6,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON6 },
		[s_DPHY2_RD_DESKEW_LEFT_CS1_CON7] = {
			DPHY2_RD_DESKEW_LEFT_CS1_CON7,
			DPHY2_SW_RD_DESKEW_LEFT_CS1_CON7 },
		[s_DPHY2_RD_DQS_VWMC_CS0_CON0] = {
			DPHY2_RD_DQS_VWMC_CS0_CON0,
			DPHY2_SW_RD_DQS_VWMC_CS0_CON0 },
		[s_DPHY2_RD_DQS_VWMC_CS1_CON0] = {
			DPHY2_RD_DQS_VWMC_CS1_CON0,
			DPHY2_SW_RD_DQS_VWMC_CS1_CON0 },
		[s_DPHY2_RD_DQS_VWML_CS0_CON0] = {
			DPHY2_RD_DQS_VWML_CS0_CON0,
			DPHY2_SW_RD_DQS_VWML_CS0_CON0 },
		[s_DPHY2_RD_DQS_VWML_CS1_CON0] = {
			DPHY2_RD_DQS_VWML_CS1_CON0,
			DPHY2_SW_RD_DQS_VWML_CS1_CON0 },
		[s_DPHY2_WR_DESKEWC_CS0_CON0] = {
			DPHY2_WR_DESKEWC_CS0_CON0,
			DPHY2_SW_WR_DESKEWC_CS0_CON0 },
		[s_DPHY2_WR_DESKEWC_CS0_CON1] = {
			DPHY2_WR_DESKEWC_CS0_CON1,
			DPHY2_SW_WR_DESKEWC_CS0_CON1 },
		[s_DPHY2_WR_DESKEWC_CS0_CON2] = {
			DPHY2_WR_DESKEWC_CS0_CON2,
			DPHY2_SW_WR_DESKEWC_CS0_CON2 },
		[s_DPHY2_WR_DESKEWC_CS0_CON3] = {
			DPHY2_WR_DESKEWC_CS0_CON3,
			DPHY2_SW_WR_DESKEWC_CS0_CON3 },
		[s_DPHY2_WR_DESKEWC_CS0_CON4] = {
			DPHY2_WR_DESKEWC_CS0_CON4,
			DPHY2_SW_WR_DESKEWC_CS0_CON4 },
		[s_DPHY2_WR_DESKEWC_CS0_CON5] = {
			DPHY2_WR_DESKEWC_CS0_CON5,
			DPHY2_SW_WR_DESKEWC_CS0_CON5 },
		[s_DPHY2_WR_DESKEWC_CS0_CON6] = {
			DPHY2_WR_DESKEWC_CS0_CON6,
			DPHY2_SW_WR_DESKEWC_CS0_CON6 },
		[s_DPHY2_WR_DESKEWC_CS0_CON7] = {
			DPHY2_WR_DESKEWC_CS0_CON7,
			DPHY2_SW_WR_DESKEWC_CS0_CON7 },
		[s_DPHY2_DM_DESKEWC_CS0_CON0] = {
			DPHY2_DM_DESKEWC_CS0_CON0,
			DPHY2_SW_DM_DESKEWC_CS0_CON0 },
		[s_DPHY2_WR_DESKEWC_CS1_CON0] = {
			DPHY2_WR_DESKEWC_CS1_CON0,
			DPHY2_SW_WR_DESKEWC_CS1_CON0 },
		[s_DPHY2_WR_DESKEWC_CS1_CON1] = {
			DPHY2_WR_DESKEWC_CS1_CON1,
			DPHY2_SW_WR_DESKEWC_CS1_CON1 },
		[s_DPHY2_WR_DESKEWC_CS1_CON2] = {
			DPHY2_WR_DESKEWC_CS1_CON2,
			DPHY2_SW_WR_DESKEWC_CS1_CON2 },
		[s_DPHY2_WR_DESKEWC_CS1_CON3] = {
			DPHY2_WR_DESKEWC_CS1_CON3,
			DPHY2_SW_WR_DESKEWC_CS1_CON3 },
		[s_DPHY2_WR_DESKEWC_CS1_CON4] = {
			DPHY2_WR_DESKEWC_CS1_CON4,
			DPHY2_SW_WR_DESKEWC_CS1_CON4 },
		[s_DPHY2_WR_DESKEWC_CS1_CON5] = {
			DPHY2_WR_DESKEWC_CS1_CON5,
			DPHY2_SW_WR_DESKEWC_CS1_CON5 },
		[s_DPHY2_WR_DESKEWC_CS1_CON6] = {
			DPHY2_WR_DESKEWC_CS1_CON6,
			DPHY2_SW_WR_DESKEWC_CS1_CON6 },
		[s_DPHY2_WR_DESKEWC_CS1_CON7] = {
			DPHY2_WR_DESKEWC_CS1_CON7,
			DPHY2_SW_WR_DESKEWC_CS1_CON7 },
		[s_DPHY2_DM_DESKEWC_CS1_CON0] = {
			DPHY2_DM_DESKEWC_CS1_CON0,
			DPHY2_SW_DM_DESKEWC_CS1_CON0 },
		[s_DPHY2_WR_DESKEWL_CS0_CON0] = {
			DPHY2_WR_DESKEWL_CS0_CON0,
			DPHY2_SW_WR_DESKEWL_CS0_CON0 },
		[s_DPHY2_WR_DESKEWL_CS0_CON1] = {
			DPHY2_WR_DESKEWL_CS0_CON1,
			DPHY2_SW_WR_DESKEWL_CS0_CON1 },
		[s_DPHY2_WR_DESKEWL_CS0_CON2] = {
			DPHY2_WR_DESKEWL_CS0_CON2,
			DPHY2_SW_WR_DESKEWL_CS0_CON2 },
		[s_DPHY2_WR_DESKEWL_CS0_CON3] = {
			DPHY2_WR_DESKEWL_CS0_CON3,
			DPHY2_SW_WR_DESKEWL_CS0_CON3 },
		[s_DPHY2_WR_DESKEWL_CS0_CON4] = {
			DPHY2_WR_DESKEWL_CS0_CON4,
			DPHY2_SW_WR_DESKEWL_CS0_CON4 },
		[s_DPHY2_WR_DESKEWL_CS0_CON5] = {
			DPHY2_WR_DESKEWL_CS0_CON5,
			DPHY2_SW_WR_DESKEWL_CS0_CON5 },
		[s_DPHY2_WR_DESKEWL_CS0_CON6] = {
			DPHY2_WR_DESKEWL_CS0_CON6,
			DPHY2_SW_WR_DESKEWL_CS0_CON6 },
		[s_DPHY2_WR_DESKEWL_CS0_CON7] = {
			DPHY2_WR_DESKEWL_CS0_CON7,
			DPHY2_SW_WR_DESKEWL_CS0_CON7 },
		[s_DPHY2_DM_DESKEWL_CS0_CON0] = {
			DPHY2_DM_DESKEWL_CS0_CON0,
			DPHY2_SW_DM_DESKEWL_CS0_CON0 },
		[s_DPHY2_WR_DESKEWL_CS1_CON0] = {
			DPHY2_WR_DESKEWL_CS1_CON0,
			DPHY2_SW_WR_DESKEWL_CS1_CON0 },
		[s_DPHY2_WR_DESKEWL_CS1_CON1] = {
			DPHY2_WR_DESKEWL_CS1_CON1,
			DPHY2_SW_WR_DESKEWL_CS1_CON1 },
		[s_DPHY2_WR_DESKEWL_CS1_CON2] = {
			DPHY2_WR_DESKEWL_CS1_CON2,
			DPHY2_SW_WR_DESKEWL_CS1_CON2 },
		[s_DPHY2_WR_DESKEWL_CS1_CON3] = {
			DPHY2_WR_DESKEWL_CS1_CON3,
			DPHY2_SW_WR_DESKEWL_CS1_CON3 },
		[s_DPHY2_WR_DESKEWL_CS1_CON4] = {
			DPHY2_WR_DESKEWL_CS1_CON4,
			DPHY2_SW_WR_DESKEWL_CS1_CON4 },
		[s_DPHY2_WR_DESKEWL_CS1_CON5] = {
			DPHY2_WR_DESKEWL_CS1_CON5,
			DPHY2_SW_WR_DESKEWL_CS1_CON5 },
		[s_DPHY2_WR_DESKEWL_CS1_CON6] = {
			DPHY2_WR_DESKEWL_CS1_CON6,
			DPHY2_SW_WR_DESKEWL_CS1_CON6 },
		[s_DPHY2_WR_DESKEWL_CS1_CON7] = {
			DPHY2_WR_DESKEWL_CS1_CON7,
			DPHY2_SW_WR_DESKEWL_CS1_CON7 },
		[s_DPHY2_DM_DESKEWL_CS1_CON0] = {
			DPHY2_DM_DESKEWL_CS1_CON0,
			DPHY2_SW_DM_DESKEWL_CS1_CON0 },
		[s_DPHY2_PRBS_CON2] = {
			DPHY2_PRBS_CON2,
			DPHY2_PRBS_CON4 },
		[s_DPHY2_PRBS_CON3] = {
			DPHY2_PRBS_CON3,
			DPHY2_PRBS_CON5 },
		[s_DPHY2_ZQ_CON9] = {
			DPHY2_ZQ_CON9,
			DPHY2_ZQ_CON9 },
	};

	return &train_save_restore_regs[idx];
}

static int ab_ddr_pcie_link_listener(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct ab_ddr_context *ddr_ctx = container_of(nb,
			struct ab_ddr_context, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		pr_debug("%s: pcie link enable\n", __func__);
		mutex_lock(&ddr_ctx->ddr_lock);
		ddr_ctx->pcie_link_ready = true;
		mutex_unlock(&ddr_ctx->ddr_lock);
		return NOTIFY_OK;
	}

	if (action & (ABC_PCIE_LINK_PRE_DISABLE | ABC_PCIE_LINK_ERROR)) {
		pr_debug("%s: pcie link disable\n", __func__);
		mutex_lock(&ddr_ctx->ddr_lock);
		ddr_ctx->pcie_link_ready = false;
		mutex_unlock(&ddr_ctx->ddr_lock);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static unsigned int ddr_freq_param(enum ddr_freq_t freq, unsigned int index)
{
	static const unsigned int ddr_reg_freq[f_reg_max][AB_DRAM_FREQ_MAX] = {
	    { DVFS_CON_1866, DVFS_CON_1600, DVFS_CON_1200, DVFS_CON_933,
	      DVFS_CON_800 },
	    { CON2_FREQ_HIGH, CON2_FREQ_HIGH, CON2_FREQ_HIGH, CON2_FREQ_LOW,
	      CON2_FREQ_LOW},
	    { GNRCON_INIT_1866, GNRCON_INIT_1600, GNRCON_INIT_1200,
	      GNRCON_INIT_933, GNRCON_INIT_800 },
	    { TMGPBR_1866, TMGPBR_1600, TMGPBR_1200, TMGPBR_933, TMGPBR_800 },
	    { TMGROW_1866, TMGROW_1600, TMGROW_1200, TMGROW_933, TMGROW_800 },
	    { TMGDTA_1866, TMGDTA_1600, TMGDTA_1200, TMGDTA_933, TMGDTA_800 },
	    { TMGPWR_1866, TMGPWR_1600, TMGPWR_1200, TMGPWR_933, TMGPWR_800 },
	};

	return ddr_reg_freq[index][freq];
};

static inline void ddr_reg_wr_otp(struct ab_ddr_context *ddr_ctx,
		uint32_t addr, uint32_t otp_idx)
{
	ddr_reg_wr(ddr_ctx, addr, ddr_otp_rd(otp_idx));
}

static int ddr_reg_poll(struct ab_ddr_context *ddr_ctx,
		uint32_t reg, uint32_t poll_idx)
{
	static const struct ddr_reg_poll_t ddr_reg_poll_array[] = {
		[p_pll_con0_pll_phy_mif]	= { PLL_STABLE,
						    PLL_STABLE,
						    POLL_TIMEOUT_USEC },
		/* zq_done[0] bit */
		[p_DPHY_ZQ_CON1]		= { ZQ_DONE,
						    ZQ_DONE,
						    POLL_TIMEOUT_USEC },
		/* ctrl_locked[18] bit */
		[p_DPHY_MDLL_CON1]		= { CTRL_LOCKED,
						    CTRL_LOCKED,
						    POLL_TIMEOUT_USEC },
		/* dfi_init_complete[4:3] */
		[p_DREX_PHYSTATUS_dfi]		= { DFI_INIT_COMPLETE_ALL,
						    DFI_INIT_COMPLETE_ALL,
						    POLL_TIMEOUT_USEC },
		/* train_complete[15:14] */
		[p_DREX_PHYSTATUS_train]	= { TRAIN_COMPLETE_ALL,
						    TRAIN_COMPLETE_ALL,
						    POLL_TIMEOUT_USEC },
		/* chip_sref_state[8]=1 */
		[p_DREX_CHIPSTATUS_sr_enter]	= { CHIP_SREF_STATE(0),
						    CHIP_SREF_ENTRY(0),
						    POLL_TIMEOUT_USEC },
		/* chip_sref_state[8]=0 */
		[p_DREX_CHIPSTATUS_sr_exit]	= { CHIP_SREF_STATE(0),
						    CHIP_SREF_EXIT(0),
						    POLL_TIMEOUT_USEC },
		/* prbs_done[0] = 1 */
		[p_DPHY_PRBS_CON0_prbs_done]	= { PRBS_DONE,
						    PRBS_DONE,
						    POLL_TIMEOUT_USEC },
		/* prbs_disable[0] = 0 */
		[p_DPHY_PRBS_CON0_prbs_disable]	= { PRBS_DONE,
						    0x00000000,
						    POLL_TIMEOUT_USEC},
	};

	const struct ddr_reg_poll_t *poll = &ddr_reg_poll_array[poll_idx];
	uint32_t reg_val;
	unsigned long timeout;

	/* Reading the register before calculating the timeout makes sure the
	 * previous write request to DREX or DPHY (for example training or
	 * mdll lock request) is completed.
	 * As ddr_reg_rd() translates to pcie_config_read() (which is a
	 * non-posted pcie transaction), the previous pcie_config_write() has
	 * to be completed (which is posted pcie transaction) before completing
	 * the current read request.
	 */
	reg_val = ddr_reg_rd(ddr_ctx, reg);

	if (ddr_ctx->poll_multiplier == -1)
		ddr_ctx->poll_multiplier = ddr_otp_rd(o_SECURE_JTAG2) + 1;
	timeout = jiffies +
		usecs_to_jiffies(poll->usec_timeout * ddr_ctx->poll_multiplier);

	while (((reg_val & poll->mask) != poll->val) &&
			time_before(jiffies, timeout)) {
		ddr_usleep(DDR_POLL_USLEEP_MIN);
		reg_val = ddr_reg_rd(ddr_ctx, reg);
	}

	/* There is a chance that the above loop can exit because of the current
	 * task is scheduled out for more than the "timeout" jiffies. In this
	 * case the reg_val can contain the old value.
	 *
	 * If the poll condition doesn't meet, then read the register one more
	 * time which will handle the above scenario.
	 */
	if ((reg_val & poll->mask) != poll->val)
		reg_val = ddr_reg_rd(ddr_ctx, reg);

	if ((reg_val & poll->mask) != poll->val) {
		pr_err("ddr status poll failed for idx: 0x%x\n", poll_idx);
		pr_err("reg: 0x%x, val: 0x%x, poll_msk: 0x%x, poll_val: 0x%x\n",
			reg, reg_val, poll->mask, poll->val);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static int ddr_config_cmu_mif_lowfreq(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_wr_otp(ddr_ctx, PLL_LOCKTIME_PLL_PHY_MIF,
			o_Reserved_DDR_INIT_0);
	ddr_reg_wr_otp(ddr_ctx, PLL_CON0_PLL_PHY_MIF, o_SECURE_JTAG0);

	if (ddr_reg_poll(ddr_ctx, PLL_CON0_PLL_PHY_MIF,
				p_pll_con0_pll_phy_mif)) {
		pr_err("%s, mif pll lock failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_reg_wr_otp(ddr_ctx, PLL_CON0_PLL_PHY_MIF, o_SECURE_JTAG1);
	ddr_reg_wr_otp(ddr_ctx, CLK_CON_DIV_DIV4_PLLCLK_MIF,
			o_Reserved_DDR_INIT_3);
	ddr_reg_wr_otp(ddr_ctx, CLK_CON_DIV_DIV2_PLLCLK_MIF,
			o_Reserved_DDR_INIT_4);
	ddr_reg_wr_otp(ddr_ctx, CLK_CON_DIV_DFI_DIV2, o_Reserved_DDR_INIT_3);

	return DDR_SUCCESS;
}

static void ddr_set_pll_to_oscillator(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_set(ddr_ctx, DREX_MEMCONTROL, CLK_STOP_EN);
	ddr_reg_clr(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);

	/* Select the oscillator clock from SYSREG_MIF */
	ddr_reg_clr(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);

	/* Disable PLL and move PLL to oscillator clock */
	ddr_reg_wr(ddr_ctx, PLL_CON0_PLL_PHY_MIF,
			PLL_MUX_SEL(PLL_MUX_SEL_OSCCLK));

	ddr_reg_set(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
	ddr_reg_clr(ddr_ctx, DREX_MEMCONTROL, CLK_STOP_EN);
}

static int ddr_set_pll_freq(struct ab_ddr_context *ddr_ctx,
		enum ddr_freq_t freq)
{
	static const struct airbrush_ddr_pll_t
				ab_ddr_pll_pms_table[AB_DRAM_FREQ_MAX] = {
		[AB_DRAM_FREQ_MHZ_1866] = { 3, 583, 0},
		[AB_DRAM_FREQ_MHZ_1600] = { 3, 500, 0},
		[AB_DRAM_FREQ_MHZ_1200] = { 3, 375, 0},
		[AB_DRAM_FREQ_MHZ_933]  = { 3, 583, 1},
		[AB_DRAM_FREQ_MHZ_800]  = { 3, 500, 1},
	};
	const struct airbrush_ddr_pll_t *pms;

	if (freq >= AB_DRAM_FREQ_MAX)
		return -EINVAL;

	ab_sm_start_ts(AB_SM_TS_DDR_SET_PLL);
	/* With the given PMS values ab_ddr_pll_pms_table[freq],
	 * try to configure the PLL for MIF.
	 */
	pms = &ab_ddr_pll_pms_table[freq];

	ddr_reg_set(ddr_ctx, DREX_MEMCONTROL, CLK_STOP_EN);
	ddr_reg_clr(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);

	/* Select the oscillator clock from SYSREG_MIF */
	ddr_reg_clr(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);

	ddr_reg_wr(ddr_ctx, PLL_CON0_PLL_PHY_MIF,
			PLL_MUX_SEL(PLL_MUX_SEL_OSCCLK));
	ddr_reg_clr_set(ddr_ctx, PLL_CON0_PLL_PHY_MIF, PLL_PMS_MSK,
			PLL_PMS(pms->p, pms->m, pms->s) | PLL_ENABLE);

	ab_sm_record_ts(AB_SM_TS_DDR_SET_PLL);
	ab_sm_start_ts(AB_SM_TS_DDR_SET_PLL_POLL);
	if (ddr_reg_poll(ddr_ctx, PLL_CON0_PLL_PHY_MIF,
				p_pll_con0_pll_phy_mif)) {
		pr_err("%s, mif pll lock failed\n", __func__);
		return DDR_FAIL;
	}
	ab_sm_record_ts(AB_SM_TS_DDR_SET_PLL_POLL);

	ab_sm_start_ts(AB_SM_TS_DDR_FINISH_PLL);
	ddr_reg_set(ddr_ctx, PLL_CON0_PLL_PHY_MIF,
			PLL_MUX_SEL(PLL_MUX_SEL_PLLOUT));

	/* Select the mif pll fout from SYSREG_MIF */
	ddr_reg_set(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);

	ddr_reg_set(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
	ddr_reg_clr(ddr_ctx, DREX_MEMCONTROL, CLK_STOP_EN);

	ab_sm_record_ts(AB_SM_TS_DDR_FINISH_PLL);
	return DDR_SUCCESS;
}

static void ddr_ungate_phy_clock(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_set(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, WRAP_PLL_IS_STABLE);
	ddr_reg_set(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, SEL_CLKMUX_PLL);
	ddr_reg_set(ddr_ctx, MIF_PLL_WRAP_CTRL_REG, DDRPHY2XCLKGATE_ENABLE);
}

static void ddr_deassert_phy_reset_while_phy_is_gated(
		struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_set(ddr_ctx, PHY0_INIT_CTRL_REG, INIT_PLL_IS_STABLE);
	ddr_reg_set(ddr_ctx, PHY1_INIT_CTRL_REG, INIT_PLL_IS_STABLE);
	ddr_reg_set(ddr_ctx, PHY0_RST_CTRL_REG, RST_N | DIV_RST_N);
	ddr_reg_set(ddr_ctx, PHY1_RST_CTRL_REG, RST_N | DIV_RST_N);
}

static void ddr_enable_hw_periodic_training(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_wr(ddr_ctx, DREX_HWP_TRAIN_PERIOD0,
		   T_PTRAIN_PERIOD(DDR_HW_P_W_TRAIN_INTERVAL_MSEC));
	ddr_reg_set(ddr_ctx, DREX_HWPR_TRAIN_CONFIG0, PERIODIC_WRITE_CHIP0);
	ddr_reg_set(ddr_ctx, DREX_HWPR_TRAIN_CONTROL0, HW_PERIODIC_TRAIN_EN);
}

static void ddr_disable_hw_periodic_training(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_clr(ddr_ctx, DREX_HWPR_TRAIN_CONTROL0, HW_PERIODIC_TRAIN_EN);
	ddr_reg_clr(ddr_ctx, DREX_HWPR_TRAIN_CONFIG0, PERIODIC_WRITE_CHIP0);
}

static int ddr_block_axi_transactions(struct ab_ddr_context *ddr_ctx)
{
	unsigned long timeout;

	timeout = jiffies + usecs_to_jiffies(DDR_AXI_BLOCK_TIMEOUT);
	ddr_reg_clr(ddr_ctx, DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);

	while (((ddr_reg_rd(ddr_ctx, DREX_EMPTY_STATE) & IDLE_STATE_MASK) !=
	       IDLE_STATE_MASK) && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	/* check for the ddr empty state */
	if ((ddr_reg_rd(ddr_ctx, DREX_EMPTY_STATE) & IDLE_STATE_MASK) !=
	    IDLE_STATE_MASK) {
		pr_err("Block AXI: Error!! DDR AXI Block timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void __ddr_enable_power_features(struct ab_ddr_context *ddr_ctx)
{
	/* Enabling DDR Power features */
	ddr_reg_set(ddr_ctx, DREX_CGCONTROL, PHY_CG_EN);
	ddr_reg_set(ddr_ctx, DPHY_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_set(ddr_ctx, DPHY2_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr_set(ddr_ctx, DREX_MEMCONTROL, DPWRDN_TYPE_MSK,
			DPWRDN_TYPE(FORCED_PRECHARGE_PD) | DPWRDN_EN);
	ddr_reg_set(ddr_ctx, DREX_MEMCONTROL, CLK_STOP_EN);
}

static void __ddr_disable_power_features(struct ab_ddr_context *ddr_ctx)
{
	/* Disable DDR Power features */
	ddr_reg_clr(ddr_ctx, DREX_CGCONTROL, PHY_CG_EN);
	ddr_reg_clr(ddr_ctx, DPHY_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr(ddr_ctx, DPHY2_LP_CON0, PCL_PD | MDLL_CG_EN | DS_IO_PD);
	ddr_reg_clr(ddr_ctx, DREX_MEMCONTROL, DPWRDN_EN);
	ddr_reg_clr(ddr_ctx, DREX_MEMCONTROL, CLK_STOP_EN);
}

static void ddr_initialize_phy_pre(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_clr(ddr_ctx, DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);
	ddr_reg_wr(ddr_ctx, DREX_PHYCONTROL0, PAUSE_NO_RELOCK);

	/* set ignore_dic[31] (ignore dfi_init_complete) &
	 * pclk_async PCLK Async
	 */
	ddr_reg_clr(ddr_ctx, DREX_DFIRSTCONTROL, PB_WA_EN);
	ddr_reg_clr(ddr_ctx, DREX_MEMCONTROL, PB_REF_EN);
	ddr_reg_wr_otp(ddr_ctx, DREX_CONCONTROL, o_DREX_CONCONTROL_0);
}

static void ddr_initialize_phy(struct ab_ddr_context *ddr_ctx,
		enum ddr_freq_t freq)
{
	ddr_reg_set(ddr_ctx, DPHY_MON_CON0, MDLL_MONITOR_EN);
	ddr_reg_set(ddr_ctx, DPHY2_MON_CON0, MDLL_MONITOR_EN);
	ddr_reg_clr(ddr_ctx, DREX_CONCONTROL,
			DFI_INIT_START | DFI_INIT_START_PHY2);

	ddr_reg_clr_set(ddr_ctx, DPHY_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_wr(ddr_ctx, DPHY_GNR_CON0,
			ddr_freq_param(freq, f_DPHY_GNR_CON0));
	ddr_reg_wr_otp(ddr_ctx, DPHY_CAL_CON0, o_DPHY_CAL_CON0_0);
	ddr_reg_wr(ddr_ctx, DPHY_CAL_CON2,
			ddr_freq_param(freq, f_DPHY_CAL_CON2));
	ddr_reg_wr_otp(ddr_ctx, DPHY_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_2);

	ddr_reg_clr_set(ddr_ctx, DPHY2_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_wr(ddr_ctx, DPHY2_GNR_CON0,
			ddr_freq_param(freq, f_DPHY_GNR_CON0));
	ddr_reg_wr_otp(ddr_ctx, DPHY2_CAL_CON0, o_DPHY_CAL_CON0_0);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_CON2,
			ddr_freq_param(freq, f_DPHY_CAL_CON2));
	ddr_reg_wr_otp(ddr_ctx, DPHY2_LP_CON0, o_DPHY_LP_CON0_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_2);

	/* Disable Periodic Write Training */
	ddr_disable_hw_periodic_training(ddr_ctx);

	/* Disable power features till ddr training completes */
	__ddr_disable_power_features(ddr_ctx);
}

static int ddr_initialize_dfi(struct ab_ddr_context *ddr_ctx, int is_init)
{
	ddr_reg_wr_otp(ddr_ctx, DPHY_MDLL_CON0, o_DPHY_MDLL_CON0_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_MDLL_CON0, o_DPHY_MDLL_CON0_1);

	if (is_init) {
		ddr_reg_set(ddr_ctx, DREX_CONCONTROL,
			    DFI_INIT_START_PHY2 | DFI_INIT_START);
		if (ddr_reg_poll(ddr_ctx, DREX_PHYSTATUS, p_DREX_PHYSTATUS_dfi))
			return DDR_FAIL;

		ddr_reg_clr(ddr_ctx, DREX_CONCONTROL,
			    DFI_INIT_START_PHY2 | DFI_INIT_START);
	}

	ddr_reg_clr(ddr_ctx, DPHY_MDLL_CON0, CTRL_DLL_ON);
	ddr_reg_clr(ddr_ctx, DPHY2_MDLL_CON0, CTRL_DLL_ON);

	return DDR_SUCCESS;
}

static void ddr_dram_reset_sequence(struct ab_ddr_context *ddr_ctx)
{
	/* RESET_DRAM */
	ddr_reg_set(ddr_ctx, DREX_DFIRSTCONTROL, DFI_RESET_CONTROL);
	ddr_usleep(DDR_INIT_TIMING_tINIT1_USEC);
	ddr_reg_clr(ddr_ctx, DREX_DFIRSTCONTROL, DFI_RESET_CONTROL);
	ddr_usleep(DDR_INIT_TIMING_tINIT3_USEC);
}

static void ddr_power_down_exit_sequence(struct ab_ddr_context *ddr_ctx)
{
	/* ExitPD start */
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_PD_EXIT);
}

static void ddr_mrw_set_vref_odt_etc(struct ab_ddr_context *ddr_ctx,
		int freq)
{
	/* MRW Settings for each frequency supported */
	static const struct airbrush_ddr_mrw_set_t
				ab_ddr_mrw_set_table[AB_DRAM_FREQ_MAX] = {
		[AB_DRAM_FREQ_MHZ_1866] = { MRW1_1866, MRW2_1866,
					    MRW3_DEFAULT, MRW11_DEFAULT,
					    MRW13_DEFAULT, MRW22_DEFAULT,
					    MRR8_READ},
		[AB_DRAM_FREQ_MHZ_1600] = { MRW1_1600, MRW2_1600,
					    MRW3_DEFAULT, MRW11_DEFAULT,
					    MRW13_DEFAULT, MRW22_DEFAULT,
					    MRR8_READ},
		[AB_DRAM_FREQ_MHZ_1200] = { MRW1_1200, MRW2_1200,
					    MRW3_DEFAULT, MRW11_DEFAULT,
					    MRW13_DEFAULT, MRW22_DEFAULT,
					    MRR8_READ},
		[AB_DRAM_FREQ_MHZ_933]  = { MRW1_933, MRW2_933,
					    MRW3_DEFAULT, MRW11_DEFAULT,
					    MRW13_DEFAULT, MRW22_DEFAULT,
					    MRR8_READ },
		[AB_DRAM_FREQ_MHZ_800]  = { MRW1_800, MRW2_800,
					    MRW3_DEFAULT, MRW11_DEFAULT,
					    MRW13_DEFAULT, MRW22_DEFAULT,
					    MRR8_READ },
	};

	/* LPDDR4_chip_Init */
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr1);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr2);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr3);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr11);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr13);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr22);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, ab_ddr_mrw_set_table[freq].mr8);
}

static void ddr_dphy_ctrl_resync(struct ab_ddr_context *ddr_ctx)
{
	/* When CTRL_RESYNC bit transits from LOW to HIGH, pointers of FIFO
	 * within PHY and all of the DLL information (Read/Write/CA/CS DLL)
	 * is updated.
	 */
	ddr_reg_set(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, CTRL_RESYNC);

	ddr_reg_set(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, CTRL_RESYNC);
}

static void ddr_dram_dctrl_resync(struct ab_ddr_context *ddr_ctx)
{
	/* force DLL resync enable/disable */
	ddr_reg_set(ddr_ctx, DREX_PHYCONTROL0, FP_RESYNC);
	ddr_reg_clr(ddr_ctx, DREX_PHYCONTROL0, FP_RESYNC);

	ddr_dphy_ctrl_resync(ddr_ctx);
}

static void ddr_dram_zq_calibration(struct ab_ddr_context *ddr_ctx)
{
	/* lpddr4 zq calibration */
	ddr_reg_wr_otp(ddr_ctx, DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_2);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_11);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_12);
	ddr_reg_wr_otp(ddr_ctx, DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_0);
	ddr_reg_wr_otp(ddr_ctx, DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_1);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_11);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_12);
	ddr_reg_wr_otp(ddr_ctx, DREX_1CHIP_MASKING, o_DREX_1CHIP_MASKING_0);

	ddr_dram_dctrl_resync(ddr_ctx);
}

static int ddr_zq_calibration(struct ab_ddr_context *ddr_ctx)
{
	/* zq calibration is implemented as one-time long calibration */
	ddr_reg_clr_set(ddr_ctx, DPHY_ZQ_CON2,
			CTRL_ZQ_CLK_DIV_MSK, CTRL_ZQ_CLK_DIV(7));
	ddr_reg_set(ddr_ctx, DPHY_ZQ_CON0, ZQ_CLK_EN);
	ddr_reg_clr_set(ddr_ctx, DPHY_ZQ_CON0, ZQ_MANUAL_MODE_MSK,
			ZQ_MANUAL_MODE(ZQ_MANUAL_MODE_LONG));
	ddr_reg_set(ddr_ctx, DPHY_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_set(ddr_ctx, DPHY_ZQ_CON0, ZQ_MANUAL_STR);

	ddr_reg_clr_set(ddr_ctx, DPHY2_ZQ_CON2,
			CTRL_ZQ_CLK_DIV_MSK, CTRL_ZQ_CLK_DIV(7));
	ddr_reg_set(ddr_ctx, DPHY2_ZQ_CON0, ZQ_CLK_EN);
	ddr_reg_clr_set(ddr_ctx, DPHY2_ZQ_CON0, ZQ_MANUAL_MODE_MSK,
			ZQ_MANUAL_MODE(ZQ_MANUAL_MODE_LONG));
	ddr_reg_set(ddr_ctx, DPHY2_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_set(ddr_ctx, DPHY2_ZQ_CON0, ZQ_MANUAL_STR);

	/* wait zq calibration */
	if (ddr_reg_poll(ddr_ctx, DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_clr(ddr_ctx, DPHY_ZQ_CON0, ZQ_MANUAL_STR);
	ddr_reg_clr(ddr_ctx, DPHY_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_clr(ddr_ctx, DPHY2_ZQ_CON0, ZQ_MANUAL_STR);
	ddr_reg_clr(ddr_ctx, DPHY2_ZQ_CON0, ZQ_CLK_DIV_EN);

	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, UPD_MODE);
	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, UPD_MODE);

	ddr_reg_clr(ddr_ctx, DREX_CONCONTROL, UPDATE_MODE);

	return DDR_SUCCESS;
}

static int ddr_io_initialization(struct ab_ddr_context *ddr_ctx,
		int from_suspend)
{
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_3);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON6, o_DPHY_ZQ_CON6_2);

	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_3);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_12);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_15);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON6, o_DPHY_ZQ_CON6_2);

	/* perform ZQ calibration */
	if (ddr_zq_calibration(ddr_ctx)) {
		pr_err("%s: ddr zq calibration failed\n", __func__);
		return DDR_FAIL;
	}

	if (!from_suspend) {
		/* set dram odt settings */
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_0);
		/* set dram drive-strength */
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_30);
	}

	/* set host drive strength */
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_3);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON3, o_DPHY_ZQ_CON3_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_3);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON3, o_DPHY_ZQ_CON3_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	/* wait zq calibration */
	if (ddr_reg_poll(ddr_ctx, DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_2);

	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_1);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_2);

	if (!from_suspend) {
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_1);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_2);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_7);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_13);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_5);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_3);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_8);
		ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_14);
	}

	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_10);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_11);

	/* wait zq calibration */
	if (ddr_reg_poll(ddr_ctx, DPHY_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_ZQ_CON1, p_DPHY_ZQ_CON1))
		return DDR_FAIL;

	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_9);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON0, o_DPHY_ZQ_CON0_8);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	ddr_reg_wr_otp(ddr_ctx, DPHY_ZQ_CON9, o_DPHY_ZQ_CON9_2);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_ZQ_CON9, o_DPHY_ZQ_CON9_2);

	return DDR_SUCCESS;
}

static int ddr_enable_dll(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_wr(ddr_ctx, DPHY_GATE_CON0, GATE_CON0_DEFAULT);
	ddr_reg_wr(ddr_ctx, DPHY2_GATE_CON0, GATE_CON0_DEFAULT);

	ddr_reg_set(ddr_ctx, DPHY_MDLL_CON0, CTRL_DLL_ON);
	ddr_reg_set(ddr_ctx, DPHY2_MDLL_CON0, CTRL_DLL_ON);

	ddr_reg_clr(ddr_ctx, DPHY_MDLL_CON0, CTRL_START);
	ddr_reg_clr(ddr_ctx, DPHY2_MDLL_CON0, CTRL_START);

	udelay(DDR_DLL_CTRL_OFF_ON_UDELAY);

	ddr_reg_set(ddr_ctx, DPHY_MDLL_CON0, CTRL_START);
	ddr_reg_set(ddr_ctx, DPHY2_MDLL_CON0, CTRL_START);

	/* Poll for ctrl_lock */
	if (ddr_reg_poll(ddr_ctx, DPHY_MDLL_CON1, p_DPHY_MDLL_CON1)) {
		pr_err("%s: phy dll stable lock fail\n", __func__);
		return DDR_FAIL;
	}

	if (ddr_reg_poll(ddr_ctx, DPHY2_MDLL_CON1, p_DPHY_MDLL_CON1)) {
		pr_err("%s: phy2 dll stable lock fail\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static void ddr_set_drex_timing_parameters(struct ab_ddr_context *ddr_ctx,
		enum ddr_freq_t freq)
{
	struct ddr_refresh_info_t *ref_info = &ddr_ctx->ref_info;

	ddr_reg_clr_set(ddr_ctx, DREX_PRECHCONFIG0,
			TP_EN_MSK | PORT_POLICY_MSK,
			PORT_POLICY(PORT_POLICY_OPEN_PAGE));

	ddr_reg_wr(ddr_ctx, DREX_PWRDNCONFIG, PWRDNCONFIG_DEFAULT);
	ddr_reg_wr(ddr_ctx, DREX_TIMINGSETSW, TIMING_SET_SW_CON);
	ddr_reg_wr(ddr_ctx, DREX_TIMINGRFCPB,
			ddr_freq_param(freq, f_DREX_TIMINGRFCPB));
	ddr_reg_wr(ddr_ctx, DREX_TIMINGROW0,
			ddr_freq_param(freq, f_DREX_TIMINGROW));
	ddr_reg_wr(ddr_ctx, DREX_TIMINGROW1,
			ddr_freq_param(freq, f_DREX_TIMINGROW));
	ddr_reg_wr(ddr_ctx, DREX_TIMINGDATA0,
			ddr_freq_param(freq, f_DREX_TIMINGDATA));
	ddr_reg_wr(ddr_ctx, DREX_TIMINGDATA1,
			ddr_freq_param(freq, f_DREX_TIMINGDATA));
	ddr_reg_wr(ddr_ctx, DREX_TIMINGPOWER0,
			ddr_freq_param(freq, f_DREX_TIMINGPOWER));
	ddr_reg_wr(ddr_ctx, DREX_TIMINGPOWER1,
			ddr_freq_param(freq, f_DREX_TIMINGPOWER));

	/* Set the all-bank and per-bank auto refresh timings */
	ddr_reg_wr(ddr_ctx, DREX_TIMINGARE,
		   TIMINGARE(ref_info->t_refi, ref_info->t_refipb));

	ddr_reg_wr_otp(ddr_ctx, DREX_ETCTIMING, o_Reserved_DDR_INIT_7);
	ddr_reg_wr_otp(ddr_ctx, DREX_RDFETCH0, o_Reserved_DDR_INIT_8);
	ddr_reg_wr_otp(ddr_ctx, DREX_RDFETCH1, o_Reserved_DDR_INIT_9);

	/* DRAM DCTL Resync */
	ddr_reg_wr_otp(ddr_ctx, DPHY_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_OFFSETD_CON0, o_DPHY_OFFSETD_CON0_0);
	ddr_reg_wr_otp(ddr_ctx, DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	/* DRAM DCTL Resync */
	ddr_dram_dctrl_resync(ddr_ctx);
}

static int ddr_set_drex_address_parameters(struct ab_ddr_context *ddr_ctx,
		enum ddr_freq_t freq)
{
	/* DRAM Density Check */
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_25);
	ddr_reg_rd(ddr_ctx, DREX_MRSTATUS);

	ddr_dram_dctrl_resync(ddr_ctx);

	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_26);
	ddr_reg_rd(ddr_ctx, DREX_MRSTATUS);

	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_29);
	ddr_reg_rd(ddr_ctx, DREX_MRSTATUS);
	ddr_dram_dctrl_resync(ddr_ctx);

	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_28);
	ddr_reg_rd(ddr_ctx, DREX_MRSTATUS);
	ddr_dram_dctrl_resync(ddr_ctx);

	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
	ddr_reg_rd(ddr_ctx, DREX_MRSTATUS);
	ddr_dram_dctrl_resync(ddr_ctx);

	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_27);
	ddr_reg_set(ddr_ctx, DPHY_GNR_CON0, CTRL_DFDQS | DVFS_GATE_UPD_MODE);
	ddr_reg_set(ddr_ctx, DPHY2_GNR_CON0, CTRL_DFDQS | DVFS_GATE_UPD_MODE);
	ddr_dram_dctrl_resync(ddr_ctx);

	/* DRAM Density Setting */
	ddr_reg_wr_otp(ddr_ctx, DREX_MEMCONTROL, o_Reserved_DDR_INIT_10);
	ddr_reg_wr(ddr_ctx, DREX_ASP_MEMBASECONFIG0, CHUNK_START_END);
	ddr_reg_wr_otp(ddr_ctx, DREX_ASP_MEMCONFIG0, o_Reserved_DDR_INIT_11);
	ddr_reg_wr(ddr_ctx, DREX_ASP_CHIP0SIZECONFIG, CHIP_SIZE_512MB);

	/* Enable dbi_en at controller side. MR3 is already updated with
	 * read and write DBI enable
	 */
	ddr_reg_set(ddr_ctx, DREX_MEMCONTROL, DBI_EN);

	return DDR_SUCCESS;
}

static void ddr_prepare_training(struct ab_ddr_context *ddr_ctx,
		enum ddr_freq_t freq)
{
	ddr_reg_clr(ddr_ctx, DPHY_OFFSETD_CON0, UPD_MODE);
	ddr_reg_clr_set(ddr_ctx, DPHY_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_set(ddr_ctx, DPHY_MDLL_CON0, CLKM_CG_EN_SW);

	ddr_reg_clr(ddr_ctx, DPHY2_OFFSETD_CON0, UPD_MODE);
	ddr_reg_clr_set(ddr_ctx, DPHY2_DVFS_CON, DVFS_CON_MSK,
			ddr_freq_param(freq, f_DPHY_DVFS_CON));
	ddr_reg_set(ddr_ctx, DPHY2_MDLL_CON0, CLKM_CG_EN_SW);
}

static void ddr_ca_training(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_clr(ddr_ctx, DPHY_LP_CON0, DS_IO_PD);
	ddr_reg_clr(ddr_ctx, DPHY2_LP_CON0, DS_IO_PD);

	ddr_reg_clr_set(ddr_ctx, DPHY_CBT_CON0,
			CBT_CS_MSK, CBT_CS(CBT_CS_RANK0));
	ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, WRLVL_MODE);
	ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, CA_CAL_MODE);

	ddr_reg_clr_set(ddr_ctx, DPHY2_CBT_CON0,
			CBT_CS_MSK, CBT_CS(CBT_CS_RANK0));
	ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, WRLVL_MODE);
	ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, CA_CAL_MODE);

	ddr_dram_dctrl_resync(ddr_ctx);

	ddr_reg_clr(ddr_ctx, DPHY_CAL_CON0, WRLVL_MODE | CA_CAL_MODE);
	ddr_reg_clr(ddr_ctx, DPHY_CBT_CON0,
			CBT_CA_VREF_MODE_DS0 | CBT_CA_VREF_MODE_DS1);
	ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON0, WRLVL_MODE | CA_CAL_MODE);
	ddr_reg_clr(ddr_ctx, DPHY2_CBT_CON0, CBT_CA_VREF_MODE_DS0 |
				    CBT_CA_VREF_MODE_DS1);

	ddr_reg_set(ddr_ctx, DPHY_LP_CON0, DS_IO_PD);
	ddr_reg_set(ddr_ctx, DPHY2_LP_CON0, DS_IO_PD);
}

static int ddr_odt_training(struct ab_ddr_context *ddr_ctx)
{
	/* On_Die_Termination */
	ddr_reg_clr(ddr_ctx, DPHY_LP_CON0, CTRL_PULLD_DQS);
	ddr_reg_clr_set(ddr_ctx, DPHY_ZQ_CON0,
			ZQ_MODE_NOTERM | ZQ_RGDDR3, ZQ_MODE_LP4);
	ddr_reg_clr(ddr_ctx, DPHY_ZQ_CON6, ZQ_DS0_NOTERM | ZQ_DS1_NOTERM);
	ddr_reg_clr(ddr_ctx, DPHY_CAL_CON2, CTRL_RODT_DISABLE);
	ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON2,
			CTRL_READADJ_MSK | CTRL_READDURADJ_MSK,
			CTRL_READADJ(1) | CTRL_READDURADJ(5));
	ddr_reg_clr(ddr_ctx, DPHY_GNR_CON0, CTRL_DFDQS);

	ddr_reg_clr(ddr_ctx, DPHY2_LP_CON0, CTRL_PULLD_DQS);
	ddr_reg_clr_set(ddr_ctx, DPHY2_ZQ_CON0,
			ZQ_MODE_NOTERM | ZQ_RGDDR3, ZQ_MODE_LP4);
	ddr_reg_clr(ddr_ctx, DPHY2_ZQ_CON6, ZQ_DS0_NOTERM | ZQ_DS1_NOTERM);
	ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
	ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON2,
			CTRL_READADJ_MSK | CTRL_READDURADJ_MSK,
			CTRL_READADJ(1) | CTRL_READDURADJ(5));
	ddr_reg_clr(ddr_ctx, DPHY2_GNR_CON0, CTRL_DFDQS);

	/* ZQ One-time forced calibration */
	if (ddr_zq_calibration(ddr_ctx)) {
		pr_err("%s: one time forced zq calibration failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_dphy_ctrl_resync(ddr_ctx);
	ddr_reg_wr_otp(ddr_ctx, DREX_CONCONTROL, o_DREX_CONCONTROL_0);

	return DDR_SUCCESS;
}

static void ddr_autodqs_clean_gate_training(struct ab_ddr_context *ddr_ctx,
		enum ddr_freq_t freq)
{
	if (is_low_freq(freq)) {
		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK |
				CTRL_GATEADJ_MSK |
				CTRL_GATEDURADJ_MSK |
				CTRL_SHGATE,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(5) |
				CTRL_GATEADJ(0) |
				CTRL_GATEDURADJ(4));
		ddr_reg_clr(ddr_ctx, DPHY_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(ddr_ctx, DPHY_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x0) | DQS1_TESTIRCV(0x0));
		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON2, CTRL_RODT_DISABLE);

		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK |
				CTRL_GATEADJ_MSK |
				CTRL_GATEDURADJ_MSK |
				CTRL_SHGATE,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(5) |
				CTRL_GATEADJ(0) |
				CTRL_GATEDURADJ(4));
		ddr_reg_clr(ddr_ctx, DPHY2_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(ddr_ctx, DPHY2_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x0) | DQS1_TESTIRCV(0x0));
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
	} else {
		ddr_reg_set(ddr_ctx, DPHY_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_set(ddr_ctx, DPHY_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(7));
		ddr_reg_clr_set(ddr_ctx, DPHY_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x3) | DQS1_TESTIRCV(0x3));
		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_set(ddr_ctx, DPHY_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON2, CTRL_RODT_DISABLE);

		ddr_reg_set(ddr_ctx, DPHY2_CAL_CON4, GLITCH_REMOVAL_EN);
		ddr_reg_set(ddr_ctx, DPHY2_GNR_CON0, CTRL_DFDQS);
		ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON2,
				CTRL_READADJ_MSK |
				CTRL_READDURADJ_MSK,
				CTRL_READADJ(0) |
				CTRL_READDURADJ(7));
		ddr_reg_clr_set(ddr_ctx, DPHY2_TESTIRCV_CON0,
				DQS0_TESTIRCV_MSK | DQS1_TESTIRCV_MSK,
				DQS0_TESTIRCV(0x3) | DQS1_TESTIRCV(0x3));
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON0, GATE_CAL_MODE);
		ddr_reg_set(ddr_ctx, DPHY2_CAL_CON3, AUTO_DQS_CLEAN);
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON2, CTRL_RODT_DISABLE);
	}
}

static int ddr_read_dq_calibration(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_wr(ddr_ctx, DPHY_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_16);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_19);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_9);
	ddr_reg_wr_otp(ddr_ctx, DREX_DIRECTCMD, o_DREX_DIRECTCMD_6);
	ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, RD_CAL_MODE);
	ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, RD_CAL_MODE);

	ddr_reg_wr(ddr_ctx, DREX_INIT_TRAIN_CONFIG, 0x0);
	ddr_reg_set(ddr_ctx, DREX_INIT_TRAIN_CONFIG, INIT_READ_TRAIN_CHIP0);
	ddr_reg_set(ddr_ctx, DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	/* poll for train complete	*/
	if (ddr_reg_poll(ddr_ctx, DREX_PHYSTATUS, p_DREX_PHYSTATUS_train))
		return DDR_FAIL;

	ddr_reg_clr(ddr_ctx, DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	return DDR_SUCCESS;
}

static int ddr_write_dq_calibration(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON4, PATTERN_5555);

	ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
	ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON4, PATTERN_5555);

	ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN0, PATTERN_AA55AA55);
	ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN1, PATTERN_AA55AA55);
	ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN2, PATTERN_5555);

	ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, WR_CAL_MODE);
	ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, WR_CAL_MODE);

	ddr_reg_wr(ddr_ctx, DREX_INIT_TRAIN_CONFIG, 0x0);
	ddr_reg_set(ddr_ctx, DREX_INIT_TRAIN_CONFIG, INIT_WRITE_TRAIN_CHIP0);
	ddr_reg_set(ddr_ctx, DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	/* poll for train complete	*/
	if (ddr_reg_poll(ddr_ctx, DREX_PHYSTATUS, p_DREX_PHYSTATUS_train))
		return DDR_FAIL;

	ddr_reg_clr(ddr_ctx, DREX_INIT_TRAIN_CONTROL, INIT_TRAIN_START);

	return DDR_SUCCESS;
}

void ddr_prbs_training_init(struct ab_ddr_context *ddr_ctx)
{
	/* DDRPHY_SET_PRBS_TRAINING_INIT */
	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON0, PRBS_CON0_DEFAULT);
	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON0, PRBS_CON0_DEFAULT);
	ddr_reg_wr_otp(ddr_ctx, DPHY_PRBS_CON1, o_PCIe_reg_address_79);
	ddr_reg_wr_otp(ddr_ctx, DPHY2_PRBS_CON1, o_PCIe_reg_address_79);
}

static int ddr_prbs_training_read(struct ab_ddr_context *ddr_ctx)
{
	/* DDRPHY_RUN_PRBS_TRAINING - READ */
	ddr_reg_set(ddr_ctx, DPHY_PRBS_CON0, PRBS_READ_START);
	ddr_reg_set(ddr_ctx, DPHY2_PRBS_CON0, PRBS_READ_START);
	if (ddr_reg_poll(ddr_ctx, DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;

	ddr_reg_clr(ddr_ctx, DPHY_PRBS_CON0, PRBS_READ_START);
	ddr_reg_clr(ddr_ctx, DPHY2_PRBS_CON0, PRBS_READ_START);
	if (ddr_reg_poll(ddr_ctx, DPHY_PRBS_CON0,
				p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_PRBS_CON0,
				p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;

	/* DRAMDCTLResync:  start */
	ddr_dram_dctrl_resync(ddr_ctx);

	return DDR_SUCCESS;
}

static int ddr_prbs_training_write(struct ab_ddr_context *ddr_ctx)
{
	/* DDRPHY_RUN_PRBS_TRAINING - WRITE */
	ddr_reg_set(ddr_ctx, DPHY_PRBS_CON0, PRBS_WRITE_START);
	ddr_reg_set(ddr_ctx, DPHY2_PRBS_CON0, PRBS_WRITE_START);

	if (ddr_reg_poll(ddr_ctx, DPHY_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_PRBS_CON0, p_DPHY_PRBS_CON0_prbs_done))
		return DDR_FAIL;

	ddr_reg_clr(ddr_ctx, DPHY_PRBS_CON0, PRBS_WRITE_START);
	ddr_reg_clr(ddr_ctx, DPHY2_PRBS_CON0, PRBS_WRITE_START);

	if (ddr_reg_poll(ddr_ctx, DPHY_PRBS_CON0,
				p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;
	if (ddr_reg_poll(ddr_ctx, DPHY2_PRBS_CON0,
				p_DPHY_PRBS_CON0_prbs_disable))
		return DDR_FAIL;

	/* DRAMDCTLResync start */
	ddr_dram_dctrl_resync(ddr_ctx);

	return DDR_SUCCESS;
}

static void ddr_axi_enable_after_all_training(struct ab_ddr_context *ddr_ctx)
{
	ddr_reg_clr(ddr_ctx, DPHY_MDLL_CON0, CLKM_CG_EN_SW);
	ddr_reg_clr(ddr_ctx, DPHY2_MDLL_CON0, CLKM_CG_EN_SW);

	/* post init config and enabling the ddr power features */
	ddr_config_post_initialization(ddr_ctx);

	/* Enable Periodic Write Training */
	ddr_enable_hw_periodic_training(ddr_ctx);

	/* enable refresh and axi access controls */
	ddr_reg_set(ddr_ctx, DREX_MEMCONTROL, PB_REF_EN | DBI_EN);
	ddr_reg_clr(ddr_ctx, DREX_CONCONTROL,
			DFI_INIT_START_PHY2 | DFI_INIT_START);
	ddr_reg_set(ddr_ctx, DREX_CONCONTROL, AREF_EN);
	ddr_reg_set(ddr_ctx, DREX_ALL_INIT_INDI, ALL_INIT_DONE);
	ddr_reg_set(ddr_ctx, DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);
}

int ddr_enter_self_refresh_mode(struct ab_ddr_context *ddr_ctx, int ref_ctrl)
{
	if (ref_ctrl) {
		/* Disable PB Refresh & Auto Refresh */
		ddr_reg_clr(ddr_ctx, DREX_MEMCONTROL, PB_REF_EN);
		ddr_reg_clr(ddr_ctx, DREX_DFIRSTCONTROL, PB_WA_EN);
		ddr_reg_clr(ddr_ctx, DREX_CONCONTROL, AREF_EN);

		/* safe to send a manual all bank refresh command */
		ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_REFA);
	}

	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_SREF_ENTR);
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_CKEL);

	/* poll for self refresh entry */
	if (ddr_reg_poll(ddr_ctx, DREX_CHIPSTATUS,
				p_DREX_CHIPSTATUS_sr_enter)) {
		pr_err("%s: self-refresh entry failed.\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

int ddr_exit_self_refresh_mode(struct ab_ddr_context *ddr_ctx, int ref_ctrl)
{
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_SREF_EXIT);

	/* poll for self refresh exit */
	if (ddr_reg_poll(ddr_ctx, DREX_CHIPSTATUS, p_DREX_CHIPSTATUS_sr_exit)) {
		pr_err("%s: self-refresh exit failed.\n", __func__);
		return DDR_FAIL;
	}

	if (ref_ctrl) {
		/* safe to send a manual all bank refresh command */
		ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_REFA);

		/* Enable PB Refresh & Auto Refresh */
		ddr_reg_set(ddr_ctx, DREX_MEMCONTROL, PB_REF_EN);
		ddr_reg_set(ddr_ctx, DREX_CONCONTROL, AREF_EN);
	}

	return DDR_SUCCESS;
}

static void ddr_copy_train_results_from_sram(struct ab_ddr_context *ddr_ctx)
{
	int i;
	unsigned int sram_train_base;
	int freq = ddr_ctx->cur_freq;

	sram_train_base = ddr_ctx->ddr_train_sram_location;
	pr_info("freq: %d, sram_train_base: 0x%x\n", freq, sram_train_base);

	for (i = 0; i < s_train_max_index; i++)
		ddr_ctx->ddr_train_save_value[freq][i] =
				ddr_mem_rd(sram_train_base + (i * 4));
}

static void ddr_train_save_configuration(struct ab_ddr_context *ddr_ctx)
{
	int i;
	int freq = ddr_ctx->cur_freq;
	const struct ddr_train_save_restore_t *train_reg;

	for (i = 0; i < s_train_max_index; i++) {
		train_reg = get_train_save_restore_regs(i);
		ddr_ctx->ddr_train_save_value[freq][i] =
				ddr_reg_rd(ddr_ctx, train_reg->reg_save);
	}
}

static void ddr_train_restore_configuration(struct ab_ddr_context *ddr_ctx,
					enum ddr_freq_t freq, int restore_mode)
{
	int idx;
	const struct ddr_train_save_restore_t *train_reg;
	uint32_t *ddr_train_save_value =
		ddr_ctx->ddr_train_save_value[freq];

	/* Training related setting to be applied before Restore */
	ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON1, RDLVL_PASS_ADJ_MSK,
			RDLVL_PASS_ADJ(0x4));
	ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON1, GLVL_PERIODIC_INCR_ADJ_MSK,
			GLVL_PERIODIC_INCR_ADJ(0x40));
	ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON4, GLVL_PERIODIC_FINE_INCR_ADJ_MSK,
			GLVL_PERIODIC_FINE_INCR_ADJ(0x0));
	ddr_reg_clr_set(ddr_ctx, DPHY_CAL_CON1, RDLVL_PERIODIC_INCR_ADJ_MSK,
			RDLVL_PERIODIC_INCR_ADJ(0x1));

	ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON1, RDLVL_PASS_ADJ_MSK,
			RDLVL_PASS_ADJ(0x4));
	ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON1, GLVL_PERIODIC_INCR_ADJ_MSK,
			GLVL_PERIODIC_INCR_ADJ(0x40));
	ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON4,
			GLVL_PERIODIC_FINE_INCR_ADJ_MSK,
			GLVL_PERIODIC_FINE_INCR_ADJ(0x0));
	ddr_reg_clr_set(ddr_ctx, DPHY2_CAL_CON1, RDLVL_PERIODIC_INCR_ADJ_MSK,
			RDLVL_PERIODIC_INCR_ADJ(0x1));

	/* Restore command bus training result */
	ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, CA_CAL_MODE);
	ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, CA_CAL_MODE);

	ddr_reg_wr(ddr_ctx, DPHY_CA_DESKEW_CON0,
		   ddr_train_save_value[s_DPHY_CA_DESKEW_CON0]);
	ddr_reg_wr(ddr_ctx, DPHY_CA_DESKEW_CON1,
		   ddr_train_save_value[s_DPHY_CA_DESKEW_CON1]);
	ddr_reg_wr(ddr_ctx, DPHY_CA_DESKEW_CON2,
		   ddr_train_save_value[s_DPHY_CA_DESKEW_CON2]);

	ddr_reg_wr(ddr_ctx, DPHY2_CA_DESKEW_CON0,
		   ddr_train_save_value[s_DPHY2_CA_DESKEW_CON0]);
	ddr_reg_wr(ddr_ctx, DPHY2_CA_DESKEW_CON1,
		   ddr_train_save_value[s_DPHY2_CA_DESKEW_CON1]);
	ddr_reg_wr(ddr_ctx, DPHY2_CA_DESKEW_CON2,
		   ddr_train_save_value[s_DPHY2_CA_DESKEW_CON2]);

	ddr_reg_clr(ddr_ctx, DPHY_CAL_CON0, CA_CAL_MODE);
	ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON0, CA_CAL_MODE);

	if (restore_mode == AB_RESTORE_FULL) {

		/* Auto DQS clean / Gate training */
		ddr_autodqs_clean_gate_training(ddr_ctx, freq);

		ddr_reg_wr(ddr_ctx, DPHY_CAL_RD_PATTERN_CON0, PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY2_CAL_RD_PATTERN_CON0,
				PATTERN_55AA55AA);

		ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON0, PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON1, PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON2, PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON3, PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY_CAL_WR_PATTERN_CON4, PATTERN_5555);
		ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN0, PATTERN_AA55AA55);
		ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN1, PATTERN_AA55AA55);
		ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN2, PATTERN_5555);

		ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON0,
				PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON1,
				PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON2,
				PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON3,
				PATTERN_55AA55AA);
		ddr_reg_wr(ddr_ctx, DPHY2_CAL_WR_PATTERN_CON4, PATTERN_5555);
		ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN0, PATTERN_AA55AA55);
		ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN1, PATTERN_AA55AA55);
		ddr_reg_wr(ddr_ctx, DREX_WRTRA_PATTERN2, PATTERN_5555);

		/* Restore Read Training Results */
		ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, RD_CAL_MODE);
		ddr_reg_set(ddr_ctx, DPHY_CAL_CON3, RD_SW_MODE);
		ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, RD_CAL_MODE);
		ddr_reg_set(ddr_ctx, DPHY2_CAL_CON3, RD_SW_MODE);

		for (idx = s_DPHY_RD_DESKEW_CENTER_CS0_CON_DM;
			idx <= s_DPHY2_RD_DQS_VWML_CS1_CON0; idx++) {
			train_reg = get_train_save_restore_regs(idx);
			ddr_reg_wr(ddr_ctx, train_reg->reg_restore,
				   ddr_train_save_value[idx]);
		}

		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON3, RD_SW_MODE);
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON3, RD_SW_MODE);

		/* Restore Write Training Results */
		ddr_reg_set(ddr_ctx, DPHY_CAL_CON0, WR_CAL_MODE);
		ddr_reg_set(ddr_ctx, DPHY_CAL_CON3, WR_SW_MODE);
		ddr_reg_set(ddr_ctx, DPHY2_CAL_CON0, WR_CAL_MODE);
		ddr_reg_set(ddr_ctx, DPHY2_CAL_CON3, WR_SW_MODE);

		for (idx = s_DPHY_WR_DESKEWC_CS0_CON0;
			idx <= s_DPHY2_DM_DESKEWL_CS1_CON0; idx++) {
			train_reg = get_train_save_restore_regs(idx);
			ddr_reg_wr(ddr_ctx, train_reg->reg_restore,
				   ddr_train_save_value[idx]);
		}

		ddr_reg_clr(ddr_ctx, DPHY_CAL_CON3, WR_SW_MODE);
		ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON3, WR_SW_MODE);
	}

	/* Restore PRBS training result */
	ddr_reg_set(ddr_ctx, DPHY_CAL_CON3, PRBS_SW_MODE);
	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON4,
			ddr_train_save_value[s_DPHY_PRBS_CON2]);
	ddr_reg_wr(ddr_ctx, DPHY_PRBS_CON5,
			ddr_train_save_value[s_DPHY_PRBS_CON3]);
	ddr_reg_clr(ddr_ctx, DPHY_CAL_CON3, PRBS_SW_MODE);

	ddr_reg_set(ddr_ctx, DPHY2_CAL_CON3, PRBS_SW_MODE);
	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON4,
			ddr_train_save_value[s_DPHY2_PRBS_CON2]);
	ddr_reg_wr(ddr_ctx, DPHY2_PRBS_CON5,
			ddr_train_save_value[s_DPHY2_PRBS_CON3]);
	ddr_reg_clr(ddr_ctx, DPHY2_CAL_CON3, PRBS_SW_MODE);

	ddr_reg_wr(ddr_ctx, DPHY_ZQ_CON9,
			ddr_train_save_value[s_DPHY_ZQ_CON9]);
	ddr_reg_wr(ddr_ctx, DPHY2_ZQ_CON9,
			ddr_train_save_value[s_DPHY2_ZQ_CON9]);

	/* Update pointers of FIFO within PHY and all of the DLL information */
	ddr_dphy_ctrl_resync(ddr_ctx);
}

static int ab_ddr_initial_trainings(struct ab_ddr_context *ddr_ctx)
{
	ddr_prepare_training(ddr_ctx, ddr_ctx->cur_freq);

	ddr_ca_training(ddr_ctx);

	if (ddr_odt_training(ddr_ctx)) {
		pr_err("%s: ddr on-die-termination training failed.\n",
			__func__);
		goto ddr_init_train_fail;
	}

	ddr_autodqs_clean_gate_training(ddr_ctx, ddr_ctx->cur_freq);

	if (ddr_read_dq_calibration(ddr_ctx)) {
		pr_err("%s: ddr read dq calibration failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	if (ddr_write_dq_calibration(ddr_ctx)) {
		pr_err("%s: ddr write dq calibration failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	/* reset the prbs_dram_act_enable */
	ddr_reg_clr(ddr_ctx, DPHY_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);
	ddr_reg_clr(ddr_ctx, DPHY2_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);

	ddr_prbs_training_init(ddr_ctx);

	if (ddr_prbs_training_read(ddr_ctx)) {
		pr_err("%s: ddr prbs read training failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	if (ddr_prbs_training_write(ddr_ctx)) {
		pr_err("%s: ddr prbs write training failed.\n", __func__);
		goto ddr_init_train_fail;
	}

	/* Set the prbs_dram_act_enable */
	ddr_reg_set(ddr_ctx, DPHY_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);
	ddr_reg_set(ddr_ctx, DPHY2_PRBS_CON8, PRBS_DRAM_ACT_ENABLE);

	return DDR_SUCCESS;

ddr_init_train_fail:
	pr_err("Error!!! ddr initial training sequence failed\n");
	return DDR_FAIL;
}

static int ab_ddr_train(struct ab_ddr_context *ddr_ctx, uint32_t ddr_sr)
{
	/* Check ddr_reg_rd(ddr_ctx, TRN_ADDR) value and if non zero,
	 * the previous training results are stored in here.
	 * Restore the Training parameters.
	 */
	if (GET_REG_TRN_ADDR()) {

		if (ddr_enter_self_refresh_mode(ddr_ctx, REF_CTRL_DISABLE))
			goto ddr_train_fail;

		/* Copy training results from TRN_ADDR to PHY/DRAM */
		ddr_train_restore_configuration(ddr_ctx,
						AB_DRAM_FREQ_MHZ_1866,
						AB_RESTORE_FULL);

		if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_DISABLE))
			goto ddr_train_fail;

		ddr_axi_enable_after_all_training(ddr_ctx);

	} else {

		if (!ddr_sr) {
			if (ddr_enter_self_refresh_mode(ddr_ctx,
							REF_CTRL_DISABLE))
				goto ddr_train_fail;
		}

		if (ddr_set_pll_freq(ddr_ctx, AB_DRAM_FREQ_MHZ_1866)) {
			pr_err("%s: setting pll freq [1866MHz] failed.\n",
				__func__);
			goto ddr_train_fail;
		}

		/* for CKE High to CS delay */
		udelay(DDR_INIT_CKE2CS_DELAY_USEC);

		if (ddr_enable_dll(ddr_ctx)) {
			pr_err("%s: enable dll failed.\n", __func__);
			goto ddr_train_fail;
		}

		ddr_power_down_exit_sequence(ddr_ctx);

		ddr_set_drex_timing_parameters(ddr_ctx, ddr_ctx->cur_freq);

		if (ddr_set_drex_address_parameters(ddr_ctx,
					ddr_ctx->cur_freq)) {
			pr_err("%s: drex address parameter settings failed.\n",
				__func__);
			goto ddr_train_fail;
		}

		if (ab_ddr_initial_trainings(ddr_ctx)) {
			pr_err("%s: initial training failed.\n", __func__);
			goto ddr_train_fail;
		}

		if (!ddr_sr) {
			if (ddrphy_run_vref_training(ddr_ctx)) {
				pr_err("%s: vref training failed.\n", __func__);
				goto ddr_train_fail;
			}

			if (ab_ddr_initial_trainings(ddr_ctx)) {
				pr_err("%s: re-initial training failed.\n",
						__func__);
				goto ddr_train_fail;
			}
		}

		if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_DISABLE))
			goto ddr_train_fail;

		ddr_axi_enable_after_all_training(ddr_ctx);

		/* Save training results to local array */
		ddr_train_save_configuration(ddr_ctx);
		ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1866] = 1;
	}

	return DDR_SUCCESS;

ddr_train_fail:
	pr_err("%s: error!! ddr training failed\n", __func__);
	return DDR_FAIL;
}

static int32_t ab_ddr_init_internal_isolation(struct ab_ddr_context *ddr_ctx)
{
	uint32_t ddr_sr;

	ddr_sr = GPIO_DDR_SR();

	/* deassert PHY reset while PHY clock is gated */
	ddr_deassert_phy_reset_while_phy_is_gated(ddr_ctx);

	/* configure PLL_MIF via CMU_MIF_for_Lowfreq and wait for pll lock */
	if (ddr_config_cmu_mif_lowfreq(ddr_ctx))
		goto ddr_init_fail;

	/* Ungate PHY Clock */
	ddr_ungate_phy_clock(ddr_ctx);

	/* Deassert Internal Isolation
	 * SYSREG_Central_PMU ::
	 * PMU_CONTROL[0x4] :: PHY_RET_ON[7:7] = 0
	 */
	PMU_CONTROL_PHY_RET_OFF();

	/* Initialize PHY */
	ddr_initialize_phy_pre(ddr_ctx);

	ddr_initialize_phy(ddr_ctx, ddr_ctx->cur_freq);

	/* Initialize DFI Interface */
	if (ddr_initialize_dfi(ddr_ctx, 1)) {
		pr_err("%s: DFI initialization failed\n", __func__);
		goto ddr_init_fail;
	}

	if (ddr_sr) { /* SUSPEND to ACTIVE sequence */

		/* IO Initialization for suspend */
		if (ddr_io_initialization(ddr_ctx, 1)) {
			pr_err("%s: ddr IO init from suspend failed\n",
				__func__);
			goto ddr_init_fail;
		}
	} else { /* OFF to ACTIVE sequence */

		/* DRAM reset sequence */
		ddr_dram_reset_sequence(ddr_ctx);

		/* Power-down exit sequence */
		ddr_power_down_exit_sequence(ddr_ctx);

		/* IO Initialization */
		if (ddr_io_initialization(ddr_ctx, 0)) {
			pr_err("%s: ddr IO initialization failed\n", __func__);
			goto ddr_init_fail;
		}

		/* MRWs (Set VREF, ODT, etc) */
		ddr_mrw_set_vref_odt_etc(ddr_ctx, AB_DRAM_FREQ_MHZ_1866);

		/* DRAM ZQ Calibration */
		ddr_dram_zq_calibration(ddr_ctx);
	}

	/* DDR training */
	if (ab_ddr_train(ddr_ctx, ddr_sr))
		goto ddr_init_fail;

	/* Now M0 can enter WFI for GPIO_DDR_TRAIN or REG_DDR_TRAIN */
	return DDR_SUCCESS;

ddr_init_fail:
	pr_err("%s: error!! ddr initialization failed\n", __func__);
	return DDR_FAIL;
}

static int32_t ab_ddr_init_freq_change(struct ab_ddr_context *ddr_ctx)
{
	/* Initialize PHY */
	ddr_initialize_phy_pre(ddr_ctx);

	ddr_initialize_phy(ddr_ctx, ddr_ctx->cur_freq);

	/* Initialize DFI Interface */
	if (ddr_initialize_dfi(ddr_ctx, 0))
		goto ddr_init_fail;

	/* Power-down exit sequence */
	ddr_power_down_exit_sequence(ddr_ctx);

	/* configure MRWs (Set VREF, ODT, etc) as per the frequency */
	ddr_mrw_set_vref_odt_etc(ddr_ctx, ddr_ctx->cur_freq);

	/* Enable DLL */
	if (ddr_enable_dll(ddr_ctx)) {
		pr_err("%s: enable dll failed.\n", __func__);
		goto ddr_init_fail;
	}

	ddr_set_drex_timing_parameters(ddr_ctx, ddr_ctx->cur_freq);

	if (ddr_set_drex_address_parameters(ddr_ctx, ddr_ctx->cur_freq))
		goto ddr_init_fail;

	return DDR_SUCCESS;

ddr_init_fail:
	pr_err("%s: Error!! init frequency change failed.\n", __func__);
	return DDR_FAIL;
}

static void ddr_sanity_test(void *ctx, unsigned int test_data)
{
#ifdef CONFIG_AB_DDR_SANITY_TEST
	/* only consider read & write bits. clear all other fields. */
	test_data &= (DDR_BOOT_TEST_READ | DDR_BOOT_TEST_WRITE);

	if (test_data & DDR_BOOT_TEST_READ)
		test_data |= DDR_TEST_PCIE_DMA_READ(
				CONFIG_AB_DDR_SANITY_SZ_MBYTES);

	if (test_data & DDR_BOOT_TEST_WRITE)
		test_data |= DDR_TEST_PCIE_DMA_WRITE(
				CONFIG_AB_DDR_SANITY_SZ_MBYTES);

	__ab_ddr_read_write_test(ctx, test_data);
#endif
}

static int ddr_phy_reinit_train(void *ctx, enum ddr_freq_t freq)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	/* DDR Init flow (reduced) without training */
	if (ab_ddr_init_freq_change(ddr_ctx)) {
		pr_err("%s: frequency switch failed\n", __func__);
		return DDR_FAIL;
	}

	/* The below settings were taken care duing different training
	 * sequences like io initialization, odt, zq calibration during normal
	 * ddr initialization and training. As part of frequency change, we are
	 * not executing above sequences. So, the below settings are required
	 * for normal ddr operation.
	 */
	ddr_reg_clr(ddr_ctx, DPHY_LP_CON0, CTRL_PULLD_DQ | CTRL_PULLD_DQS);
	ddr_reg_clr(ddr_ctx, DPHY_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_clr_set(ddr_ctx, DPHY_ZQ_CON0, ZQ_MODE_TERM_MSK,
			ZQ_MODE_TERM(ZQ_MODE_TERM_60_OHM));

	ddr_reg_clr(ddr_ctx, DPHY2_LP_CON0, CTRL_PULLD_DQ | CTRL_PULLD_DQS);
	ddr_reg_clr(ddr_ctx, DPHY2_ZQ_CON0, ZQ_CLK_DIV_EN);
	ddr_reg_clr_set(ddr_ctx, DPHY2_ZQ_CON0, ZQ_MODE_TERM_MSK,
			ZQ_MODE_TERM(ZQ_MODE_TERM_60_OHM));

	if (ddr_ctx->ddr_train_completed[freq]) {
		ddr_train_restore_configuration(ddr_ctx, freq, AB_RESTORE_FULL);
	} else {
		/* RESTORE the training results of CBT, PRBS and VREF training
		 * of 1866MHz from SRAM location
		 */
		ddr_train_restore_configuration(ddr_ctx, AB_DRAM_FREQ_MHZ_1866,
						AB_RESTORE_DVFS);

		/* Prepare Training */
		ddr_prepare_training(ddr_ctx, freq);

		/* autodqs clean gate training for low freq */
		ddr_autodqs_clean_gate_training(ddr_ctx, freq);

		/* Read & Write dq calibration for modified frequency */
		if (ddr_read_dq_calibration(ddr_ctx)) {
			pr_err("Error!! read dq calibration failed\n");
			goto freq_change_fail;
		}

		if (ddr_write_dq_calibration(ddr_ctx)) {
			pr_err("Error!! write dq calibration failed\n");
			goto freq_change_fail;
		}
	}

	/* save the training results to local array */
	if (!ddr_ctx->ddr_train_completed[freq])
		ddr_train_save_configuration(ddr_ctx);

	ddr_ctx->ddr_train_completed[freq] = 1;

	return DDR_SUCCESS;

freq_change_fail:
	pr_err("%s: ddr phy reinit and train seqeunce failed!!\n", __func__);
	return DDR_FAIL;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int ddr_set_mif_freq(void *ctx, enum ddr_freq_t freq)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (ddr_ctx->cur_freq == freq)
		return DDR_SUCCESS;

	ddr_ctx->cur_freq = freq;

	ddr_sanity_test(ctx, DDR_BOOT_TEST_WRITE);

	/* Block AXI Before entering self-refresh */
	if (ddr_block_axi_transactions(ddr_ctx))
		return -ETIMEDOUT;

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode(ddr_ctx, REF_CTRL_ENABLE)) {
		pr_err("%s: self-refresh entry failed\n", __func__);
		return DDR_FAIL;
	}

	/* Configure the PLL for the required frequency */
	if (ddr_set_pll_freq(ddr_ctx, freq)) {
		pr_err("%s: pll setting failed\n", __func__);
		return DDR_FAIL;
	}

	if (ddr_phy_reinit_train(ddr_ctx, freq)) {
		pr_err("%s: phy reinit and train sequence failed\n", __func__);
		return DDR_FAIL;
	}

	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_DISABLE)) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		return DDR_FAIL;
	}

	ddr_axi_enable_after_all_training(ddr_ctx);

	ddr_sanity_test(ctx, DDR_BOOT_TEST_READ);

	return DDR_SUCCESS;
}

/* Perform ddr re-training or train result restore */
int32_t ab_ddr_train_gpio(void *ctx)
{
	uint32_t ddr_sr;
	unsigned long timeout;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	ddr_sr = GPIO_DDR_SR();

	timeout = jiffies + usecs_to_jiffies(DDR_CLK_IN_SENSE_TIMEOUT);
	while (GPIO_CKE_IN_SENSE() && time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	if (GPIO_CKE_IN_SENSE()) {
		pr_err("error!! CLK_IN_SENSE transition timeout\n");
		return DDR_FAIL;
	}

	/* self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_DISABLE)) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		return DDR_FAIL;
	}

	if (ab_ddr_train(ddr_ctx, ddr_sr)) {
		pr_err("%s: ddr training failed\n", __func__);
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int32_t __ab_ddr_train_all(struct ab_ddr_context *ddr_ctx)
{
	int idx = 0;

	/* Clear the previous trainings done for all Non-1866MHz frequencies */
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1600] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1200] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_933] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_800] = 0;

	for (idx = AB_DRAM_FREQ_MHZ_1600; idx <= AB_DRAM_FREQ_MHZ_800; idx++) {
		if (ddr_set_mif_freq(ddr_ctx, idx)) {
			pr_err("train-all: freq idx %d train fail\n", idx);
			goto ddr_train_all_fail;
		}
	}

	/* revert back the ddr frequency to 1866MHz */
	if (ddr_set_mif_freq(ddr_ctx, AB_DRAM_FREQ_MHZ_1866)) {
		pr_err("train-all: revert back to 1866MHz fail\n");
		goto ddr_train_all_fail;
	}

	return DDR_SUCCESS;

ddr_train_all_fail:
	pr_err("ddr_train_all: Error!! ddr all frequency training failed\n");
	return DDR_FAIL;
}

static int32_t ab_ddr_train_all(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret;

	if (!ddr_ctx->is_setup_done) {
		pr_err("error: ddr setup is not called\n");
		return -EAGAIN;
	}

	/* Allow retrain only in 1866MHz frequency */
	if (ddr_ctx->cur_freq != AB_DRAM_FREQ_MHZ_1866) {
		pr_err("ddr train-all only supported during 1866MHz freq\n");
		return -EINVAL;
	}

	/* Perform train-all only during the ddr ON and SLEEP states */
	if ((ddr_ctx->ddr_state != DDR_ON) &&
	    (ddr_ctx->ddr_state != DDR_SLEEP)) {
		pr_err("ddr train-all: Invalid ddr state for retrain\n");
		return -EINVAL;
	}

	mutex_lock(&ddr_ctx->ddr_lock);
	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}
	ret = __ab_ddr_train_all(ddr_ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

/* Perform ddr re-training */
int32_t ab_ddr_train_sysreg(void *ctx)
{
	uint32_t ddr_sr;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("error: ddr setup is not called\n");
		return -EAGAIN;
	}

	/* Read the DDR_SR */
	ddr_sr = GPIO_DDR_SR();

	/* Set the train address to zero to make sure re-train happens instead
	 * of training restore
	 */
	SET_REG_TRN_ADDR(0);

	if (ab_ddr_train(ddr_ctx, ddr_sr)) {
		pr_err("train_sysreg: ddr training failed\n");
		return DDR_FAIL;
	}

	return DDR_SUCCESS;
}

static int ddr_config_post_initialization(struct ab_ddr_context *ddr_ctx)
{
	struct ddr_refresh_info_t *ref_info = &ddr_ctx->ref_info;

	/* Block AXI Before entering self-refresh */
	if (ddr_block_axi_transactions(ddr_ctx))
		return -ETIMEDOUT;

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode(ddr_ctx, REF_CTRL_ENABLE)) {
		pr_err("self-refresh entry fail during pwr features enable\n");
		ddr_reg_set(ddr_ctx, DREX_ACTIVATE_AXI_READY,
				ACTIVATE_AXI_READY);
		return -ETIMEDOUT;
	}

	/* Set MR13 register to control below fields for power optimization
	 * VRCG (VREF Current Generator) OP[3]
	 *       0B: Normal Operation (default)
	 *       1B: VREF Fast Response (high current) mode 3
	 * RRO Refresh rate option OP[4]
	 *       0B: Disable codes 001 and 010 in MR4 OP[2:0]
	 *       1B: Enable all codes in MR4 OP[2:0]
	 */
	ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, MRW13_FAST_RESP_DIS);

	/* Set the all-bank and per-bank auto refresh timings */
	ddr_reg_wr(ddr_ctx, DREX_TIMINGARE,
		   TIMINGARE(ref_info->t_refi, ref_info->t_refipb));

	/* Enabling DDR Power features */
	__ddr_enable_power_features(ddr_ctx);

	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_ENABLE)) {
		pr_err("self-refresh exit fail during power features enable\n");
		ddr_reg_set(ddr_ctx, DREX_ACTIVATE_AXI_READY,
				ACTIVATE_AXI_READY);
		return -ETIMEDOUT;
	}

	/* Enable Periodic Write Training */
	ddr_enable_hw_periodic_training(ddr_ctx);

	ddr_reg_set(ddr_ctx, DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);

	return 0;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int __ab_ddr_wait_for_m0_ddr_init(struct ab_ddr_context *ddr_ctx)
{
	unsigned long timeout;
	uint32_t ddr_train_status;

	timeout = jiffies + AB_DDR_INIT_TIMEOUT;
	while ((!(ddr_reg_rd(ddr_ctx, REG_DDR_TRAIN_STATUS) &
					DDR_TRAIN_COMPLETE))
			&& time_before(jiffies, timeout))
		ddr_usleep(DDR_POLL_USLEEP_MIN);

	/* check for the ddr training failure condition */
	ddr_train_status = ddr_reg_rd(ddr_ctx, REG_DDR_TRAIN_STATUS);
	if ((ddr_train_status & DDR_TRAIN_FAIL) ||
			!(ddr_train_status & DDR_TRAIN_COMPLETE)) {
		pr_err("%s: DDR Training failed during M0 boot. Status: 0x%x\n",
				__func__, ddr_train_status);
		return -EIO;
	}

	return 0;
}

int ab_ddr_wait_for_m0_ddr_init(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret;

	if (!ddr_ctx->is_setup_done) {
		pr_err("wait_for_ddr_init: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	ret = __ab_ddr_wait_for_m0_ddr_init(ddr_ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int32_t __ab_ddr_resume(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	struct ab_state_context *sc = ddr_ctx->ab_state_ctx;

	if (IS_DDR_OTP_FLASHED() && (ab_get_chip_id(sc) == CHIP_ID_A0) &&
	    IS_M0_DDR_INIT()) {

		/* IMPORTANT:
		 * ------------------------------------------------------------
		 * This path should not be HIT as there are high chances
		 * of data loss because of the below reasons. Please avoid
		 * the below combination.
		 *  [A0 SAMPLE + DDR OTP Flashed + NORMAL BOOT + M0_DDR_INIT]
		 *
		 * 1] In A0 BootROM, self-refresh exit sequence is called way
		 *    before Auto Refresh settings are enabled.
		 * 2] From  Suspend -> Resume scenario,
		 *    "AXI_Enable_After_All_training" sequence is skipped.
		 *
		 * Issue[1] can't be hadled. To fix Issue[2] call
		 * "AXI_Enable_After_All_training" sequence here.
		 */
		ddr_axi_enable_after_all_training(ddr_ctx);

		pr_err("%s: Error!! This path should not be used\n", __func__);
		WARN_ON(1);

		goto ddr_resume_fail;
	}

	/* Disable the DDR_SR GPIO */
	ab_gpio_disable_ddr_sr(sc);

	ddr_sanity_test(ctx, DDR_BOOT_TEST_READ);

	return DDR_SUCCESS;

ddr_resume_fail:
	pr_err("ddr_resume: unable to resume from suspend mode\n");
	return DDR_FAIL;
}

static int32_t ab_ddr_resume(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr_resume: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	ret = __ab_ddr_resume(ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int32_t __ab_ddr_suspend(void *ctx)
{
	int ret;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	struct ab_state_context *sc = ddr_ctx->ab_state_ctx;
	int prev_ldo2_state = 0;

	if (ddr_ctx->ddr_state == DDR_ON) {
		ddr_sanity_test(ctx, DDR_BOOT_TEST_WRITE);

		/* Block AXI Before entering self-refresh */
		if (ddr_block_axi_transactions(ddr_ctx))
			return -ETIMEDOUT;

		/* Self-refresh entry sequence */
		if (ddr_enter_self_refresh_mode(ddr_ctx, REF_CTRL_ENABLE))
			goto ddr_suspend_fail;

	} else if ((ddr_ctx->ddr_state == DDR_SLEEP) &&
		   (ddr_ctx->cur_freq != AB_DRAM_FREQ_MHZ_1866)) {

		/* This sections handles the case, when ddr suspend is requested
		 * from the sleep state. During sleep state, MIF clock is set to
		 * Oscillator. But to configure the MR settings, we need to move
		 * the MIF clock to PLL. Configuring MR registers may fail when
		 * MIF clock is set to lower frequencies like Oscillator clock.
		 */

		/* Move MIF clock to PLL to configure MR registers @1866MHz */
		if (ddr_set_pll_freq(ddr_ctx, ddr_ctx->cur_freq)) {
			pr_err("ddr suspend: mif pll config failed.\n");
			goto ddr_suspend_fail;
		}
	}

	/* No need to configure MR registers if current frequency is already
	 * set to 1866MHz.
	 */
	if (ddr_ctx->cur_freq != AB_DRAM_FREQ_MHZ_1866) {
		/* get the current LDO2 state (VDDQ rail) */
		prev_ldo2_state = regulator_is_enabled(sc->ldo2);

		/* If LDO2 was not enabled, momentarily enable LDO2 rail for
		 * configuring MR Registers.
		 */
		if (!prev_ldo2_state) {
			ret = regulator_enable(sc->ldo2);
			if (ret) {
				dev_err(sc->dev,
					"failed to enable LDO2 (%d)\n", ret);
				goto ddr_suspend_fail;
			}
		}

		/* MR Register configuration is not allowed when DRAM is in
		 * Power Down Entry mode. But during self-refresh entry command,
		 * DREX sends both Self-Refresh and Power Down Entry commands
		 * to DRAM.
		 *
		 * For the above reason, we need to bring the DRAM out of Power
		 * Down Entry mode.
		 */
		ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_PD_EXIT);

		/* During suspend -> resume (DDR_SR = 1), M0 bootrom will not
		 * update the MR registers specific to 1866MHz. As the ddr
		 * initialization happens at 1866MHz, make sure the MR registers
		 * are udpated for 1866MHz before entering to suspend state.
		 * Otherwise the read/write trainings will fail during ddr
		 * initialization (during suspend -> resume) in BootROM
		 */
		ddr_mrw_set_vref_odt_etc(ddr_ctx, AB_DRAM_FREQ_MHZ_1866);

		/* Set MR13 VRCG to default (Normal operation) for power saving
		 * VRCG (VREF Current Generator) OP[3]
		 *       0B: Normal Operation (default)
		 *       1B: VREF Fast Response (high current) mode 3
		 */
		ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, MRW13_FAST_RESP_DIS);

		/* As we are going to suspend state, keep the DRAM in
		 * Power Down Entry Mode.
		 */
		ddr_reg_wr(ddr_ctx, DREX_DIRECTCMD, CMD_TYPE_CKEL);

		/* Disable LDO2 if the previous LDO2 state was disabled */
		if (!prev_ldo2_state) {
			ret = regulator_disable(sc->ldo2);
			if (ret) {
				dev_err(sc->dev,
					"failed to disable LDO2 (%d)\n", ret);
				goto ddr_suspend_fail;
			}
		}
	}

	/* Move the MIF clock to oscillator and switch off the PLL for
	 * better power savings.
	 */
	ddr_set_pll_to_oscillator(ddr_ctx);

	/* Enable the PMU Retention */
	PMU_CONTROL_PHY_RET_ON();

	/* Enable GPIOs to inform DDR is in suspend mode */
	ab_gpio_enable_ddr_iso(sc);

	/* Airbrush will resume with 1866MHz */
	ddr_ctx->cur_freq = AB_DRAM_FREQ_MHZ_1866;

	return DDR_SUCCESS;

ddr_suspend_fail:
	pr_err("ddr_suspend: unable to enter suspend mode\n");
	return DDR_FAIL;
}

static int32_t ab_ddr_suspend(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret;

	if (!ddr_ctx->is_setup_done) {
		pr_err("ddr_suspend: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	ret = __ab_ddr_suspend(ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int32_t __ab_ddr_selfrefresh_exit(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	enum ddr_freq_t freq = ddr_ctx->cur_freq;

	/* Config MIF_PLL and move the MIF clock to PLL output */
	if (ddr_set_pll_freq(ddr_ctx, ddr_ctx->cur_freq)) {
		pr_err("%s: mif pll config failed.\n", __func__);
		return DDR_FAIL;
	}

	ab_sm_start_ts(AB_SM_TS_DDR_EXIT_SR_POWER_DIS);
	/* Disable power features for DLL locking */
	__ddr_disable_power_features(ddr_ctx);
	ab_sm_record_ts(AB_SM_TS_DDR_EXIT_SR_POWER_DIS);

	ab_sm_start_ts(AB_SM_TS_DDR_EXIT_SR_ENABLE_DLL);
	if (ddr_enable_dll(ddr_ctx)) {
		ab_sm_record_ts(AB_SM_TS_DDR_EXIT_SR_ENABLE_DLL);
		pr_warn("%s: enable dll failed.\n", __func__);

		/* Incase DLL Lock failure is observed during the self-refresh
		 * exit, perform the re-train sequence at the current frequency.
		 * This will include the re-initialization of PHY and the
		 * re-training sequences.
		 */
		ddr_ctx->ddr_train_completed[freq] = 0;
		if (ddr_phy_reinit_train(ddr_ctx, freq)) {
			pr_err("%s: phy reint and train failed\n", __func__);
			return DDR_FAIL;
		}

		/* Self-refresh exit sequence */
		if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_DISABLE)) {
			pr_err("%s: self-refresh exit failed\n", __func__);
			return DDR_FAIL;
		}

		ddr_axi_enable_after_all_training(ddr_ctx);
		return DDR_SUCCESS;
	}
	ab_sm_record_ts(AB_SM_TS_DDR_EXIT_SR_ENABLE_DLL);

	ab_sm_start_ts(AB_SM_TS_DDR_EXIT_SR_POWER_EN);
	/* Enable power features after DLL locking */
	__ddr_enable_power_features(ddr_ctx);
	ab_sm_record_ts(AB_SM_TS_DDR_EXIT_SR_POWER_EN);

	ab_sm_start_ts(AB_SM_TS_DDR_EXIT_SR_MODE);
	/* Self-refresh exit sequence */
	if (ddr_exit_self_refresh_mode(ddr_ctx, REF_CTRL_ENABLE)) {
		pr_err("%s: self-refresh exit failed\n", __func__);
		return DDR_FAIL;
	}
	ab_sm_record_ts(AB_SM_TS_DDR_EXIT_SR_MODE);

	ab_sm_start_ts(AB_SM_TS_DDR_EXIT_SR_FINISH);

	/* Enable Periodic Write Training */
	ddr_enable_hw_periodic_training(ddr_ctx);

	/* Allow AXI after exiting from self-refresh */
	ddr_reg_set(ddr_ctx, DREX_ACTIVATE_AXI_READY, ACTIVATE_AXI_READY);

	ddr_sanity_test(ctx, DDR_BOOT_TEST_READ);

	ab_sm_record_ts(AB_SM_TS_DDR_EXIT_SR_FINISH);
	return DDR_SUCCESS;
}

static int32_t ab_ddr_selfrefresh_exit(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret;

	if (!ddr_ctx->is_setup_done) {
		pr_err("SRX: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	ret = __ab_ddr_selfrefresh_exit(ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int32_t __ab_ddr_selfrefresh_enter(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	ddr_sanity_test(ddr_ctx, DDR_BOOT_TEST_WRITE);

	/* Disable Periodic Write Training */
	ddr_disable_hw_periodic_training(ddr_ctx);

	/* Block AXI Before entering self-refresh */
	if (ddr_block_axi_transactions(ddr_ctx))
		return -ETIMEDOUT;

	/* Self-refresh entry sequence */
	if (ddr_enter_self_refresh_mode(ddr_ctx, REF_CTRL_ENABLE)) {
		pr_err("%s: self-refresh entry failed\n", __func__);
		return DDR_FAIL;
	}

	/* Move the MIF clock to oscillator */
	ddr_set_pll_to_oscillator(ddr_ctx);

	return DDR_SUCCESS;
}

static int32_t ab_ddr_selfrefresh_enter(void *ctx)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	int ret;

	if (!ddr_ctx->is_setup_done) {
		pr_err("SRE: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	ret = __ab_ddr_selfrefresh_enter(ctx);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

static int ab_ddr_get_freq(void *ctx, u64 *val)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	u64 freq = 0;
	int ret = DDR_SUCCESS;

	if (!ddr_ctx->is_setup_done) {
		pr_err("get_freq: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	switch (ddr_ctx->cur_freq) {
	case AB_DRAM_FREQ_MHZ_1866:
		freq = DRAM_CLK_1866MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_1600:
		freq = DRAM_CLK_1600MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_1200:
		freq = DRAM_CLK_1200MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_933:
		freq = DRAM_CLK_933MHZ;
		break;
	case AB_DRAM_FREQ_MHZ_800:
		freq = DRAM_CLK_800MHZ;
		break;
	default:
		ret = DDR_FAIL;
		break;
	}

	if ((ddr_ctx->ddr_state == DDR_SLEEP) ||
	    (ddr_ctx->ddr_state == DDR_SUSPEND)) {
		freq = DRAM_CLK_OSC;
	} else if (ddr_ctx->ddr_state == DDR_OFF) {
		freq = 0;
	}

	*val = freq;
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

/* Caller must hold ddr_ctx->ddr_lock */
static int __ab_ddr_set_freq(void *ctx, u64 val)
{
	int ret = DDR_FAIL;

	switch (val) {
	case DRAM_CLK_1866MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_1866);
		break;
	case DRAM_CLK_1600MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_1600);
		break;
	case DRAM_CLK_1200MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_1200);
		break;
	case DRAM_CLK_933MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_933);
		break;
	case DRAM_CLK_800MHZ:
	case DRAM_CLK_200MHZ:
		ret = ddr_set_mif_freq(ctx, AB_DRAM_FREQ_MHZ_800);
		break;
	default:
		break;
	}

	return ret;
}

static int ab_ddr_set_freq(void *ctx, u64 val)
{
	int ret = DDR_FAIL;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;

	if (!ddr_ctx->is_setup_done) {
		pr_err("set_freq: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	ret = __ab_ddr_set_freq(ctx, val);
	mutex_unlock(&ddr_ctx->ddr_lock);

	return ret;
}

static int ab_ddr_set_state(const struct block_property *prop_from,
			const struct block_property *prop_to,
			enum block_state block_state_id, void *data)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)data;
	unsigned long old_rate;
	unsigned long new_rate;
	unsigned long extra_pre_notify_flag = 0;
	unsigned long extra_post_notify_flag = 0;
	int ret = 0;

	if (!ddr_ctx->is_setup_done) {
		pr_err("set_state: Error!! ddr setup is not called\n");
		return -EAGAIN;
	}

	if (!prop_from || !prop_to)
		return -EINVAL;

	/*
	 * If DRAM is going to enter block_state_0 (power off), data will
	 * be lost.  Notify subscribers in advance both pre-rate and post-rate.
	 */
	if (block_state_id == BLOCK_STATE_0) {
		extra_pre_notify_flag = AB_DRAM_DATA_PRE_OFF;
		extra_post_notify_flag = AB_DRAM_DATA_POST_OFF;
	}

	/*
	 * DDR state will not be ON if block_state_id < 300. Make sure to
	 * cancel already scheduled work before DDR operations.
	 */
	if (block_state_id < BLOCK_STATE_300)
		cancel_delayed_work_sync(&ddr_ctx->ddr_ref_control_work);

	mutex_lock(&ddr_ctx->ddr_lock);

	/* entering BLOCK_STATE_0 doesn't require PCIe access */
	if (!ddr_ctx->pcie_link_ready && block_state_id != BLOCK_STATE_0) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	old_rate = prop_from->clk_frequency;
	new_rate = prop_to->clk_frequency;
	ab_sm_clk_notify(AB_DRAM_PRE_RATE_CHANGE | extra_pre_notify_flag,
			 old_rate, new_rate);

	switch (block_state_id) {
	case BLOCK_STATE_300 ... BLOCK_STATE_305:
		if (ddr_ctx->ddr_state == DDR_SLEEP)
			ret |= __ab_ddr_selfrefresh_exit(ddr_ctx);
		else if (ddr_ctx->ddr_state == DDR_SUSPEND)
			ret |= __ab_ddr_resume(ddr_ctx);

		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_ON;

		mutex_unlock(&ddr_ctx->ddr_lock);
		ddr_refresh_control_wkqueue(
				&ddr_ctx->ddr_ref_control_work.work);
		mutex_lock(&ddr_ctx->ddr_lock);
		break;

	case BLOCK_STATE_101:
		/* ddr sleep/deep-sleep functionality */

		/* Allow all the state transitions where DDR state is in
		 * Self-refresh mode.
		 */
		if (ddr_ctx->ddr_state == DDR_SLEEP)
			goto set_state_complete;

		if (ddr_ctx->ddr_state == DDR_SUSPEND) {
			if (__ab_ddr_resume(ddr_ctx))
				goto set_state_fail;
		}

		ret |= __ab_ddr_selfrefresh_enter(ddr_ctx);

		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_SLEEP;
		break;

	case BLOCK_STATE_100:
		/* ddr suspend functionality */
		if ((ddr_ctx->ddr_state == DDR_SUSPEND) ||
			(ddr_ctx->ddr_state == DDR_OFF))
			goto set_state_complete;

		ret |= __ab_ddr_suspend(ddr_ctx);

		ab_ddr_clear_cache(ddr_ctx);
		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_SUSPEND;
		break;

	case BLOCK_STATE_0:
		if (ddr_ctx->ddr_state == DDR_SUSPEND) {
			ab_gpio_disable_ddr_sr(ddr_ctx->ab_state_ctx);
			ab_gpio_disable_ddr_iso(ddr_ctx->ab_state_ctx);
		}
		ab_ddr_clear_cache(ddr_ctx);
		ddr_ctx->prev_ddr_state = ddr_ctx->ddr_state;
		ddr_ctx->ddr_state = DDR_OFF;
		break;

	default:
		break;
	}

	if (ddr_ctx->ddr_state == DDR_ON)
		ret |= __ab_ddr_set_freq(ddr_ctx, prop_to->clk_frequency);

	if (ret)
		goto set_state_fail;

set_state_complete:
	ab_sm_clk_notify(AB_DRAM_POST_RATE_CHANGE | extra_post_notify_flag,
			 old_rate, new_rate);
	mutex_unlock(&ddr_ctx->ddr_lock);
	return 0;

set_state_fail:
	ab_sm_clk_notify(AB_DRAM_ABORT_RATE_CHANGE, old_rate, new_rate);
	mutex_unlock(&ddr_ctx->ddr_lock);
	return -EINVAL;
}

static int32_t ab_ddr_setup(void *ctx, void *ab_ctx)
{
#ifdef DEBUG
	int otp_idx;
#endif
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	struct ab_state_context *sc = (struct ab_state_context *)ab_ctx;

	if (!sc) {
		pr_err("ddr_setup: Invalid state context\n");
		return -EINVAL;
	}

	/* ab_ddr_setup is already called */
	if (ddr_ctx->is_setup_done)
		return DDR_SUCCESS;

	ab_sm_start_ts(AB_SM_TS_DDR_SETUP);
	/* Incase of OTPs are flashed, get all information from OTPs.
	 * If ddr OTPs are not flashed, depend on dt properties
	 */
	if (IS_DDR_OTP_FLASHED()) {
		/* DDR OTPs are flashed, use OTP wrapper to read ddr OTPs*/
		dev_dbg(ddr_ctx->dev, "ddr_setup: DDR OTPs fused\n");
		ddr_otp_rd = &read_otp_wrapper;

		if (ab_get_chip_id(sc) == CHIP_ID_A0) {
			dev_info(ddr_ctx->dev, "Airbrush Revision: A0\n");
		} else if (ab_get_chip_id(sc) == CHIP_ID_B0) {
			dev_info(ddr_ctx->dev, "Airbrush Revision: B0\n");
		} else {
			/* add support for other revisions */
			pr_err("%s: Invalid version ID\n", __func__);
			WARN_ON(1);
		}
	} else {

		/* If DDR OTP is not flashed, then use the OTP array instead */
		dev_warn(ddr_ctx->dev,
			 "%s: DDR OTPs NOT fused. Use local array.\n",
			 __func__);
		ddr_otp_rd = &read_otp_array;
	}

#ifdef DEBUG
	for (otp_idx = 0; otp_idx < o_DDR_OTP_MAX; otp_idx++)
		dev_dbg(ddr_ctx->dev,
			"%d: 0x%x\n", otp_idx, ddr_otp_rd(otp_idx));
#endif

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	/* Keeps track of setup call.
	 * Should be checked in all other dram_ops callbacks
	 */
	ddr_ctx->is_setup_done = 1;
	ddr_ctx->ab_state_ctx = sc;
	mutex_unlock(&ddr_ctx->ddr_lock);

	ab_sm_record_ts(AB_SM_TS_DDR_SETUP);
	return DDR_SUCCESS;
}

static int32_t ab_ddr_init(void *ctx)
{
	int32_t ret = 0;
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)ctx;
	uint32_t ddr_sr;

	if (!ddr_ctx->is_setup_done) {
		pr_err("%s, error: ddr setup is not called", __func__);
		return -EAGAIN;
	}

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_err("%s: pcie link not ready\n", __func__);
		mutex_unlock(&ddr_ctx->ddr_lock);
		return -EINVAL;
	}

	/* set 1866MHz ddr clock during airbrush normal and resume boot */
	ddr_ctx->cur_freq = AB_DRAM_FREQ_MHZ_1866;

	if (IS_M0_DDR_INIT()) {
		ab_sm_start_ts(AB_SM_TS_DDR_M0_INIT_INTERNAL);
		/* Perform the post ddr init sequences which are only applicable
		 * when ddr init is done by M0 bootrom.
		 */
		/* post init config and enabling the ddr power features */
		ret = ddr_config_post_initialization(ddr_ctx);
		if (ret) {
			pr_err("ddr_init: error!! enabling power features failed\n");
			goto ddr_init_done;
		}
		/* In A0 BootROM Auto Refresh setting is missing.
		 * To fix this issue, set the Auto-Refresh enable from HOST
		 * immediately after M0 DDR init.
		 */
		if (ab_get_chip_id(ddr_ctx->ab_state_ctx) == CHIP_ID_A0)
			ddr_reg_set(ddr_ctx, DREX_CONCONTROL, AREF_EN);

		/* DDR training is completed as part of bootrom code execution.
		 * Copy the training results to the local array for future use.
		 */
		ddr_ctx->ddr_train_sram_location =
				ddr_reg_rd(ddr_ctx, SYSREG_REG_TRN_ADDR);

		/* Check the ddr training data is saved to SRAM and the valid
		 * SRAM location is updated to SYSREG_REG_TRN_ADDR by bootrom.
		 */
		if (!ddr_ctx->ddr_train_sram_location) {
			ret = -EFAULT;
			pr_err("error!! ddr train data location not updated\n");
			goto ddr_init_done;
		}

		ddr_copy_train_results_from_sram(ddr_ctx);
		ab_sm_record_ts(AB_SM_TS_DDR_M0_INIT_INTERNAL);
	} else {
		ab_sm_start_ts(AB_SM_TS_DDR_INIT_INTERNAL);
		/* HOST should perform DDR init and train sequences */
		ret = ab_ddr_init_internal_isolation(ddr_ctx);
		ab_sm_record_ts(AB_SM_TS_DDR_INIT_INTERNAL);
		if (ret) {
			pr_err("ddr_init: error!! ddr initialization failed\n");
			goto ddr_init_done;
		}
	}

	/* Clear all training results except for AB_DRAM_FREQ_MHZ_1866.
	 * AB_DRAM_FREQ_MHZ_1866 frequency is trained as part of the above
	 * init sequence. As all other frequencies training is dependent on
	 * AB_DRAM_FREQ_MHZ_1866 train results, we are clearing the train
	 * complete flags for all other frequencies. This will allow the other
	 * frequencies to be re-trained during the ddr frequency switch.
	 */
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1866] = 1;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1600] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_1200] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_933] = 0;
	ddr_ctx->ddr_train_completed[AB_DRAM_FREQ_MHZ_800] = 0;

	/* Read the DDR_SR */
	ddr_sr = GPIO_DDR_SR();

	if (!ddr_sr)
		ddr_sanity_test(ctx, DDR_BOOT_TEST_READ_WRITE);

ddr_init_done:
	mutex_unlock(&ddr_ctx->ddr_lock);
	return ret;
}

#ifndef CONFIG_AB_DDR_RW_TEST
static int ab_ddr_read_write_test(void *ctx, unsigned int read_write)
{
	return -ENODEV;
}
#endif

#ifndef CONFIG_AB_DDR_EYE_MARGIN
static int ab_ddr_eye_margin(void *ctx, unsigned int test_data)
{
	return -ENODEV;
}

static int ab_ddr_eye_margin_plot(void *ctx)
{
	return -ENODEV;
}
#endif

#ifndef CONFIG_AB_DDR_PPC
static int ab_ddr_ppc_set_event(void *ctx, unsigned int counter_idx,
				  unsigned int event)
{
	return -ENODEV;
}

static void ab_ddr_ppc_ctrl(void *ctx, int ppc_start)
{
}
#endif

static struct ab_sm_dram_ops dram_ops = {
	.ctx = NULL,

	.setup = &ab_ddr_setup,
	.wait_for_m0_ddr_init = &ab_ddr_wait_for_m0_ddr_init,
	.init = &ab_ddr_init,
	.train_all = &ab_ddr_train_all,
	.get_freq = &ab_ddr_get_freq,
	.set_freq = &ab_ddr_set_freq,
	.suspend = &ab_ddr_suspend,
	.resume = &ab_ddr_resume,
	.sref_enter = &ab_ddr_selfrefresh_enter,
	.sref_exit = &ab_ddr_selfrefresh_exit,
	.rw_test = &ab_ddr_read_write_test,
	.eye_margin = &ab_ddr_eye_margin,
	.eye_margin_plot = &ab_ddr_eye_margin_plot,
	.ppc_set_event = &ab_ddr_ppc_set_event,
	.ppc_ctrl = &ab_ddr_ppc_ctrl,
};

static void ddr_refresh_control_wkqueue(struct work_struct *refresh_ctrl_wq)
{
	struct ab_ddr_context *ddr_ctx = (struct ab_ddr_context *)dram_ops.ctx;
	uint32_t mr4_reg, rr;
	const struct ddr_refresh_info_t *r_rate;
	static const struct ddr_refresh_info_t refresh_info[RR_MAX] = {
		[RR_4x]     = { T_REFI_4x,    T_REFIPB_4x    },
		[RR_2x]     = { T_REFI_2x,    T_REFIPB_2x    },
		[RR_1x]     = { T_REFI_1x,    T_REFIPB_1x    },
		[RR_0_5x]   = { T_REFI_0_50x, T_REFIPB_0_50x },
		[RR_0_25x]  = { T_REFI_0_25x, T_REFIPB_0_25x },
		[RR_0_25xd] = { T_REFI_0_25x, T_REFIPB_0_25x },
	};

	mutex_lock(&ddr_ctx->ddr_lock);

	if (!ddr_ctx->pcie_link_ready) {
		pr_debug("%s: pcie link not ready\n", __func__);
		goto mr_poll_thread_sleep;
	}

	/* Refresh rate control is only required during ddr ON state */
	if (ddr_ctx->ddr_state != DDR_ON)  {
		WARN(1, "refresh control work while not in DDR_ON");
		mutex_unlock(&ddr_ctx->ddr_lock);
		return;
	}

	/* Read MR4 register to get the refresh rate to be
	 * applied at the DREX controller side.
	 */
	mr4_reg = ddr_read_mr_reg(ddr_ctx, 4);

	/* Refresh Rate OP[2:0] */
	rr = mr4_reg & 0x7;
	if (rr == ddr_ctx->ref_rate)
		goto mr_poll_thread_sleep;

	if ((rr < RR_4x) || (rr > RR_0_25xd))
		goto mr_poll_thread_sleep;

	pr_info("Refresh Rate: current: %d, prev: %d\n",
			rr, ddr_ctx->ref_rate);

	r_rate = &refresh_info[rr];

	/* Set the all-bank and per-bank auto refresh timings */
	ddr_reg_wr(ddr_ctx, DREX_TIMINGARE,
			TIMINGARE(r_rate->t_refi, r_rate->t_refipb));

	ddr_ctx->ref_info.t_refipb = r_rate->t_refipb;
	ddr_ctx->ref_info.t_refi = r_rate->t_refi;
	ddr_ctx->ref_rate = rr;

mr_poll_thread_sleep:
	mutex_unlock(&ddr_ctx->ddr_lock);
	schedule_delayed_work(&ddr_ctx->ddr_ref_control_work,
				msecs_to_jiffies(DDR_REFCTRL_POLL_TIME_MSEC));
}

static int ab_ddr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ab_ddr_context *ddr_ctx;
	int err;

	ddr_ctx = kzalloc(sizeof(struct ab_ddr_context), GFP_KERNEL);
	if (ddr_ctx == NULL)
		return -ENOMEM;

#ifdef CONFIG_AB_DDR_EYE_MARGIN
	ddr_ctx->eye_data = vmalloc(sizeof(struct ddr_eyemargin_data));
	if (ddr_ctx->eye_data == NULL)
		pr_err("ab_ddr: memory alloc for eyemargin data failed\n");
#endif

	/* initialize ddr state to off */
	ddr_ctx->ddr_state = DDR_OFF;

	/* Set the default refresh rate to 1x (RR_1x) */
	ddr_ctx->ref_rate = RR_1x;
	ddr_ctx->ref_info.t_refipb = T_REFIPB_1x;
	ddr_ctx->ref_info.t_refi = T_REFI_1x;

	ddr_ctx->dev = dev;
	ab_ddr_clear_cache(ddr_ctx);
	ddr_ctx->poll_multiplier = -1;

	mutex_init(&ddr_ctx->ddr_lock);

	/* Create Work Queue for DDR Refresh Control */
	INIT_DELAYED_WORK(&ddr_ctx->ddr_ref_control_work,
			  ddr_refresh_control_wkqueue);

	ddr_ctx->pcie_link_ready = true;
	ddr_ctx->pcie_link_blocking_nb.notifier_call =
			ab_ddr_pcie_link_listener;
	err = abc_register_pcie_link_blocking_event(
			&ddr_ctx->pcie_link_blocking_nb);
	if (err) {
		dev_err(dev,
			"fail: PCIe blocking link event subscribe, ret %d\n",
			err);
		return err;
	}

	dram_ops.ctx = ddr_ctx;
	ab_sm_register_dram_ops(&dram_ops);

	/* Register the Airbrush State Manager (ASM) callback */
	ab_sm_register_blk_callback(DRAM, &ab_ddr_set_state, ddr_ctx);

	return 0;
}

static int ab_ddr_remove(struct platform_device *pdev)
{
	struct ab_ddr_context *ddr_ctx;

	ddr_ctx = (struct ab_ddr_context *)dram_ops.ctx;
	cancel_delayed_work_sync(&ddr_ctx->ddr_ref_control_work);
	ab_sm_unregister_dram_ops();

	return 0;
}

static const struct of_device_id ab_ddr_of_match[] = {
		{ .compatible = "abc,airbrush-ddr", },
		{ },
};

static struct platform_driver ab_ddr_driver = {
	.probe = ab_ddr_probe,
	.remove = ab_ddr_remove,
	.driver = {
		.name = "airbrush-ddr",
		.of_match_table = ab_ddr_of_match,
	},
};
module_platform_driver(ab_ddr_driver);
