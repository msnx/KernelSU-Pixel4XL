/*
 * iaxxx-regmap.c -- Register map data for IAxxx series devices
 *
 * Copyright 2016 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ao.h>
#include <linux/mfd/adnc/iaxxx-register-defs-i2s.h>
#include <linux/mfd/adnc/iaxxx-register-defs-af.h>
#include <linux/mfd/adnc/iaxxx-register-defs-cnr0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm0.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm1.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm2.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm3.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm4.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pcm5.h>
#include <linux/mfd/adnc/iaxxx-register-defs-ioctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pad-ctrl.h>
#include <linux/mfd/adnc/iaxxx-register-defs-gpio.h>
#include <linux/mfd/adnc/iaxxx-channel-registers.h>
#include <linux/mfd/adnc/iaxxx-sensor-registers.h>
#include <linux/mfd/adnc/iaxxx-stream-registers.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-tunnel-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pkg-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-in-endpoint-group.h>
#include <linux/mfd/adnc/iaxxx-register-defs-out-endpoint-group.h>
#include <linux/mfd/adnc/iaxxx-register-defs-debug.h>
#include <linux/mfd/adnc/iaxxx-register-defs-event-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-script-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-register-defs-bulk-transfer.h>
#include "iaxxx.h"
#include <linux/mfd/adnc/iaxxx-core.h>


#define IAXXX_CH_MAX 32
#define IAXXX_PLUGIN_MAX 16
#define IAXXX_PLUGIN_UPDATE_BLOCKS_MAX 3
#define IAXXX_STRM_MAX 16
#define IAXXX_TNL_MAX 32
#define IAXXX_SENSOR_MAX  4
#define IAXXX_MAX_MULTI_PLUGIN_HDRS 2

#define IAXXX_SRB_ARB_N_SIZE(N)		(IAXXX_SRB_ARB_0_SIZE_ADDR+(N*8))
#define IAXXX_SRB_ARB_N_BASE_ADDRESS(N)	(IAXXX_SRB_ARB_0_BASE_ADDR_ADDR+(N*8))

#define IAXXX_MAX_REGISTER	  (IAXXX_SRB_ARB_N_SIZE(31))
#define IAXXX_SRB_SIZE		  (IAXXX_SRB_REG_NUM*4)

#define IAXXX_BLOCK_CHANNEL				1
#define IAXXX_BLOCK_STREAM				2
#define IAXXX_BLOCK_TUNNEL				3
#define IAXXX_BLOCK_PACKAGE				4
#define IAXXX_BLOCK_PLUGIN				5
#define IAXXX_BLOCK_DBGLOG				0xa
#define IAXXX_BLOCK_BTP 				0xb
#define IAXXX_BLOCK_SENSOR				0xd
#define IAXXX_BLOCK_POWER				0xe
#define IAXXX_BLOCK_EVENT				0xf
#define IAXXX_BLOCK_SCRIPT				0x10

#define IAXXX_VIRTUAL_ADDR_START		0x01000000
#define IAXXX_VIRTUAL_ADDR_END			0x10FFFFFF

#define IAXXX_VIRTUAL_BASE_ADDR(B)				     ((B) << 24)
#define IAXXX_INDEX_FROM_VIRTUAL(A)				     ((A) >> 24)

#define RBDT_OFFSET(X)	(((X) - IAXXX_SRB_ARB_0_BASE_ADDR_ADDR)/sizeof(u32))
#define RBDT_N_SIZE(N)	RBDT_OFFSET(IAXXX_SRB_ARB_N_SIZE(N))
#define RBDT_N_ADDR(N)	RBDT_OFFSET(IAXXX_SRB_ARB_N_BASE_ADDRESS(N))

#define SRB_BLOCK_SELECT_REG	IAXXX_SRB_HOST_DEFINED_1_ADDR
#define SRB_BLOCK_SELECT_MASK	0x1F
#define SRB_BLOCK_SELECT_SHIFT	0

#define IAXXX_REGMAP_NO_PM_NUM_ARBS     4

#define IAXXX_PHYSICAL_ADDRESS_BASE	0x50000000
#define IAXXX_I2S_SIZE			0x00001000
#define IAXXX_PCM_SIZE			0x00001000
#define IAXXX_CNR_SIZE			0x00001000
#define IAXXX_IOCTRL_SIZE		0x00001000
#define IAXXX_PAD_CTRL_SIZE		0x00001000
#define IAXXX_GPIO_SIZE			0x00001000
#define IAXXX_SRB_BASE			IAXXX_SRB_REGS_ADDR
#define IAXXX_BOSS_DRAM_BASE		0xA0000000

#define IAXXX_I2S_REGS_END_ADDR	 (IAXXX_I2S_REGS_ADDR + IAXXX_I2S_SIZE)
#define IAXXX_AO_REGS_END_ADDR	 (IAXXX_AO_REGS_ADDR + (4 * IAXXX_AO_REG_NUM))
#define IAXXX_CNR0_REGS_END_ADDR (IAXXX_CNR0_REGS_ADDR + IAXXX_CNR_SIZE)
#define IAXXX_IOCTRL_REGS_END_ADDR (IAXXX_IO_CTRL_REGS_ADDR + IAXXX_IOCTRL_SIZE)
#define IAXXX_PAD_CTRL_REGS_END_ADDR (IAXXX_PAD_CTRL_REGS_ADDR + \
	IAXXX_PAD_CTRL_SIZE)
#define IAXXX_GPIO_REGS_END_ADDR (IAXXX_GPIO_REGS_ADDR + IAXXX_GPIO_SIZE)

#define PCM_REG_NAME(N)  IAXXX_PCM ## N ## _REGS_ADDR
#define IAXXX_PCM_BASE_ADDR(N) PCM_REG_NAME(N)
#define IAXXX_PCM_REGS_END_ADDR(N) (IAXXX_PCM_BASE_ADDR(N) + IAXXX_PCM_SIZE)

/* Virtual addresses */
#define IAXXX_REG_CHANNEL_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_CHANNEL)
#define IAXXX_REG_STREAM_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_STREAM)
#define IAXXX_REG_PACKAGE_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_PACKAGE)
#define IAXXX_REG_PLUGIN_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_PLUGIN)
#define IAXXX_REG_TUNNEL_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_TUNNEL)
#define IAXXX_REG_DBGLOG_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_DBGLOG)
#define IAXXX_REG_EVENT_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_EVENT)
#define IAXXX_REG_SENSOR_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_SENSOR)
#define IAXXX_REG_SCRIPT_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_SCRIPT)
#define IAXXX_REG_POWER_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_POWER)
#define IAXXX_REG_BTP_BASE	IAXXX_VIRTUAL_BASE_ADDR(IAXXX_BLOCK_BTP)

static const uint32_t iaxxx_phy_addr_ranges[][2] = {
	/* Start phy address */  /* End phy address */
	{IAXXX_SRB_BASE, (IAXXX_SRB_BASE + IAXXX_SRB_SIZE)},
	{IAXXX_I2S_REGS_ADDR, IAXXX_I2S_REGS_END_ADDR},
	{IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR,
			IAXXX_AF_WCPT_WALL_CLOCK_RD_1_ADDR},
	{IAXXX_AO_REGS_ADDR, IAXXX_AO_REGS_END_ADDR},
	{IAXXX_CNR0_REGS_ADDR, IAXXX_CNR0_REGS_END_ADDR},
	{IAXXX_PCM_BASE_ADDR(0), IAXXX_PCM_REGS_END_ADDR(0)},
	{IAXXX_PCM_BASE_ADDR(1), IAXXX_PCM_REGS_END_ADDR(1)},
	{IAXXX_PCM_BASE_ADDR(2), IAXXX_PCM_REGS_END_ADDR(2)},
	{IAXXX_PCM_BASE_ADDR(3), IAXXX_PCM_REGS_END_ADDR(3)},
	{IAXXX_PCM_BASE_ADDR(4), IAXXX_PCM_REGS_END_ADDR(4)},
	{IAXXX_PCM_BASE_ADDR(5), IAXXX_PCM_REGS_END_ADDR(5)},
	{IAXXX_IO_CTRL_REGS_ADDR, IAXXX_IOCTRL_REGS_END_ADDR},
	{IAXXX_PAD_CTRL_REGS_ADDR, IAXXX_PAD_CTRL_REGS_END_ADDR},
	{IAXXX_GPIO_REGS_ADDR, IAXXX_GPIO_REGS_END_ADDR},
};

/* Returns true if register is in the supported physical address range */
static inline bool iaxxx_is_physical_address(uint32_t reg)
{
	int i;
	int phy_addr_size = sizeof(iaxxx_phy_addr_ranges) /
					sizeof(iaxxx_phy_addr_ranges[0]);

	for (i = 0; i < phy_addr_size; i++) {
		if (reg >= iaxxx_phy_addr_ranges[i][0] &&
				reg < iaxxx_phy_addr_ranges[i][1])
			return true;
	}
	return false;
}

/*
 * readable_register needed to optimize access and bus usage
 */
static bool iaxxx_readable_register(struct device *dev, unsigned int reg)
{
	int i;
	const struct regmap_range_cfg *cfg;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (WARN_ON(!priv))
		return false;	/* Something went wrong */

	/* ONLY if FW is booted, allow reads to this regmap */
	if (!iaxxx_is_firmware_ready(priv))
		return false;

	cfg = priv->regmap_config->ranges;

	/* Virtual addresses are only supported when mapped */
	if (reg >= IAXXX_VIRTUAL_ADDR_START && reg <= IAXXX_VIRTUAL_ADDR_END) {
		if (!cfg)
			return false;

		/* TODO: use lookup table Block Index -> *regmap_range_cfg */
		/* For now, walk through the ranges and check the address */
		for (i = 0; i < priv->regmap_config->num_ranges; ++i, ++cfg)
			if ((reg >= cfg->range_min) && (reg <= cfg->range_max))
				return true;
	}

	/* Support physical address */
	return iaxxx_is_physical_address(reg);
}

static bool iaxxx_writeable_register(struct device *dev, unsigned int reg)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (WARN_ON(!priv))
		return false;	/* Something went wrong */

	/* ONLY if FW is booted allow writes to this regmap */
	if (!iaxxx_is_firmware_ready(priv))
		return false;

	return true;
}

static bool iaxxx_volatile_register(struct device *dev, unsigned int reg)
{
		return true;
}

static bool iaxxx_application_volatile_reg(struct device *dev, unsigned int reg)
{
	int i;
	int j;

	switch (reg) {
	/* I2S Registers */
	case IAXXX_I2S_I2S_TRIGGER_GEN_ADDR:
	/* PCM registers */
	case IAXXX_PCM0_RIS_ADDR:
	case IAXXX_PCM1_RIS_ADDR:
	case IAXXX_PCM2_RIS_ADDR:
	case IAXXX_PCM3_RIS_ADDR:
	case IAXXX_PCM4_RIS_ADDR:
	case IAXXX_PCM5_RIS_ADDR:
	/* Wall clock registers */
	case IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR:
	case IAXXX_AF_WCPT_WALL_CLOCK_RD_1_ADDR:
	/* Channel ARB Regs */
	case IAXXX_CH_HDR_CH_CNT_ADDR:
	case IAXXX_CH_HDR_CH_GAIN_ADDR:
	/* Stream ARB regs */
	case IAXXX_STR_HDR_STR_CNT_ADDR:
	/* Stream status reg */
	case IAXXX_STR_HDR_STR_ST_ADDR:
	/* Package ARB regs */
	case IAXXX_PKG_MGMT_PKG_REQ_ADDR:
	case IAXXX_PKG_MGMT_PKG_ERROR_ADDR:
	case IAXXX_PKG_MGMT_PKG_IADDR_P_ADDR:
	case IAXXX_PKG_MGMT_PKG_DADDR_P_ADDR:
	/* Tunnel ARB Regs */
	case IAXXX_TNL_HDR_TNL_COUNT_ADDR:
	/* Endpoints ARB Regs */
	case IAXXX_IN_EP_GRP_STATUS_ADDR:
		return true;
	}

	/* All SRB Registers except SRB Block Selct Register.
	 * SRB Block Select register is excluded because FW does
	 * not require Page Select to be able to read/write
	 */
	if ((reg != SRB_BLOCK_SELECT_REG) && reg >= IAXXX_SRB_BASE &&
		reg < (IAXXX_SRB_BASE + IAXXX_SRB_SIZE))
		return true;

	/* All Power Management Registers */
	if (reg >= IAXXX_PWR_MGMT_REGS_ADDR && reg <
		(IAXXX_PWR_MGMT_REGS_ADDR + (IAXXX_PWR_MGMT_REG_NUM*4)))
		return true;

	/* All event Management Registers */
	if (reg >= IAXXX_EVT_MGMT_EVT_ADDR &&
		reg < (IAXXX_EVT_MGMT_EVT_ADDR + (IAXXX_EVT_MGMT_REG_NUM*4)))
		return true;

	/* PCM Registers */
	if (reg >= IAXXX_PCM_BASE_ADDR(0) && reg < IAXXX_PCM_REGS_END_ADDR(0))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(1) && reg < IAXXX_PCM_REGS_END_ADDR(1))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(2) && reg < IAXXX_PCM_REGS_END_ADDR(2))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(3) && reg < IAXXX_PCM_REGS_END_ADDR(3))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(4) && reg < IAXXX_PCM_REGS_END_ADDR(4))
		return true;
	if (reg >= IAXXX_PCM_BASE_ADDR(5) && reg < IAXXX_PCM_REGS_END_ADDR(5))
		return true;

	/* I2S Registers */
	if (reg >= IAXXX_I2S_I2S0_HL_ADDR &&
			reg <= IAXXX_I2S_I2S_TRIGGER_GEN_ADDR)
		return true;

	/* AO registers */
	if (reg >= IAXXX_AO_REGS_ADDR && reg <= IAXXX_AO_REGS_END_ADDR)
		return true;
	/* CNR registers */
	if (reg >= IAXXX_CNR0_REGS_ADDR && reg <= IAXXX_CNR0_REGS_END_ADDR)
		return true;
	/* IOCTL registers */
	if (reg >= IAXXX_IO_CTRL_REGS_ADDR && reg <= IAXXX_IOCTRL_REGS_END_ADDR)
		return true;
	/* PAD CTRL registers */
	if (reg >= IAXXX_PAD_CTRL_REGS_ADDR && reg <
		IAXXX_PAD_CTRL_REGS_END_ADDR)
		return true;
	/* GPIO registers */
	if (reg >= IAXXX_GPIO_REGS_ADDR && reg < IAXXX_GPIO_REGS_END_ADDR)
		return true;
	/* Tunnel header registers */
	if (reg >= IAXXX_TNL_HDR_TNL_OUT_BUF_SIZE_ADDR &&
			reg <= IAXXX_TNL_HDR_TNL_OUT_BUF_TAIL_ADDR)
		return true;
	/* Sensor header registers */
	if (reg >= IAXXX_SENSOR_HDR_SENSOR_CNT_ADDR &&
			reg <= IAXXX_SENSOR_HDR_GET_PARAM_REQ_ADDR)
		return true;

	/* Stream group registers */
	for (i = 0; i < IAXXX_STRM_MAX; i++) {
		if (reg == IAXXX_STR_GRP_STR_STATUS_REG(i))
			return true;
		if ((reg >=
			IAXXX_STR_GRP_STR_AF_ERROR_AFS_FIFO_OVERFLOW_CNT_REG
			(i)) && (reg <=
			IAXXX_STR_GRP_STR_AF_ERROR_ACCESS_CNT_REG(i)))
			return true;
	}

	/* Plugin group registers */
	for (i = 0; i < IAXXX_PLUGIN_MAX; i++) {
		if (reg >= IAXXX_PLUGIN_INS_GRP_PARAM_REG(i) &&
			reg <= IAXXX_PLUGIN_INS_GRP_OUT_15_STATUS_REG(i))
			return true;
	}

	/* Sensor group registers */
	for (i = 0; i < IAXXX_SENSOR_MAX; i++) {
		if (reg >= IAXXX_SENSOR_GRP_PARAM_ID_REG(i) &&
			reg <= IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_SIZE_REG(i))
			return true;
	}

	/* Plugin Header registers*/
	for (i = 0; i < IAXXX_PLUGIN_UPDATE_BLOCKS_MAX; i++) {
		for (j = 0; j < IAXXX_MAX_MULTI_PLUGIN_HDRS; j++) {
			if (reg >=
				IAXXX_PLUGIN_HDR_RESET_BLOCK_ADDR(i, j) &&
				reg <=
				IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_ADDR(i, j))
				return true;
		}
	}

	/* Channel group registers */
	for (i = 0; i < IAXXX_CH_MAX; i++) {
		if (reg == IAXXX_OUT_CH_GRP_CH_GAIN_STATUS_REG(i))
			return true;
		if (reg == IAXXX_IN_CH_GRP_CH_GAIN_STATUS_REG(i))
			return true;
		if ((reg >= IAXXX_OUT_CH_GRP_CH_PEAK_REG(i)) &&
			(reg <= IAXXX_OUT_CH_GRP_CH_DROP_CNT_REG(i)))
			return true;
		if ((reg >= IAXXX_IN_CH_GRP_CH_PEAK_REG(i)) &&
			(reg <= IAXXX_IN_CH_GRP_CH_DROP_CNT_REG(i)))
			return true;
	}

	/* Tunnel group registers */
	for (i = 0; i < IAXXX_TNL_MAX; i++) {
		if ((reg >= IAXXX_IN_TNL_GRP_TNL_NFRAME_DROPS_REG(i)) &&
			(reg <= IAXXX_IN_TNL_GRP_TNL_NRECVD_REG(i)))
			return true;

		if ((reg >= IAXXX_OUT_TNL_GRP_TNL_NFRAME_DROPS_REG(i)) &&
			(reg <= IAXXX_OUT_TNL_GRP_TNL_NRECVD_REG(i)))
			return true;
	}

	if (reg == IAXXX_SCRIPT_MGMT_SCRIPT_ADDR_ADDR)
		return true;

	if (reg >= IAXXX_DEBUG_REGS_ADDR && reg <=
			IAXXX_DEBUG_BLOCK_2_CRASHLOG_SIZE_ADDR)
		return true;

	/* BTP registers */
	if (reg >=  IAXXX_BULK_TRANSFER_BULK_TRANSFER_SIZE_ADDR &&
		reg <= IAXXX_BULK_TRANSFER_BULK_TRANSFER_D_ADDR_BLOCK_2_ADDR)
		return true;

	return false;
}

static bool iaxxx_application_volatile_reg_no_pm
		(struct device *dev, unsigned int reg)
{

	/* All SRB Registers except SRB Block Selct Register.
	 * SRB Block Select register is excluded because FW does
	 * not require Page Select to be able to read/write
	 */
	if ((reg != SRB_BLOCK_SELECT_REG) && reg >= IAXXX_SRB_BASE &&
		reg < (IAXXX_SRB_BASE + IAXXX_SRB_SIZE))
		return true;

	/* All Power Management Registers */
	if (reg >= IAXXX_PWR_MGMT_REGS_ADDR && reg <
		(IAXXX_PWR_MGMT_REGS_ADDR + IAXXX_PWR_MGMT_REG_NUM*4))
		return true;

	/* All event Management Registers */
	if (reg >= IAXXX_EVT_MGMT_EVT_ADDR &&
		reg < (IAXXX_EVT_MGMT_EVT_ADDR + (IAXXX_EVT_MGMT_REG_NUM*4)))
		return true;

	if ((reg == IAXXX_STR_HDR_STR_ST_ADDR) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_1, IAXXX_HOST_0)) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_1, IAXXX_HOST_1)) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_2, IAXXX_HOST_0)) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_2, IAXXX_HOST_1)))
		return true;

	return false;
}

/*
 * readable_register needed to optimize access and bus usage
 */
static bool iaxxx_readable_register_no_pm(struct device *dev, unsigned int reg)
{

	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (WARN_ON(!priv))
		return false;	/* Something went wrong */

	/* All SRB Registers */
	if (reg >= IAXXX_SRB_BASE && reg < (IAXXX_SRB_BASE + IAXXX_SRB_SIZE))
		return true;

	/* All Power Management Registers */
	if (reg >= IAXXX_PWR_MGMT_REGS_ADDR && reg <=
		(IAXXX_PWR_MGMT_REGS_ADDR + IAXXX_PWR_MGMT_REG_NUM*4))
		return true;

	/* All event Management Registers */
	if (reg >= IAXXX_EVT_MGMT_EVT_ADDR &&
		reg < (IAXXX_EVT_MGMT_EVT_ADDR + (IAXXX_EVT_MGMT_REG_NUM*4)))
		return true;

	/* Electrical control register used during boot */
	if (reg == IAXXX_AO_MEM_ELEC_CTRL_ADDR)
		return true;

	if ((reg == IAXXX_STR_HDR_STR_ST_ADDR) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_1, IAXXX_HOST_0)) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_1, IAXXX_HOST_1)) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_2, IAXXX_HOST_0)) ||
			(reg == IAXXX_PLUGIN_HDR_CREATE_STATUS_BLOCK_ADDR(
						IAXXX_BLOCK_2, IAXXX_HOST_1)))
		return true;

	return false;
}

int iaxxx_regmap_drop_regions(struct iaxxx_priv *priv)
{
	uint32_t start_addr, end_addr;
	int rc;
	int i;
	int phy_addr_size = sizeof(iaxxx_phy_addr_ranges) /
					sizeof(iaxxx_phy_addr_ranges[0]);

	for (i = 0; i < phy_addr_size; i++) {
		rc = regcache_drop_region(priv->regmap,
					iaxxx_phy_addr_ranges[i][0],
					iaxxx_phy_addr_ranges[i][1]);
		if (rc != 0) {
			dev_err(priv->dev,
				"%s() Failed to drop physical[%d] reg: %d\n",
				__func__, i, rc);
			goto drop_regions_exit;
		}
	}

	/* regmap cache drop */
	for (i = 0 ; i < priv->regmap_config->num_ranges; i++) {
		start_addr = priv->regmap_config->ranges[i].range_min;
		end_addr = priv->regmap_config->ranges[i].range_min +
				priv->regmap_config->ranges[i].window_len;
		rc = regcache_drop_region(priv->regmap, start_addr, end_addr);
		if (rc != 0) {
			dev_err(priv->dev,
				"%s() Failed to drop arb range[%d] reg: %d\n",
				__func__, i, rc);
			goto drop_regions_exit;
		}
	}
drop_regions_exit:
	return rc;
}

/*
 * Use ranges to define the Application Register Blocks
 * This needs to be populated at boot-time after firmware download.
 * This is just a template, we copy this over to allocated memory
 */
static const struct regmap_range_cfg iaxxx_ranges[] = {
	{
		.name = "Channel",
		.range_min = IAXXX_REG_CHANNEL_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Stream",
		.range_min = IAXXX_REG_STREAM_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Tunnel",
		.range_min = IAXXX_REG_TUNNEL_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Package Management",
		.range_min = IAXXX_REG_PACKAGE_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Plugin Instance",
		.range_min = IAXXX_REG_PLUGIN_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Crash Log",
		.range_min = IAXXX_REG_DBGLOG_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Event",
		.range_min = IAXXX_REG_EVENT_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Sensor",
		.range_min = IAXXX_REG_SENSOR_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Script",
		.range_min = IAXXX_REG_SCRIPT_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		.name = "Power",
		.range_min = IAXXX_REG_POWER_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
	{
		/*
		 * Btp stands for bulk transfer protocol. This protocol is being
		 * used to avoid ROM hole related issues. Memory read and write
		 * will happen through these registers
		 */
		.name = "Btp",
		.range_min = IAXXX_REG_BTP_BASE,
		.selector_reg = SRB_BLOCK_SELECT_REG,
		.selector_mask = SRB_BLOCK_SELECT_MASK,
		.selector_shift = SRB_BLOCK_SELECT_SHIFT,
	},
};

static struct regmap_config iaxxx_regmap_config = {
	.name = IAXXX_REGMAP_NAME,
	.reg_bits = 32,		/* 32-bit register offsets */
	.val_bits = 32,		/* 32-bit register values */
	.max_register = IAXXX_MAX_REGISTER,
	.readable_reg = iaxxx_readable_register,
	.writeable_reg = iaxxx_writeable_register,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.volatile_reg = iaxxx_volatile_register,
	.cache_type = REGCACHE_RBTREE,

	/* These will be set before the second regmap_init() */
	/*.ranges = iaxxx_ranges,
	 * .num_ranges = ARRAY_SIZE(iaxxx_ranges),
	 */
};

static struct regmap_config iaxxx_regmap_no_pm_config = {
	.name = IAXXX_REGMAP_NO_PM_NAME,
	.reg_bits = 32,		/* 32-bit register offsets */
	.val_bits = 32,		/* 32-bit register values */
	.max_register = IAXXX_MAX_REGISTER,
	.readable_reg = iaxxx_readable_register_no_pm,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.volatile_reg = iaxxx_volatile_register,
	.cache_type = REGCACHE_NONE,
};

/**
 * get_rbdt_block_count: determines the number of RBDT blocks to be mapped
 *
 * @priv: iaxxx private data
 *
 * The iaxxx_ranges table above defines the known Application Register
 * Blocks. This function will search through the Register Block
 * Descriptor Table and count the number of blocks that match the blocks
 * in iaxxx_ranges. This is used to determine how much space to allocate
 * for the range configuration table.
 *
 * Returns the number of blocks in the RBDT that match the range table.
 */
static inline unsigned int get_rbdt_block_count(struct iaxxx_priv *priv)
{
	int i;
	unsigned int num_blks = 0;
	unsigned int blk_index;
	const struct regmap_range_cfg *def_range_cfg;

	/* Count the number of known relocated blocks */
	for (i = 0; i < ARRAY_SIZE(iaxxx_ranges); ++i) {
		def_range_cfg = &iaxxx_ranges[i];
		blk_index = IAXXX_INDEX_FROM_VIRTUAL(def_range_cfg->range_min);
		WARN_ON(blk_index >= IAXXX_RBDT_NUM_ENTRIES);

		if (priv->sys_rbdt[RBDT_N_ADDR(blk_index)])
			++num_blks;
	}

	WARN_ON(num_blks > IAXXX_RBDT_NUM_ENTRIES);
	return num_blks;
}

/**
 * iaxxx_update_relocatable_blocks: builds regmap range configuration data
 *
 * @priv: iaxxx private data
 *
 * The regmap framework provides the concept of range configurations that can
 * be used for indirectly accessed or paged registers. This mechanism will be
 * used to let regmap handle the virtual-to-physical address translation.
 * The range_min and range_max fields specify the min and max virtual register
 * addresses for a given Application Register Block. The window_start and
 * window_len fields specify the physical address and length of the physical
 * register block.
 *
 * The Register Descriptor Block Table (RBDT) is index by Block Index and
 * provides the physical start address and length for each Application
 * Register Block (ARB).
 *
 * The range configuration table is allocated, initialized, and then assigned
 * to the regmap configuration.
 */
static int iaxxx_update_relocatable_blocks(struct iaxxx_priv *priv)
{
	int i;
	uint32_t addr, size;
	unsigned int nblks;
	unsigned long blk_index;

	struct device *dev = priv->dev;
	struct regmap_range_cfg *range_cfg;
	struct regmap_range_cfg *range_cfg_1;
	struct regmap_range_cfg *curr_range_cfg;
	struct regmap_range_cfg *curr_range_cfg_1;
	const struct regmap_range_cfg *def_range_cfg;

	/* Get the count of the number of valid RBDT entries */
	nblks = get_rbdt_block_count(priv);
	if (!nblks) {
		dev_warn(dev, "%s() No relocatable blocks\n", __func__);
		return 0;
	}

	/* Allocate a range configuration for the number of blocks */
	range_cfg = devm_kzalloc(dev, nblks * sizeof(*range_cfg), GFP_KERNEL);
	if (!range_cfg)
		return -ENOMEM;

	/* Allocate a range configuration for second regmap */
	range_cfg_1 = devm_kzalloc(dev,
			IAXXX_REGMAP_NO_PM_NUM_ARBS * sizeof(*range_cfg),
			GFP_KERNEL);
	if (!range_cfg_1) {
		devm_kfree(priv->dev, range_cfg);
		return -ENOMEM;
	}

	/* Build the range configuration table */
	curr_range_cfg   = range_cfg;
	curr_range_cfg_1 = range_cfg_1;

	for (i = 0; i < ARRAY_SIZE(iaxxx_ranges); ++i) {
		def_range_cfg = &iaxxx_ranges[i];
		blk_index = IAXXX_INDEX_FROM_VIRTUAL(def_range_cfg->range_min);

		/* Read the physical address from the RBDT */
		addr = priv->sys_rbdt[RBDT_N_ADDR(blk_index)];
		size = priv->sys_rbdt[RBDT_N_SIZE(blk_index)];
		pr_debug("ARB[%lu] = 0x%.08x, size = %d bytes\n",
				 blk_index, addr, size);

		if (addr && size) {
			/* Copy over the defaults */
			*curr_range_cfg = *def_range_cfg;

			curr_range_cfg->window_len = size;
			curr_range_cfg->window_start = addr;

			curr_range_cfg->range_max =
			curr_range_cfg->range_min + size - sizeof(u32);

			++curr_range_cfg;

			/* Second regmap only has range for below ARBS, which
			 * might be needed in pm ops
			 */
			if ((blk_index == IAXXX_BLOCK_POWER) ||
				(blk_index == IAXXX_BLOCK_EVENT) ||
				(blk_index == IAXXX_BLOCK_STREAM) ||
				(blk_index == IAXXX_BLOCK_PLUGIN)) {
				*curr_range_cfg_1 = *def_range_cfg;
				curr_range_cfg_1->window_len   = size;
				curr_range_cfg_1->window_start = addr;
				curr_range_cfg_1->range_max    =
						curr_range_cfg_1->range_min +
						size - sizeof(u32);
				++curr_range_cfg_1;
			}
		}
	}

	priv->regmap_config->ranges = range_cfg;
	priv->regmap_config->num_ranges = nblks;
	WARN_ON((curr_range_cfg - range_cfg) != nblks);

	priv->regmap_no_pm_config->ranges = range_cfg_1;
	priv->regmap_no_pm_config->num_ranges = IAXXX_REGMAP_NO_PM_NUM_ARBS;
	WARN_ON((curr_range_cfg_1 - range_cfg_1) !=
			IAXXX_REGMAP_NO_PM_NUM_ARBS);
	return 0;
}

/**
 * iaxxx_conv_physical_to_virtual_register_address
 *   - Converts register physical address to its virtual address
 *
 * @priv: iaxxx private data
 * @phy_addr: register physical address
 *
 * Loops through all configured regmap ranges and see where
 * the physical address falls in. If so, calculate the virtual
 * address using range start address and offset from start address.
 *
 */
uint32_t iaxxx_conv_physical_to_virtual_register_address(
		struct iaxxx_priv *priv,
		const uint32_t phy_addr)
{
	int i;
	uint32_t virt_addr, addr, size;
	unsigned int blk_index;
	const struct regmap_range_cfg *def_range_cfg;

	/* Using the array of virtual address ranges and
	 * ARB address/size array, find the virtual address
	 * for the physical address
	 */
	for (i = 0 ; i < ARRAY_SIZE(iaxxx_ranges) ; i++) {
		def_range_cfg = &iaxxx_ranges[i];
		blk_index = IAXXX_INDEX_FROM_VIRTUAL(def_range_cfg->range_min);
		if (blk_index < IAXXX_RBDT_NUM_ENTRIES) {
			addr = priv->sys_rbdt[RBDT_N_ADDR(blk_index)];
			size = priv->sys_rbdt[RBDT_N_SIZE(blk_index)];

			if (addr && size && (phy_addr >= addr) &&
				(phy_addr < addr + size)) {
				virt_addr = phy_addr - addr;
				virt_addr += def_range_cfg->range_min;
				return virt_addr;
			}
		}
	}
	return phy_addr;
}

/**
 * iaxxx_conv_virtual_to_physical_register_address
 *   - Converts register virtual address to its physical address
 *
 * @priv: iaxxx private data
 * @phy_addr: register virtual address
 *
 *
 *
 */
uint32_t iaxxx_conv_virtual_to_physical_register_address(
		struct iaxxx_priv *priv,
		const uint32_t virt_addr)
{
	/* Get ARB block index to get the physical address
	 * it is mapped to
	 */
	int blk_index = IAXXX_INDEX_FROM_VIRTUAL(virt_addr);
	uint32_t virt_base_addr = IAXXX_VIRTUAL_BASE_ADDR(blk_index);

	/* Return the physical address from the RBDT +
	 * offset of the virtual address from its base
	 */
	return priv->sys_rbdt[RBDT_N_ADDR(blk_index)] +
			(virt_addr-virt_base_addr);
}

/**
 * iaxxx_update_rbdt - updates the cached copy of the RBDT
 *
 * @priv: iaxxx private data
 *
 * Saves a copy of the Register Block Descriptor Table (RBDT) to the sys_rbdt
 * fields of the driver private data.
 */
static int iaxxx_update_rbdt(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	rc = regmap_bulk_read(priv->regmap_no_pm,
			IAXXX_SRB_ARB_0_BASE_ADDR_ADDR,
			&priv->sys_rbdt, ARRAY_SIZE(priv->sys_rbdt));
	if (rc)
		dev_err(dev, "%s: regmap_read failed, rc = %d\n", __func__, rc);

	return rc;
}

/* Register map initialization */
int iaxxx_regmap_init(struct iaxxx_priv *priv)
{
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	if (!priv->regmap_init_bus) {
		dev_err(dev, "%s: missing regmap_init func\n", __func__);
		return -EINVAL;
	}

	priv->regmap_config = &iaxxx_regmap_config;
	priv->regmap_no_pm_config = &iaxxx_regmap_no_pm_config;

	return priv->regmap_init_bus(priv);
}

int iaxxx_sbl_regmap_init(struct iaxxx_priv *priv)
{
	/* Free the existing regmaps */
	if (priv->regmap) {
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}

	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}


	priv->regmap_config->volatile_reg = iaxxx_volatile_register;
	priv->regmap_no_pm_config->volatile_reg = iaxxx_volatile_register;

	priv->is_application_mode = false;

	return iaxxx_regmap_init(priv);
}

/**
 * iaxxx_application_regmap_init: application register map initialization
 *
 * @priv: iaxxx private data
 *
 * Once the firmware download has completed and the device has booted into
 * application mode, the regmap needs to be updated to include the application
 * registers that have virtual addresses. These registers are not available in
 * SBL mode.
 *
 * 1) Update the cached SRB register to include the new RBDT settings
 * 2) Release the SBL regmap (it served us well)
 * 3) Setup the range configurations for the Application Register Blocks
 * 4) Initialize a new regmap that includes the range configurations
 */
int iaxxx_application_regmap_init(struct iaxxx_priv *priv)
{
	int rc;
	struct device *dev = priv->dev;

	dev_dbg(dev, "%s()\n", __func__);

	/* Update System Register Block using the SBL regmap. */
	rc = iaxxx_update_rbdt(priv);
	if (rc) {
		dev_err(dev, "%s: update SRB failed, rc = %d\n", __func__, rc);
		goto err;
	}

	/* Free the existing regmaps */
	if (priv->regmap) {
		regmap_exit(priv->regmap);
		priv->regmap = NULL;
	}

	if (priv->regmap_no_pm) {
		regmap_exit(priv->regmap_no_pm);
		priv->regmap_no_pm = NULL;
	}

	/* Update regmap configuration for the relocatable blocks */
	rc = iaxxx_update_relocatable_blocks(priv);
	if (rc) {
		dev_err(dev, "%s: range cfg failed, rc = %d\n", __func__, rc);
		goto err;
	}

	priv->regmap_config->volatile_reg = iaxxx_application_volatile_reg;
	priv->regmap_no_pm_config->volatile_reg =
			iaxxx_application_volatile_reg_no_pm;

	priv->is_application_mode = true;

	/* Initialize the "Application" regmap */
	rc = iaxxx_regmap_init(priv);
	if (rc) {
		devm_kfree(dev, (void *)priv->regmap_config->ranges);
		priv->regmap_config->ranges = &iaxxx_ranges[0];
		priv->regmap_config->num_ranges = 0;
		devm_kfree(priv->dev,
				(void *)priv->regmap_no_pm_config->ranges);
		priv->regmap_no_pm_config->ranges = NULL;
	}

err:
	return rc;
}
