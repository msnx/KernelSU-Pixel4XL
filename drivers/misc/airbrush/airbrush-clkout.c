/*
 * Copyright (C) 2019 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush CLKOUT driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "airbrush-pmic-ctrl.h"
#include "airbrush-regs.h"

struct ab_blk_clkout {
	const char *blk_name;		    /* cmu block name */
	const char * const *clkout0_names;  /* clockout0 names of this block */
	uint32_t clkout0_num;		    /* number of clocks in clkout0 */
	uint32_t clkout0_cmureg;	    /* clkout0 mux register address */
	const char * const *clkout1_names;  /* clockout1 names of this block */
	uint32_t clkout1_num;		    /* number of clocks in clkout1 */
	uint32_t clkout1_cmureg;	    /* clkout1 mux register address */

	/* Mux index of AB_REG_PMU_DEBUGx for selecting this cmu block */
	uint32_t pmu_mux_sel;
};

#define AB_BASE_CMU_AON		0x10B10000
#define AB_BASE_CMU_CORE	0x10F10000
#define AB_BASE_CMU_FSYS	0x10710000
#define AB_BASE_CMU_MIF		0x10510000
#define AB_BASE_CMU_IPU		0x10240000
#define AB_BASE_CMU_TPU		0x10040000
#define OFFSET_CLKOUT0		(0x810)
#define OFFSET_CLKOUT1		(0x814)
#define CLKOUT_ENABLE		(0x1 << 29)
#define CLKOUT_SEL(x)		(((x) & 0x1f) << 8)

enum ab_clkout_idx {
	AB_CLKOUT0,
	AB_CLKOUT1,
	AB_CLKOUT_MAX
};

enum ab_clkout_blk_idx {
	AB_CLKOUT_BLK_AON,
	AB_CLKOUT_BLK_IPU,
	AB_CLKOUT_BLK_TPU,
	AB_CLKOUT_BLK_FSYS,
	AB_CLKOUT_BLK_MIF,
	AB_CLKOUT_BLK_CORE,
	AB_CLKOUT_BLK_MAX
};

#define AB_BASE_CENTRAL_PMU	0x10BA0000
#define AB_REG_PMU_DEBUG0	(AB_BASE_CENTRAL_PMU + 0x3000)
#define AB_REG_PMU_DEBUG1	(AB_BASE_CENTRAL_PMU + 0x3004)
#define CLKOUT_DISABLE		(0x1 << 0)
#define CLKOUT_SEL_BLK_MSK	(0x3f << 8)
#define CLKOUT_SEL_BLK(x)	(((x) & 0x3f) << 8)
#define CLKOUT_SEL_BLK_AON	(2)
#define CLKOUT_SEL_BLK_IPU	(3)
#define CLKOUT_SEL_BLK_TPU	(4)
#define CLKOUT_SEL_BLK_FSYS	(5)
#define CLKOUT_SEL_BLK_MIF	(6)
#define CLKOUT_SEL_BLK_CORE	(7)

#define AB_BASE_SYSREG_AON			0x10B30000
#define AB_CLKOUT_COUNTER_REG_COUNTER0_LOW	(AB_BASE_SYSREG_AON + 0x354)
#define AB_CLKOUT_COUNTER_REG_COUNTER0_HIGH	(AB_BASE_SYSREG_AON + 0x350)
#define AB_CLKOUT_COUNTER_REG_COUNTER1_LOW	(AB_BASE_SYSREG_AON + 0x35c)
#define AB_CLKOUT_COUNTER_REG_COUNTER1_HIGH	(AB_BASE_SYSREG_AON + 0x358)

#define AB_CLKOUT_COUNTER_REG_CONTROL		(AB_BASE_SYSREG_AON + 0x360)
#define COUNTER_RESET				(0x1 << 0)
#define COUNTER_STOP				(0x1 << 1)
#define COUNTER_START				(0x1 << 2)
#define CLKGATE_EN_COUNTER0			(0x1 << 3)
#define CLKGATE_EN_COUNTER1			(0x1 << 4)

#define ABC_OSCCLK_FREQ				(19200000)
#define CLKOUT_SAMPLE_FREQ_MSEC			(500)

static inline void clkout_reg_set(uint32_t addr, uint32_t mask)
{
	WR_REG(addr, RD_REG(addr) | mask);
}

static inline void clkout_reg_clr(uint32_t addr, uint32_t mask)
{
	WR_REG(addr, RD_REG(addr) & (~mask));
}

static const struct ab_blk_clkout *clkout_get_aon_blk(void)
{
	static const char * const ab_blk_aon_clkout0_names[] = {
		[0] = "OSCCLK_AON",
		[1] = "DFT_OSCCLK_AON",
		[2] = "AON_PCLK",
		[3] = "DIV4_PLLCLK",
		[4] = "PLL_AON_CLK",
		[5] = "CLK_BLK_AON_UID_AON_CMU_AON_IPCLKPORT_PCLK",
		[6] = "CLK_BLK_AON_UID_ABC_IntMEM_IPCLKPORT_ACLK",
		[7] = "CLK_BLK_AON_UID_M0PLUS_RAM_IPCLKPORT_CLK",
		[8] = "CLK_BLK_AON_UID_M0PLUS_ROM_IPCLKPORT_CLK",
		[9] = "CLK_BLK_AON_UID_WDT1_TOP_IPCLKPORT_CLK",
		[10] = "CLK_BLK_AON_UID_WDT1_TOP_IPCLKPORT_PCLK",
		[11] = "CLK_BLK_AON_UID_WDT0_TOP_IPCLKPORT_CLK",
		[12] = "CLK_BLK_AON_UID_WDT0_TOP_IPCLKPORT_PCLK",
		[13] = "CLK_BLK_AON_UID_SYSREG_AON_IPCLKPORT_CLK",
		[14] = "CLK_BLK_AON_UID_SYSREG_AON_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_AON_UID_OTP_AON_IPCLKPORT_OSCCLK",
		[16] = "CLK_BLK_AON_UID_OTP_AON_IPCLKPORT_PCLK",
		[17] = "CLK_BLK_AON_UID_NIC_AON_IPCLKPORT_mainclk",
		[18] = "CLK_BLK_AON_UID_M0PLUS_ROM_IPCLKPORT_HCLK",
		[19] = "CLK_BLK_AON_UID_M0PLUS_RAM_IPCLKPORT_HCLK",
		[20] = "CLK_BLK_AON_UID_M0PLUS_IPCLKPORT_DCLK",
		[21] = "CLK_BLK_AON_UID_M0PLUS_IPCLKPORT_FCLK",
		[22] = "CLK_BLK_AON_UID_M0PLUS_IPCLKPORT_HCLK",
		[23] = "CLK_BLK_AON_UID_M0PLUS_IPCLKPORT_SCLK",
		[24] = "CLK_BLK_AON_UID_GPIO_ASYNCBR_TOP_AON_IPCLKPORT_PCLK",
		[25] = "CLK_BLK_AON_UID_Central_PMU_IPCLKPORT_PCLK",
		[26] = "CLK_BLK_AON_UID_BR_S_AON_IPCLKPORT_I_CLK",
		[27] = "CLK_BLK_AON_UID_BR_IntMEM_IPCLKPORT_I_CLK",
		[28] = "CLK_BLK_AON_UID_BR_D_AON_IPCLKPORT_I_CLK",
		[29] = "CLK_BLK_AON_UID_AXI2APB_AON_IPCLKPORT_ACLK",
		[30] = "SHARED_DIV_MIF",
		[31] = "SHARED_DIV_AON_PLL",
	};

	static const char * const ab_blk_aon_clkout1_names[] = {
		[0] = "OSCCLK_AON",
		[1] = "DFT_OSCCLK_AON",
		[2] = "AON_PCLK",
		[3] = "DIV4_PLLCLK",
		[4] = "PLL_AON_CLK",
		[5] = "CLK_BLK_AON_UID_AON_CMU_AON_IPCLKPORT_PCLK",
		[6] = "CLK_BLK_AON_UID_ABC_IntMEM_IPCLKPORT_ACLK",
		[7] = "CLK_BLK_AON_UID_M0PLUS_RAM_IPCLKPORT_CLK",
		[8] = "CLK_BLK_AON_UID_M0PLUS_ROM_IPCLKPORT_CLK",
		[9] = "CLK_BLK_AON_UID_WDT1_TOP_IPCLKPORT_CLK",
		[10] = "CLK_BLK_AON_UID_WDT1_TOP_IPCLKPORT_PCLK",
		[11] = "CLK_BLK_AON_UID_WDT0_TOP_IPCLKPORT_CLK",
		[12] = "CLK_BLK_AON_UID_WDT0_TOP_IPCLKPORT_PCLK",
		[13] = "CLK_BLK_AON_UID_SYSREG_AON_IPCLKPORT_CLK",
		[14] = "CLK_BLK_AON_UID_SYSREG_AON_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_AON_UID_OTP_AON_IPCLKPORT_OSCCLK",
		[16] = "CLK_BLK_AON_UID_OTP_AON_IPCLKPORT_PCLK",
		[17] = "CLK_BLK_AON_UID_NIC_AON_IPCLKPORT_mainclk",
		[18] = "CLK_BLK_AON_UID_M0PLUS_ROM_IPCLKPORT_HCLK",
		[19] = "CLK_BLK_AON_UID_M0PLUS_RAM_IPCLKPORT_HCLK",
		[20] = "CLK_BLK_AON_UID_M0PLUS_IPCLKPORT_DCLK",
		[21] = "CLK_BLK_AON_UID_AON_UART_IPCLKPORT_i_PCLK",
		[22] = "CLK_BLK_AON_UID_AON_UART_IPCLKPORT_i_SCLK_UART",
		[23] = "CLK_BLK_AON_UID_AON_SPI0_IPCLKPORT_i_aon_clk",
		[24] = "CLK_BLK_AON_UID_DAP_ABC_AON_IPCLKPORT_DAP_CLK",
		[25] = "CLK_BLK_AON_UID_APB_ASYNC_BRIDGE_TMU_AON_IPCLKPORT_PCLK",
		[26] = "DIV_OTP",
		[27] = "CLK_BLK_AON_UID_TEST_WRAP_OTP_AON_IPCLKPORT_CLK",
		[28] = "DIV_TMU",
		[29] = "CLK_BLK_AON_UID_TMU_AON_IPCLKPORT_I_CLK",
		[30] = "SHARED_DIV_MIF",
		[31] = "SHARED_DIV_AON_PLL",
	};

	static const struct ab_blk_clkout ab_blk_aon_clkout = {
		"AON",
		ab_blk_aon_clkout0_names,
		ARRAY_SIZE(ab_blk_aon_clkout0_names),
		AB_BASE_CMU_AON + OFFSET_CLKOUT0,
		ab_blk_aon_clkout1_names,
		ARRAY_SIZE(ab_blk_aon_clkout1_names),
		AB_BASE_CMU_AON + OFFSET_CLKOUT1,
		CLKOUT_SEL_BLK_AON
	};

	return &ab_blk_aon_clkout;
}

static const struct ab_blk_clkout *clkout_get_ipu_blk(void)
{
	static const char * const ab_blk_ipu_clkout0_names[] = {
		[0] = "DFT_OSCCLK_IPU",
		[1] = "DFT_MOUT_IPU_AXI",
		[2] = "DFT_MOUT_IPUCLK",
		[3] = "DIV4_PLLCLK_IPU",
		[4] = "DFT_MOUT_IPU_APB",
		[5] = "CLK_BLK_IPU_UID_IPU_IPCLKPORT_clk_ipu",
		[6] = "CLK_BLK_IPU_UID_HPM0_IPU_IPCLKPORT_hpm_targetclk_c",
		[7] = "CLK_BLK_IPU_UID_IPU_IPCLKPORT_clk_axi",
		[8] = "CLK_BLK_IPU_UID_IPU_IPCLKPORT_clk_apb",
		[9] = "CLK_BLK_IPU_UID_IPU_CMU_IPU_IPCLKPORT_PCLK",
		[10] = "CLK_BLK_IPU_UID_AXI2APB_BRIDGE_IPU_IPCLKPORT_ACLK",
		[11] = "CLK_BLK_IPU_UID_CORE_2_IPU_LH_IPCLKPORT_I_CLK",
		[12] = "CLK_BLK_IPU_UID_IPU_2_CORE_LH_IPCLKPORT_I_CLK",
		[13] = "CLK_BLK_IPU_UID_IPU_HPM_APBIF_IPCLKPORT_CLK",
		[14] = "CLK_BLK_IPU_UID_IPU_HPM_APBIF_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_IPU_UID_SYSREG_IPU_IPCLKPORT_CLK",
		[16] = "CLK_BLK_IPU_UID_SYSREG_IPU_IPCLKPORT_PCLK",
		[17] = "MOUT_AONCLK_PLLCLK1",
	};

	static const char * const ab_blk_ipu_clkout1_names[] = {
		[0] = "DFT_OSCCLK_IPU",
		[1] = "DFT_MOUT_IPU_AXI",
		[2] = "DFT_MOUT_IPUCLK",
		[3] = "DIV4_PLLCLK_IPU",
		[4] = "DFT_MOUT_IPU_APB",
		[5] = "CLK_BLK_IPU_UID_IPU_IPCLKPORT_clk_ipu",
		[6] = "CLK_BLK_IPU_UID_HPM0_IPU_IPCLKPORT_hpm_targetclk_c",
		[7] = "CLK_BLK_IPU_UID_IPU_IPCLKPORT_clk_axi",
		[8] = "CLK_BLK_IPU_UID_IPU_IPCLKPORT_clk_apb",
		[9] = "CLK_BLK_IPU_UID_IPU_CMU_IPU_IPCLKPORT_PCLK",
		[10] = "CLK_BLK_IPU_UID_AXI2APB_BRIDGE_IPU_IPCLKPORT_ACLK",
		[11] = "CLK_BLK_IPU_UID_CORE_2_IPU_LH_IPCLKPORT_I_CLK",
		[12] = "CLK_BLK_IPU_UID_IPU_2_CORE_LH_IPCLKPORT_I_CLK",
		[13] = "CLK_BLK_IPU_UID_IPU_HPM_APBIF_IPCLKPORT_CLK",
		[14] = "CLK_BLK_IPU_UID_IPU_HPM_APBIF_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_IPU_UID_SYSREG_IPU_IPCLKPORT_CLK",
		[16] = "CLK_BLK_IPU_UID_SYSREG_IPU_IPCLKPORT_PCLK",
		[17] = "MOUT_AONCLK_PLLCLK1",
	};

	static const struct ab_blk_clkout ab_blk_ipu_clkout = {
		"IPU",
		ab_blk_ipu_clkout0_names,
		ARRAY_SIZE(ab_blk_ipu_clkout0_names),
		AB_BASE_CMU_IPU + OFFSET_CLKOUT0,
		ab_blk_ipu_clkout1_names,
		ARRAY_SIZE(ab_blk_ipu_clkout1_names),
		AB_BASE_CMU_IPU + OFFSET_CLKOUT1,
		CLKOUT_SEL_BLK_IPU
	};

	return &ab_blk_ipu_clkout;
}

static const struct ab_blk_clkout *clkout_get_tpu_blk(void)
{
	static const char * const ab_blk_tpu_clkout0_names[] = {
		[0] = "DFT_OSCCLK_TPU",
		[1] = "DFT_MOUT_TPU_AXI",
		[2] = "DFT_MOUT_TPUCLK",
		[3] = "DIV4_PLLCLK_TPU",
		[4] = "DFT_MOUT_TPU_APB",
		[5] = "CLK_BLK_TPU_UID_TPU_IPCLKPORT_clk_tpu",
		[6] = "CLK_BLK_TPU_UID_TPU_IPCLKPORT_axi_clk",
		[7] = "CLK_BLK_TPU_UID_TPU_IPCLKPORT_apb_pclk",
		[8] = "CLK_BLK_TPU_UID_HPM0_TPU_IPCLKPORT_hpm_targetclk_c",
		[9] = "CLK_BLK_TPU_UID_TPU_HPM_APBIF_IPCLKPORT_CLK",
		[10] = "CLK_BLK_TPU_UID_TPU_CMU_TPU_IPCLKPORT_PCLK",
		[11] = "CLK_BLK_TPU_UID_AXI2APB_BRIDGE_TPU_IPCLKPORT_ACLK",
		[12] = "CLK_BLK_TPU_UID_CORE_2_TPU_LH_IPCLKPORT_I_CLK",
		[13] = "CLK_BLK_TPU_UID_SYSREG_TPU_IPCLKPORT_CLK",
		[14] = "CLK_BLK_TPU_UID_SYSREG_TPU_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_TPU_UID_TPU_2_CORE_LH_IPCLKPORT_I_CLK",
		[16] = "CLK_BLK_TPU_UID_TPU_HPM_APBIF_IPCLKPORT_PCLK",
		[17] = "MOUT_TPU_AONCLK_PLLCLK",
	};

	static const char * const ab_blk_tpu_clkout1_names[] = {
		[0] = "DFT_OSCCLK_TPU",
		[1] = "DFT_MOUT_TPU_AXI",
		[2] = "DFT_MOUT_TPUCLK",
		[3] = "DIV4_PLLCLK_TPU",
		[4] = "DFT_MOUT_TPU_APB",
		[5] = "CLK_BLK_TPU_UID_TPU_IPCLKPORT_clk_tpu",
		[6] = "CLK_BLK_TPU_UID_TPU_IPCLKPORT_axi_clk",
		[7] = "CLK_BLK_TPU_UID_TPU_IPCLKPORT_apb_pclk",
		[8] = "CLK_BLK_TPU_UID_HPM0_TPU_IPCLKPORT_hpm_targetclk_c",
		[9] = "CLK_BLK_TPU_UID_TPU_HPM_APBIF_IPCLKPORT_CLK",
		[10] = "CLK_BLK_TPU_UID_TPU_CMU_TPU_IPCLKPORT_PCLK",
		[11] = "CLK_BLK_TPU_UID_AXI2APB_BRIDGE_TPU_IPCLKPORT_ACLK",
		[12] = "CLK_BLK_TPU_UID_CORE_2_TPU_LH_IPCLKPORT_I_CLK",
		[13] = "CLK_BLK_TPU_UID_SYSREG_TPU_IPCLKPORT_CLK",
		[14] = "CLK_BLK_TPU_UID_SYSREG_TPU_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_TPU_UID_TPU_2_CORE_LH_IPCLKPORT_I_CLK",
		[16] = "CLK_BLK_TPU_UID_TPU_HPM_APBIF_IPCLKPORT_PCLK",
		[17] = "MOUT_TPU_AONCLK_PLLCLK",
	};

	static const struct ab_blk_clkout ab_blk_tpu_clkout = {
		"TPU",
		ab_blk_tpu_clkout0_names,
		ARRAY_SIZE(ab_blk_tpu_clkout0_names),
		AB_BASE_CMU_TPU + OFFSET_CLKOUT0,
		ab_blk_tpu_clkout1_names,
		ARRAY_SIZE(ab_blk_tpu_clkout1_names),
		AB_BASE_CMU_TPU + OFFSET_CLKOUT1,
		CLKOUT_SEL_BLK_TPU
	};

	return &ab_blk_tpu_clkout;
}

static const struct ab_blk_clkout *clkout_get_fsys_blk(void)
{
	static const char * const ab_blk_fsys_clkout0_names[] = {
		[0] = "DFT_OSCCLK_FSYS",
		[1] = "PCLK_FSYS",
		[2] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_mstr_aclk_soc",
		[3] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_mstr_aclk_soc",
		[4] = "CLK_BLK_FSYS_UID_LH_SI_FSYS_IPCLKPORT_I_CLK",
		[5] = "CLK_BLK_FSYS_UID_AXI2APB_FSYS_IPCLKPORT_ACLK",
		[6] = "CLK_BLK_FSYS_UID_BUS_S_FSYS_IPCLKPORT_mainclk",
		[7] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_ABC_pcie_sub_ctrl_i_driver_apb_clk",
		[8] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_dbi_aclk_soc",
		[9] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_apb_pclk",
		[10] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_apb_pclk_PCS",
		[11] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_ln0_apb_pclk",
		[12] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_ln1_apb_pclk",
		[13] = "CLK_BLK_FSYS_UID_SYSREG_FSYS_IPCLKPORT_CLK",
		[14] = "CLK_BLK_FSYS_UID_SYSREG_FSYS_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_pcs_hs_scan_clk",
		[16] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_slv_aclk_soc",
		[17] = "CLK_BLK_FSYS_UID_FSYS_CMU_FSYS_IPCLKPORT_PCLK",
		[18] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_ABC_pcie_sub_ctrl_aux_clk_soc",
		[19] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_immortal_clk",
	};

	static const char * const ab_blk_fsys_clkout1_names[] = {
		[0] = "DFT_OSCCLK_FSYS",
		[1] = "PCLK_FSYS",
		[2] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_mstr_aclk_soc",
		[3] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_mstr_aclk_soc",
		[4] = "CLK_BLK_FSYS_UID_LH_SI_FSYS_IPCLKPORT_I_CLK",
		[5] = "CLK_BLK_FSYS_UID_AXI2APB_FSYS_IPCLKPORT_ACLK",
		[6] = "CLK_BLK_FSYS_UID_BUS_S_FSYS_IPCLKPORT_mainclk",
		[7] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_ABC_pcie_sub_ctrl_i_driver_apb_clk",
		[8] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_dbi_aclk_soc",
		[9] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_apb_pclk",
		[10] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_apb_pclk_PCS",
		[11] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_ln0_apb_pclk",
		[12] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_ln1_apb_pclk",
		[13] = "CLK_BLK_FSYS_UID_SYSREG_FSYS_IPCLKPORT_CLK",
		[14] = "CLK_BLK_FSYS_UID_SYSREG_FSYS_IPCLKPORT_PCLK",
		[15] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_pcs_hs_scan_clk",
		[16] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_slv_aclk_soc",
		[17] = "CLK_BLK_FSYS_UID_FSYS_CMU_FSYS_IPCLKPORT_PCLK",
		[18] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_ABC_pcie_sub_ctrl_aux_clk_soc",
		[19] = "CLK_BLK_FSYS_UID_PCIe_MEM_TOP_FSYS_IPCLKPORT_i_immortal_clk",
	};

	static const struct ab_blk_clkout ab_blk_fsys_clkout = {
		"FSYS",
		ab_blk_fsys_clkout0_names,
		ARRAY_SIZE(ab_blk_fsys_clkout0_names),
		AB_BASE_CMU_FSYS + OFFSET_CLKOUT0,
		ab_blk_fsys_clkout1_names,
		ARRAY_SIZE(ab_blk_fsys_clkout1_names),
		AB_BASE_CMU_FSYS + OFFSET_CLKOUT1,
		CLKOUT_SEL_BLK_FSYS
	};

	return &ab_blk_fsys_clkout;
}

static const struct ab_blk_clkout *clkout_get_mif_blk(void)
{
	static const char * const ab_blk_mif_clkout0_names[] = {
		[0] = "OSCCLK_AON",
		[1] = "PCLK_MIF",
		[2] = "CLK_BLK_MIF_UID_AXI2APB_MIF_IPCLKPORT_ACLK",
		[3] = "CLK_BLK_MIF_UID_PHY0_MIF_IPCLKPORT_PCLK",
		[4] = "CLK_BLK_MIF_UID_PHY1_MIF_IPCLKPORT_PCLK",
		[5] = "CLK_BLK_MIF_UID_PPC_MIF_IPCLKPORT_PCLK",
		[6] = "CLK_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_PCLK",
		[7] = "CLK_BLK_MIF_UID_AXI_ASYNC_BR_AXI2APB_MIF_IPCLKPORT_I_CLK",
		[8] = "CLK_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_CLK",
		[9] = "CLK_BLK_MIF_UID_MIF_CMU_MIF_IPCLKPORT_PCLK",
		[10] = "DFT_OSCCLK_MIF",
		[11] = "PLL_PHY_MIF",
		[12] = "CLK_BLK_MIF_UID_VT_MON_MIF_IPCLKPORT_PCLK",
		[13] = "CLK_BLK_MIF_UID_VT_MON_MIF_IPCLKPORT_OSC_CLK_24M",
		[14] = "DIV2_PLLCLK_MIF",
		[15] = "DFT_OSCCLK_MIF",
		[16] = "CLK_BLK_MIF_UID_PLL_WRP_PHY_MIF_IPCLKPORT_clk_user",
		[17] = "DMC_PCLK",
		[18] = "CLK_BLK_MIF_UID_DMC_MIF_IPCLKPORT_RCLK",
		[19] = "CLK_BLK_MIF_UID_DMC_MIF_IPCLKPORT_PCLK",
		[20] = "CLK_BLK_MIF_UID_DMC_MIF_IPCLKPORT_PCLK_TZ",
	};

	static const char * const ab_blk_mif_clkout1_names[] = {
		[0] = "OSCCLK_AON",
		[1] = "PCLK_MIF",
		[2] = "CLK_BLK_MIF_UID_AXI2APB_MIF_IPCLKPORT_ACLK",
		[3] = "CLK_BLK_MIF_UID_PHY0_MIF_IPCLKPORT_PCLK",
		[4] = "CLK_BLK_MIF_UID_PHY1_MIF_IPCLKPORT_PCLK",
		[5] = "CLK_BLK_MIF_UID_PPC_MIF_IPCLKPORT_PCLK",
		[6] = "CLK_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_PCLK",
		[7] = "CLK_BLK_MIF_UID_AXI_ASYNC_BR_AXI2APB_MIF_IPCLKPORT_I_CLK",
		[8] = "CLK_BLK_MIF_UID_SYSREG_MIF_IPCLKPORT_CLK",
		[9] = "CLK_BLK_MIF_UID_MIF_CMU_MIF_IPCLKPORT_PCLK",
		[10] = "DFT_OSCCLK_MIF",
		[11] = "PLL_PHY_MIF",
		[12] = "CLK_BLK_MIF_UID_VT_MON_MIF_IPCLKPORT_PCLK",
		[13] = "CLK_BLK_MIF_UID_VT_MON_MIF_IPCLKPORT_OSC_CLK_24M",
		[14] = "DIV2_PLLCLK_MIF",
		[15] = "DFT_OSCCLK_MIF",
		[16] = "CLK_BLK_MIF_UID_PLL_WRP_PHY_MIF_IPCLKPORT_clk_user",
		[17] = "DMC_PCLK",
		[18] = "CLK_BLK_MIF_UID_DMC_MIF_IPCLKPORT_RCLK",
		[19] = "CLK_BLK_MIF_UID_DMC_MIF_IPCLKPORT_PCLK",
		[20] = "CLK_BLK_MIF_UID_DMC_MIF_IPCLKPORT_PCLK_TZ",
	};

	static const struct ab_blk_clkout ab_blk_mif_clkout = {
		"MIF",
		ab_blk_mif_clkout0_names,
		ARRAY_SIZE(ab_blk_mif_clkout0_names),
		AB_BASE_CMU_MIF + OFFSET_CLKOUT0,
		ab_blk_mif_clkout1_names,
		ARRAY_SIZE(ab_blk_mif_clkout1_names),
		AB_BASE_CMU_MIF + OFFSET_CLKOUT1,
		CLKOUT_SEL_BLK_MIF
	};

	return &ab_blk_mif_clkout;
}

static const struct ab_blk_clkout *clkout_get_core_blk(void)
{
	static const char * const ab_blk_core_clkout0_names[] = {
		[0] = "DFT_OSCCLK_CORE",
		[1] = "BLK_CORE_AON_DIV2_CLK",
		[2] = "BLK_CORE_AON_DIV4_CLK",
		[3] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_CONFIGclk",
		[4] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_DMCclk",
		[5] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_DMCclk",
		[6] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_DMCclk",
		[7] = "CLK_BLK_CORE_UID_AXI2APB_CORE_IPCLKPORT_ACLK",
		[8] = "CLK_BLK_CORE_UID_ppmuIPU_IPCLKPORT_PCLK",
		[9] = "CLK_BLK_CORE_UID_ppmuTPU_IPCLKPORT_PCLK",
		[10] = "CLK_BLK_CORE_UID_ppmuFSYS_M_IPCLKPORT_PCLK",
		[11] = "CLK_BLK_CORE_UID_ppmuFSYS_S_IPCLKPORT_PCLK",
		[12] = "CLK_BLK_CORE_UID_SYSREG_CORE_IPCLKPORT_PCLK",
		[13] = "CLK_BLK_CORE_UID_SYSREG_CORE_IPCLKPORT_CLK",
		[14] = "DFT_OSCCLK_CORE",
		[15] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_mainclk",
		[16] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_mainclk_r",
		[17] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_mainclk",
		[18] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_mainclk_r",
		[19] = "CLK_BLK_CORE_UID_nic400_TPU_IPCLKPORT_mainclk",
		[20] = "CLK_BLK_CORE_UID_nic400_TPU_IPCLKPORT_mainclk_r",
		[21] = "CLK_BLK_CORE_UID_CORE_CMU_CORE_IPCLKPORT_PCLK",
		[22] = "CLK_BLK_CORE_UID_SI_S_AON_CFG_IPCLKPORT_I_CLK",
		[23] = "CLK_BLK_CORE_UID_SI_S_IPU_CFG_IPCLKPORT_I_CLK",
		[24] = "CLK_BLK_CORE_UID_SI_S_TPU_CFG_IPCLKPORT_I_CLK",
		[25] = "CLK_BLK_CORE_UID_SI_S_MIF_CFG_IPCLKPORT_I_CLK",
		[26] = "CLK_BLK_CORE_UID_nic400_CONFIG_IPCLKPORT_mainclk",
		[27] = "CLK_BLK_CORE_UID_nic400_CONFIG_IPCLKPORT_mainclk_r",
		[28] = "CLK_BLK_CORE_UID_MI_BLK_IPU_IPCLKPORT_I_CLK",
		[29] = "DIV4_PLLCLK_CORE",
		[30] = "DIV2_PLLCLK_CORE",
		[31] = "DFT_OSCCLK_CORE",
	};

	static const char * const ab_blk_core_clkout1_names[] = {
		[0] = "DFT_OSCCLK_CORE",
		[1] = "BLK_CORE_AON_DIV2_CLK",
		[2] = "BLK_CORE_AON_DIV4_CLK",
		[3] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_CONFIGclk",
		[4] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_DMCclk",
		[5] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_DMCclk",
		[6] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_DMCclk",
		[7] = "CLK_BLK_CORE_UID_AXI2APB_CORE_IPCLKPORT_ACLK",
		[8] = "CLK_BLK_CORE_UID_ppmuIPU_IPCLKPORT_PCLK",
		[9] = "CLK_BLK_CORE_UID_ppmuTPU_IPCLKPORT_PCLK",
		[10] = "CLK_BLK_CORE_UID_ppmuFSYS_M_IPCLKPORT_PCLK",
		[11] = "CLK_BLK_CORE_UID_ppmuFSYS_S_IPCLKPORT_PCLK",
		[12] = "CLK_BLK_CORE_UID_SYSREG_CORE_IPCLKPORT_PCLK",
		[13] = "CLK_BLK_CORE_UID_SYSREG_CORE_IPCLKPORT_CLK",
		[14] = "DFT_OSCCLK_CORE",
		[15] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_mainclk",
		[16] = "CLK_BLK_CORE_UID_nic400_IPU_IPCLKPORT_mainclk_r",
		[17] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_mainclk",
		[18] = "CLK_BLK_CORE_UID_nic400_PRIMARY_IPCLKPORT_mainclk_r",
		[19] = "CLK_BLK_CORE_UID_nic400_TPU_IPCLKPORT_mainclk",
		[20] = "CLK_BLK_CORE_UID_SI_S_FSYS_PRIMARY_IPCLKPORT_I_CLK",
		[21] = "CLK_BLK_CORE_UID_SI_S_INTMEM_PRIMARY_IPCLKPORT_I_CLK",
		[22] = "CLK_BLK_CORE_UID_ppmuFSYS_M_IPCLKPORT_ACLK",
		[23] = "CLK_BLK_CORE_UID_ppmuFSYS_S_IPCLKPORT_ACLK",
		[24] = "CLK_BLK_CORE_UID_ppmuIPU_IPCLKPORT_ACLK",
		[25] = "CLK_BLK_CORE_UID_ppmuTPU_IPCLKPORT_ACLK",
		[26] = "CLK_BLK_CORE_UID_SI_DMC0_IPU_IPCLKPORT_I_CLK",
		[27] = "CLK_BLK_CORE_UID_SI_DMC0_TPU_IPCLKPORT_I_CLK",
		[28] = "CLK_BLK_CORE_UID_SI_DMC0_PRIMARY_IPCLKPORT_I_CLK",
		[29] = "DIV4_PLLCLK_CORE",
		[30] = "DIV2_PLLCLK_CORE",
		[31] = "DFT_OSCCLK_CORE",
	};

	static const struct ab_blk_clkout ab_blk_core_clkout = {
		"CORE",
		ab_blk_core_clkout0_names,
		ARRAY_SIZE(ab_blk_core_clkout0_names),
		AB_BASE_CMU_CORE + OFFSET_CLKOUT0,
		ab_blk_core_clkout1_names,
		ARRAY_SIZE(ab_blk_core_clkout1_names),
		AB_BASE_CMU_CORE + OFFSET_CLKOUT1,
		CLKOUT_SEL_BLK_CORE
	};

	return &ab_blk_core_clkout;
}

static const struct ab_blk_clkout *clkout_get_blk(unsigned int blk_idx)
{
	switch (blk_idx) {
	case AB_CLKOUT_BLK_AON:
		return clkout_get_aon_blk();
	case AB_CLKOUT_BLK_IPU:
		return clkout_get_ipu_blk();
	case AB_CLKOUT_BLK_TPU:
		return clkout_get_tpu_blk();
	case AB_CLKOUT_BLK_FSYS:
		return clkout_get_fsys_blk();
	case AB_CLKOUT_BLK_MIF:
		return clkout_get_mif_blk();
	case AB_CLKOUT_BLK_CORE:
		return clkout_get_core_blk();
	default:
		break;
	}

	return NULL;
}

static void clkout_set(const struct ab_blk_clkout *clkout, uint32_t clk_sel,
		  uint32_t clockout_idx)
{
	/* Configure the clkout GPIO registers */
	WR_REG(0x10B40000, 0x22);

	switch (clockout_idx) {
	case AB_CLKOUT0:
		WR_REG(AB_REG_PMU_DEBUG0, CLKOUT_SEL_BLK(clkout->pmu_mux_sel));
		WR_REG(clkout->clkout0_cmureg,
		       CLKOUT_ENABLE | CLKOUT_SEL(clk_sel));
		break;
	case AB_CLKOUT1:
		WR_REG(AB_REG_PMU_DEBUG1, CLKOUT_SEL_BLK(clkout->pmu_mux_sel));
		WR_REG(clkout->clkout1_cmureg,
		       CLKOUT_ENABLE | CLKOUT_SEL(clk_sel));
		break;
	default:
		pr_err("%s: Invalid index. Use either 0 or 1\n", __func__);
		break;
	}
}

static void clkout_disable(uint32_t clockout_idx)
{
	switch (clockout_idx) {
	case AB_CLKOUT0:
		clkout_reg_set(AB_REG_PMU_DEBUG0, CLKOUT_DISABLE);
		clkout_reg_clr(AB_CLKOUT_COUNTER_REG_CONTROL,
			       CLKGATE_EN_COUNTER0);
		break;
	case AB_CLKOUT1:
		clkout_reg_set(AB_REG_PMU_DEBUG1, CLKOUT_DISABLE);
		clkout_reg_clr(AB_CLKOUT_COUNTER_REG_CONTROL,
			       CLKGATE_EN_COUNTER1);
		break;
	default:
		pr_err("%s: Invalid index. Use either 0 or 1\n", __func__);
		break;
	}
}

void ab_clkout_sel(struct ab_state_context *sc, unsigned int clkout_idx)
{
	if (!sc)
		return;

	if (clkout_idx >= AB_CLKOUT_MAX) {
		pr_err("%s: Invalid clkout index(%d). Should be 0 or 1\n",
		       __func__, clkout_idx);
		return;
	}

	sc->clkout_idx = clkout_idx;
}

void ab_clkout_blksel(struct ab_state_context *sc, unsigned int blk_idx)
{
	if (!sc)
		return;

	sc->clkout_blk_idx = blk_idx;
}

void ab_clkout_clksel(struct ab_state_context *sc, unsigned int clk_idx)
{
	if (!sc)
		return;

	sc->clkout_clk_idx = clk_idx;
}

int ab_clkout_enable(struct ab_state_context *sc, unsigned int enable)
{
	const struct ab_blk_clkout *blk_clkout;

	if (!sc)
		return -EINVAL;

	if (sc->clkout_idx >= AB_CLKOUT_MAX) {
		pr_err("%s: Invalid clkout index(%d). Should be 0 or 1\n",
		       __func__, sc->clkout_idx);
		return -EINVAL;
	}

	if (sc->clkout_blk_idx >= AB_CLKOUT_BLK_MAX) {
		pr_err("%s: Invalid clkout blk index(%d). Should be < %d\n",
		       __func__, sc->clkout_blk_idx, AB_CLKOUT_BLK_MAX);
		return -EINVAL;
	}

	if (!enable) {
		clkout_disable(sc->clkout_idx);
		return 0;
	}

	blk_clkout = clkout_get_blk(sc->clkout_blk_idx);

	if (((sc->clkout_idx == AB_CLKOUT0) &&
	    (sc->clkout_clk_idx >= blk_clkout->clkout0_num)) ||
	    ((sc->clkout_idx == AB_CLKOUT1) &&
	    (sc->clkout_clk_idx >= blk_clkout->clkout1_num))) {
		pr_err("%s: Invalid clk index\n", __func__);
		return -EINVAL;
	}

	clkout_set(blk_clkout, sc->clkout_clk_idx, sc->clkout_idx);

	pr_info("clkout%d: cmu_blk: %s, clksel: %d, clk: %s\n",
		sc->clkout_idx, blk_clkout->blk_name, sc->clkout_clk_idx,
		sc->clkout_idx ?
		blk_clkout->clkout1_names[sc->clkout_clk_idx] :
		blk_clkout->clkout0_names[sc->clkout_clk_idx]);

	return 0;
}

int ab_clkout_freq(struct ab_state_context *sc, u64 *val)
{
	const struct ab_blk_clkout *blk_clkout;
	unsigned long counter_cnt0_low, counter_cnt1_low;
	unsigned long counter_cnt0_high, counter_cnt1_high;
	unsigned long counter_cnt0, counter_cnt1;
	u64 freq;

	if (!sc)
		return -EINVAL;

	if (ab_clkout_enable(sc, 1))
		return -EINVAL;

	blk_clkout = clkout_get_blk(AB_CLKOUT_BLK_AON);
	if (sc->clkout_idx == AB_CLKOUT0) {
		/* Select AON OSCCLK as reference clk at CLKOUT1 */
		clkout_set(blk_clkout, 0, AB_CLKOUT1);
	} else {
		/* Select AON OSCCLK as reference clk at CLKOUT0 */
		clkout_set(blk_clkout, 0, AB_CLKOUT0);
	}

	WR_REG(AB_CLKOUT_COUNTER_REG_CONTROL,
	       CLKGATE_EN_COUNTER0 | CLKGATE_EN_COUNTER1 | COUNTER_RESET);
	WR_REG(AB_CLKOUT_COUNTER_REG_CONTROL,
	       CLKGATE_EN_COUNTER0 | CLKGATE_EN_COUNTER1 | COUNTER_START);

	/* Sleep for clkout counter to capture the freq data */
	msleep(CLKOUT_SAMPLE_FREQ_MSEC);

	WR_REG(AB_CLKOUT_COUNTER_REG_CONTROL,
	       CLKGATE_EN_COUNTER0 | CLKGATE_EN_COUNTER1 | COUNTER_STOP);

	counter_cnt0_low = RD_REG(AB_CLKOUT_COUNTER_REG_COUNTER0_LOW);
	counter_cnt1_low = RD_REG(AB_CLKOUT_COUNTER_REG_COUNTER1_LOW);
	counter_cnt0_high = RD_REG(AB_CLKOUT_COUNTER_REG_COUNTER0_HIGH);
	counter_cnt1_high = RD_REG(AB_CLKOUT_COUNTER_REG_COUNTER1_HIGH);

	/* Once the counter values are read, reset the counters */
	WR_REG(AB_CLKOUT_COUNTER_REG_CONTROL,
	       CLKGATE_EN_COUNTER0 | CLKGATE_EN_COUNTER1 | COUNTER_RESET);

	counter_cnt0 = (counter_cnt0_high << 32) | counter_cnt0_low;
	counter_cnt1 = (counter_cnt1_high << 32) | counter_cnt1_low;

	if (sc->clkout_idx == AB_CLKOUT0)
		freq = (counter_cnt0 * ABC_OSCCLK_FREQ) / (counter_cnt1 * 1000);
	else
		freq = (counter_cnt1 * ABC_OSCCLK_FREQ) / (counter_cnt0 * 1000);

	pr_info("clkout: frequency calculated is :%lld KHz\n", freq);

	*val = freq * 1000;

	return 0;
}
