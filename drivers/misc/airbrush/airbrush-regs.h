/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha(shaik.ameer@samsung.com)
 *
 * Airbrush DDR Interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#ifndef _AIRBRUSH_REGS_H_
#define _AIRBRUSH_REGS_H_

static inline uint32_t RD_REG(uint32_t addr)
{
	uint32_t data = 0xffffffff;

	WARN_ON(abc_pcie_config_read(addr & 0xFFFFFF, 0x4, &data));

	return data;
}

static inline void WR_REG(uint32_t addr, uint32_t data)
{
	WARN_ON(abc_pcie_config_write(addr & 0xFFFFFF, 0x4, data));
}

#define ABC_BASE_CMU_AON		0x10b10000
#define ABC_BASE_CMU_CORE		0x10f10000
#define ABC_BASE_CMU_FSYS		0x10710000
#define ABC_BASE_CMU_MIF		0x10510000
#define ABC_BASE_UART 			0x10B60000
#define ABC_BASE_SYSREG_AON		0x10B30000
#define ABC_BASE_GPIO_AON		0x10B40000
#define ABC_BASE_SYSREG_CENTRAL_PMU	0x10BA0000
#define ABC_BASE_OTP_WRAPPER		0x10BB0000

#define PLL_CON0_PLL_PHY_MIF		0x10510140
#define CLK_CON_DIV_DIV4_PLLCLK		0x10b11800
#define CLK_CON_DIV_PLL_AON_CLK		0x10b1180c

#define ABC_BASE_PCIE_PCS		0x10750000
#define PCS_OUT_VEC_4			(ABC_BASE_PCIE_PCS + 0x154)

/* --------------------------------------------------
 * 1.7.1.65 AP_HW_OPTION_1
 *    Base Address: 0x10BB_0000
 *    Address = Base Address + 0x4014, Reset Value = 0x0000_0000
 *    OTP_DBG_DIS[2] (OTP_FW_PATCH_DIS)
 *    OTP_UART_PRINT_DISABLE[3] (Disable UART prints when set)
 *    OTP_DDR_OTP_FLASHED[7]
 *    OTP_AP_DDR [9] (0: M0_DDR_INIT, 1: HOST_DDR_INIT)
 * --------------------------------------------------
 */
#define OTP_AP_HW_OPTION_1()	(RD_REG(ABC_BASE_OTP_WRAPPER + 0x4014))
#define IS_OTP_FW_PATCH_DIS()	(!!(OTP_AP_HW_OPTION_1() & (1 << 2)))
#define IS_UART_PRINT_DISABLE() (!!(OTP_AP_HW_OPTION_1() & (1 << 3)))
#define IS_DDR_OTP_FLASHED()	(!!(OTP_AP_HW_OPTION_1() & (1 << 7)))
#define IS_HOST_DDR_INIT()	(!!(OTP_AP_HW_OPTION_1() & (1 << 9)))
#define IS_M0_DDR_INIT()	(!IS_HOST_DDR_INIT())

/* SYSREG REGISTERS */
#define SYSREG_AON_IPU_REG31		(ABC_BASE_SYSREG_AON + 0x440)
#define GET_DDR_TRAIN_STATE()		(RD_REG(SYSREG_AON_IPU_REG31))
#define SYSREG_AON_SPI0_AHB_ENABLE	(ABC_BASE_SYSREG_AON + 0x328)
#define IS_SPI_FSM_ENABLE()		(RD_REG(SYSREG_AON_SPI0_AHB_ENABLE) & 0x1)
#define SYSREG_REG_SRAM_ADDR		(ABC_BASE_SYSREG_AON + 0x374)
#define SYSREG_REG_DDR_INIT		(ABC_BASE_SYSREG_AON + 0x378)
#define SYSREG_REG_TRN_ADDR		(ABC_BASE_SYSREG_AON + 0x10)
#define GET_REG_TRN_ADDR()		(RD_REG(SYSREG_REG_TRN_ADDR))
#define SET_REG_TRN_ADDR(x)		(WR_REG(SYSREG_REG_TRN_ADDR, x))

/* SYSREG CENTRAL PMU REGISTERS */
#define PMU_CONTROL			(ABC_BASE_SYSREG_CENTRAL_PMU + 0x4)
#define PMU_CONTROL_PHY_RET_OFF()	(WR_REG(PMU_CONTROL, RD_REG(PMU_CONTROL) & ~( 0x1 << 7)))
#define PMU_CONTROL_PHY_RET_ON()	(WR_REG(PMU_CONTROL, RD_REG(PMU_CONTROL) | (0x1 << 7)))

/* --------------------------------------------------
 * GPIOs used in finding the OSCCLK
 *	GPF0_1 : XOSC_CLK_IS_38P4
 * ------------------------------------------------- */
#define GPIO_GPF0_CON			(ABC_BASE_GPIO_AON + 0xC0)
#define GPIO_GPF0_DAT			(ABC_BASE_GPIO_AON + 0xC4)
#define IS_OSCCLK_38P4()		(!!(RD_REG(GPIO_GPF0_DAT) & (0x1 << 1)))

/* --------------------------------------------------
 * Configure the GPIOs used in DDR initialization/training
 *	GPG0_4 : DDR_SR (XGPIO10)
 *	GPG0_5 : MIF_INIT_DONE (XGPIO11)
 *	GPG0_6 : CLK_IN_SENSE (XCKEIN)
 * ------------------------------------------------- */
#define GPIO_GPG0_CON			(ABC_BASE_GPIO_AON + 0xE0)
#define GPIO_GPG0_DAT			(ABC_BASE_GPIO_AON + 0xE4)
#define GPIO_CKE_IN_SENSE()		(!!(RD_REG(GPIO_GPG0_DAT) & (1 << 6)))
#define GPIO_DDR_SR()			(!!(RD_REG(GPIO_GPG0_DAT) & (1 << 4)))

#endif	/* _AIRBRUSH_REGS_H_ */
