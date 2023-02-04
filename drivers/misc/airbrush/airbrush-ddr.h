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

#ifndef _AIRBRUSH_DDR_H_
#define _AIRBRUSH_DDR_H_

#include <linux/airbrush-sm-ctrl.h>

#define DDR_SUCCESS	(0)
#define DDR_FAIL	(-1)

#define DRAM_CLK_1866MHZ		(1867000000)
#define DRAM_CLK_1600MHZ		(1600000000)
#define DRAM_CLK_1200MHZ		(1200000000)
#define DRAM_CLK_933MHZ			(934000000)
#define DRAM_CLK_800MHZ			(800000000)
#define DRAM_CLK_200MHZ			(200000000)
#define DRAM_CLK_OSC			(19200000)

#define REG_DDR_TRAIN_STATUS		SYSREG_AON_IPU_REG31
#define DDR_TRAIN_STARTED		(0x1 << 0)
#define DDR_TRAIN_INIT_DONE		(0x1 << 1)
#define DDR_TRAIN_VREF_DONE		(0x1 << 2)
#define DDR_TRAIN_REPEAT_DONE		(0x1 << 3)
#define DDR_TRAIN_COMPLETE		(0x1 << 4)
#define DDR_TRAIN_RESTORE_STARTED	(0x1 << 5)
#define DDR_TRAIN_RESTORE_COMPLETE	(0x1 << 6)
#define DDR_TRAIN_SAVE_STARTED		(0x1 << 7)
#define DDR_TRAIN_SAVE_COMPLETE		(0x1 << 8)
#define DDR_TRAIN_FAIL			(0x1 << 9)

#define AB_DDR_INIT_TIMEOUT		msecs_to_jiffies(50)

#define DDR_BOOT_TEST_WRITE		(0x1 << 0)
#define DDR_BOOT_TEST_READ		(0x1 << 1)
#define DDR_TEST_MEMTESTER		(0x1 << 4)
#define DDR_TEST_PCIE_DMA		(0x1 << 5)
#define DDR_TEST_NOPRINT		(0x1 << 31)
/* size should be multiple of 1MB */
#define DDR_TEST_PCIE_DMA_SIZE_MB(x)	(((x) & (0x3ff << 8)) >> 8)
#define DDR_TEST_EYE_MARGIN_SAMPLES(x)	(((x) & (0xff << 20)) >> 20)
#define DDR_BOOT_TEST_READ_WRITE (DDR_BOOT_TEST_READ | DDR_BOOT_TEST_WRITE)

#define DDR_TEST_PCIE_DMA_READ(x)	\
	(DDR_TEST_PCIE_DMA | DDR_BOOT_TEST_READ | (x << 8))

#define DDR_TEST_PCIE_DMA_WRITE(x)	\
	(DDR_TEST_PCIE_DMA | DDR_BOOT_TEST_WRITE | (x << 8))

#define DDR_TEST_PCIE_DMA_READ_WRITE(x)	\
	(DDR_TEST_PCIE_DMA_READ(x) | DDR_TEST_PCIE_DMA_WRITE(x))

/* DDR PPC driver related defines */
#define PPC_COUNTER_0			(0)
#define PPC_COUNTER_1			(1)
#define PPC_COUNTER_2			(2)
#define PPC_COUNTER_3			(3)
#define PPC_COUNTER_MAX			(4)

#define PPC_EVENT_WR_PORT0		(0x0)
#define PPC_EVENT_RD_PORT0		(0x2)
#define PPC_EVENT_WR_PORT1		(0x4)
#define PPC_EVENT_RD_PORT1		(0x6)
#define PPC_EVENT_WR_PORT2		(0x8)
#define PPC_EVENT_RD_PORT2		(0xa)
#define PPC_EVENT_WR_PORT3		(0xc)
#define PPC_EVENT_RD_PORT3		(0xe)
#define PPC_EVENT_WR2BANK0_PORT2	(0x38)
#define PPC_EVENT_WR2BANK1_PORT2	(0x39)
#define PPC_EVENT_WR2BANK2_PORT2	(0x3a)
#define PPC_EVENT_WR2BANK3_PORT2	(0x3b)
#define PPC_EVENT_WR_CAS		(0x64)
#define PPC_EVENT_RD_CAS		(0x65)
#define PPC_EVENT_RW_CAS		(0x66)
#define PPC_EVENT_WR_SHED		(0x68)
#define PPC_EVENT_RD_SHED		(0x69)
#define PPC_EVENT_RW_SHED		(0x6a)
#define PPC_EVENT_WR_XFER		(0x6c)
#define PPC_EVENT_RD_XFER		(0x6d)
#define PPC_EVENT_RW_XFER		(0x6e)
#define PPC_EVENT_PD_CHIP0		(0x88)
#define PPC_EVENT_PD_CHIP1		(0x89)
#define PPC_EVENT_SREF_CHIP0		(0x8a)
#define PPC_EVENT_SREF_CHIP1		(0x8b)
#define PPC_EVENT_AXI_RD_REQ_PORT0	(0x90)
#define PPC_EVENT_AXI_RD_REQ_PORT1	(0x91)
#define PPC_EVENT_AXI_RD_REQ_PORT2	(0x92)
#define PPC_EVENT_AXI_RD_REQ_PORT3	(0x93)
#define PPC_EVENT_AXI_WR_REQ_PORT0	(0x94)
#define PPC_EVENT_AXI_WR_REQ_PORT1	(0x95)
#define PPC_EVENT_AXI_WR_REQ_PORT2	(0x96)
#define PPC_EVENT_AXI_WR_REQ_PORT3	(0x97)

#endif /* _AIRBRUSH_DDR_H_ */
