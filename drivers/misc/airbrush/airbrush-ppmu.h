/*
 * airbrush-ppmu.h - Airbrush PPMU header file
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 * Author : Nishant Prajapati <nishant.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __AIRBRUSH_PPMU_H__
#define __AIRBRUSH_PPMU_H__

#define MAX_COUNTER	9
#define PPMU_DATA_WIDTH	4

struct airbrush_ppmu_data {
	unsigned long long event_count;
	unsigned long long clk_freq;
};

/*
 * struct airbrush_ppmu_state: structure to hold the state of ppmu device
 * @conf_events: number of currently configured events
 * @over_flow: array to hold overflow values
 */
struct airbrush_ppmu_state {
	int conf_events;
	int bmp;
	int over_flow[MAX_COUNTER];
};

/*
 * struct airbrush_ppmu: airbrush_ppmu device structure
 * @dev: device linked to this structure
 * @ppmu: structure holding data realated to airbrush_ppmu
 * @state: structure holding the current state of ppmu device
 */
struct airbrush_ppmu {
	struct device dev;
	u32 base;
	u32 irq;
	struct notifier_block nb;
	struct airbrush_ppmu_data ppmu_data;
	struct airbrush_ppmu_state *state;
};


enum ppmu_state {
	PPMU_DISABLE = 0,
	PPMU_ENABLE,
};

enum ppmu_counter {
	PPMU_PMNCNT0 = 0,
	PPMU_PMNCNT1,
	PPMU_PMNCNT2,
	PPMU_PMNCNT3,
	PPMU_PMNCNT4,
	PPMU_PMNCNT5,
	PPMU_PMNCNT6,
	PPMU_PMNCNT7,
	PPMU_PMNCNT_MAX,
};


/***
 * PPMU_V2.0 definitions
 */

enum {
	PPMU25_VER              = (0x0),
	PPMU25_PMNC             = (0x4),
	PPMU25_CNTENS           = (0x8),
	PPMU25_CNTENC           = (0xC),
	PPMU25_INTENS           = (0x10),
	PPMU25_INTENC           = (0x14),
	PPMU25_FLAG             = (0x18),
	PPMU25_CIG_CFG0         = (0x1C),
	PPMU25_CIG_CFG1         = (0x20),
	PPMU25_CIG_CFG2         = (0x24),
	PPMU25_CIG_RESULT       = (0x28),
	PPMU25_CNT_RESET        = (0x2C),
	PPMU25_CNT_AUTO         = (0x30),
	PPMU25_PMCNT0           = (0x34),
	PPMU25_PMCNT1           = (0x38),
	PPMU25_PMCNT2           = (0x3C),
	PPMU25_PMCNT3           = (0x40),
	PPMU25_PMCNT3_HIGH      = (0x44),
	PPMU25_CCNT             = (0x48),
	PPMU25_CONFIG_INFO_SET0 = (0X5C),
	PPMU25_INTERRUPT_TEST   = (0X60),
	PPMU25_PMCNT4           = (0XB4),
	PPMU25_PMCNT5           = (0xb8),
	PPMU25_PMCNT6           = (0xbc),
	PPMU25_PMCNT7           = (0xc0),
	PPMU25_PMCNT7_HIGH      = (0xc4),
	PPMU25_CONFIG_INFO_SET_1        = (0xdc),
	PPMU25_EVENT_EV0_TYPE           = (0x200),
	PPMU25_EVENT_EV1_TYPE   = (0x204),
	PPMU25_EVENT_EV2_TYPE   = (0x208),
	PPMU25_EVENT_EV3_TYPE   = (0x20C),
	PPMU25_EVENT_EV4_TYPE   = (0x210),
	PPMU25_EVENT_EV5_TYPE   = (0x214),
	PPMU25_EVENT_EV6_TYPE   = (0x218),
	PPMU25_EVENT_EV7_TYPE   = (0x21C),
	PPMU25_SM_ID_MASK       = (0x220),
	PPMU25_SM_ID_V          = (0x224),
	PPMU25_SM_ID_A          = (0x228),
	PPMU25_SM_OTH_V         = (0x22c),
	PPMU25_SM_OTH_A         = (0x230),
};

enum {
	/* Active Cycles */
	PPMU_EVENT_READ_BUSY            = 0,
	PPMU_EVENT_WRITE_BUSY           = 1,

	/* Number of transfer */
	PPMU_EVENT_READ_REQUEST         = 2,
	PPMU_EVENT_WRITE_REQUEST        = 3,
	PPMU_EVENT_READ_DATA            = 4,
	PPMU_EVENT_WRITE_DATA           = 5,
	PPMU_EVENT_WRITE_RESP           = 6,
	PPMU_EVENT_READ_LAST            = 7,
	PPMU_EVENT_WRITE_LAST           = 8,

	/* Blocked Cycles */
	PPMU_EVENT_READ_REQ_BLOCK       = 0x10,
	PPMU_EVENT_WRITE_REQ_BLOCK      = 0x11,
	PPMU_EVENT_READ_DATA_BLOCK      = 0x12,
	PPMU_EVENT_WRITE_DATA_BLOCK     = 0x13,
	PPMU_EVENT_WRITE_RESP_BLOCK     = 0x14,

	/* External Event */
	PPMU_EVENT_EXT_0                = 0x30,
	PPMU_EVENT_EXT_1                = 0x31,
	PPMU_EVENT_EXT_2                = 0x32,

	PPMU_EVENT_RW_BUSY              = 0x20,
	PPMU_EVENT_RW_REQUEST           = 0x21,
	PPMU_EVENT_RW_DATA              = 0x22,
	PPMU_EVENT_RW_REQ_BLOCK         = 0x23,
	/* Latency */
	PPMU_EVENT_READ_LATENCY         = 0x24,         //ARV_RLAST
	PPMU_EVENT_WRITE_LATENCY        = 0x25,         //AWV_BRESP
	PPMU_EVENT_READ_PEAK_LATENCY    = 0x26,
	PPMU_EVENT_WRITE_PEAK_LATENCY   = 0x27,

	/* External Event */
	PPMU_EVENT_EXT_MULTI_0          = 0x33,
};

/* PMNC register */
#define PPMU_V2_PMNC_START_MODE_SHIFT	20
#define PPMU_V2_PMNC_START_MODE_MASK	(0x3 << PPMU_V2_PMNC_START_MODE_SHIFT)

#define PPMU_PMNC_CC_RESET_SHIFT	2
#define PPMU_PMNC_COUNTER_RESET_SHIFT	1
#define PPMU_PMNC_ENABLE_SHIFT		0
#define PPMU_PMNC_START_MODE_MASK	BIT(16)
#define PPMU_PMNC_CC_DIVIDER_MASK	BIT(3)
#define PPMU_PMNC_CC_RESET_MASK		BIT(2)
#define PPMU_PMNC_COUNTER_RESET_MASK	BIT(1)
#define PPMU_PMNC_ENABLE_MASK		BIT(0)

#define PPMU25_PMNCT(x)			(PPMU25_PMCNT0 + (0x4 * x))
#define PPMU25_EVENT_EVx_TYPE(x)	(PPMU25_EVENT_EV0_TYPE + (0x4 * x))

#define PPMU_PMCNT(x)			BIT(x)
#define PPMU_PMCNT_ALL			0xff
#define PPMU_CNT_CC			BIT(31)
#define PPMU_CIG_UP_INT			BIT(4)
#define PPMU_CIG_LW_INT			BIT(0)
#define PPMU_RESET_VAL			0

#endif /* __AIRBRUSH_PPMU_H__ */
