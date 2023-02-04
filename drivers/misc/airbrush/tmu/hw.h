/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _AIRBRUSH_TMU_HW_
#define _AIRBRUSH_TMU_HW_

#include <linux/bitops.h>
#include <linux/types.h>

#include <dt-bindings/thermal/airbrush_tmu.h>

struct ab_tmu_hw;
struct device;

struct ab_tmu_hw *devm_ab_tmu_hw_create(struct device *dev, u32 base);
void devm_ab_tmu_hw_destroy(struct ab_tmu_hw *hw);

struct ab_tmu_hw_events {
	void (*pcie_link_post_enable)(struct ab_tmu_hw *hw, void *data);
	void (*pcie_link_pre_disable)(struct ab_tmu_hw *hw, void *data);
	void (*post_enable)(struct ab_tmu_hw *hw, void *data);
};

void ab_tmu_hw_register_events(struct ab_tmu_hw *hw,
		const struct ab_tmu_hw_events *events, void *data);

bool ab_tmu_hw_pcie_link_lock(struct ab_tmu_hw *hw);
void ab_tmu_hw_pcie_link_unlock(struct ab_tmu_hw *hw);

u32 ab_tmu_hw_read(struct ab_tmu_hw *hw, u32 offset);
void ab_tmu_hw_write(struct ab_tmu_hw *hw, u32 offset, u32 value);

/* These should be hided after all dependents are moved to here. */
int ab_tmu_hw_initialize(struct ab_tmu_hw *hw);
void ab_tmu_hw_control(struct ab_tmu_hw *hw, bool on);
void ab_tmu_hw_set_irqs(struct ab_tmu_hw *hw, bool enable);
void ab_tmu_hw_clear_irqs(struct ab_tmu_hw *hw);

u32 ab_tmu_hw_read_current_temp(struct ab_tmu_hw *hw, int id);

/* Airbrush TMU registers */
#define AB_TMU_TRIMINFO0		0x0
#define AB_TMU_TRIMINFO(n)		(AB_TMU_TRIMINFO0 + ((n) * 4))
#define AB_TMU_TRIMINFO_ERROR1_FIELD	GENMASK(8, 0)
#define AB_TMU_TRIMINFO_ERROR2_FIELD	GENMASK(17, 9)
#define AB_TMU_TRIMINFO_CAL_FIELD	GENMASK(19, 18)

#define AB_TMU_CONTROL			0x20
#define AB_TMU_CONTROL_CORE_EN_FIELD	BIT_MASK(0)
#define AB_TMU_CONTROL_BUF_SLOPE_SEL	GENMASK(11, 8)
#define AB_TMU_CONTROL_TRIP_EN_FIELD	BIT_MASK(12)
#define AB_TMU_CONTROL_EN_FIELDS \
	(AB_TMU_CONTROL_CORE_EN_FIELD | AB_TMU_CONTROL_TRIP_EN_FIELD)
#define AB_TMU_CONTROL_BUF_SLOPE_SEL_VAL	0x5

#define AB_TMU_CONTROL1			0x24
#define AB_TMU_CONTROL1_REMOTE_PROBE	GENMASK(19, 16)

#define AB_TMU_STATUS			0x28
#define AB_TMU_STATUS_IDLE_FIELD	BIT_MASK(0)

#define AB_TMU_SAMPLING_INTERVAL	0x2c
#define AB_TMU_COUNTER_VALUE0		0x30

#define AB_TMU_COUNTER_VALUE1		0x34

#define AB_TMU_AVG_CONTROL		0x38
#define AB_TMU_AVG_CONTROL_MODE		GENMASK(2, 0)
#define AB_TMU_AVG_CONTROL_EN_DEM	BIT_MASK(4)
#define AB_TMU_AVG_CONTROL_MODE_VAL	0x6

#define AB_TMU_CURRENT_TEMP0_1		0x040
#define AB_TMU_CURRENT_TEMP2_4		0x044
#define AB_TMU_CURRENT_TEMP5_7		0x048

#define AB_TMU_THD_TEMP_STEP		0x020

#define AB_TMU_THD0_TEMP_RISE7_6	0x050
#define AB_TMU_THD0_TEMP_FALL7_6	0x060
#define AB_TMU_THD1_TEMP_RISE7_6	0x170
#define AB_TMU_THD1_TEMP_FALL7_6	0x180

#define AB_TMU_THD_TEMP_X(x, n, trip) \
({ \
	const unsigned int _n = n, _trip = trip; \
	const unsigned int reg_base = (_n == 0) ? \
		AB_TMU_THD0_TEMP_##x##7_6 : \
		AB_TMU_THD1_TEMP_##x##7_6 + (_n - 1) * AB_TMU_THD_TEMP_STEP; \
	const unsigned int trip_offset = ((7 - _trip) / 2) * 4; \
	reg_base + trip_offset; \
})
#define AB_TMU_THD_TEMP_RISE(n, trip) \
	AB_TMU_THD_TEMP_X(RISE, n, trip)
#define AB_TMU_THD_TEMP_FALL(n, trip) \
	AB_TMU_THD_TEMP_X(FALL, n, trip)

#define AB_TMU_THD_TEMP_X_SHIFT(trip) (16 * ((8 - trip) % 2))
#define AB_TMU_THD_TEMP_RISE_SHIFT(_n, trip) \
	AB_TMU_THD_TEMP_X_SHIFT(trip)
#define AB_TMU_THD_TEMP_FALL_SHIFT(_n, trip) \
	AB_TMU_THD_TEMP_X_SHIFT(trip)

#define AB_TMU_INTEN0			0x110
#define AB_TMU_INTSTAT0			0x114
#define AB_TMU_INTPEND0			0x118
#define AB_TMU_INTEN5			0x310
#define AB_TMU_INTSTAT5			0x314
#define AB_TMU_INTPEND5			0x318

#define AB_TMU_INTX_STEP			0x10
#define AB_TMU_INTX(x, n) \
({ \
	const unsigned int _n = (n); \
	(_n <= 4) ? \
		AB_TMU_INT##x##0 + _n * AB_TMU_INTX_STEP : \
		AB_TMU_INT##x##5 + ((_n - 5) * AB_TMU_INTX_STEP); \
})
#define AB_TMU_INTEN(n)			AB_TMU_INTX(EN, n)
#define AB_TMU_INTSTAT(n)		AB_TMU_INTX(STAT, n)
#define AB_TMU_INTPEND(n)		AB_TMU_INTX(PEND, n)

#define AB_TMU_INTX_RISE_SHIFT(n)	(n)
#define AB_TMU_INTX_FALL_SHIFT(n)	((n) + 16)
#define AB_TMU_INTEN_ALL		0xff01ff

#define AB_TMU_TEMP_MASK		0x1ff

#define AB_TMU_EMUL_CON			0x160
#define AB_TMU_EMUL_CON_ENABLE_FIELD	BIT_MASK(0)
#define AB_TMU_EMUL_CON_NEXTDATA_FIELD	GENMASK(15, 7)
#define AB_TMU_EMUL_CON_NEXTTIME_FIELD	GENMASK(31, 16)

#define AB_TMU_DEBUG_CURRENT_TEMP	0x164

#define AB_TMU_NO_TRIMMING		0
#define AB_TMU_ONE_POINT_TRIMMING	1
#define AB_TMU_TWO_POINT_TRIMMING	2

#define AB_TMU_NUM_REMOTE_PROBE		0x6
#define AB_TMU_NUM_ALL_PROBE		7
#define AB_TMU_TEMP_SHIFT		9

#endif /* _AIRBRUSH_TMU_HW_ */
