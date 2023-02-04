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
#ifndef _AIRBRUSH_TMU_TRIM_
#define _AIRBRUSH_TMU_TRIM_

#include <linux/types.h>

/*
 * struct ab_tmu_trim - trimming information descriptor
 * @cal_type: calibration type for temperature calculation
 * @error1: fused value of the first point trim
 * @error2: fused value of the second point trim
 */
struct ab_tmu_trim {
	u8 cal_type;
	u16 error1;
	u16 error2;
};

/*
 * Converters between raw temperature code and Celsius degree. The
 * temperature is converted differently depending on the calibration
 * type.
 */
int ab_tmu_trim_raw_to_cel(const struct ab_tmu_trim *trim, u16 raw);
u16 ab_tmu_trim_cel_to_raw(const struct ab_tmu_trim *trim, int cel);

#endif /* _AIRBRUSH_TMU_TRIM_ */
