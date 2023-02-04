// SPDX-License-Identifier: GPL-2.0
/*
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
#include "trim.h"

#include "hw.h"

#define AB_TMU_FIRST_POINT_TRIM			25
#define AB_TMU_SECOND_POINT_TRIM		85
#define AB_TMU_DIFF_POINT_TRIM \
	(AB_TMU_SECOND_POINT_TRIM - AB_TMU_FIRST_POINT_TRIM)
#define AB_TMU_DEFAULT_TEMP_OFFSET		50

int ab_tmu_trim_raw_to_cel(const struct ab_tmu_trim *trim, u16 raw)
{
	int diff_error, cel;

	switch (trim->cal_type) {
	case AB_TMU_NO_TRIMMING:
	case AB_TMU_TWO_POINT_TRIMMING:
		diff_error = trim->error2 > trim->error1 ?
			trim->error2 - trim->error1 : 1;
		cel = (raw - trim->error1) * AB_TMU_DIFF_POINT_TRIM /
			diff_error + AB_TMU_FIRST_POINT_TRIM;
		break;
	case AB_TMU_ONE_POINT_TRIMMING:
		cel = raw - trim->error1 + AB_TMU_FIRST_POINT_TRIM;
		break;
	default:
		cel = raw - AB_TMU_DEFAULT_TEMP_OFFSET;
		break;
	}

	return cel;
}

u16 ab_tmu_trim_cel_to_raw(const struct ab_tmu_trim *trim, int cel)
{
	u16 raw;

	switch (trim->cal_type) {
	case AB_TMU_NO_TRIMMING:
	case AB_TMU_TWO_POINT_TRIMMING:
		raw = (cel - AB_TMU_FIRST_POINT_TRIM) *
			(trim->error2 - trim->error1) /
			AB_TMU_DIFF_POINT_TRIM + trim->error1;
		break;
	case AB_TMU_ONE_POINT_TRIMMING:
		raw = cel + trim->error1 - AB_TMU_FIRST_POINT_TRIM;
		break;
	default:
		raw = cel + AB_TMU_DEFAULT_TEMP_OFFSET;
		break;
	}

	return raw;
}
