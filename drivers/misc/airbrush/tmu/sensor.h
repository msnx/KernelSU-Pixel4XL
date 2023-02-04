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
#ifndef _AIRBRUSH_TMU_SENSOR_
#define _AIRBRUSH_TMU_SENSOR_

#include <linux/types.h>

struct ab_tmu_hw;
struct ab_tmu_sensor;
struct device;

struct ab_tmu_sensor *devm_ab_tmu_sensor_create(struct device *dev,
		struct ab_tmu_hw *hw, int id);
// TODO destroy

void ab_tmu_sensor_load_trim_info(struct ab_tmu_sensor *sensor);
void ab_tmu_sensor_save_threshold(struct ab_tmu_sensor *sensor);
void ab_tmu_sensor_update(struct ab_tmu_sensor *sensor);
void ab_tmu_sensor_notify(struct ab_tmu_sensor *sensor);

int ab_tmu_sensor_raw_to_cel(const struct ab_tmu_sensor *sensor, u16 raw);

#endif /* _AIRBRUSH_TMU_SENSOR_ */
