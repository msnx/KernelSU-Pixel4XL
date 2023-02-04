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
#include "sensor.h"

#include <linux/bitfield.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include "../../../thermal/thermal_core.h"

#include "hw.h"
#include "trim.h"

#define MCELSIUS	1000

#define AB_TMU_SENSOR_TRIP_NUM 8

static const u16 no_trimming_error1[] = {287, 286, 287, 287, 286, 286, 286};
static const u16 no_trimming_error2[] = {346, 346, 347, 347, 347, 346, 346};

struct ab_tmu_sensor {
	struct device *dev;
	struct ab_tmu_hw *hw;
	int id;
	bool of_trip_loaded;
	int of_trip_temp[AB_TMU_SENSOR_TRIP_NUM];
	int of_trip_hyst[AB_TMU_SENSOR_TRIP_NUM];
	struct ab_tmu_trim trim;
	struct thermal_zone_device *tzd;
};

static void ab_tmu_sensor_set_threshold(struct ab_tmu_sensor *sensor,
		u32 thd_off, u32 thd_shift, int temp)
{
	struct ab_tmu_hw *hw = sensor->hw;
	u32 code, thd_val;

	code = ab_tmu_trim_cel_to_raw(&sensor->trim, temp);
	thd_val = ab_tmu_hw_read(hw, thd_off);
	thd_val &= ~(AB_TMU_TEMP_MASK << thd_shift);
	thd_val |= code << thd_shift;
	ab_tmu_hw_write(hw, thd_off, thd_val);
}

static void ab_tmu_sensor_set_threshold_rise(struct ab_tmu_sensor *sensor,
		int trip, int temp)
{
	u32 thd_off = AB_TMU_THD_TEMP_RISE(sensor->id, trip);
	u32 thd_shift = AB_TMU_THD_TEMP_RISE_SHIFT(sensor->id, trip);

	ab_tmu_sensor_set_threshold(sensor, thd_off, thd_shift, temp);
}

static void ab_tmu_sensor_set_threshold_fall(struct ab_tmu_sensor *sensor,
		int trip, int temp)
{
	u32 thd_off = AB_TMU_THD_TEMP_FALL(sensor->id, trip);
	u32 thd_shift = AB_TMU_THD_TEMP_FALL_SHIFT(sensor->id, trip);

	ab_tmu_sensor_set_threshold(sensor, thd_off, thd_shift, temp);
}

static void ab_tmu_sensor_set_thresholds(struct ab_tmu_sensor *sensor,
		int trip, int temp, int hyst)
{
	ab_tmu_sensor_set_threshold_rise(sensor, trip, temp);
	ab_tmu_sensor_set_threshold_fall(sensor, trip, temp - hyst);
}

static int ab_tmu_sensor_op_get_temp(void *data, int *temp)
{
	struct ab_tmu_sensor *sensor = data;
	struct ab_tmu_hw *hw = sensor->hw;
	bool pcie_link_ready;
	u32 code;

	if (!data)
		return -EINVAL;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		code = ab_tmu_hw_read_current_temp(hw, sensor->id);
		*temp = ab_tmu_trim_raw_to_cel(&sensor->trim, code) * MCELSIUS;
	} else {
		/*
		 * Return 0 degree Celsius while TMU is not ready to
		 * suppress the error log while reading the temperature.
		 */
		*temp = 0;
	}
	ab_tmu_hw_pcie_link_unlock(hw);
	return 0;
}

static void ab_tmu_enable_emulate(struct ab_tmu_hw *hw)
{
	u32 emul_con;

	emul_con = ab_tmu_hw_read(hw, AB_TMU_EMUL_CON);
	emul_con |= FIELD_PREP(AB_TMU_EMUL_CON_ENABLE_FIELD, 1);

	ab_tmu_hw_write(hw, AB_TMU_EMUL_CON, emul_con);
}

static void ab_tmu_disable_emulate(struct ab_tmu_hw *hw)
{
	u32 emul_con;

	emul_con = ab_tmu_hw_read(hw, AB_TMU_EMUL_CON);
	emul_con &= ~AB_TMU_EMUL_CON_ENABLE_FIELD;

	ab_tmu_hw_write(hw, AB_TMU_EMUL_CON, emul_con);
}

static void ab_tmu_set_emulate_data(struct ab_tmu_hw *hw, u16 next_time,
		u16 next_data)
{
	u32 emul_con;

	if (next_time == 0x0)
		next_time = 0x1;

	emul_con = ab_tmu_hw_read(hw, AB_TMU_EMUL_CON);
	emul_con &= ~(AB_TMU_EMUL_CON_NEXTTIME_FIELD |
		AB_TMU_EMUL_CON_NEXTDATA_FIELD);
	emul_con |= FIELD_PREP(AB_TMU_EMUL_CON_NEXTTIME_FIELD, next_time) |
		FIELD_PREP(AB_TMU_EMUL_CON_NEXTDATA_FIELD, next_data);

	ab_tmu_hw_write(hw, AB_TMU_EMUL_CON, emul_con);
}

static int ab_tmu_sensor_op_set_emul_temp(void *data, int temp)
{
	struct ab_tmu_sensor *sensor = data;
	struct ab_tmu_hw *hw = sensor->hw;
	bool pcie_link_ready;
	u16 code;
	int ret;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		if (temp != 0) {
			ab_tmu_enable_emulate(hw);
			code = ab_tmu_trim_cel_to_raw(&sensor->trim,
					temp / MCELSIUS);
			ab_tmu_set_emulate_data(hw, 1, code);
		} else {
			ab_tmu_disable_emulate(hw);
		}
		ret = 0;
	} else {
		ret = -ENODEV;
	}
	ab_tmu_hw_pcie_link_unlock(hw);

	return ret;
}

static int ab_tmu_sensor_op_set_trip_temp(void *data, int trip, int temp)
{
	struct ab_tmu_sensor *sensor = data;
	struct ab_tmu_hw *hw = sensor->hw;
	struct thermal_zone_device *tzd = sensor->tzd;
	bool pcie_link_ready;
	int temp_cel, hyst, hyst_cel, ret;

	if (trip < 0 || trip >= AB_TMU_SENSOR_TRIP_NUM)
		return -EINVAL;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		if (temp != 0) {
			temp_cel = temp / MCELSIUS;
		} else {
			temp_cel = sensor->of_trip_temp[trip] / MCELSIUS;
			tzd->ops->set_trip_hyst(tzd, trip,
					sensor->of_trip_hyst[trip]);
		}
		tzd->ops->get_trip_hyst(tzd, trip, &hyst);
		hyst_cel = hyst / MCELSIUS;
		ab_tmu_sensor_set_thresholds(sensor, trip, temp_cel, hyst_cel);
		ret = 0;
	} else {
		ret = -ENODEV;
	}
	ab_tmu_hw_pcie_link_unlock(hw);

	return ret;
}

static int ab_tmu_sensor_op_get_trip_temp(void *data, int trip, int *temp)
{
	struct ab_tmu_sensor *sensor = data;
	struct ab_tmu_hw *hw = sensor->hw;
	bool pcie_link_ready;
	u32 thd_rise;

	if (trip < 0 || trip >= AB_TMU_SENSOR_TRIP_NUM)
		return -EINVAL;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		thd_rise = ab_tmu_hw_read(hw,
				AB_TMU_THD_TEMP_RISE(sensor->id, trip));
		thd_rise >>= AB_TMU_THD_TEMP_RISE_SHIFT(sensor->id, trip);
		thd_rise &= AB_TMU_TEMP_MASK;
		*temp = ab_tmu_trim_raw_to_cel(&sensor->trim, thd_rise) *
				MCELSIUS;
	} else {
		/*
		 * Thermal governor would get trip point and ignore the
		 * error code even TMU is not ready. Return a highest
		 * invalid Celsius in such case to prevent trigger any
		 * trip point. The value named 'LOW' since it is a high
		 * value indicating invalid for *low* temperature
		 * tracking zone.
		 */
		*temp = THERMAL_TEMP_INVALID_LOW;
	}
	ab_tmu_hw_pcie_link_unlock(hw);

	return 0;
}

/*
 * All of the sensor ops should get pcie lock before accessing tmu
 * registers.
 */
static struct thermal_zone_of_device_ops ab_tmu_sensor_ops = {
	.get_temp = ab_tmu_sensor_op_get_temp,
	.set_emul_temp = ab_tmu_sensor_op_set_emul_temp,
	.set_trip_temp = ab_tmu_sensor_op_set_trip_temp,
	.get_trip_temp = ab_tmu_sensor_op_get_trip_temp,
};

struct ab_tmu_sensor *devm_ab_tmu_sensor_create(struct device *dev,
		struct ab_tmu_hw *hw, int id)
{
	int err;
	struct ab_tmu_sensor *sensor;

	if (id < 0 || id >= AB_TMU_NUM_ALL_PROBE)
		return ERR_PTR(-EINVAL);

	sensor = devm_kzalloc(dev, sizeof(struct ab_tmu_sensor), GFP_KERNEL);
	if (!sensor)
		return ERR_PTR(-ENOMEM);

	sensor->dev = dev;
	sensor->hw = hw;
	sensor->id = id;
	sensor->trim.error1 = no_trimming_error1[id];
	sensor->trim.error2 = no_trimming_error2[id];
	sensor->tzd = devm_thermal_zone_of_sensor_register(dev, id, sensor,
			&ab_tmu_sensor_ops);
	if (IS_ERR(sensor->tzd)) {
		err = PTR_ERR(sensor->tzd);
		devm_kfree(dev, sensor);
		return ERR_PTR(err);
	}

	return sensor;
}

void ab_tmu_sensor_load_trim_info(struct ab_tmu_sensor *sensor)
{
	struct ab_tmu_hw *hw = sensor->hw;
	struct ab_tmu_trim *trim = &sensor->trim;
	u32 trim_info;

	trim_info = ab_tmu_hw_read(hw, AB_TMU_TRIMINFO(sensor->id));

	trim->cal_type = FIELD_GET(AB_TMU_TRIMINFO_CAL_FIELD, trim_info);
	if (sensor->trim.cal_type == AB_TMU_NO_TRIMMING)
		return;

	trim->error1 = FIELD_GET(AB_TMU_TRIMINFO_ERROR1_FIELD, trim_info);
	trim->error2 = FIELD_GET(AB_TMU_TRIMINFO_ERROR2_FIELD, trim_info);
}

static void ab_tmu_sensor_load_of_trip(struct ab_tmu_sensor *sensor)
{
	struct thermal_zone_device *tz = sensor->tzd;
	const struct thermal_trip *trips;
	int i, ntrips;

	if (sensor->of_trip_loaded)
		return;

	trips = of_thermal_get_trip_points(tz);
	if (!trips) {
		dev_warn(sensor->dev, "Failed to get trip points from OF");
		return;
	}

	ntrips = of_thermal_get_ntrips(tz);
	if (ntrips != AB_TMU_SENSOR_TRIP_NUM) {
		dev_warn(sensor->dev, "ntrip count mismatch to sensor trip num: %d",
				ntrips);
		return;
	}
	for (i = 0; i < AB_TMU_SENSOR_TRIP_NUM; i++) {
		sensor->of_trip_temp[i] = trips[i].temperature;
		sensor->of_trip_hyst[i] = trips[i].hysteresis;
	}
	sensor->of_trip_loaded = true;
}

void ab_tmu_sensor_save_threshold(struct ab_tmu_sensor *sensor)
{
	int i;

	ab_tmu_sensor_load_of_trip(sensor);
	if (!sensor->of_trip_loaded)
		return;

	for (i = 0; i < AB_TMU_SENSOR_TRIP_NUM; i++) {
		ab_tmu_sensor_set_thresholds(sensor, i,
				sensor->of_trip_temp[i] / MCELSIUS,
				sensor->of_trip_hyst[i] / MCELSIUS);
	}
}

void ab_tmu_sensor_update(struct ab_tmu_sensor *sensor)
{
	struct thermal_zone_device *tzd = sensor->tzd;

	thermal_zone_device_update(tzd, THERMAL_EVENT_UNSPECIFIED);
}

void ab_tmu_sensor_notify(struct ab_tmu_sensor *sensor)
{
	struct thermal_zone_device *tzd = sensor->tzd;
	int i, temp;
	char *envp[2];

	mutex_lock(&tzd->lock);
	/* Find the level for which trip happened */
	for (i = 0; i < of_thermal_get_ntrips(tzd); i++) {
		tzd->ops->get_trip_temp(tzd, i, &temp);
		if (tzd->last_temperature < temp)
			break;
	}

	envp[0] = kasprintf(GFP_KERNEL, "%d", i);
	envp[1] = NULL;
	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, envp);
	kfree(envp[0]);
	mutex_unlock(&tzd->lock);
}

int ab_tmu_sensor_raw_to_cel(const struct ab_tmu_sensor *sensor, u16 raw)
{
	return ab_tmu_trim_raw_to_cel(&sensor->trim, raw);
}
