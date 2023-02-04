// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "airbrush-thermal.h"
#include "airbrush-cooling.h"

#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/printk.h>
#include <linux/thermal.h>

static enum throttle_state to_throttle_state(unsigned long state)
{
	switch (state) {
	case 0:
		return THROTTLE_NONE;
	case 1:
		return THROTTLE_TO_MID;
	case 2:
		return THROTTLE_TO_LOW;
	case 3:
		return THROTTLE_TO_MIN;
	case 4:
		return THROTTLE_NOCOMPUTE;
	default:
		pr_warn("Bad throttle state, defaulting to THROTTLE_NOCOMPUTE\n");
		return THROTTLE_NOCOMPUTE;
	}
}

struct ab_thermal_cooling {
	struct ab_thermal *thermal;
	unsigned long state;
	struct ab_cooling *cooling;
};

struct ab_thermal {
	struct device *dev;

	struct ab_thermal_ops ops;
	void *op_data;

	struct mutex throttle_state_lock;
	bool throttle_ready;
	/* Combined from cooling_external and cooling_internal */
	enum throttle_state raw_throttle_state;

	/* For system wide phone skin temperature sensor */
	struct ab_thermal_cooling cooling_external;
	/* For battery current limiter (BCL) */
	struct ab_thermal_cooling cooling_bcl;
	/* For sensors probed by Airbrush TMU */
	struct ab_thermal_cooling cooling_internal;
};

static enum throttle_state ab_thermal_get_throttle_state(
		struct ab_thermal *thermal)
{
	return thermal->throttle_ready ? thermal->raw_throttle_state :
			THROTTLE_NONE;
}

static void ab_thermal_exit_cooling(struct ab_thermal_cooling *thermal_cooling)
{
	if (!IS_ERR_OR_NULL(thermal_cooling->cooling))
		ab_cooling_unregister(thermal_cooling->cooling);
}

static void ab_thermal_exit(struct ab_thermal *thermal)
{
	ab_thermal_exit_cooling(&thermal->cooling_internal);
	ab_thermal_exit_cooling(&thermal->cooling_bcl);
	ab_thermal_exit_cooling(&thermal->cooling_external);
}

static void ab_thermal_cooling_op_state_updated(
		const struct ab_cooling *cooling, unsigned long old_state,
		unsigned long new_state, void *cooling_op_data)
{
	struct ab_thermal_cooling *thermal_cooling = cooling_op_data;
	struct ab_thermal *thermal = thermal_cooling->thermal;
	unsigned long cooling_state = 0;
	enum throttle_state old_throttle_state, new_throttle_state;

	mutex_lock(&thermal->throttle_state_lock);

	/* Make sure that other cooling state would not be updated */
	thermal_cooling->state = new_state;

	cooling_state = max(cooling_state, thermal->cooling_external.state);
	cooling_state = max(cooling_state, thermal->cooling_bcl.state);
	cooling_state = max(cooling_state, thermal->cooling_internal.state);

	dev_info(thermal->dev, "Cooling state updated: ext=%d, bcl=%d, int=%d, ready=%d",
		thermal->cooling_external.state, thermal->cooling_bcl.state,
		thermal->cooling_internal.state, thermal->throttle_ready);

	old_throttle_state = ab_thermal_get_throttle_state(thermal);
	thermal->raw_throttle_state = to_throttle_state(cooling_state);
	new_throttle_state = ab_thermal_get_throttle_state(thermal);
	if (old_throttle_state != new_throttle_state) {
		thermal->ops.throttle_state_updated(new_throttle_state,
				thermal->op_data);
	}
	mutex_unlock(&thermal->throttle_state_lock);
}

const static struct ab_cooling_ops ab_thermal_cooling_ops = {
	.state_updated = ab_thermal_cooling_op_state_updated,
};

static struct ab_cooling *ab_thermal_cooling_register(
		struct ab_thermal *thermal,
		struct ab_thermal_cooling *thermal_cooling,
		const char *cooling_node_name, char *type, bool enable)
{
	struct device_node *cooling_node = NULL;

	if (cooling_node_name) {
		cooling_node = of_find_node_by_name(NULL, cooling_node_name);
		if (!cooling_node) {
			dev_warn(thermal->dev, "failed to find OF node for cooling device \"%s\".",
					cooling_node_name);
		}
	}
	return ab_cooling_register(cooling_node, type, &ab_thermal_cooling_ops,
			thermal_cooling, enable);
}

static int ab_thermal_init_cooling(struct ab_thermal_cooling *thermal_cooling,
		struct ab_thermal *thermal, const char *cooling_node_name,
		char *type, bool enable)
{
	thermal_cooling->thermal = thermal;
	thermal_cooling->state = 0;
	thermal_cooling->cooling = ab_thermal_cooling_register(thermal,
			thermal_cooling, cooling_node_name, type, enable);
	if (IS_ERR(thermal_cooling->cooling))
		return PTR_ERR(thermal_cooling->cooling);
	return 0;
}

static int ab_thermal_init(struct ab_thermal *thermal, struct device *dev,
		const struct ab_thermal_ops *ops, void *op_data)
{
	int err;

	thermal->dev = dev;
	thermal->ops = *ops;
	thermal->op_data = op_data;

	mutex_init(&thermal->throttle_state_lock);
	thermal->throttle_ready = false;
	thermal->raw_throttle_state = THROTTLE_NONE;

	err = ab_thermal_init_cooling(&thermal->cooling_external, thermal,
			AB_OF_CDEV_NAME, AB_CDEV_NAME, true);
	if (err) {
		dev_err(dev, "failed to initialize external cooling\n");
		ab_thermal_exit(thermal);
		return err;
	}

	err = ab_thermal_init_cooling(&thermal->cooling_bcl, thermal,
			AB_OF_CDEV_BCL_NAME, AB_CDEV_BCL_NAME, true);
	if (err) {
		dev_err(dev, "failed to initialize bcl cooling\n");
		ab_thermal_exit(thermal);
		return err;
	}

	err = ab_thermal_init_cooling(&thermal->cooling_internal, thermal,
			AB_OF_CDEV_INTERNAL_NAME, AB_CDEV_INTERNAL_NAME, true);
	if (err) {
		dev_err(dev, "failed to initialize internal cooling\n");
		ab_thermal_exit(thermal);
		return err;
	}

	return 0;
}

static void devm_ab_thermal_release(struct device *dev, void *res)
{
	struct ab_thermal *thermal = res;

	ab_thermal_exit(thermal);
}

struct ab_thermal *devm_ab_thermal_create(struct device *dev,
		const struct ab_thermal_ops *ops, void *op_data)
{
	struct ab_thermal *thermal;
	int err;

	thermal = devres_alloc(devm_ab_thermal_release,
			sizeof(struct ab_thermal), GFP_KERNEL);
	if (!thermal)
		return ERR_PTR(-ENOMEM);

	err = ab_thermal_init(thermal, dev, ops, op_data);
	if (err < 0) {
		devres_free(thermal);
		return ERR_PTR(err);
	}

	devres_add(dev, thermal);
	return thermal;
}

static int devm_ab_thermal_match(struct device *dev, void *res, void *data)
{
	struct ab_thermal *r = res;
	struct ab_thermal *d = data;

	return r == d;
}

void devm_ab_thermal_destroy(struct ab_thermal *thermal)
{
	devres_release(thermal->dev, devm_ab_thermal_release,
			devm_ab_thermal_match, thermal);
}

static void ab_thermal_set_throttle_ready(struct ab_thermal *thermal,
		bool throttle_ready)
{
	enum throttle_state old_throttle_state, new_throttle_state;

	if (IS_ERR_OR_NULL(thermal)) {
		dev_warn_once(thermal->dev, "Thermal not initialized properly.");
		return;
	}

	mutex_lock(&thermal->throttle_state_lock);
	old_throttle_state = ab_thermal_get_throttle_state(thermal);
	thermal->throttle_ready = throttle_ready;
	new_throttle_state = ab_thermal_get_throttle_state(thermal);
	if (old_throttle_state != new_throttle_state) {
		thermal->ops.throttle_state_updated(new_throttle_state,
				thermal->op_data);
	}
	mutex_unlock(&thermal->throttle_state_lock);
}

void ab_thermal_enable(struct ab_thermal *thermal)
{
	ab_thermal_set_throttle_ready(thermal, true);
}

void ab_thermal_disable(struct ab_thermal *thermal)
{
	ab_thermal_set_throttle_ready(thermal, false);
}
