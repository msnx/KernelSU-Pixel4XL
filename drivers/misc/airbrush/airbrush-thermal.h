/* SPDX-License-Identifier: GPL-2.0
 *
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
#ifndef _AIRBRUSH_THERMAL_H
#define _AIRBRUSH_THERMAL_H

#include <linux/airbrush-sm-ctrl.h>

#define AB_OF_CDEV_NAME "abc-cooling"
#define AB_CDEV_NAME "ab"

#define AB_OF_CDEV_BCL_NAME "abc-cooling-bcl"
#define AB_CDEV_BCL_NAME "ab-bcl"

#define AB_OF_CDEV_INTERNAL_NAME "abc-cooling-internal"
#define AB_CDEV_INTERNAL_NAME "ab-internal"

struct ab_thermal;
struct device;

struct ab_thermal_ops {
	void (*throttle_state_updated)(
			enum throttle_state throttle_state_id, void *op_data);
};

struct ab_thermal *devm_ab_thermal_create(struct device *dev,
		const struct ab_thermal_ops *ops, void *op_data);
void devm_ab_thermal_destroy(struct ab_thermal *thermal);

void ab_thermal_enable(struct ab_thermal *thermal);
void ab_thermal_disable(struct ab_thermal *thermal);

#endif /* _AIRBRUSH_THERMAL_H */
