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
#include "airbrush-cooling.h"

#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/thermal.h>

static struct dentry *ab_cooling_droot;

struct ab_cooling {
	struct thermal_cooling_device *cdev;

	struct ab_cooling_ops ops;
	void *op_data;

	struct mutex lock;
	bool enable;
	unsigned long state;

	struct dentry *droot;
};

static unsigned long ab_cooling_get_state_nolock(struct ab_cooling *cooling)
{
	return cooling->enable ? cooling->state : 0;
}

static int ab_cooling_op_get_max_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	*state = AIRBRUSH_COOLING_STATE_MAX;
	return 0;
}

static int ab_cooling_op_get_cur_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct ab_cooling *cooling = cdev->devdata;

	mutex_lock(&cooling->lock);
	*state = ab_cooling_get_state_nolock(cooling);
	mutex_unlock(&cooling->lock);
	return 0;
}

static int ab_cooling_op_set_cur_state(struct thermal_cooling_device *cdev,
		unsigned long state)
{
	struct ab_cooling *cooling = cdev->devdata;
	unsigned long old_state, new_state;

	mutex_lock(&cooling->lock);
	old_state = ab_cooling_get_state_nolock(cooling);
	cooling->state = state;
	new_state = ab_cooling_get_state_nolock(cooling);
	if (new_state != old_state) {
		cooling->ops.state_updated(cooling, old_state, state,
				cooling->op_data);
	}
	mutex_unlock(&cooling->lock);
	return 0;
}

static const struct thermal_cooling_device_ops ab_cooling_ops = {
	.get_max_state = ab_cooling_op_get_max_state,
	.get_cur_state = ab_cooling_op_get_cur_state,
	.set_cur_state = ab_cooling_op_set_cur_state,
};

static int ab_cooling_enable_get(void *data, u64 *val)
{
	struct ab_cooling *cooling = data;

	mutex_lock(&cooling->lock);
	*val = cooling->enable;
	mutex_unlock(&cooling->lock);
	return 0;
}

static int ab_cooling_enable_set(void *data, u64 val)
{
	struct ab_cooling *cooling = data;
	unsigned long old_state, new_state;

	mutex_lock(&cooling->lock);
	old_state = ab_cooling_get_state_nolock(cooling);
	cooling->enable = val;
	new_state = ab_cooling_get_state_nolock(cooling);
	if (new_state != old_state) {
		cooling->ops.state_updated(cooling, old_state, new_state,
				cooling->op_data);
	}
	mutex_unlock(&cooling->lock);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_ab_cooling_enable, ab_cooling_enable_get,
		ab_cooling_enable_set, "%llu\n");

static void ab_cooling_create_debugfs(struct ab_cooling *cooling)
{
	struct dentry *d;

	if (!ab_cooling_droot) {
		ab_cooling_droot = debugfs_create_dir("airbrush_cooling", NULL);
		if (!ab_cooling_droot)
			ab_cooling_droot = ERR_PTR(-ENOENT);
	}

	if (IS_ERR(ab_cooling_droot))
		goto err_out;

	cooling->droot = debugfs_create_dir(cooling->cdev->type,
			ab_cooling_droot);
	if (!cooling->droot)
		goto err_out;

	d = debugfs_create_file("enable", 0666, cooling->droot, cooling,
			&fops_ab_cooling_enable);
	if (!d)
		goto err_out;

	return;

err_out:
	dev_warn(&cooling->cdev->device,
			"Some error occurred while creating debugfs entry for airbrush cooling %s",
			cooling->cdev->type);
}

static void ab_cooling_remove_debugfs(struct ab_cooling *cooling)
{
	debugfs_remove_recursive(cooling->droot);
}

struct ab_cooling *ab_cooling_register(struct device_node *np, char *type,
		const struct ab_cooling_ops *ops, void *op_data, bool enable)
{
	struct ab_cooling *cooling;
	int err;

	cooling = kzalloc(sizeof(struct ab_cooling), GFP_KERNEL);
	if (!cooling)
		return ERR_PTR(-ENOMEM);

	cooling->ops = *ops;
	cooling->op_data = op_data;

	mutex_init(&cooling->lock);
	cooling->enable = enable;
	cooling->state = 0;
	cooling->cdev = thermal_of_cooling_device_register(np, type,
			cooling, &ab_cooling_ops);
	if (IS_ERR(cooling->cdev)) {
		err = PTR_ERR(cooling->cdev);
		kfree(cooling);
		return ERR_PTR(err);
	}

	ab_cooling_create_debugfs(cooling);
	return cooling;
}

void ab_cooling_unregister(struct ab_cooling *cooling)
{
	ab_cooling_remove_debugfs(cooling);
	thermal_cooling_device_unregister(cooling->cdev);
	kfree(cooling);
}
