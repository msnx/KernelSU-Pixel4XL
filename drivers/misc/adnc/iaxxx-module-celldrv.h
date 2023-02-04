/*
 * iaxxx-module-celldrv.h  --  IAXXX module Cell Driver header
 *
 * Copyright 2017 Knowles, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __IAXXX_MODULE_CELLDRV_H__
#define __IAXXX_MODULE_CELLDRV_H__

#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/device.h>

/* Base name used by character devices. */
#define MODULE_CDEV_MAX_DEVICES	1

/* MODULE CELL parameters */
struct module_cell_params {
	int cdev_minor;
	dev_t dev_num;
	struct class *cdev_class;
};

struct module_device_priv {
	struct cdev cdev;
	dev_t dev_num;
	struct device *dev;
	struct device *parent;
	/*
	 * Add lock to protect cell params
	 * spinlock_t module_cell_lock
	 */
	struct regmap *regmap;
	int static_mem_blk_id;
};
#endif
