/*
 * iaxxx-odsp-celldrv.h  --  IAXXX odsp Cell Driver header
 *
 * Copyright 2017 Knowles, Inc.
 *
 * Author: Sharada Kumar <Sharada.Kumar@knowles.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __IAXXX_ODSP_CELLDRV_H__
#define __IAXXX_ODSP_CELLDRV_H__

#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/device.h>

/* Base name used by character devices. */
#define ODSP_CDEV_MAX_DEVICES	1


enum plugin_state {
	NULL_STATE = 0,
	CREATE,
	INIT,
	RESET,
	ENABLE,
	DISABLE,
	DESTROY
};


/* ODSP CELL parameters */
struct odsp_cell_params {
	int cdev_minor;
	dev_t dev_num;
	struct class *cdev_class;
};


struct odsp_device_priv {
	struct cdev cdev;
	dev_t dev_num;
	struct device *dev;
	struct device *parent;
	struct regmap *regmap;
	struct mutex ioctl_lock;
	int  static_mem_blk_id;

};

#endif
