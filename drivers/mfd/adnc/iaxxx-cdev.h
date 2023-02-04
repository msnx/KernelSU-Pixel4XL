/*
 * iaxxx-cdev.h  --  iaxxx character device support
 *
 * Copyright 2017 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __IAXXX_CDEV__
#define __IAXXX_CDEV__

#include <linux/cdev.h>

enum iaxxx_cdev_types {
	IAXXX_CDEV_TUNNEL = 0,
	IAXXX_CDEV_REGDUMP,
	IAXXX_CDEV_DEBUG,
	IAXXX_CDEV_CUSTOM,
	IAXXX_CDEV_CRASHDUMP,
	IAXXX_CDEV_SENSOR_TUNNEL_DEV,
	IAXXX_CDEV_MISC,
	/* Last available enum index */
	IAXXX_CDEV_LAST,
};

struct iaxxx_cdev {
	struct cdev cdev;
	struct device *dev;
};

int iaxxx_cdev_init(void);
void iaxxx_cdev_exit(void);

int iaxxx_cdev_create(struct iaxxx_cdev *cdev,
	struct device *parent,
	const struct file_operations *fops,
	void *drvdata, enum iaxxx_cdev_types type);
int iaxxx_cdev_create_name(struct iaxxx_cdev *cdev,
	struct device *parent,
	const struct file_operations *fops,
	void *drvdata, char *name, ...);
void iaxxx_cdev_destroy(struct iaxxx_cdev *cdev);

#endif /* __IAXXX_CDEV__ */
