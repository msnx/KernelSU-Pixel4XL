/*
 * iaxxx-cdev.c  --  iaxxx character device support
 *
 * Copyright 2017 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h>

#include "iaxxx.h"
#include "iaxxx-cdev.h"

#define IAXXX_CDEV_MINOR	0
#define IAXXX_CDEV_COUNT	3

static struct class *iaxxx_cdev_class;
static dev_t iaxxx_cdev_major;
static dev_t iaxxx_cdev_minor;
static dev_t iaxxx_cdev_indexes[IAXXX_CDEV_LAST] = { 0 };
static char *iaxxx_cdev_name;

static char *iaxxx_cdev_get_name(enum iaxxx_cdev_types type)
{
	switch (type) {
	case IAXXX_CDEV_TUNNEL:
		return "tunnel%d";
	case IAXXX_CDEV_REGDUMP:
		return "regdump";
	case IAXXX_CDEV_DEBUG:
		return "debug%d";
	case IAXXX_CDEV_CUSTOM:
		return iaxxx_cdev_name;
	case IAXXX_CDEV_CRASHDUMP:
		return "crashdump";
	case IAXXX_CDEV_SENSOR_TUNNEL_DEV:
		return "sensor_tunnel";
	case IAXXX_CDEV_MISC:
		return "iaxxx_misc";
	default:
		break;
	}

	return NULL;
}

int iaxxx_cdev_create(struct iaxxx_cdev *cdev,
	struct device *parent,
	const struct file_operations *fops,
	void *drvdata, enum iaxxx_cdev_types type)
{
	dev_t devno;
	char *name;
	int idx, err = -EINVAL;

	name = iaxxx_cdev_get_name(type);
	if (!name)
		return -EINVAL;

	cdev_init(&cdev->cdev, fops);
	cdev->cdev.owner = THIS_MODULE;

	devno = MKDEV(iaxxx_cdev_major, iaxxx_cdev_minor);
	err = cdev_add(&cdev->cdev, devno, 1);
	if (err) {
		pr_err("failed to add cdev=%04x error: %d", devno, err);
		goto exit_cdev_add;
	}

	idx = iaxxx_cdev_indexes[type];
	cdev->dev = device_create(iaxxx_cdev_class, parent, devno,
				drvdata, name, idx);
	if (IS_ERR(cdev->dev)) {
		err = PTR_ERR(cdev->dev);
		pr_err("device_create cdev=%04x failed: %d\n", devno, err);
		goto exit_dev_create;
	}

	iaxxx_cdev_minor++;
	iaxxx_cdev_indexes[type]++;

	return 0;

exit_dev_create:
	cdev_del(&cdev->cdev);
exit_cdev_add:
	return err;
}

int iaxxx_cdev_create_name(struct iaxxx_cdev *cdev,
	struct device *parent,
	const struct file_operations *fops,
	void *drvdata, char *name, ...)
{
	va_list args;
	int ret;

	va_start(args, name);
	iaxxx_cdev_name = kvasprintf(GFP_KERNEL, name, args);
	va_end(args);

	if (!iaxxx_cdev_name)
		return -ENOMEM;

	ret = iaxxx_cdev_create(cdev, parent, fops, drvdata, IAXXX_CDEV_CUSTOM);

	kfree(iaxxx_cdev_name);
	iaxxx_cdev_name = NULL;

	return ret;
}

void iaxxx_cdev_destroy(struct iaxxx_cdev *cdev)
{
	device_destroy(iaxxx_cdev_class, cdev->cdev.dev);
	cdev_del(&cdev->cdev);
}

int iaxxx_cdev_init(void)
{
	static const char *cdev_name = "iaxxx-dev";
	dev_t devno;
	int err;

	if (iaxxx_cdev_class)
		return 0;

	err = alloc_chrdev_region(&devno, 0, IAXXX_CDEV_COUNT, cdev_name);
	if (err) {
		pr_err("unable to allocate char dev = %d", err);
		return err;
	}
	iaxxx_cdev_major = MAJOR(devno);

	/* register device class */
	iaxxx_cdev_class = class_create(THIS_MODULE, cdev_name);
	if (IS_ERR(iaxxx_cdev_class)) {
		err = PTR_ERR(iaxxx_cdev_class);
		iaxxx_cdev_class = NULL;
		pr_err("unable to create %s class = %d\n", cdev_name, err);
		return err;
	}

	return 0;
}

void iaxxx_cdev_exit(void)
{
	class_destroy(iaxxx_cdev_class);
	unregister_chrdev_region(MKDEV(iaxxx_cdev_major, 0), IAXXX_CDEV_COUNT);
}

