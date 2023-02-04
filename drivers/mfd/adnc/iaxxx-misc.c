/*
 * Copyright (c) 2019-, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include "iaxxx.h"
#include "iaxxx-cdev.h"
#include "iaxxx-misc.h"

struct iaxxx_misc_hwinfo_type {
	char part[128];
	bool initialized;
};

/*
 * DSP Speech Up/Down time and counts
 */
struct wdsp_stat_priv_type {
	ktime_t ktime_zero;
	ktime_t uptime;
	ktime_t crashtime;
	s64 total_uptime;
	s64 total_downtime;
	u64 crash_count;
	u64 recover_count;
	uint32_t action;
	struct mutex lock;
};

/*
 * IAXXX Misc data
 */
struct iaxxx_misc_priv_type {
	struct iaxxx_priv *priv;
	struct wdsp_stat_priv_type wdsp_stat_priv;
	struct notifier_block nb_core;	/* IAXXX Core notifier */
	struct device *dev;
	struct iaxxx_cdev misc_cdev;
	struct iaxxx_misc_hwinfo_type hwinfo_priv;
};

static ssize_t iaxxx_misc_hwinfo_part_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_misc_priv_type *misc_priv = dev_get_drvdata(dev);
	struct iaxxx_misc_hwinfo_type *hwinfo = (misc_priv ?
			&misc_priv->hwinfo_priv : NULL);
	ssize_t ret = 0;


	if (!hwinfo || !(hwinfo->initialized))
		return scnprintf(buf, PAGE_SIZE, "Unknown\n");

	ret = scnprintf(buf, PAGE_SIZE, "%s", hwinfo->part);

	return ret;
}
static DEVICE_ATTR(hwinfo_part_number, 0440,
		iaxxx_misc_hwinfo_part_number_show, NULL);

static ssize_t iaxxx_misc_update_pn(struct iaxxx_misc_priv_type *misc_priv)
{
	struct iaxxx_priv *priv = misc_priv ? misc_priv->priv : NULL;
	struct iaxxx_misc_hwinfo_type *hwinfo = (misc_priv ?
			&misc_priv->hwinfo_priv : NULL);

	int ret = 0;
	uint32_t trim_ldo_bg = 0;
	char tmpbuf[64];

	if (!misc_priv || !priv || !hwinfo)
		return -EFAULT;

	scnprintf(tmpbuf, sizeof(tmpbuf),
			"%s", priv->hwinfo.revision);
	trim_ldo_bg = priv->hwinfo.trim_ldo_bg;

	if (strnlen(tmpbuf, sizeof(tmpbuf)) > 0) {
		ret = scnprintf(hwinfo->part,
				sizeof(hwinfo->part),
				"IA8508%s%s",
				tmpbuf,
				(trim_ldo_bg > 0) ? "C" : "");
	} else {
		ret = scnprintf(hwinfo->part, sizeof(hwinfo->part), "%s",
			"Unknown");
	}

	return ret;
}

void iaxxx_misc_update_hwinfo(struct iaxxx_misc_priv_type *misc_priv,
				unsigned long action)
{
	struct iaxxx_misc_hwinfo_type *hwinfo = (misc_priv ?
			&misc_priv->hwinfo_priv : NULL);

	switch (action) {
	case IAXXX_EV_STARTUP:
		/* only do it once after boot no matter hw status */
		if (!hwinfo->initialized) {
			iaxxx_misc_update_pn(misc_priv);
			hwinfo->initialized = true;
		}
		break;
	default:
		break;
	}
}

static struct attribute *iaxxx_misc_hwinfo_attrs[] = {
	&dev_attr_hwinfo_part_number.attr,
	NULL,
};

static const struct attribute_group iaxxx_misc_hwinfo_attr_group = {
	.attrs = iaxxx_misc_hwinfo_attrs,
};

/*
 * wdsp_stat show function
 */
static ssize_t iaxxx_misc_wdsp_stat_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_misc_priv_type *misc_priv = dev_get_drvdata(dev);
	struct wdsp_stat_priv_type *wdsp = &misc_priv->wdsp_stat_priv;
	ssize_t ret = 0;
	ktime_t current_time;

	mutex_lock(&wdsp->lock);

	current_time = ktime_get_boottime();
	if (ktime_compare(wdsp->uptime, wdsp->ktime_zero)) {
		wdsp->total_uptime +=
			ktime_ms_delta(current_time, wdsp->uptime);
		wdsp->uptime = current_time;
	}

	if (ktime_compare(wdsp->crashtime, wdsp->ktime_zero)) {
		wdsp->total_downtime +=
			ktime_ms_delta(current_time, wdsp->crashtime);
		wdsp->crashtime = current_time;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%lld,%lld,%llu,%llu",
			wdsp->total_uptime,
			wdsp->total_downtime,
			wdsp->crash_count,
			wdsp->recover_count);

	wdsp->total_uptime = 0;
	wdsp->total_downtime = 0;
	wdsp->crash_count = 0;
	wdsp->recover_count = 0;

	mutex_unlock(&wdsp->lock);
	return ret;
}
static DEVICE_ATTR(wdsp_stat, 0440, iaxxx_misc_wdsp_stat_show, NULL);

enum {
	WDSP_STAT_CRASH = 0,
	WDSP_STAT_DOWN,
	WDSP_STAT_UP,
};

static void iaxxx_misc_update_wdsp_stat(struct iaxxx_misc_priv_type *misc_priv,
				int action)
{
	struct wdsp_stat_priv_type *wdsp = (misc_priv ?
		&misc_priv->wdsp_stat_priv : NULL);
	ktime_t current_time;


	if (!wdsp) {
		pr_warn("%s invalid device", __func__);
		return;
	}

	mutex_lock(&wdsp->lock);
	wdsp->action = action;
	current_time = ktime_get_boottime();

	switch (action) {
	case WDSP_STAT_CRASH:
		wdsp->crashtime = current_time;
		if (ktime_compare(wdsp->uptime, wdsp->ktime_zero)) {
			wdsp->total_uptime +=
				ktime_ms_delta(wdsp->crashtime, wdsp->uptime);
			wdsp->uptime = wdsp->ktime_zero;
		}
		wdsp->crash_count++;
		break;

	case WDSP_STAT_UP:
		wdsp->uptime = current_time;
		if (ktime_compare(wdsp->crashtime, wdsp->ktime_zero)) {
			wdsp->total_downtime +=
				ktime_ms_delta(wdsp->uptime,
						wdsp->crashtime);
			wdsp->crashtime = wdsp->ktime_zero;
		}
		break;

	case WDSP_STAT_DOWN:
		if (ktime_compare(wdsp->uptime, wdsp->ktime_zero)) {
			wdsp->total_uptime +=
				ktime_ms_delta(current_time, wdsp->uptime);
			wdsp->uptime = wdsp->ktime_zero;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&wdsp->lock);

}

/*
 * codec_state enumerations
 */
enum {
	IAXXX_CODEC_STATE_UNKNOWN = -99,
	IAXXX_CODEC_STATE_ONLINE = 0,
};

/*
 * codec_state show function
 */
static ssize_t iaxxx_misc_codec_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_misc_priv_type *misc_priv = dev_get_drvdata(dev);
	struct iaxxx_priv *priv = misc_priv ? misc_priv->priv : NULL;
	int codec_state = 0;

	if (priv && test_bit(IAXXX_FLG_FW_READY, &priv->flags))
		codec_state = IAXXX_CODEC_STATE_ONLINE;
	else
		codec_state = IAXXX_CODEC_STATE_UNKNOWN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", codec_state);
}
static DEVICE_ATTR(codec_state, 0440, iaxxx_misc_codec_state_show, NULL);

/*
 * iaxxx misc notify callback
 * booted : IAXXX_EV_APP_MODE -> IAXXX_EV_STARTUP
 * crashed : IAXXX_EV_CRASH -> IAXXX_EV_APP_MODE -> IAXXX_EV_RECOVERY
 * refer to EV & FLG enum in iaxxx_core.h
 */
static int iaxxx_misc_notify(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct iaxxx_misc_priv_type *misc_priv =
		container_of(nb, struct iaxxx_misc_priv_type, nb_core);

	dev_info(misc_priv->dev, "action %d\n", action);
	switch (action) {
	/* defined in iaxxx-core.h */
	case IAXXX_EV_CRASH:
		iaxxx_misc_update_wdsp_stat(misc_priv, WDSP_STAT_CRASH);
		break;
	case IAXXX_EV_APP_MODE:
		break;
	case IAXXX_EV_STARTUP:
		iaxxx_misc_update_wdsp_stat(misc_priv, WDSP_STAT_DOWN);
		iaxxx_misc_update_wdsp_stat(misc_priv, WDSP_STAT_UP);
		iaxxx_misc_update_hwinfo(misc_priv, action);
		break;
	case IAXXX_EV_RECOVERY:
		iaxxx_misc_update_wdsp_stat(misc_priv, WDSP_STAT_UP);
		break;
	default:
		break;
	}
	return 0;
}

int iaxxx_misc_init(struct iaxxx_priv *priv)
{
	struct iaxxx_misc_priv_type *misc_priv = NULL;
	int ret;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private\n");
		return -EINVAL;
	}

	misc_priv = devm_kzalloc(priv->dev, sizeof(*misc_priv), GFP_KERNEL);
	if (!misc_priv)
		return -ENOMEM;

	priv->misc_priv = misc_priv;
	misc_priv->priv = priv;

	ret = iaxxx_cdev_create(&misc_priv->misc_cdev, priv->dev,
		NULL, misc_priv, IAXXX_CDEV_MISC);
	if (ret) {
		pr_err("%s() error in creating the iaxxx_misc\n",
				__func__);
		ret = -EIO;
		goto misc_cdev_err;
	}

	misc_priv->dev = misc_priv->misc_cdev.dev;

	ret = device_create_file(misc_priv->dev, &dev_attr_codec_state);
	if (ret) {
		dev_err(misc_priv->dev, "%s: failed to create codec_state\n",
			__func__);
		goto err_iaxxx_misc_file_codec_failed;
	}

	ret = device_create_file(misc_priv->dev, &dev_attr_wdsp_stat);
	if (ret) {
		dev_err(misc_priv->dev, "%s: failed to create wdsp_stat\n",
			__func__);
		goto err_iaxxx_misc_file_wdsp_failed;
	}

	misc_priv->wdsp_stat_priv.ktime_zero = ktime_set(0, 0);
	mutex_init(&misc_priv->wdsp_stat_priv.lock);

	misc_priv->nb_core.notifier_call = iaxxx_misc_notify;
	ret = iaxxx_fw_notifier_register(misc_priv->priv->dev,
		&misc_priv->nb_core);
	if (ret) {
		dev_err(misc_priv->dev, "%s: failed to register notifier\n",
			__func__);
		goto err_iaxxx_misc_notifier_failed;
	}

	ret = sysfs_create_group(&misc_priv->dev->kobj,
		&iaxxx_misc_hwinfo_attr_group);
	if (ret) {
		dev_err(misc_priv->dev,
			"%s sysfs_create_group for hwinfo failed\n", __func__);
		goto err_iaxxx_misc_hwinfo_failed;
	}

	return ret;

err_iaxxx_misc_hwinfo_failed:
err_iaxxx_misc_notifier_failed:
err_iaxxx_misc_file_wdsp_failed:
	device_remove_file(misc_priv->dev, &dev_attr_wdsp_stat);
err_iaxxx_misc_file_codec_failed:
	device_remove_file(misc_priv->dev, &dev_attr_codec_state);
misc_cdev_err:
	devm_kfree(misc_priv->dev, misc_priv);
	iaxxx_cdev_destroy(&misc_priv->misc_cdev);
	devm_kfree(priv->dev, misc_priv);
	priv->misc_priv = NULL;
	return ret;

}
EXPORT_SYMBOL(iaxxx_misc_init);

void iaxxx_misc_exit(struct iaxxx_priv *priv)
{
	struct iaxxx_misc_priv_type *misc_priv = NULL;

	if (priv == NULL) {
		pr_err("Invalid iaxxx private data pointer\n");
		return;
	}

	misc_priv = (struct iaxxx_misc_priv_type *) priv->misc_priv;

	sysfs_remove_group(&misc_priv->dev->kobj,
		&iaxxx_misc_hwinfo_attr_group);

	iaxxx_cdev_destroy(&misc_priv->misc_cdev);
	devm_kfree(priv->dev, misc_priv);
	priv->misc_priv = NULL;
}
EXPORT_SYMBOL(iaxxx_misc_exit);
