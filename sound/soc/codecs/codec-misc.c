// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019-, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/codec-misc.h>

#define DRIVER_NAME "codec_detect"

static struct platform_device *codec_pdev;

/*
 * Amplifier status
 */
struct amp_stat_priv_type {
	uint32_t action;
	struct mutex lock;
	struct device *ampdev_l;
	struct device *ampdev_r;
	long imp_l;
	long imp_r;
	long thermal_l;
	long thermal_r;
	int active;
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
	u64 recover_count; /* re-use this field to fill FW ver. & chip rev.*/
	uint32_t action;
	struct mutex lock;
};

/*
 * codec Misc data
 */
struct codec_misc_priv_type {
	struct wdsp_stat_priv_type wdsp_stat_priv;
	struct notifier_block nb_core;	/* codec Core notifier */
	struct srcu_notifier_head core_notifier_list;
	state_cb st_cb;
	state_cb hs_cb;
	number_cb nb_cb;
	struct amp_stat_priv_type amp_stat_priv;
};

/*
 * wdsp_stat show function
 */
static ssize_t wdsp_stat_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct codec_misc_priv_type *misc_priv = dev_get_drvdata(dev);
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
static DEVICE_ATTR_RO(wdsp_stat);

static int codec_misc_notify(struct notifier_block *nb,
			unsigned long action, void *data)
{
	struct codec_misc_priv_type *misc_priv =
		container_of(nb, struct codec_misc_priv_type, nb_core);
	struct wdsp_stat_priv_type *wdsp = (misc_priv ?
		&misc_priv->wdsp_stat_priv : NULL);

	ktime_t current_time;

	if (!wdsp) {
		pr_warn("%s invalid device", __func__);
		return 0;
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
	return 0;
}

bool codec_detect_available(void)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv)
			return true;
	}

	return false;
}
EXPORT_SYMBOL(codec_detect_available);

static ssize_t hwinfo_part_number_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv && priv->nb_cb)
			return scnprintf(buf, PAGE_SIZE, "%s", priv->nb_cb());
	}

	return scnprintf(buf, PAGE_SIZE, "Unknown");
}
static DEVICE_ATTR_RO(hwinfo_part_number);

void codec_detect_number_callback(number_cb cb)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv)
			priv->nb_cb = cb;
	}
}
EXPORT_SYMBOL(codec_detect_number_callback);

static ssize_t codec_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv && priv->st_cb)
			return scnprintf(buf, PAGE_SIZE, "%d", priv->st_cb());
	}

	return scnprintf(buf, PAGE_SIZE, "%d", CODEC_STATE_UNKNOWN);
}
static DEVICE_ATTR_RO(codec_state);

void codec_detect_state_callback(state_cb cb)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv)
			priv->st_cb = cb;
	}
}
EXPORT_SYMBOL(codec_detect_state_callback);

static ssize_t headset_codec_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv && priv->hs_cb)
			return scnprintf(buf, PAGE_SIZE, "%d", priv->hs_cb());
	}

	return scnprintf(buf, PAGE_SIZE, "%d", CODEC_STATE_UNKNOWN);
}
static DEVICE_ATTR_RO(headset_codec_state);

void codec_detect_hs_state_callback(state_cb cb)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv)
			priv->hs_cb = cb;
	}
}
EXPORT_SYMBOL(codec_detect_hs_state_callback);

int codec_detect_status_notifier(unsigned long val)
{
	struct codec_misc_priv_type *priv = NULL;

	if (!IS_ERR_OR_NULL(codec_pdev) && !IS_ERR_OR_NULL(&codec_pdev->dev)) {
		priv = dev_get_drvdata(&codec_pdev->dev);
		if (priv)
			return srcu_notifier_call_chain(
					&priv->core_notifier_list, val, NULL);
	}

	return -EINVAL;
}
EXPORT_SYMBOL(codec_detect_status_notifier);

/*
 * update amplify impedance function
 */
int codec_misc_amp_put(int ch, long val)
{
	int ret = 0;
	struct codec_misc_priv_type *misc_priv = NULL;
	struct amp_stat_priv_type *amp_priv = NULL;

	if (IS_ERR(codec_pdev) || IS_ERR(&codec_pdev->dev))
		return -EINVAL;

	misc_priv = dev_get_drvdata(&codec_pdev->dev);

	if (!misc_priv)
		return -EINVAL;

	amp_priv = &misc_priv->amp_stat_priv;

	if (!amp_priv)
		return -EINVAL;

	mutex_lock(&amp_priv->lock);

	if (ch == 0)
		amp_priv->imp_l = val;
	else
		amp_priv->imp_r = val;

	dev_dbg(&codec_pdev->dev, "%s: (%d, %ld) %ld, %ld\n", __func__,
		ch, val, amp_priv->imp_l, amp_priv->imp_r);

	mutex_unlock(&amp_priv->lock);

	return ret;
}
EXPORT_SYMBOL(codec_misc_amp_put);

/*
 * amplify impedance show function
 */
static ssize_t resistance_left_right_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct codec_misc_priv_type *misc_priv = dev_get_drvdata(dev);
	struct amp_stat_priv_type *amp_priv = &misc_priv->amp_stat_priv;
	int ret = 0;

	char resistance[128];
	int scale = 100000;

	mutex_lock(&amp_priv->lock);

	scnprintf(resistance,
		sizeof(resistance), "%d.%d,%d.%d",
		amp_priv->imp_l/scale,
		amp_priv->imp_l%scale,
		amp_priv->imp_r/scale,
		amp_priv->imp_r%scale);

	dev_info(dev, "%s: %s (%ld, %ld, %ld)\n",
		__func__, resistance, amp_priv->imp_l, amp_priv->imp_r, scale);
	ret = scnprintf(buf, PAGE_SIZE, "%s", resistance);

	mutex_unlock(&amp_priv->lock);

	return ret;
}
static DEVICE_ATTR_RO(resistance_left_right);

static int misc_probe(struct platform_device *pdev)
{
	int ret;
	struct codec_misc_priv_type *misc_priv = NULL;

	codec_pdev = pdev;

	misc_priv = devm_kzalloc(&codec_pdev->dev,
		sizeof(*misc_priv), GFP_KERNEL);
	if (!misc_priv)
		return -ENOMEM;

	dev_set_drvdata(&codec_pdev->dev, misc_priv);

	ret = device_create_file(&codec_pdev->dev,
		&dev_attr_codec_state);
	if (ret) {
		dev_err(&codec_pdev->dev, "failed to create codec_state\n");
		goto err_codec_state;
	}

	ret = device_create_file(&codec_pdev->dev,
		&dev_attr_headset_codec_state);
	if (ret) {
		dev_err(&codec_pdev->dev, "failed to create headset_codec_state\n");
		goto err_headset_codec_state;
	}

	ret = device_create_file(&codec_pdev->dev,
		&dev_attr_hwinfo_part_number);
	if (ret) {
		dev_err(&codec_pdev->dev,
			"failed to create codec hw info numbers\n");
		goto err_hwinfo;
	}

	ret = device_create_file(&codec_pdev->dev, &dev_attr_wdsp_stat);
	if (ret) {
		dev_err(&codec_pdev->dev, "failed to create wdsp state\n");
		goto err_wdsp_stat;
	}

	ret = device_create_file(&codec_pdev->dev,
		&dev_attr_resistance_left_right);
	if (ret) {
		dev_err(&codec_pdev->dev,
			"failed to create resistance_left_right\n");
		goto err_resistance_left_right;
	}

	srcu_init_notifier_head(&misc_priv->core_notifier_list);
	misc_priv->nb_core.notifier_call = codec_misc_notify;

	ret = srcu_notifier_chain_register(&misc_priv->core_notifier_list,
		&misc_priv->nb_core);
	if (ret) {
		dev_err(&codec_pdev->dev, "%s: failed to register notifier\n",
			__func__);
		goto err_notifier_reg;
	}

	misc_priv->st_cb = NULL;
	misc_priv->hs_cb = NULL;
	misc_priv->nb_cb = NULL;

	misc_priv->wdsp_stat_priv.ktime_zero = ktime_set(0, 0);
	mutex_init(&misc_priv->wdsp_stat_priv.lock);
	mutex_init(&misc_priv->amp_stat_priv.lock);

	dev_set_drvdata(&codec_pdev->dev, misc_priv);
	dev_info(&codec_pdev->dev, "created codec_detect\n");
	return ret;

err_notifier_reg:
	device_remove_file(&pdev->dev, &dev_attr_resistance_left_right);
err_resistance_left_right:
	device_remove_file(&codec_pdev->dev, &dev_attr_wdsp_stat);
err_wdsp_stat:
	device_remove_file(&codec_pdev->dev, &dev_attr_hwinfo_part_number);
err_hwinfo:
	device_remove_file(&codec_pdev->dev, &dev_attr_headset_codec_state);
err_headset_codec_state:
	device_remove_file(&codec_pdev->dev, &dev_attr_codec_state);
err_codec_state:
	mutex_destroy(&misc_priv->wdsp_stat_priv.lock);
	mutex_destroy(&misc_priv->amp_stat_priv.lock);
	devm_kfree(&pdev->dev, misc_priv);
	return ret;
}

static int misc_remove(struct platform_device *pdev)
{
	struct codec_misc_priv_type *misc_priv =
		dev_get_drvdata(&pdev->dev);

	device_remove_file(&pdev->dev, &dev_attr_wdsp_stat);
	device_remove_file(&pdev->dev, &dev_attr_hwinfo_part_number);
	device_remove_file(&pdev->dev, &dev_attr_headset_codec_state);
	device_remove_file(&pdev->dev, &dev_attr_codec_state);
	device_remove_file(&pdev->dev, &dev_attr_resistance_left_right);
	mutex_destroy(&misc_priv->wdsp_stat_priv.lock);
	mutex_destroy(&misc_priv->amp_stat_priv.lock);
	devm_kfree(&pdev->dev, misc_priv);

	return 0;
}

struct platform_driver misc_driver = {
	.probe = misc_probe,
	.remove = misc_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	}
};

static struct platform_device *codec_detect_dev;

static int codec_detect_init(void)
{
	int ret;

	codec_detect_dev =
		platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);

	if (IS_ERR(codec_detect_dev))
		return PTR_ERR(codec_detect_dev);

	ret = platform_driver_register(&misc_driver);
	if (ret != 0)
		platform_device_unregister(codec_detect_dev);

	return ret;
}

static void codec_detect_exit(void)
{
	platform_driver_unregister(&misc_driver);
	platform_device_unregister(codec_detect_dev);
}

subsys_initcall(codec_detect_init);
module_exit(codec_detect_exit);
