/*
 * iaxxx-sysfs.c -- IAxxx Sysfs attributes
 *
 * Copyright 2018 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-af.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include "iaxxx.h"
#include "iaxxx-build-info.h"

#define IAXXX_MAX_PLUGIN		(IAXXX_PLGIN_ID_MASK+1)
#define IAXXX_MAX_PACKAGE		(IAXXX_PKG_ID_MASK+1)
#define IAXXX_FW_DOWNLOAD_TIMEOUT	10000		/* 10 secs */
#define IAXXX_VER_STR_SIZE		60

/**
 * Sysfs save isr_disable option
 */
static ssize_t iaxxx_isr_disable_save(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	if (val) {
		dev_info(dev, "%s() ISR Disabled\n", __func__);
		priv->debug_isr_disable  = true;
	} else {
		priv->debug_isr_disable = false;
		dev_info(dev, "%s() ISR Enabled\n", __func__);
	}
	mutex_unlock(&priv->test_mutex);
	return count;
}

/**
 * sysfs show isr disable option
 */
static ssize_t iaxxx_isr_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
		priv->debug_isr_disable?"ISR Event Handling Disabled\n" :
		"ISR Event Handling Enabled\n");
}

static DEVICE_ATTR(isr_disable, 0600,
		iaxxx_isr_disable_show, iaxxx_isr_disable_save);

/**
 * iaxxx_firmware_version_show - sys node show function for firmware version
 */
static ssize_t iaxxx_firmware_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	rc = iaxxx_get_firmware_version(dev, verbuf, len);
	if (rc) {
		dev_err(dev, "%s() FW version read fail\n", __func__);
		return -EIO;
	}
	return scnprintf(buf, PAGE_SIZE, "%s\n", verbuf);
}
static DEVICE_ATTR(fw_version, 0400, iaxxx_firmware_version_show, NULL);


/**
 * iaxxx_host_version_show - sys node show function hsw veriso
 * and fw version built with
 */
static ssize_t iaxxx_host_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"HSW v%s built with Firmware v%s\n",
			HOST_SOFTWARE_VERSION_STR, FW_VERSION_IN_HOST_STR);
}
static DEVICE_ATTR(host_version, 0400, iaxxx_host_version_show, NULL);

/*
 * Sysfs - firmware update
 */
static ssize_t iaxxx_fw_update(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	/* Kick work queue for firmware loading */
	iaxxx_work(priv, fw_update_work);
	return count;
}
static DEVICE_ATTR(fw_update, 0200, NULL, iaxxx_fw_update);

/**
 * iaxxx_plugin_version_show - sys node show function for plugin version
 */
static ssize_t iaxxx_plugin_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	uint32_t buf_len = 0;
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
							plugin_node);
		rc = iaxxx_core_plg_get_plugin_version(dev,
					plugin_data->inst_id, verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Plugin version read fail\n",
								__func__);
			buf_len = 0;
			goto exit;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE,
			"plugin-%d:proc-id-%d\t%s\n", plugin_data->inst_id,
						plugin_data->proc_id, verbuf);
	}

exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return buf_len;
}
static DEVICE_ATTR(plugin_version, 0400, iaxxx_plugin_version_show, NULL);

/**
 * iaxxx_package_version_show - sys node show function for package version
 */
static ssize_t iaxxx_package_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	int32_t len = IAXXX_VER_STR_SIZE;
	char verbuf[IAXXX_VER_STR_SIZE];
	uint32_t buf_len = 0;
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	if (!priv) {
		dev_err(dev, "%s() Device's priv data is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
								plugin_node);
		rc = iaxxx_core_plg_get_package_version(dev,
					plugin_data->inst_id, verbuf, len);
		if (rc) {
			dev_err(dev, "%s() Package version read fail\n",
								__func__);
			buf_len = 0;
			goto exit;
		}
		buf_len += scnprintf(buf + buf_len, PAGE_SIZE,
			"package-%d:\t%s\n", plugin_data->inst_id, verbuf);
	}
exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return buf_len;
}
static DEVICE_ATTR(package_version, 0400, iaxxx_package_version_show, NULL);

/**
 * iaxxx_firmware_timestamp_show - sys node show function for firmware timestamp
 */
static ssize_t iaxxx_firmware_timestamp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t fw_clk_rd[2];
	uint64_t timestamp;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "Invalid device\n");

	/* Read Firmware wall clock timestamp */
	rc = regmap_bulk_read(priv->regmap,
			IAXXX_AF_WCPT_WALL_CLOCK_RD_0_ADDR,
			fw_clk_rd, ARRAY_SIZE(fw_clk_rd));

	if (rc) {
		dev_err(dev,
			"Failed IAXXX_AF_WCPT_WALL_CLOCK_RD, rc:%d\n", rc);
		return rc;
	}
	timestamp = (((long)((fw_clk_rd[1] & 0xFFFF)) << 32) | fw_clk_rd[0]);

	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", timestamp);
}
static DEVICE_ATTR(fw_timestamp, 0400, iaxxx_firmware_timestamp_show, NULL);

/*
 * iaxxx_set_spi_speed
 */
static ssize_t iaxxx_set_spi_speed(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val > priv->spi_app_speed)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);

	if (priv->spi_speed_setup)
		priv->spi_speed_setup(dev, val);
	priv->spi_app_speed = val;

	mutex_unlock(&priv->test_mutex);

	dev_info(dev, "%s() Success\n", __func__);
	return count;
}
static DEVICE_ATTR(set_spi_speed, 0200, NULL, iaxxx_set_spi_speed);

static ssize_t iaxxx_max_spi_speed_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int rc;
	uint32_t max_spi_speed = 0;

	if (!priv)
		return scnprintf(buf, PAGE_SIZE, "ERROR, invalid device\n");

	rc = iaxxx_get_max_spi_speed(dev, &max_spi_speed);
	if (rc) {
		dev_err(dev, "Failed to get max spi speed, ret=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%u\n", max_spi_speed);
}
static DEVICE_ATTR(max_spi_speed, 0400, iaxxx_max_spi_speed_show, NULL);

/*
 * iaxxx_pm_wakeup_chip
 */
static ssize_t iaxxx_sysfs_pm_wakeup_chip(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, reg_val, rc;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;
	/* SPI transcation should wake up the chip
	 * reading SYS_STATUS reg
	 */
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);
	msleep(50);
	rc = regmap_read(priv->regmap, IAXXX_SRB_SYS_STATUS_ADDR, &reg_val);

	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return count;
}
static DEVICE_ATTR(pm_wakeup_chip, 0200, NULL, iaxxx_sysfs_pm_wakeup_chip);

/*
 * iaxxx_pm_set_aclk, input value as follows.
 * 0 - 3.072 MHz
 * 1 - 6.144 MHz
 * 2 - 12.288 MHz
 * 3 - 24.576 MHz
 * 4 - 49.152 MHz
 * 5 - 98.304 MHz
 * 6 - 368.640 MHz
 * otherwise - Invalid
 */
static ssize_t iaxxx_sysfs_pm_set_aclk(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc = -1;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val > 6)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	rc = iaxxx_pm_set_aclk(dev, val);

	mutex_unlock(&priv->test_mutex);

	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return count;
}
static DEVICE_ATTR(pm_set_aclk, 0200, NULL, iaxxx_sysfs_pm_set_aclk);

/*
 * iaxxx_pm_set_optimal_power_mode (host0)
 * Need to do wake up to come out
 */
static ssize_t iaxxx_sysfs_pm_set_optimal_power_mode_host0(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc;
	uint32_t status;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = iaxxx_pm_set_optimal_power_mode_host0(dev);

	if (rc) {
		dev_info(dev, "%s() Fail\n", __func__);
		return count;
	}

	rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_0, priv->regmap,
			20,
			UPDATE_BLOCK_FIXED_WAIT_OPTION,
			&status);
	dev_info(dev, "%s() Success\n", __func__);
	mutex_unlock(&priv->test_mutex);
	return count;
}
static DEVICE_ATTR(pm_set_optimal_power_mode_host0, 0200, NULL,
		iaxxx_sysfs_pm_set_optimal_power_mode_host0);

/*
 * iaxxx_pm_set_optimal_power_mode (host1)
 * Need to do wake up to come out
 */
static ssize_t iaxxx_sysfs_pm_set_optimal_power_mode_host1(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc;
	uint32_t status;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	/* Disable both the control interfaces and the chip will go to
	 * optimal power mode
	 */
	rc = iaxxx_pm_set_optimal_power_mode_host1(dev, false);

	if (rc) {
		dev_info(dev, "%s() Fail\n", __func__);
		return count;
	}

	rc = iaxxx_send_update_block_request_with_options(
			priv->dev, IAXXX_BLOCK_0,
			IAXXX_HOST_1, priv->regmap,
			20,
			UPDATE_BLOCK_FIXED_WAIT_OPTION,
			&status);

	dev_info(dev, "%s() Success\n", __func__);
	mutex_unlock(&priv->test_mutex);
	return count;
}
static DEVICE_ATTR(pm_set_optimal_power_mode_host1, 0200, NULL,
		iaxxx_sysfs_pm_set_optimal_power_mode_host1);

/* sysfs chip_reset function
 */
static ssize_t iaxxx_chip_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;
	iaxxx_reset_to_sbl(priv);
	return count;
}
static DEVICE_ATTR(chip_reset, 0200, NULL, iaxxx_chip_reset_store);

/* sysfs debug simulate fwcrash
 */
static ssize_t iaxxx_simulate_fwcrash(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;
	iaxxx_fw_crash(dev, IAXXX_FW_CRASH_SIMULATED, IAXXX_NO_PROC);
	return count;
}
static DEVICE_ATTR(simulate_fwcrash, 0200, NULL,
		iaxxx_simulate_fwcrash);

/* sysfs disable runtime pm
 */
static ssize_t iaxxx_runtime_pm_disable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		/* Wake up and disable runtime pm */
		if (!priv->debug_runtime_pm_disable) {

			/* wake up chip */
			iaxxx_wakeup_chip(priv);
			pm_runtime_forbid(dev);
			dev_info(dev, "%s() Runtime PM Disabled\n", __func__);
			priv->debug_runtime_pm_disable = true;
		} else
			dev_info(dev, "%s() Runtime PM Already Disabled\n",
					__func__);

	} else {
		if (priv->debug_runtime_pm_disable) {
			pm_runtime_allow(dev);
			dev_info(dev, "%s() Runtime PM Enabled\n", __func__);
			priv->debug_runtime_pm_disable = false;
		} else
			dev_info(dev, "%s() Runtime PM Already Enabled\n",
					__func__);
	}

	return count;
}
static DEVICE_ATTR(runtime_pm_disable, 0200, NULL,
		iaxxx_runtime_pm_disable);

/* sysfs disable fw crash handling
 */
static ssize_t iaxxx_fwcrash_handling_disable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val != 0 && val != 1)
		return -EINVAL;

	if (val) {
		priv->debug_fwcrash_handling_disable = true;
		dev_info(dev, "%s() FW Crash Handling Disabled\n", __func__);

	} else {
		priv->debug_fwcrash_handling_disable = false;
		dev_info(dev, "%s() FW Crash Handling Enabled\n", __func__);
	}

	return count;
}
static DEVICE_ATTR(fwcrash_handling_disable, 0200, NULL,
		iaxxx_fwcrash_handling_disable);

static ssize_t iaxxx_sysfs_set_osc_trim_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iaxxx_priv *priv = dev ? to_iaxxx_priv(dev) : NULL;
	int val, rc = -1;

	if (!buf)
		return -EINVAL;
	if (kstrtoint(buf, 0, &val))
		return -EINVAL;
	if (val < 0)
		return -EINVAL;

	mutex_lock(&priv->test_mutex);
	rc = iaxxx_set_osc_trim_period(priv, val);
	mutex_unlock(&priv->test_mutex);

	if (rc)
		dev_info(dev, "%s() Fail\n", __func__);
	else
		dev_info(dev, "%s() Success\n", __func__);
	return count;
}
static DEVICE_ATTR(osc_trim_period, 0200, NULL,
		iaxxx_sysfs_set_osc_trim_period);

/*
 * sysfs attr info
 */
static struct attribute *iaxxx_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_host_version.attr,
	&dev_attr_fw_timestamp.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_plugin_version.attr,
	&dev_attr_package_version.attr,
	&dev_attr_pm_wakeup_chip.attr,
	&dev_attr_pm_set_aclk.attr,
	&dev_attr_pm_set_optimal_power_mode_host0.attr,
	&dev_attr_pm_set_optimal_power_mode_host1.attr,
	&dev_attr_set_spi_speed.attr,
	&dev_attr_max_spi_speed.attr,
	&dev_attr_chip_reset.attr,
	&dev_attr_isr_disable.attr,
	&dev_attr_simulate_fwcrash.attr,
	&dev_attr_runtime_pm_disable.attr,
	&dev_attr_fwcrash_handling_disable.attr,
	&dev_attr_osc_trim_period.attr,
	NULL,
};

/*
 * sysfs attr group info
 */
static const struct attribute_group iaxxx_attr_group = {
	.attrs = iaxxx_attrs,
	.name	= "iaxxx"
};

int iaxxx_init_sysfs(struct iaxxx_priv *priv)
{
	int ret;
	/* Create sysfs */
	ret = sysfs_create_group(&priv->dev->kobj, &iaxxx_attr_group);
	if (ret) {
		dev_err(priv->dev,
			"%s [ERROR] sysfs_create_group\n", __func__);
	}
	return ret;
}

void iaxxx_remove_sysfs(struct iaxxx_priv *priv)
{
	sysfs_remove_group(&priv->dev->kobj, &iaxxx_attr_group);
}
