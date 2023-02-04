/*
 * Authors:
 * Shaun Kelsey <shaunkelsey@google.com>
 *
 * Airbrush State Manager Sysfs Entries
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/airbrush-sm-ctrl.h>
#include <linux/stat.h>
#include <linux/sysfs.h>

#include "airbrush-regs.h"

static ssize_t mapped_chip_state_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	u32 val;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	val = ab_sm_get_state((struct ab_state_context *)sc);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t mapped_chip_state_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret;
	int val;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	if (kstrtoint(buf, 10, &val) < 0) {
		dev_err(sc->dev, "Bad input format\n");
		return -EIO;
	}

	ret = ab_sm_set_state(sc, val);
	if (ret < 0) {
		dev_err(sc->dev, "%s: State change failed, ret %d\n",
				__func__, ret);
		return ret;
	}

	return count;
}

static ssize_t version_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	enum ab_chip_id chip_id;

	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	chip_id = ab_get_chip_id(sc);

	switch (chip_id) {
	case CHIP_ID_A0:
		return scnprintf(buf, PAGE_SIZE, "A0\n");
	case CHIP_ID_B0:
		return scnprintf(buf, PAGE_SIZE, "B0\n");
	default:
		return scnprintf(buf, PAGE_SIZE, "Unknown\n");
	}

	return 0;
}

static ssize_t state_stats_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	/* keep contents of states array in sync with stat_state enum */
	const char * const states[STAT_STATE_SIZE] = {"ACTIVE", "SLEEP",
		"DEEP SLEEP", "SUSPEND", "OFF", "UNKNOWN"};
	u32 pos = 0;
	u32 i;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	pos += scnprintf(buf + pos, PAGE_SIZE - pos,
			"Pixel Visual Core Subsystem Power Stats\n");

	mutex_lock(&sc->state_transitioning_lock);

	for (i = 0; i < STAT_STATE_SIZE; i++) {
		ktime_t adjusted_duration = sc->state_stats[i].duration;
		ktime_t last_entry = sc->state_stats[i].last_entry;
		ktime_t last_exit = sc->state_stats[i].last_exit;

		/* adjust duration for current state */
		if (ab_chip_state_to_stat_state(sc->curr_chip_substate_id) == i
				&& ktime_after(last_entry, last_exit)) {
			ktime_t partial_duration = ktime_sub(
					ktime_get_boottime(), last_entry);
			adjusted_duration = ktime_add(adjusted_duration,
					partial_duration);
		}

		pos += scnprintf(buf + pos, PAGE_SIZE - pos, "%s\n"
				"\tCumulative count: %llu\n"
				"\tCumulative duration msec:  %llu\n"
				"\tLast entry timestamp msec: %llu\n"
				"\tLast exit timestamp msec:  %llu\n",
				states[i], sc->state_stats[i].counter,
				ktime_to_ms(adjusted_duration),
				ktime_to_ms(last_entry),
				ktime_to_ms(last_exit));
	}

	mutex_unlock(&sc->state_transitioning_lock);

	return pos;
}

/* Trigger an error event for testing purpose; input is ignored */
static ssize_t gen_error_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	dev_warn(dev, "Triggering an error event for testing.\n");
	sysfs_notify(&dev->kobj, NULL, "error_event");
	return count;
}

/* Supposed to be polled by userspace daemon. */
static ssize_t error_event_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Return fixed string "1" */
	return scnprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static ssize_t clk_expect_frequency_show(struct device *dev,
		char *buf, enum block_name blk_id)
{
	u64 clk_frequency_expt;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);

	mutex_lock(&sc->state_transitioning_lock);

	clk_frequency_expt = sc->blocks[blk_id].current_state->clk_frequency;

	mutex_unlock(&sc->state_transitioning_lock);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", clk_frequency_expt);
}

static ssize_t ipu_clk_expect_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_expect_frequency_show(dev, buf, BLK_IPU);
}

static ssize_t tpu_clk_expect_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_expect_frequency_show(dev, buf, BLK_TPU);
}

static ssize_t aon_clk_expect_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_expect_frequency_show(dev, buf, BLK_AON);
}

static ssize_t mif_clk_expect_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_expect_frequency_show(dev, buf, BLK_MIF);
}

static ssize_t clk_meas_frequency_show(struct device *dev,
		char *buf, enum block_name blk_id)
{
	unsigned int clkout_idx, clkout_blk_idx, clkout_clk_idx;
	u64 val;
	int retval;
	struct ab_state_context *sc =
		(struct ab_state_context *)dev_get_drvdata(dev);
	clkout_idx = 0;

	switch (blk_id) {
	case BLK_IPU:
		clkout_blk_idx = 1;
		clkout_clk_idx = 2;
		break;
	case BLK_TPU:
		clkout_blk_idx = 2;
		clkout_clk_idx = 2;
		break;
	case BLK_AON:
		clkout_blk_idx = 0;
		clkout_clk_idx = 4;
		break;
	case BLK_MIF:
		clkout_blk_idx = 4;
		clkout_clk_idx = 11;
		break;
	default:
		return scnprintf(buf, PAGE_SIZE, "UnKnown\n");
	}

	ab_clkout_sel(sc, clkout_idx);
	ab_clkout_blksel(sc, clkout_blk_idx);
	ab_clkout_clksel(sc, clkout_clk_idx);
	retval = ab_clkout_freq(sc, &val);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t ipu_clk_meas_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_meas_frequency_show(dev, buf, BLK_IPU);
}

static ssize_t tpu_clk_meas_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_meas_frequency_show(dev, buf, BLK_TPU);
}

static ssize_t aon_clk_meas_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_meas_frequency_show(dev, buf, BLK_AON);
}

static ssize_t mif_clk_meas_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return clk_meas_frequency_show(dev, buf, BLK_MIF);
}

static struct device_attribute ab_sm_attrs[] = {
	__ATTR_RW(mapped_chip_state),
	__ATTR_RO(version),
	__ATTR_RO(state_stats),
	__ATTR_RO(error_event),
	__ATTR_WO(gen_error),
	__ATTR_RO(ipu_clk_expect_frequency),
	__ATTR_RO(tpu_clk_expect_frequency),
	__ATTR_RO(aon_clk_expect_frequency),
	__ATTR_RO(mif_clk_expect_frequency),
	__ATTR_RO(ipu_clk_meas_frequency),
	__ATTR_RO(tpu_clk_meas_frequency),
	__ATTR_RO(aon_clk_meas_frequency),
	__ATTR_RO(mif_clk_meas_frequency)
};

void ab_sm_create_sysfs(struct ab_state_context *sc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ab_sm_attrs); i++)
		device_create_file(sc->dev, &ab_sm_attrs[i]);
}

void ab_sm_remove_sysfs(struct ab_state_context *sc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ab_sm_attrs); i++)
		device_remove_file(sc->dev, &ab_sm_attrs[i]);
}
