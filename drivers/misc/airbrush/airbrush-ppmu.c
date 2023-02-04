/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Author: Nishant Prajapati <nishant.p@samsung.com>
 *
 * Airbrush PPMU driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/mfd/abc-pcie.h>
#include "airbrush-ppmu.h"

/**
 * ppmu_write:
 * ppmu_read: functions for Reading/Writing register through PCIe channel
 */
void ppmu_write(u32 value, u32 offset)
{
	abc_pcie_config_write(offset, 4, value);
}

u32 ppmu_read(u32 offset)
{
	u32 data;

	abc_pcie_config_read(offset, 4, &data);
	return data;
}

/**
 * reset_counter: reset all the counters
 * @ppmu: airbrush_ppmu device structure
 */
static void reset_counter(struct airbrush_ppmu *ppmu)
{
	u32 base, regvalue, i;

	base = ppmu->base;
	regvalue = ppmu_read(base + PPMU25_PMNC);

	/* Reset CCNT and PMCNT's */
	ppmu_write((regvalue | (1 << PPMU_PMNC_CC_RESET_SHIFT) |
		(1 << PPMU_PMNC_COUNTER_RESET_SHIFT)), base + PPMU25_PMNC);

	for (i = 0; i < MAX_COUNTER - 1; i++)
		ppmu_write(PPMU_RESET_VAL, base + PPMU25_EVENT_EVx_TYPE(i));
}

/**
 * ppmu_reset: reset whole PPMU device
 */
static int ppmu_reset(struct airbrush_ppmu *ppmu)
{
	u32 base, i, regvalue;

	base = ppmu->base;

	/* Number of configured event = 0 */
	ppmu->state->conf_events = 0;
	ppmu->state->bmp = 0;

	for (i = 0; i < MAX_COUNTER ; i++)
		ppmu->state->over_flow[i] = 0;

	reset_counter(ppmu);

	regvalue = ppmu_read(base + PPMU25_PMNC);
	ppmu_write(regvalue & ~0x1 << PPMU_PMNC_ENABLE_SHIFT,
			base + PPMU25_PMNC);

	/* Disable all 8 PMCNTs [7:0] */
	ppmu_write(PPMU_PMCNT_ALL, base + PPMU25_CNTENC);

	/* Enable CCNT Clock event [31:31] */
	ppmu_write(PPMU_CNT_CC, base + PPMU25_CNTENS);

	/* Enable interrupt for CCNT and all PMCNTs */
	ppmu_write(PPMU_CNT_CC | PPMU_PMCNT_ALL, base + PPMU25_INTENS);
	ppmu_write(PPMU_RESET_VAL, base + PPMU25_INTENC);

	/* Clear all events' overflow status Flags at starting */
	ppmu_write(PPMU_CNT_CC | PPMU_PMCNT_ALL, base + PPMU25_FLAG);

	/* reset CIG configurations*/
	ppmu_write(PPMU_RESET_VAL,	base + PPMU25_CIG_CFG0);
	ppmu_write(PPMU_RESET_VAL,	base + PPMU25_CIG_CFG1);
	ppmu_write(PPMU_RESET_VAL,	base + PPMU25_CIG_CFG2);
	ppmu_write(PPMU_CIG_UP_INT | PPMU_CIG_LW_INT, base + PPMU25_CIG_RESULT);
	ppmu_write(PPMU_CNT_CC | PPMU_PMCNT_ALL, base + PPMU25_CNT_RESET);

	return 0;

}

/*
 * print_result(): Temporary function for debugging result
 * @counter: array holding the value from PPMU event counters
 */
void print_result(unsigned long long *counter)
{
	u32 i = 0;

	for (i = 0; i < MAX_COUNTER - 1 ; i++)
		pr_info("PMCNT%d: %lld", i, counter[i]);

	pr_info("CCNT: %lld\n", counter[MAX_COUNTER - 1]);
}

static int allocate_event(unsigned int event,
					u32 *set, u32 len,
					struct airbrush_ppmu *ppmu)
{
	struct device *dev = &ppmu->dev;
	u32 base = ppmu->base;
	u32 i, regval;

	for (i = 0; i < len; i++) {
		if (!(ppmu->state->bmp & 1 << set[i])) {
			ppmu->state->bmp |= 1 << set[i];
			ppmu->state->conf_events++;
			ppmu_write(event, base +
				   PPMU25_EVENT_EVx_TYPE(set[i]));
			regval = ppmu_read(base + PPMU25_CNTENS);
			ppmu_write(regval | 1 << set[i],
				   base + PPMU25_CNTENS);
			dev_info(dev, "Event: %d -> will capture in PMCNT: %d\n",
			       event, set[i]);
			return 0;
		}
	}
	dev_err(dev, "Can't allocate counter\n");
	return -ENOSPC;
}

/**
 * ppmu_config: allocate event to counter.
 * @event: number corresponding to type of event
 */
int ppmu_config(struct airbrush_ppmu *ppmu, unsigned int event)
{
	struct device *dev = &ppmu->dev;
	u32 first_set[] = {PPMU_PMNCNT0, PPMU_PMNCNT1, PPMU_PMNCNT4,
			   PPMU_PMNCNT5};
	u32 second_set[] = {PPMU_PMNCNT2, PPMU_PMNCNT3, PPMU_PMNCNT6,
				PPMU_PMNCNT7};

	if (ppmu->state->conf_events > (PPMU_PMNCNT_MAX - 1)) {
		dev_err(dev, "CANT ALLOCATE: Limit reached\n");
		return -ENOSPC;
	}
	if (event >= PPMU_EVENT_RW_BUSY && event <= PPMU_EVENT_WRITE_LATENCY)
		return allocate_event(event, second_set,
				ARRAY_SIZE(second_set), ppmu);

	if (allocate_event(event, first_set, ARRAY_SIZE(first_set), ppmu) < 0)
		return allocate_event(event, second_set,
				ARRAY_SIZE(second_set), ppmu);

	return 0;
}

int ppmu_start(struct airbrush_ppmu *ppmu)
{
	u32 base, regvalue;

	base = ppmu->base;
	regvalue = ppmu_read(base + PPMU25_PMNC);
	regvalue = regvalue | 1 << PPMU_PMNC_ENABLE_SHIFT;
	ppmu_write(regvalue, base + PPMU25_PMNC);
	return 0;
}

int ppmu_stop(struct airbrush_ppmu *ppmu)
{
	u32 base, regvalue;

	base = ppmu->base;
	regvalue = ppmu_read(base + PPMU25_PMNC);
	ppmu_write(regvalue & ~0x1 << PPMU_PMNC_ENABLE_SHIFT,
			base + PPMU25_PMNC);
	return 0;
}

/**
 * ppmu_get_result(): function to fetch results from all counters
 * @counter: array to hold the counter values
 */
int ppmu_get_result(struct airbrush_ppmu *ppmu, unsigned long long *counter)
{
	u32 i;
	u32 base = ppmu->base;

	for (i = 0; i < MAX_COUNTER; i++)
		counter[i] = (unsigned long long)
			ppmu->state->over_flow[i] << 32;

	counter[0] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT0);
	counter[1] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT1);
	counter[2] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT2);
	counter[3] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT3) |
		(unsigned long long)(ppmu_read(
			base + PPMU25_PMCNT3_HIGH) & 0xFF) << 32;
	counter[4] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT4);
	counter[5] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT5);
	counter[6] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT6);
	counter[7] |= (unsigned long long)ppmu_read(base + PPMU25_PMCNT7) |
		(unsigned long long)(ppmu_read(
			base + PPMU25_PMCNT7_HIGH) & 0xFF) << 32;
	counter[8] |= (unsigned long long)ppmu_read(base + PPMU25_CCNT);

	return 0;
}

static const struct of_device_id airbrush_ppmu_id_match[] = {
	{
		.compatible = "abc,airbrush-ppmu-v2",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, airbrush_ppmu_id_match);

int ppmu_irq_handler(unsigned int irq, struct airbrush_ppmu *ppmu)
{
	u32 regvalue, i;
	u32 base = ppmu->base;
	struct airbrush_ppmu_state *ppmu_state = ppmu->state;

	regvalue = ppmu_read(base + PPMU25_FLAG);
	pr_info("flag: %x\n", regvalue);
	ppmu_write(regvalue, base + PPMU25_FLAG);

	/* This is to handle interrupt due to CCNT overflow.*/
	if (regvalue & PPMU_CNT_CC)
		ppmu_state->over_flow[8] += 1;

	for (i = 0; i < MAX_COUNTER - 1; i++) {
		if (regvalue & PPMU_PMCNT(i))
			ppmu_state->over_flow[i] += 1;
	}
	return 0;
}

static int airbrush_ppmu_parse_dt(struct platform_device *pdev,
	struct airbrush_ppmu *info)
{
	u32 i = 0, irq;
	u32 reg[2];
	int err;
	struct device *dev = &info->dev;
	struct device_node *np = dev->of_node;
	struct airbrush_ppmu_state *state =
		devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);

	if (!np) {
		dev_err(dev, "failed to find devicetree node\n");
		return -EINVAL;
	}

	err = of_property_read_u32_array(np, "reg", reg, 2);
	if (err)
		dev_err(dev, "error in getting base address (%d)\n", err);

	/*for getting offset for PCIe r/w function calls */
	info->base = reg[1] & 0xFFFFFF;

	err = of_property_read_u32(np, "interrupt", &irq);
	if (err)
		dev_err(dev, "error in getting interrupt number (%d)\n", err);

	info->irq = irq;

	/* reset state*/
	state->conf_events = 0;
	state->bmp = 0;
	for (i = 0; i < MAX_COUNTER ; i++)
		state->over_flow[i] = 0;

	info->state = state;
	dev_set_drvdata(&pdev->dev, info);

	return 0;
}

/*Create Sysfs attributes*/
static ssize_t test_read_store(struct device *child,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	u32 base = ppmu->base;

	/* transaction on PCIe Master */
	ppmu_read(base + PPMU25_VER);
	pr_info("1 read");

	return count;
}

static ssize_t test_write_store(struct device *child,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	u32 base = ppmu->base;
	u32 val = 0x25000;

	ppmu_write(val, base + PPMU25_VER);
	pr_info("1 write");

	return count;
}

static ssize_t set_cnt_size_store(struct device *child,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		pr_err("Error in parsing counter size\n");

	pr_info("val: %ld\n", val);
	ppmu->ppmu_data.event_count = val;
	ppmu_write(0xffffffff-(val-1), ppmu->base + PPMU25_PMCNT2);
	return count;
}

static ssize_t set_clk_freq_store(struct device *child,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		pr_err("Error in parsing frequency\n");

	pr_info("clk_freq %ld\n", val);
	ppmu->ppmu_data.clk_freq = val;
	return count;
}

static int ppmu_irq_notify(struct notifier_block *nb,
					unsigned long irq, void *data)
{
	struct airbrush_ppmu *dev =
		container_of(nb, struct airbrush_ppmu, nb);
	u32 intnc_val = (u64)data;

	if (irq == ABC_MSI_AON_INTNC &&
			(intnc_val & (1 << (dev->irq - ABC_MSI_COUNT))))
		return ppmu_irq_handler(irq, dev);

	return 0;
}

static ssize_t register_irq_store(struct device *child,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);

	ppmu->nb.notifier_call = ppmu_irq_notify;
	abc_reg_notifier_callback(&ppmu->nb);
	return count;
}

static ssize_t ppmu_reset_store(struct device *child,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);

	dev_info(child, "ppmu reset\n");
	ppmu_reset(ppmu);
	return count;
}

static ssize_t ppmu_config_show(struct device *child,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s",
		"PPMU_EVENT_READ_BUSY			= 0\n"
		"PPMU_EVENT_WRITE_BUSY			= 1,\n"
		"PPMU_EVENT_READ_REQUEST		= 2,\n"
		"PPMU_EVENT_WRITE_REQUEST		= 3,\n"
		"PPMU_EVENT_READ_DATA			= 4,\n"
		"PPMU_EVENT_WRITE_DATA			= 5,\n"
		"PPMU_EVENT_WRITE_RESP			= 6,\n"
		"PPMU_EVENT_READ_LAST			= 7,\n"
		"PPMU_EVENT_WRITE_LAST			= 8,\n"
		"PPMU_EVENT_READ_REQ_BLOCK		= 0x10,\n"
		"PPMU_EVENT_WRITE_REQ_BLOCK		= 0x11,\n"
		"PPMU_EVENT_READ_DATA_BLOCK		= 0x12,\n"
		"PPMU_EVENT_WRITE_DATA_BLOCK		= 0x13,\n"
		"PPMU_EVENT_WRITE_RESP_BLOCK		= 0x14,\n"
		"PPMU_EVENT_EXT_0			= 0x30,\n"
		"PPMU_EVENT_EXT_1			= 0x31,\n"
		"PPMU_EVENT_EXT_2			= 0x32,\n"
		"PPMU_EVENT_RW_BUSY			= 0x20,\n"
		"PPMU_EVENT_RW_REQUEST			= 0x21,\n"
		"PPMU_EVENT_RW_DATA			= 0x22,\n"
		"PPMU_EVENT_RW_REQ_BLOCK		= 0x23,\n"
		"PPMU_EVENT_READ_LATENCY		= 0x24,\n"
		"PPMU_EVENT_WRITE_LATENCY		= 0x25,\n");
}

static ssize_t ppmu_config_store(struct device *child,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{

	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	u32 para = 0;

	if (sysfs_streq(buf, "PPMU_EVENT_READ_BUSY"))
		para = 0;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_BUSY"))
		para = 1;
	else if (sysfs_streq(buf, "PPMU_EVENT_READ_REQUEST"))
		para = 2;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_REQUEST"))
		para = 3;
	else if (sysfs_streq(buf, "PPMU_EVENT_READ_DATA"))
		para = 4;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_DATA"))
		para = 5;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_RESP"))
		para = 6;
	else if (sysfs_streq(buf, "PPMU_EVENT_READ_LAST"))
		para = 7;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_LAST"))
		para = 8;
	else if (sysfs_streq(buf, "PPMU_EVENT_READ_REQ_BLOCK"))
		para = 0x10;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_REQ_BLOCK"))
		para = 0x11;
	else if (sysfs_streq(buf, "PPMU_EVENT_READ_DATA_BLOCK"))
		para = 0x12;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_DATA_BLOCK"))
		para = 0x13;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_RESP_BLOCK"))
		para = 0x14;
	else if (sysfs_streq(buf, "PPMU_EVENT_EXT_0"))
		para = 0x30;
	else if (sysfs_streq(buf, "PPMU_EVENT_EXT_1"))
		para = 0x31;
	else if (sysfs_streq(buf, "PPMU_EVENT_EXT_2"))
		para = 0x32;
	else if (sysfs_streq(buf, "PPMU_EVENT_RW_BUSY"))
		para = 0x20;
	else if (sysfs_streq(buf, "PPMU_EVENT_RW_REQUEST"))
		para = 0x21;
	else if (sysfs_streq(buf, "PPMU_EVENT_RW_DATA"))
		para = 0x22;
	else if (sysfs_streq(buf, "PPMU_EVENT_RW_REQ_BLOCK"))
		para = 0x23;
	else if (sysfs_streq(buf, "PPMU_EVENT_READ_LATENCY"))
		para = 0x24;
	else if (sysfs_streq(buf, "PPMU_EVENT_WRITE_LATENCY"))
		para = 0x25;
	else {
		dev_err(child, "ERROR in event type\n");
		return count;
	}
	ppmu_config(ppmu, para);
	return count;
}

static ssize_t ppmu_start_store(struct device *child,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);

	if (sysfs_streq(buf, "1"))
		ppmu_start(ppmu);

	return count;
}

static ssize_t ppmu_stop_store(struct device *child,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);

	dev_info(child, "PPMU stop= %s\n", buf);
	ppmu_stop(ppmu);

	return count;

}

static ssize_t ppmu_get_result_show(struct device *child,
	struct device_attribute *attr, char *buf)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	unsigned long long counter[MAX_COUNTER];

	ppmu_get_result(ppmu, counter);
	print_result(counter);
	return scnprintf(buf, PAGE_SIZE, "PPMU results:\n");
}

static ssize_t selective_measurement_mask_store(struct device *child,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* Caution: This register should be configured only when there is no
	 * outstanding transaction in the monitored AXI channel.
	 */
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 16, &val);
	if (ret) {
		dev_err(&ppmu->dev, "Error in parsing mask\n");
		return ret;
	}

	dev_info(&ppmu->dev, "mask %lu\n", val);
	ppmu_write(val, ppmu->base + PPMU25_SM_ID_MASK);
	return count;
}

static ssize_t selective_measurement_mask_show(struct device *child,
	struct device_attribute *attr, char *buf)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	u32 val = ppmu_read(ppmu->base + PPMU25_SM_ID_MASK);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t selective_measurement_value_store(struct device *child,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* Caution: This register should be configured only when there is no
	 * outstanding transaction in the monitored AXI channel.
	 */
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 16, &val);
	if (ret) {
		dev_err(&ppmu->dev, "Error in parsing value\n");
		return ret;
	}

	dev_info(&ppmu->dev, "value %lu\n", val);
	ppmu_write(val, ppmu->base + PPMU25_SM_ID_V);
	return count;
}

static ssize_t selective_measurement_value_show(struct device *child,
	struct device_attribute *attr, char *buf)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	u32 val = ppmu_read(ppmu->base + PPMU25_SM_ID_V);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t selective_measurement_allocate_store(struct device *child,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* Caution: This register should be configured only when there is no
	 * outstanding transaction in the monitored AXI channel.
	 */
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 16, &val);
	if (ret) {
		dev_err(&ppmu->dev, "Error in parsing allocation bits\n");
		return ret;
	}

	dev_info(&ppmu->dev, "allocation bits %lu\n", val);
	ppmu_write(val, ppmu->base + PPMU25_SM_ID_A);
	return count;
}

static ssize_t selective_measurement_allocate_show(struct device *child,
	struct device_attribute *attr, char *buf)
{
	struct airbrush_ppmu *ppmu = dev_get_drvdata(child);
	u32 val = ppmu_read(ppmu->base + PPMU25_SM_ID_A);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static DEVICE_ATTR(test_read, 0664, NULL, test_read_store);
static DEVICE_ATTR(test_write, 0664, NULL, test_write_store);
static DEVICE_ATTR(ppmu_reset, 0664, NULL, ppmu_reset_store);
static DEVICE_ATTR(ppmu_config, 0664, ppmu_config_show, ppmu_config_store);
static DEVICE_ATTR(ppmu_start, 0664, NULL, ppmu_start_store);
static DEVICE_ATTR(ppmu_stop, 0664, NULL, ppmu_stop_store);
static DEVICE_ATTR(ppmu_get_result, 0664, ppmu_get_result_show, NULL);
static DEVICE_ATTR(register_irq, 0664, NULL, register_irq_store);
static DEVICE_ATTR(set_cnt_size, 0664, NULL, set_cnt_size_store);
static DEVICE_ATTR(set_clk_freq, 0664, NULL, set_clk_freq_store);
static DEVICE_ATTR(selective_measurement_mask, 0664,
	selective_measurement_mask_show,
	selective_measurement_mask_store);
static DEVICE_ATTR(selective_measurement_value, 0664,
	selective_measurement_value_show,
	selective_measurement_value_store);
static DEVICE_ATTR(selective_measurement_allocate, 0664,
	selective_measurement_allocate_show,
	selective_measurement_allocate_store);

struct attribute *ppmu_attrs[] = {
&dev_attr_test_read.attr,
&dev_attr_test_write.attr,
&dev_attr_ppmu_reset.attr,
&dev_attr_ppmu_config.attr,
&dev_attr_ppmu_start.attr,
&dev_attr_ppmu_stop.attr,
&dev_attr_ppmu_get_result.attr,
&dev_attr_register_irq.attr,
&dev_attr_set_cnt_size.attr,
&dev_attr_set_clk_freq.attr,
&dev_attr_selective_measurement_mask.attr,
&dev_attr_selective_measurement_value.attr,
&dev_attr_selective_measurement_allocate.attr,
NULL,
};

struct attribute_group ppmu_attrs_grp = {
	.attrs = ppmu_attrs,
};

static int airbrush_ppmu_probe(struct platform_device *pdev)
{
	struct airbrush_ppmu *info;
	int ret = 0;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = pdev->dev;

	/* Parse dt data to get resource */
	ret = airbrush_ppmu_parse_dt(pdev, info);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"failed to parse devicetree for resource\n");
		return ret;
	}
	/* sysfs creation*/
	if (sysfs_create_group(&pdev->dev.kobj, &ppmu_attrs_grp))
		dev_err(&pdev->dev, "Sysfs Attribute Creation failed for PPMU\n");
	else
		dev_dbg(&pdev->dev, "Sysfs attribute created for PPMU\n");

	return 0;
}

static int airbrush_ppmu_remove(struct platform_device *pdev)
{
	/* todo */
	return 0;
}

static struct platform_driver airbrush_ppmu_driver = {
	.probe = airbrush_ppmu_probe,
	.remove = airbrush_ppmu_remove,
	.driver = {
		.name = "airbrush-ppmu",
		.of_match_table = airbrush_ppmu_id_match,
	},
};
module_platform_driver(airbrush_ppmu_driver);

MODULE_DESCRIPTION("Airbrush Platform Performance Monitoring Unit driver");
MODULE_AUTHOR("Nishant Prajapati <nishant.p@samsung.com>");
MODULE_LICENSE("GPL");
