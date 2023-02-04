/*
 * airbrush_tmu.c - AIRBRUSH TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2018 Samsung Electronics
 *  Alim Akhtar <alim.akhtar@samsung.com>
 *
 * Based on exynos_tmu.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "drvdata.h"
#include "hw.h"
#include "isr.h"
#include "sensor.h"

#define AB_TMU_BASE	0xB90000

static void airbrush_tmu_pcie_link_post_enable(struct ab_tmu_hw *hw,
		void *data)
{
	struct ab_tmu_drvdata *tmu_data = data;
	int i, ret;

	ret = ab_tmu_hw_initialize(hw);
	if (ret)
		return;

	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++) {
		ab_tmu_sensor_load_trim_info(tmu_data->sensor[i]);
		ab_tmu_sensor_save_threshold(tmu_data->sensor[i]);
	}
	ab_tmu_hw_control(hw, true);
};

static void airbrush_tmu_pcie_link_pre_disable(struct ab_tmu_hw *hw,
		void *data)
{
	ab_tmu_hw_control(hw, false);
};

static void airbrush_tmu_post_enable(struct ab_tmu_hw *hw, void *data)
{
	struct ab_tmu_drvdata *tmu_data = data;
	int i;

	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++)
		ab_tmu_sensor_update(tmu_data->sensor[i]);
}

static const struct ab_tmu_hw_events airbrush_tmu_events = {
	.pcie_link_post_enable = airbrush_tmu_pcie_link_post_enable,
	.pcie_link_pre_disable = airbrush_tmu_pcie_link_pre_disable,
	.post_enable = airbrush_tmu_post_enable,
};

static const struct of_device_id airbrush_tmu_match[] = {
	{ .compatible = "abc,airbrush-tmu", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, airbrush_tmu_match);

static ssize_t temp_probe_show(struct device *dev, int id, char *buf)
{
	struct ab_tmu_drvdata *data = dev_get_drvdata(dev);
	struct ab_tmu_hw *hw = data->hw;
	bool pcie_link_ready;
	u32 code;
	int temp, ret;

	pcie_link_ready = ab_tmu_hw_pcie_link_lock(hw);
	if (pcie_link_ready) {
		code = ab_tmu_hw_read_current_temp(hw, id);
		temp = ab_tmu_sensor_raw_to_cel(data->sensor[id], code);
		ret = scnprintf(buf, PAGE_SIZE, "code=%u, temp=%d\n", code,
			temp);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "pcie link down\n");
	}
	ab_tmu_hw_pcie_link_unlock(hw);
	return ret;
}

#define AB_TMU_TEMP_PROBE_ATTR_RO(name, id) \
	static ssize_t temp_probe_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		return temp_probe_show(dev, (id), buf); \
	} \
	static DEVICE_ATTR_RO(temp_probe_##name)

AB_TMU_TEMP_PROBE_ATTR_RO(main, AB_TMU_TEMP_PROBE_MAIN);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu0, AB_TMU_TEMP_PROBE_IPU0);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu1, AB_TMU_TEMP_PROBE_IPU1);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu2, AB_TMU_TEMP_PROBE_IPU2);
AB_TMU_TEMP_PROBE_ATTR_RO(ipu_tpu, AB_TMU_TEMP_PROBE_IPU_TPU);
AB_TMU_TEMP_PROBE_ATTR_RO(tpu0, AB_TMU_TEMP_PROBE_TPU0);
AB_TMU_TEMP_PROBE_ATTR_RO(tpu1, AB_TMU_TEMP_PROBE_TPU1);

static struct attribute *airbrush_tmu_attrs[] = {
	&dev_attr_temp_probe_main.attr,
	&dev_attr_temp_probe_ipu0.attr,
	&dev_attr_temp_probe_ipu1.attr,
	&dev_attr_temp_probe_ipu2.attr,
	&dev_attr_temp_probe_ipu_tpu.attr,
	&dev_attr_temp_probe_tpu0.attr,
	&dev_attr_temp_probe_tpu1.attr,
	NULL,
};

ATTRIBUTE_GROUPS(airbrush_tmu);

static int airbrush_tmu_probe(struct platform_device *pdev)
{
	struct ab_tmu_drvdata *data;
	int i;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct ab_tmu_drvdata),
					GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = devm_device_add_groups(&pdev->dev, airbrush_tmu_groups);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, data);

	/* TODO Probe from device tree instead of a hard coded base. */
	data->hw = devm_ab_tmu_hw_create(&pdev->dev, AB_TMU_BASE);
	if (IS_ERR(data->hw))
		return PTR_ERR(data->hw);

	/*
	 * data->sensor must be created before calling
	 * airbrush_tmu_initialize(), requesting irq and calling
	 * ab_tmu_hw_control().
	 */
	for (i = 0; i < AB_TMU_NUM_ALL_PROBE; i++) {
		data->sensor[i] = devm_ab_tmu_sensor_create(&pdev->dev,
				data->hw, i);
		if (IS_ERR(data->sensor[i])) {
			dev_err(&pdev->dev, "Failed to register sensor %d", i);
			return PTR_ERR(data->sensor[i]);
		}
	}

	data->isr = devm_ab_tmu_isr_request(&pdev->dev);
	if (IS_ERR(data->isr))
		return PTR_ERR(data->isr);

	ab_tmu_hw_register_events(data->hw, &airbrush_tmu_events, data);

	airbrush_tmu_pcie_link_post_enable(data->hw, data);
	dev_dbg(&pdev->dev, "%s: done.\n", __func__);
	return 0;
}

static int airbrush_tmu_remove(struct platform_device *pdev)
{
	struct ab_tmu_drvdata *data = platform_get_drvdata(pdev);

	ab_tmu_hw_control(data->hw, false);
	return 0;
}

static struct platform_driver airbrush_tmu_driver = {
	.driver = {
		.name   = "airbrush-tmu",
		.of_match_table = airbrush_tmu_match,
	},
	.probe = airbrush_tmu_probe,
	.remove	= airbrush_tmu_remove,
};

module_platform_driver(airbrush_tmu_driver);

MODULE_DESCRIPTION("AIRBRUSH TMU Driver");
MODULE_AUTHOR("Alim Akhtar <alim.akhtar@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:airbrush-tmu");
