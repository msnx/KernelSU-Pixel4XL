/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This is useful for systems with mixed controllable and
 * non-controllable regulators, as well as for allowing testing on
 * systems with no controllable regulators.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

struct regulator_tps_data {
	struct regulator_desc desc;
	struct regulator_dev *dev;
	struct mutex lock;
	bool enabled;
	const char *supply_name;
	struct regulator_init_data *init_data;
	unsigned int nr_control_gpios;
	struct gpio_desc **control_gpio_descs;
	unsigned int nr_gpio_en;
	unsigned int *gpio_enable_sequence;
	unsigned int *gpio_enable_delay;
	unsigned int nr_gpio_dis;
	unsigned int *gpio_disable_sequence;
	unsigned int *gpio_disable_delay;
};


int of_get_regulator_tps_config(struct device *dev,
				const struct regulator_desc *desc,
				struct regulator_tps_data *drvdata)
{
	struct device_node *np = dev->of_node;
	struct regulator_init_data *init_data;
	int i, ret;

	drvdata->init_data = of_get_regulator_init_data(dev, dev->of_node,
							desc);
	if (!drvdata->init_data)
		return -EINVAL;

	init_data = drvdata->init_data;
	init_data->constraints.apply_uV = 0;

	drvdata->supply_name = init_data->constraints.name;

	of_property_read_u32(np, "startup-delay-us",
			     &drvdata->desc.enable_time);

	ret = of_property_read_u32(np, "nr-control-gpios",
			     &drvdata->nr_control_gpios);
	if (ret || drvdata->nr_control_gpios <= 0) {
		dev_err(dev, "Incorrect value for nr-control-gpios\n");
		return -EINVAL;
	}

	drvdata->control_gpio_descs = devm_kcalloc(dev,
						  drvdata->nr_control_gpios,
						  sizeof(struct gpio_desc *),
						  GFP_KERNEL);
	if (!drvdata->control_gpio_descs) {
		dev_err(dev, "nr_control_gpio allocation failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < drvdata->nr_control_gpios; i++)
		drvdata->control_gpio_descs[i] = devm_gpiod_get_index(dev,
							       "control",
							       i,
							       GPIOD_OUT_LOW);

	drvdata->nr_gpio_en = of_property_count_elems_of_size(np,
							"gpio-enable-sequence",
							sizeof(u32));
	if (drvdata->nr_gpio_en <= 0) {
		dev_err(dev, "gpio-enable-sequence not set\n");
		return -EINVAL;
	}

	drvdata->gpio_enable_sequence = devm_kcalloc(dev, drvdata->nr_gpio_en,
						     sizeof(unsigned int),
						     GFP_KERNEL);
	if (!drvdata->gpio_enable_sequence) {
		dev_err(dev, "gpio_enable_sequence allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-enable-sequence",
				   drvdata->gpio_enable_sequence,
				   drvdata->nr_gpio_en);
	if (ret) {
		dev_err(dev, "Failed to read gpio-enable-sequence\n");
		return ret;
	}

	ret = of_property_count_elems_of_size(np, "gpio-enable-delay",
					      sizeof(u32));
	if (ret != drvdata->nr_gpio_en) {
		dev_err(dev,
			"gpio-enable-delay should have same length as gpio-enable-sequence\n"
			);
		return -EINVAL;
	}

	drvdata->gpio_enable_delay = devm_kcalloc(dev, drvdata->nr_gpio_en,
						  sizeof(unsigned int),
						  GFP_KERNEL);
	if (!drvdata->gpio_enable_delay) {
		dev_err(dev, "gpio_enable_delay allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-enable-delay",
				   drvdata->gpio_enable_delay,
				   drvdata->nr_gpio_en);
	if (ret) {
		dev_err(dev, "Failed to read gpio-enable-delay\n");
		return ret;
	}

	drvdata->nr_gpio_dis = of_property_count_elems_of_size(np,
						"gpio-disable-sequence",
						sizeof(u32));
	if (drvdata->nr_gpio_dis <= 0) {
		dev_err(dev, "gpio-disable-sequence not set\n");
		return -EINVAL;
	}

	drvdata->gpio_disable_sequence = devm_kcalloc(dev, drvdata->nr_gpio_dis,
						     sizeof(unsigned int),
						     GFP_KERNEL);
	if (!drvdata->gpio_disable_sequence) {
		dev_err(dev, "gpio_disable_sequence allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-disable-sequence",
				   drvdata->gpio_disable_sequence,
				   drvdata->nr_gpio_dis);
	if (ret) {
		dev_err(dev, "Failed to read gpio-disable-sequence\n");
		return ret;
	}

	ret = of_property_count_elems_of_size(np, "gpio-disable-delay",
					      sizeof(u32));
	if (ret != drvdata->nr_gpio_dis) {
		dev_err(dev,
			"gpio-disable-delay should have same length as gpio-disable-sequence\n"
			);
		return -EINVAL;
	}

	drvdata->gpio_disable_delay = devm_kcalloc(dev, drvdata->nr_gpio_dis,
						   sizeof(unsigned int),
						   GFP_KERNEL);
	if (!drvdata->gpio_disable_delay) {
		dev_err(dev, "gpio_disable_delay allocation failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_array(np, "gpio-disable-delay",
				   drvdata->gpio_disable_delay,
				   drvdata->nr_gpio_dis);
	if (ret) {
		dev_err(dev, "Failed to read gpio-disable-delay\n");
		return ret;
	}

	return 0;
}

static int tps_regulator_enable(struct regulator_dev *rdev)
{
	struct regulator_tps_data *drvdata;
	int i = 0;

	drvdata = rdev_get_drvdata(rdev);
	mutex_lock(&drvdata->lock);

	if (drvdata->enabled)
		goto exit;

	for (i = 0; i < drvdata->nr_gpio_en; i++) {
		gpiod_set_value_cansleep(drvdata->control_gpio_descs[
					 drvdata->gpio_enable_sequence[i] - 1],
					 1);
		udelay(drvdata->gpio_enable_delay[i]);
	}

	drvdata->enabled = true;
exit:
	mutex_unlock(&drvdata->lock);
	return 0;
}

static int tps_regulator_disable(struct regulator_dev *rdev)
{
	struct regulator_tps_data *drvdata;
	int i = 0;

	drvdata = rdev_get_drvdata(rdev);
	mutex_lock(&drvdata->lock);

	if (!drvdata->enabled)
		goto exit;

	for (i = 0; i < drvdata->nr_gpio_dis; i++) {
		gpiod_set_value_cansleep(drvdata->control_gpio_descs[
					 drvdata->gpio_disable_sequence[i] - 1],
					 0);
		udelay(drvdata->gpio_disable_delay[i]);
	}

	drvdata->enabled = false;
exit:
	mutex_unlock(&drvdata->lock);
	return 0;
}

static int tps_regulator_is_enabled(struct regulator_dev *dev)
{
	struct regulator_tps_data *drvdata;

	drvdata = rdev_get_drvdata(dev);

	return drvdata->enabled ? 1 : 0;
}

static struct regulator_ops regulator_tps_voltage_ops = {
	.is_enabled = tps_regulator_is_enabled,
	.enable = tps_regulator_enable,
	.disable = tps_regulator_disable,
};

static int reg_tps_probe(struct platform_device *pdev)
{
	struct regulator_tps_data *drvdata;
	struct regulator_config cfg = { };
	int ret;

	drvdata = devm_kzalloc(&pdev->dev,
			       sizeof(struct regulator_tps_data),
			       GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	ret = of_get_regulator_tps_config(&pdev->dev, &drvdata->desc, drvdata);
	if (ret)
		return ret;

	drvdata->desc.name = devm_kstrdup(&pdev->dev,
					  drvdata->supply_name,
					  GFP_KERNEL);
	if (drvdata->desc.name == NULL) {
		dev_err(&pdev->dev, "Failed to allocate supply name\n");
		return -ENOMEM;
	}
	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.ops = &regulator_tps_voltage_ops;

	cfg.dev = &pdev->dev;
	cfg.init_data = drvdata->init_data;
	cfg.driver_data = drvdata;
	cfg.of_node = pdev->dev.of_node;

	drvdata->dev = devm_regulator_register(&pdev->dev, &drvdata->desc,
					       &cfg);
	if (IS_ERR(drvdata->dev)) {
		ret = PTR_ERR(drvdata->dev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, drvdata);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id regulator_tps_of_match[] = {
	{ .compatible = "regulator-tps", },
	{},
};
MODULE_DEVICE_TABLE(of, regulator_tps_of_match);
#endif

static struct platform_driver regulator_tps_driver = {
	.probe          = reg_tps_probe,
	.driver         = {
		.name           = "reg-fixed-voltage-tps",
		.of_match_table = of_match_ptr(regulator_tps_of_match),
	},
};

static int __init regulator_tps_init(void)
{
	return platform_driver_register(&regulator_tps_driver);
}
subsys_initcall(regulator_tps_init);

static void __exit regulator_tps_exit(void)
{
	platform_driver_unregister(&regulator_tps_driver);
}
module_exit(regulator_tps_exit);

MODULE_AUTHOR("Badhri Jagan Sridharan<badhri@google.com>");
MODULE_DESCRIPTION("tps voltage regulator");
MODULE_LICENSE("GPL");

