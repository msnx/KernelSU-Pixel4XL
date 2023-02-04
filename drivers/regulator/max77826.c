/*
 * max77826.c - Regulator driver for the Maxim 77826
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Copyright (C) 2013 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver is based on max77686.c
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include "max77826.h"

#define MAX_RETRY 3

struct voltage_map_desc {
	int min;
	int max;
	int step;
	unsigned int n_bits;
};

static struct voltage_map_desc ldo_nmos_voltage_map_desc = {
	.min = 600000,	.max = 2187500,	.step = 12500,	.n_bits = 7,
};

static struct voltage_map_desc ldo_pmos_voltage_map_desc = {
	.min = 800000,	.max = 3975000,	.step = 25000,	.n_bits = 7,
};

static struct voltage_map_desc buck_voltage_map_desc = {
	.min = 500000,	.max = 1800000,	.step = 6250,	.n_bits = 8,
};

static struct voltage_map_desc buck_boost_voltage_map_desc = {
	.min = 2600000,	.max = 4187500,	.step = 12500,	.n_bits = 7,
};

static struct voltage_map_desc *reg_voltage_map[] = {
	[MAX77826_LDO1] = &ldo_nmos_voltage_map_desc,
	[MAX77826_LDO2] = &ldo_nmos_voltage_map_desc,
	[MAX77826_LDO3] = &ldo_nmos_voltage_map_desc,
	[MAX77826_LDO4] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO5] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO6] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO7] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO8] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO9] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO10] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO11] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO12] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO13] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO14] = &ldo_pmos_voltage_map_desc,
	[MAX77826_LDO15] = &ldo_pmos_voltage_map_desc,
	[MAX77826_BUCK1] = &buck_voltage_map_desc,
	[MAX77826_BUCK2] = &buck_boost_voltage_map_desc,
};

int max77826_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct max77826_dev *max77826 = i2c_get_clientdata(i2c);
	int ret, retry = 0;

	mutex_lock(&max77826->io_lock);
	do {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret >= 0)
			break;

		/* i2c read fail, sleep 3~6 ms and retry */
		pr_err("%s: i2c read error=%d retry count: %d\n",
			__func__, ret, retry);
		retry++;
		usleep_range(3000, 6000);
	} while (retry < MAX_RETRY);
	mutex_unlock(&max77826->io_lock);

	if (ret < 0)
		return ret;

	ret &= 0xff;
	*dest = ret;
	return 0;
}

int max77826_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct max77826_dev *max77826 = i2c_get_clientdata(i2c);
	int ret, retry = 0;

	mutex_lock(&max77826->io_lock);
	do {
		ret = i2c_smbus_write_byte_data(i2c, reg, value);
		if (ret >= 0)
			break;

		/* i2c wrtie fail, sleep 3~6 ms and retry */
		pr_err("%s: i2c write error=%d retry count: %d\n",
			__func__, ret, retry);
		retry++;
		usleep_range(3000, 6000);
	} while (retry < MAX_RETRY);
	mutex_unlock(&max77826->io_lock);
	return ret;
}

int max77826_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct max77826_dev *max77826 = i2c_get_clientdata(i2c);
	int ret, retry = 0;
	u8 old_val, new_val;

	mutex_lock(&max77826->io_lock);
	do {
		ret = i2c_smbus_read_byte_data(i2c, reg);
		if (ret >= 0)
			break;

		/* i2c read fail, sleep 3~6 ms and retry */
		pr_err("%s: i2c read error=%d retry count: %d\n",
			__func__, ret, retry);
		retry++;
		usleep_range(3000, 6000);
	} while (retry < MAX_RETRY);

	if (ret < 0) {
		mutex_unlock(&max77826->io_lock);
		return ret;
	}

	old_val = ret & 0xff;
	new_val = (val & mask) | (old_val & (~mask));

	retry = 0;
	do {
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
		if (ret >= 0)
			break;
		/* i2c wrtie fail, sleep 3~6 ms and retry */
		pr_err("%s: i2c write error=%d retry count: %d\n",
			__func__, ret, retry);
		retry++;
		usleep_range(3000, 6000);
	} while (retry < MAX_RETRY);

	mutex_unlock(&max77826->io_lock);
	return ret;
}

static int max77826_get_op_register(struct regulator_dev *rdev,
		int *reg, int *mask, int *pattern)
{
	unsigned int rid = rdev_get_id(rdev);

	switch (rid) {
	case MAX77826_LDO1 ... MAX77826_LDO15:
		*reg = MAX77826_REG_LDO_OPMD1 + (rid - MAX77826_LDO1) / 4;
		*mask = 1 << (((rid - MAX77826_LDO1) % 4) * 2 + 1);
		*pattern = 1 << (((rid - MAX77826_LDO1) % 4) * 2 + 1);
		break;

	case MAX77826_BUCK1 ... MAX77826_BUCK2:
		*reg = MAX77826_REG_B_BB_OPMD;
		*mask = 1 << (((rid - MAX77826_BUCK1) % 2) * 2 + 1);
		*pattern = 1 << (((rid - MAX77826_BUCK1) % 2) * 2 + 1);
		break;

	default:
		return -EINVAL;
	}

	pr_debug("%s: id=%d, reg=0x%x, mask=0x%x, pattern=0x%x\n",
		__func__, rid, *reg, *mask, *pattern);

	return 0;
}

static int max77826_get_voltage_register(struct regulator_dev *rdev,
		int *reg, int *shift, int *mask)
{
	int rid = rdev_get_id(rdev);

	switch (rid) {
	case MAX77826_LDO1 ... MAX77826_LDO15:
		*reg = MAX77826_REG_LDO1_CFG + (rid - MAX77826_LDO1);
		*shift = 0;
		*mask = 0x7F;
		break;

	case MAX77826_BUCK1:
		*reg = MAX77826_REG_BUCK_VOUT;
		*shift = 0;
		*mask = 0xFF;
		break;

	case MAX77826_BUCK2:
		*reg = MAX77826_REG_BB_VOUT;
		*shift = 0;
		*mask = 0x7F;
		break;

	default:
		return -EINVAL;
	}

	pr_debug("%s: id=%d, reg=0x%x, shift=0x%x, mask=0x%x\n",
		__func__, rid, *reg, *shift, *mask);

	return 0;
}

static int max77826_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct max77826_dev *max77826 = rdev_get_drvdata(rdev);
	int ret = 0, reg = 0, mask = 0, pattern = 0;
	u8 val;

	ret = max77826_get_op_register(rdev, &reg, &mask, &pattern);
	if (ret) {
		pr_err("%s: id=%d, error=%d\n",
			__func__, rdev_get_id(rdev), ret);
		return ret;
	}

	ret = max77826_read_reg(max77826->i2c, reg, &val);
	if (ret)
		return ret;

	pr_debug("%s: id=%d, ret=%d, val=0x%x, mask=0x%x, pattern=0x%x\n",
		__func__, rdev_get_id(rdev), (val & mask) == pattern,
		val, mask, pattern);

	return (val & mask) == pattern;
}

static int max77826_bb_enable_force_pwm(struct max77826_dev *max77826)
{
	int ret = 0;

	/* Forced PWM on Buck-Boost */
	ret = max77826_write_reg(max77826->i2c, MAX77826_REG_BB_CFG, 0x3A);
	if (ret)
		pr_err("%s: reg=0x%x, error=%d\n",
			__func__, MAX77826_REG_BB_CFG, ret);

	return ret;
}

static int max77826_bb_enable_skip_mode(struct max77826_dev *max77826)
{
	int ret = 0;

	/* Reset to high skip mode on Buck-Boost */
	ret = max77826_write_reg(max77826->i2c, MAX77826_REG_BB_CFG, 0x3C);
	if (ret)
		pr_err("%s: reg=0x%x, error=%d\n",
			__func__, MAX77826_REG_BB_CFG, ret);

	return ret;
}

static int max77826_ldo_enable(struct regulator_dev *rdev)
{
	struct max77826_dev *max77826 = rdev_get_drvdata(rdev);
	int ret = 0, reg = 0, mask = 0, pattern = 0;
	unsigned int rid;

	rid = rdev_get_id(rdev);
	if (rid == MAX77826_LDO1 || rid == MAX77826_LDO11) {
		/* LDO1 or LDO11 enable means camera is power up.
		 * Enbale PWM mode
		 */
		max77826_bb_enable_force_pwm(max77826);
	}

	ret = max77826_get_op_register(rdev, &reg, &mask, &pattern);
	if (ret) {
		pr_err("%s: id=%d, error=%d\n",
			__func__, rdev_get_id(rdev), ret);
		return ret;
	}

	pr_debug("%s: id=%d, reg=0x%x, mask=0x%x, pattern=0x%x\n",
		__func__, rdev_get_id(rdev), reg, mask, pattern);
	return max77826_update_reg(max77826->i2c, reg, pattern, mask);
}

static int max77826_ldo_disable(struct regulator_dev *rdev)
{
	struct max77826_dev *max77826 = rdev_get_drvdata(rdev);
	int ret = 0, reg = 0, mask = 0, pattern = 0;
	unsigned int rid;

	rid = rdev_get_id(rdev);
	if (rid == MAX77826_LDO1 || rid == MAX77826_LDO11) {
		/* LDO1 or LDO11 disable means camera is power off.
		 * But vd6281 might still power up, set to high skip
		 * mode to avoid power consumption
		 */
		max77826_bb_enable_skip_mode(max77826);
	}

	ret = max77826_get_op_register(rdev, &reg, &mask, &pattern);
	if (ret) {
		pr_err("%s: id=%d, error=%d\n",
			__func__, rdev_get_id(rdev), ret);
		return ret;
	}

	pr_debug("%s: id=%d, reg=0x%x, mask=0x%x, pattern=0x%x\n",
		__func__, rdev_get_id(rdev), reg, mask, pattern);

	return max77826_update_reg(max77826->i2c, reg, ~mask, mask);
}

static int max77826_ldo_list_voltage(struct regulator_dev *rdev,
		unsigned int selector)
{
	const struct voltage_map_desc *desc;
	int rid = rdev_get_id(rdev);
	int val;

	if (rid >= ARRAY_SIZE(reg_voltage_map) ||
			rid < 0)
		return -EINVAL;

	desc = reg_voltage_map[rid];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	pr_debug("%s: id=%d, val=%d\n", __func__, rdev_get_id(rdev), val);

	return val;
}

static int max77826_ldo_get_voltage(struct regulator_dev *rdev)
{
	struct max77826_dev *max77826 = rdev_get_drvdata(rdev);
	int reg, shift, mask, ret;
	u8 val;

	ret = max77826_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret) {
		pr_err("%s: id=%d, error=%d\n",
			__func__, rdev_get_id(rdev), ret);
		return ret;
	}

	ret = max77826_read_reg(max77826->i2c, reg, &val);
	if (ret)
		return ret;

	val = (val & mask) >> shift;

	pr_debug("%s: id=%d, reg=0x%x, mask=0x%x, val=0x%x\n",
		__func__, rdev_get_id(rdev), reg, mask, val);

	return max77826_ldo_list_voltage(rdev, val);
}

static inline int max77826_get_voltage_proper_val(
		const struct voltage_map_desc *desc,
		int min_vol, int max_vol)
{
	int i = 0;

	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	while (desc->min + desc->step * i < min_vol &&
		desc->min + desc->step * i < desc->max)
		i++;

	if (desc->min + desc->step * i > max_vol)
		return -EINVAL;

	if (i >= (1 << desc->n_bits))
		return -EINVAL;

	return i;
}

static int max77826_ldo_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned int *selector)
{
	struct max77826_dev *max77826 = rdev_get_drvdata(rdev);
	const struct voltage_map_desc *desc;
	int rid = rdev_get_id(rdev);
	int reg, shift, mask, ret;
	int i;
	u8 val;

	desc = reg_voltage_map[rid];

	i = max77826_get_voltage_proper_val(desc, min_uV, max_uV);
	if (i < 0)
		return i;

	ret = max77826_get_voltage_register(rdev, &reg, &shift, &mask);

	if (ret) {
		pr_err("%s: id=%d, error=%d\n",
			__func__, rdev_get_id(rdev), ret);
		return ret;
	}

	max77826_read_reg(max77826->i2c, reg, &val);
	val = (val & mask) >> shift;

	pr_debug("%s: id=%d, reg=0x%x, mask=0x%x, val=0x%x, new=0x%x\n",
		__func__, rdev_get_id(rdev), reg, mask, val, i);

	ret = max77826_update_reg(max77826->i2c,
		reg, i << shift, mask << shift);
	*selector = i;

	switch (rid) {
	case MAX77826_BUCK1:
		if (val < i)
			udelay(DIV_ROUND_UP(desc->step * (i - val), 12500));
		break;
	default:
		break;
	}

	return ret;
}

static struct regulator_ops max77826_ldo_ops = {
	.is_enabled = max77826_ldo_is_enabled,
	.enable = max77826_ldo_enable,
	.disable = max77826_ldo_disable,
	.list_voltage = max77826_ldo_list_voltage,
	.get_voltage = max77826_ldo_get_voltage,
	.set_voltage = max77826_ldo_set_voltage,
};

#define regulator_desc_ldo(num)		{\
	.name		= "max77826_ldo"#num,\
	.of_match	= of_match_ptr("max77826_ldo"#num),\
	.regulators_node	= of_match_ptr("regulators"),\
	.id			= MAX77826_LDO##num,\
	.ops		= &max77826_ldo_ops,\
	.type		= REGULATOR_VOLTAGE,\
	.owner		= THIS_MODULE,\
}
#define regulator_desc_buck(num)	{\
	.name		= "max77826_buck"#num,\
	.of_match	= of_match_ptr("max77826_buck"#num),\
	.regulators_node	= of_match_ptr("regulators"),\
	.id			= MAX77826_BUCK##num,\
	.ops		= &max77826_ldo_ops,\
	.type		= REGULATOR_VOLTAGE,\
	.owner		= THIS_MODULE,\
}

static struct regulator_desc regulators[] = {
	regulator_desc_ldo(1),
	regulator_desc_ldo(2),
	regulator_desc_ldo(3),
	regulator_desc_ldo(4),
	regulator_desc_ldo(5),
	regulator_desc_ldo(6),
	regulator_desc_ldo(7),
	regulator_desc_ldo(8),
	regulator_desc_ldo(9),
	regulator_desc_ldo(10),
	regulator_desc_ldo(11),
	regulator_desc_ldo(12),
	regulator_desc_ldo(13),
	regulator_desc_ldo(14),
	regulator_desc_ldo(15),
	regulator_desc_buck(1),
	regulator_desc_buck(2),
};

static int max77826_setup_regulators(struct max77826_dev *max77826,
	struct max77826_platform_data *pdata)
{
	int i, err;
	struct regulator_config config = { };
	int volt_range = 0;

	max77826->rdev = kcalloc(MAX77826_REG_MAX,
				sizeof(struct regulator_dev *), GFP_KERNEL);
	if (!max77826->rdev) {
		err = -ENOMEM;
		goto err_nomem;
	}

	config.dev = max77826->dev;
	config.driver_data = max77826;

	/* Register the regulators */
	for (i = 0; i < MAX77826_REG_MAX; i++) {
		volt_range =
			reg_voltage_map[i]->max - reg_voltage_map[i]->min;
		regulators[regulators[i].id].n_voltages =
			volt_range / reg_voltage_map[i]->step;
		max77826->rdev[i] = devm_regulator_register(max77826->dev,
			&regulators[regulators[i].id], &config);
		if (IS_ERR(max77826->rdev[i])) {
			err = PTR_ERR(max77826->rdev[i]);
			dev_err(max77826->dev, "regulator init failed: %d\n",
				err);
			goto error;
		}
	}
	return 0;

error:
	while (--i >= 0)
		regulator_unregister(max77826->rdev[i]);

	kfree(max77826->rdev);
	max77826->rdev = NULL;

err_nomem:
	return err;
}

static int max77826_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct max77826_dev *max77826;
	struct max77826_platform_data *pdata;
	int ret, rc = 0;

	pr_debug("%s\n", __func__);

	/*Check I2C functionality */
	ret = i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C);
	if (!ret) {
		pr_err("%s: No I2C functionality found\n", __func__);
		ret = -ENODEV;
		goto err_i2c_fail;
	}

	if (i2c->dev.of_node) {
		pr_debug("%s: of_node\n", __func__);

		pdata = devm_kzalloc(&i2c->dev,
			sizeof(struct max77826_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev,
				"%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
	} else {
		pr_debug("%s: pdata\n", __func__);

		pdata = i2c->dev.platform_data;
		if (!pdata) {
			dev_err(&i2c->dev, "%s: no pdata\n", __func__);
			ret = -ENODEV;
			goto err_i2c_fail;
		}
	}

	max77826 = kzalloc(sizeof(struct max77826_dev), GFP_KERNEL);
	if (!max77826)
		return -ENOMEM;

	max77826->i2c = i2c;
	max77826->dev = &i2c->dev;

	pdata->enable_gpio = of_get_named_gpio(
		max77826->dev->of_node, "maxim,enable-gpio", 0);
	rc = devm_gpio_request_one(
		max77826->dev, pdata->enable_gpio, GPIOF_IN, "MAX77826 ENABLE");
	gpio_direction_output(pdata->enable_gpio, 1);
	gpio_set_value(pdata->enable_gpio, 1);
	mutex_init(&max77826->io_lock);
	i2c_set_clientdata(i2c, max77826);
	ret = max77826_setup_regulators(max77826, pdata);

	if (ret < 0)
		goto err_detect;

	return 0;

err_detect:
	kfree(max77826);
err_i2c_fail:
	return ret;
}

static int max77826_i2c_remove(struct i2c_client *i2c)
{
	struct max77826_dev *max77826 = i2c_get_clientdata(i2c);
	int i, enable_gpio, rc;

	enable_gpio = of_get_named_gpio(
		max77826->dev->of_node, "maxim,enable-gpio", 0);

	for (i = 0; i < max77826->num_regulators; i++)
		regulator_unregister(max77826->rdev[i]);
	rc = devm_gpio_request_one(max77826->dev,
		enable_gpio, GPIOF_IN, "MAX77826 ENABLE");
	if (rc) {
		pr_err("%s Request GPIO %d failed", __func__, enable_gpio);
		goto request_gpio_err;
	}
	gpio_set_value(enable_gpio, 0);
	gpio_direction_input(enable_gpio);

request_gpio_err:
	kfree(max77826->rdev);
	kfree(max77826);
	return 0;
}

static const struct i2c_device_id max77826_i2c_id[] = {
	{ "max77826", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77826_i2c_id);

static const struct of_device_id max77826_match_table[] = {
	{ .compatible = "max77826,max77826-regulator",},
	{ },
};

static struct i2c_driver max77826_i2c_driver = {
	.driver = {
		.name = "max77826",
		.owner = THIS_MODULE,
		.of_match_table = max77826_match_table,
	},
	.probe    = max77826_i2c_probe,
	.remove   = max77826_i2c_remove,
	.id_table = max77826_i2c_id,
};

static int __init max77826_module_init(void)
{
	return i2c_add_driver(&max77826_i2c_driver);
}
subsys_initcall(max77826_module_init);

static void __exit max77826_module_exit(void)
{
	i2c_del_driver(&max77826_i2c_driver);
}
module_exit(max77826_module_exit);

MODULE_DESCRIPTION("MAXIM 77826 Regulator Driver");
MODULE_LICENSE("GPL");
