/*
 * Copyright (C) 2017-2018 Google, Inc.
 *
 * Author: Trevor Bunker <trevorbunker@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "s2mpg01-core.h"

#define DRIVER_NAME "s2mpg01-regulator"

struct s2mpg01_regulator {
	struct device *dev;
	struct s2mpg01_core *s2mpg01_core;
	struct regulator_dev **rdevs;

	/* bitmask for tracking enabled regulators */
	unsigned long reg_enabled_mask;
};

static struct s2mpg01_regulator *_s2mpg01_regulator;

static int s2mpg01_regulator_enable(struct regulator_dev *rdev);
static int s2mpg01_regulator_disable(struct regulator_dev *rdev);
static int s2mpg01_regulator_is_enabled(struct regulator_dev *rdev);

static struct regulator_ops s2mpg01_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = s2mpg01_regulator_enable,
	.disable = s2mpg01_regulator_disable,
	.is_enabled = s2mpg01_regulator_is_enabled,
};

/* Voltage Table for SMPS1 */
static const struct regulator_linear_range smsp1_volt_range[] = {
	REGULATOR_LINEAR_RANGE(550000, 0x28, 0x70, 6250),
};

/* Voltage Table for SMPS2 */
static const struct regulator_linear_range smsp2_volt_range[] = {
	REGULATOR_LINEAR_RANGE(650000, 0x38, 0x70, 6250),
};

/* Voltage Table for SMPS3 */
static const struct regulator_linear_range smsp3_volt_range[] = {
	REGULATOR_LINEAR_RANGE(900000, 0x60, 0xB0, 6250),
};

/* Voltage Table for LDO1/LDO5 */
static const struct regulator_linear_range ldo1_ldo5_volt_range[] = {
	REGULATOR_LINEAR_RANGE(1500000, 0x20, 0x38, 25000),
};

/* Voltage Table for LDO2 */
static const struct regulator_linear_range ldo2_volt_range[] = {
	REGULATOR_LINEAR_RANGE(550000, 0x0C, 0x18, 12500),
};

/* Voltage Table for LDO3 */
static const struct regulator_linear_range ldo3_volt_range[] = {
	REGULATOR_LINEAR_RANGE(650000, 0x1C, 0x38, 12500),
};

/* Voltage Table for LDO4 */
static const struct regulator_linear_range ldo4_volt_range[] = {
	REGULATOR_LINEAR_RANGE(650000, 0x14, 0x30, 12500),
};


static struct regulator_desc
	s2mpg01_regulator_desc[S2MPG01_NUM_REGULATORS] = {
	[S2MPG01_ID_SMPS1] = {
		.name = S2MPG01_REGLTR_NAME_SMPS1,
		.id = S2MPG01_ID_SMPS1,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x7F + 1),
		.linear_ranges = smsp1_volt_range,
		.n_linear_ranges = ARRAY_SIZE(smsp1_volt_range),
		.vsel_reg = S2MPG01_REG_BUCK1_OUT,
		.vsel_mask = 0x7F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_SMPS2] = {
		.name = S2MPG01_REGLTR_NAME_SMPS2,
		.id = S2MPG01_ID_SMPS2,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0xFF + 1),
		.linear_ranges = smsp2_volt_range,
		.n_linear_ranges = ARRAY_SIZE(smsp2_volt_range),
		.vsel_reg = S2MPG01_REG_BUCK2_OUT,
		.vsel_mask = 0xFF,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_SMPS3] = {
		.name = S2MPG01_REGLTR_NAME_SMPS3,
		.id = S2MPG01_ID_SMPS3,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0xFF + 1),
		.linear_ranges = smsp3_volt_range,
		.n_linear_ranges = ARRAY_SIZE(smsp3_volt_range),
		.vsel_reg = S2MPG01_REG_BUCK3_OUT,
		.vsel_mask = 0xFF,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_LDO1] = {
		.name = S2MPG01_REGLTR_NAME_LDO1,
		.id = S2MPG01_ID_LDO1,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x3F + 1),
		.linear_ranges = ldo1_ldo5_volt_range,
		.n_linear_ranges = ARRAY_SIZE(ldo1_ldo5_volt_range),
		.vsel_reg = S2MPG01_REG_LDO1_CTRL,
		.vsel_mask = 0x3F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_LDO2] = {
		.name = S2MPG01_REGLTR_NAME_LDO2,
		.id = S2MPG01_ID_LDO2,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x3F + 1),
		.linear_ranges = ldo2_volt_range,
		.n_linear_ranges = ARRAY_SIZE(ldo2_volt_range),
		.vsel_reg = S2MPG01_REG_LDO2_CTRL,
		.vsel_mask = 0x3F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_LDO3] = {
		.name = S2MPG01_REGLTR_NAME_LDO3,
		.id = S2MPG01_ID_LDO3,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x3F + 1),
		.linear_ranges = ldo3_volt_range,
		.n_linear_ranges = ARRAY_SIZE(ldo3_volt_range),
		.vsel_reg = S2MPG01_REG_LDO3_CTRL,
		.vsel_mask = 0x3F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_LDO4] = {
		.name = S2MPG01_REGLTR_NAME_LDO4,
		.id = S2MPG01_ID_LDO4,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x3F + 1),
		.linear_ranges = ldo4_volt_range,
		.n_linear_ranges = ARRAY_SIZE(ldo4_volt_range),
		.vsel_reg = S2MPG01_REG_LDO4_CTRL,
		.vsel_mask = 0x3F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_LDO5] = {
		.name = S2MPG01_REGLTR_NAME_LDO5,
		.id = S2MPG01_ID_LDO5,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x3F + 1),
		.linear_ranges = ldo1_ldo5_volt_range,
		.n_linear_ranges = ARRAY_SIZE(ldo1_ldo5_volt_range),
		.vsel_reg = S2MPG01_REG_LDO5_CTRL,
		.vsel_mask = 0x3F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_BOOST_SMPS1] = {
		.name = S2MPG01_REGLTR_NAME_BOOST_SMPS1,
		.id = S2MPG01_ID_BOOST_SMPS1,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x7F + 1),
		.linear_ranges = smsp1_volt_range,
		.n_linear_ranges = ARRAY_SIZE(smsp1_volt_range),
		.vsel_reg = S2MPG01_REG_BUCK1_OUT_DVS,
		.vsel_mask = 0x7F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[S2MPG01_ID_BOOST_LDO3] = {
		.name = S2MPG01_REGLTR_NAME_BOOST_LDO3,
		.id = S2MPG01_ID_BOOST_LDO3,
		.ops = &s2mpg01_regulator_ops,
		.n_voltages = (0x3F + 1),
		.linear_ranges = ldo3_volt_range,
		.n_linear_ranges = ARRAY_SIZE(ldo3_volt_range),
		.vsel_reg = S2MPG01_REG_LDO3_CTRL_DVS,
		.vsel_mask = 0x3F,
		.enable_time = 200,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static struct regulator_init_data
	s2mpg01_regulator_init_data[S2MPG01_NUM_REGULATORS] = {
	[S2MPG01_ID_SMPS1] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_SMPS1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 550000,
			.max_uV = 850000,
		},
	},
	[S2MPG01_ID_SMPS2] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_SMPS2,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 800000,
			.max_uV = 950000,
		},
	},
	[S2MPG01_ID_SMPS3] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_SMPS3,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 1050000,
			.max_uV = 1150000,
		},
	},
	[S2MPG01_ID_LDO1] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_LDO1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 1750000,
			.max_uV = 1850000,
		},
	},
	[S2MPG01_ID_LDO2] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_LDO2,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 550000,
			.max_uV = 650000,
		},
	},
	[S2MPG01_ID_LDO3] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_LDO3,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 700000,
			.max_uV = 800000,
		},
	},
	[S2MPG01_ID_LDO4] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_LDO4,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 800000,
			.max_uV = 950000,
		},
	},
	[S2MPG01_ID_LDO5] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_LDO5,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 1750000,
			.max_uV = 1850000,
		},
	},
	[S2MPG01_ID_BOOST_SMPS1] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_BOOST_SMPS1,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 800000,
			.max_uV = 950000,
		},
	},
	[S2MPG01_ID_BOOST_LDO3] = {
		.constraints = {
			.name = S2MPG01_REGLTR_NAME_BOOST_LDO3,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
			.min_uV = 800000,
			.max_uV = 900000,
		},
	},
};

/* enable the regulator */
static int s2mpg01_regulator_enable(struct regulator_dev *rdev)
{
	struct s2mpg01_regulator *s2mpg01_regulator = rdev_get_drvdata(rdev);
	struct s2mpg01_core *s2mpg01_core = s2mpg01_regulator->s2mpg01_core;
	enum s2mpg01_regulator_ids rid = rdev_get_id(rdev);
	int ret;

	dev_dbg(s2mpg01_regulator->dev, "%s: rid %d\n", __func__, rid);

	switch (rid) {
	case S2MPG01_ID_SMPS1:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_BUCK1_CTRL,
					 0xF8);
		break;
	case S2MPG01_ID_SMPS2:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_BUCK2_CTRL,
					 0xD8);
		break;
	case S2MPG01_ID_SMPS3:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_BUCK3_CTRL,
					 0xD8);
		break;
	case S2MPG01_ID_LDO1:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO1_CTRL,
					 0xEC);
		break;
	case S2MPG01_ID_LDO2:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO2_CTRL,
					 0x90);
		break;
	case S2MPG01_ID_LDO3:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO3_CTRL,
					 0xA4);
		break;
	case S2MPG01_ID_LDO4:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO4_CTRL,
					 0xA4);
		break;
	case S2MPG01_ID_LDO5:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO5_CTRL,
					 0xEC);
		break;
	case S2MPG01_ID_BOOST_SMPS1:
	case S2MPG01_ID_BOOST_LDO3:
		ret = s2mpg01_enable_boost(s2mpg01_core);
		if (!ret) {
			/*
			 * Both smps1 and ldo3 are applied boost mode at the
			 * same time.
			 */
			set_bit(S2MPG01_ID_BOOST_SMPS1,
				&s2mpg01_regulator->reg_enabled_mask);
			set_bit(S2MPG01_ID_BOOST_LDO3,
				&s2mpg01_regulator->reg_enabled_mask);
		}
		return ret;
	default:
		return -EINVAL;
	}

	if (!ret)
		set_bit(rid, &s2mpg01_regulator->reg_enabled_mask);

	return ret;
}

/* disable the regulator */
static int s2mpg01_regulator_disable(struct regulator_dev *rdev)
{
	struct s2mpg01_regulator *s2mpg01_regulator = rdev_get_drvdata(rdev);
	struct s2mpg01_core *s2mpg01_core = s2mpg01_regulator->s2mpg01_core;
	enum s2mpg01_regulator_ids rid = rdev_get_id(rdev);
	int ret;

	dev_dbg(s2mpg01_regulator->dev, "%s: rid %d\n", __func__, rid);

	switch (rid) {
	case S2MPG01_ID_SMPS1:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_BUCK1_CTRL,
					 0x38);
		break;
	case S2MPG01_ID_SMPS2:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_BUCK2_CTRL,
					 0x18);
		break;
	case S2MPG01_ID_SMPS3:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_BUCK3_CTRL,
					 0x18);
		break;
	case S2MPG01_ID_LDO1:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO1_CTRL,
					 0x2C);
		break;
	case S2MPG01_ID_LDO2:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO2_CTRL,
					 0x10);
		break;
	case S2MPG01_ID_LDO3:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO3_CTRL,
					 0x24);
		break;
	case S2MPG01_ID_LDO4:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO4_CTRL,
					 0x24);
		break;
	case S2MPG01_ID_LDO5:
		ret = s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_LDO5_CTRL,
					 0x2C);
		break;
	case S2MPG01_ID_BOOST_SMPS1:
	case S2MPG01_ID_BOOST_LDO3:
		ret = s2mpg01_disable_boost(s2mpg01_core);
		if (!ret) {
			/*
			 * Both smps1 and ldo3 are applied boost mode at the
			 * same time.
			 */
			clear_bit(S2MPG01_ID_BOOST_SMPS1,
				  &s2mpg01_regulator->reg_enabled_mask);
			clear_bit(S2MPG01_ID_BOOST_LDO3,
				  &s2mpg01_regulator->reg_enabled_mask);
		}
		return ret;
	default:
		return -EINVAL;
	}

	if (!ret)
		clear_bit(rid, &s2mpg01_regulator->reg_enabled_mask);

	return ret;
}

/* get regulator enable status */
static int s2mpg01_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct s2mpg01_regulator *s2mpg01_regulator = rdev_get_drvdata(rdev);
	enum s2mpg01_regulator_ids rid = rdev_get_id(rdev);

	dev_dbg(s2mpg01_regulator->dev, "%s: rid %d\n", __func__, rid);

	if ((rid >= 0) && (rid < S2MPG01_NUM_REGULATORS))
		return !!(s2mpg01_regulator->reg_enabled_mask & (1 << rid));
	else
		return -EINVAL;
}

void s2mpg01_regulator_notify(enum s2mpg01_regulator_ids rid,
			      unsigned long event)
{
	if (!_s2mpg01_regulator || !_s2mpg01_regulator->rdevs ||
	    !_s2mpg01_regulator->rdevs[rid])
		return;

	if (!s2mpg01_regulator_is_enabled(_s2mpg01_regulator->rdevs[rid]))
		return;

	dev_err(_s2mpg01_regulator->dev, "%s: rid %d, event 0x%lx\n", __func__,
		rid, event);

	regulator_notifier_call_chain(_s2mpg01_regulator->rdevs[rid], event,
				      NULL);
}
EXPORT_SYMBOL_GPL(s2mpg01_regulator_notify);

/* register a regulator with the kernel regulator framework */
static int
s2mpg01_regulator_register(struct s2mpg01_regulator *s2mpg01_regulator)
{
	struct device *dev = s2mpg01_regulator->dev;
	struct s2mpg01_core *s2mpg01_core = s2mpg01_regulator->s2mpg01_core;
	struct regulator_config cfg = {
		.dev = dev,
		.driver_data = s2mpg01_regulator,
		.regmap = s2mpg01_core->regmap
	};
	struct regulator_dev *rdev;
	int i;

	for (i = 0; i < S2MPG01_NUM_REGULATORS; i++) {
		cfg.init_data = &s2mpg01_regulator_init_data[i];
		rdev = devm_regulator_register(dev,
					       &s2mpg01_regulator_desc[i],
					       &cfg);
		if (IS_ERR(rdev)) {
			dev_err(dev,
				"%s: failed to register regulator %d\n",
				__func__, i);
			return PTR_ERR(rdev);
		}

		*(s2mpg01_regulator->rdevs + i) = rdev;
	}

	return 0;
}

static int s2mpg01_regulator_probe(struct platform_device *pdev)
{
	struct s2mpg01_core *s2mpg01_core = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg01_regulator *s2mpg01_regulator;
	struct device *dev = &pdev->dev;

	s2mpg01_regulator = devm_kzalloc(dev, sizeof(*s2mpg01_regulator),
					 GFP_KERNEL);
	if (!s2mpg01_regulator)
		return -ENOMEM;

	s2mpg01_regulator->dev = dev;
	s2mpg01_regulator->s2mpg01_core = s2mpg01_core;
	_s2mpg01_regulator = s2mpg01_regulator;

	platform_set_drvdata(pdev, s2mpg01_regulator);

	/* initialize and register device regulators */
	s2mpg01_regulator->rdevs =
		devm_kzalloc(dev,
			     S2MPG01_NUM_REGULATORS *
			     sizeof(struct regulator_dev *),
			     GFP_KERNEL);
	if (!s2mpg01_regulator->rdevs) {
		dev_err(dev,
			"%s: could not initialize rdevs array\n",
			__func__);
		return -ENOMEM;
	}

	return s2mpg01_regulator_register(s2mpg01_regulator);
}

static int s2mpg01_regulator_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id s2mpg01_regulator_of_match[] = {
	{ .compatible = "samsung,s2mpg01-regulator", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg01_regulator_of_match);

static struct platform_driver s2mpg01_regulator_driver = {
	.probe = s2mpg01_regulator_probe,
	.remove = s2mpg01_regulator_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = s2mpg01_regulator_of_match,
	},
};
module_platform_driver(s2mpg01_regulator_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPG01 Regulator Driver");
MODULE_LICENSE("GPL");
