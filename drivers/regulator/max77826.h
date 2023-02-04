/*
 * max77826.h - Regulator driver for the Maxim 77826
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
 * This driver is based on max77686.h
 */


#ifndef __LINUX_REGULATOR_MAX77826_H
#define __LINUX_REGULATOR_MAX77826_H

#include <linux/regulator/machine.h>

/* MAX77826 regulator ids */
enum max77826_regulators {
	MAX77826_LDO1 = 0,
	MAX77826_LDO2,
	MAX77826_LDO3,
	MAX77826_LDO4,
	MAX77826_LDO5,
	MAX77826_LDO6,
	MAX77826_LDO7,
	MAX77826_LDO8,
	MAX77826_LDO9,
	MAX77826_LDO10,
	MAX77826_LDO11,
	MAX77826_LDO12,
	MAX77826_LDO13,
	MAX77826_LDO14,
	MAX77826_LDO15,
	MAX77826_BUCK1,
	MAX77826_BUCK2,
	MAX77826_REG_MAX,
};

/* MAX77826 PMIC Registers. */
enum max77826_pmic_registers {
	MAX77826_REG_INT_SRC = 0x00,
	MAX77826_REG_SYS_INT,
	MAX77826_REG_INT1,
	MAX77826_REG_INT2,
	MAX77826_REG_BB_INT,
	MAX77826_REG_INT_SRC_M,
	MAX77826_REG_TOPSYS_INT_M,
	MAX77826_REG_INT1_M,
	MAX77826_REG_INT2_M,
	MAX77826_REG_BB_INT_M,
	MAX77826_REG_TOPSYS_STAT,
	MAX77826_REG_STAT1,
	MAX77826_REG_STAT2,
	MAX77826_REG_BB_STAT,
	/* 0x0E - 0x0F: Reserved */
	MAX77826_REG_LDO_OPMD1 = 0x10,
	MAX77826_REG_LDO_OPMD2,
	MAX77826_REG_LDO_OPMD3,
	MAX77826_REG_LDO_OPMD4,
	MAX77826_REG_B_BB_OPMD,
	/* 0x15 - 0x1F: Reserved */
	MAX77826_REG_LDO1_CFG = 0x20,
	MAX77826_REG_LDO2_CFG,
	MAX77826_REG_LDO3_CFG,
	MAX77826_REG_LDO4_CFG,
	MAX77826_REG_LDO5_CFG,
	MAX77826_REG_LDO6_CFG,
	MAX77826_REG_LDO7_CFG,
	MAX77826_REG_LDO8_CFG,
	MAX77826_REG_LDO9_CFG,
	MAX77826_REG_LDO10_CFG,
	MAX77826_REG_LDO11_CFG,
	MAX77826_REG_LDO12_CFG,
	MAX77826_REG_LDO13_CFG,
	MAX77826_REG_LDO14_CFG,
	MAX77826_REG_LDO15_CFG,
	/* 0x2F: Reserved */
	MAX77826_REG_BUCK_CFG = 0x30,
	MAX77826_REG_BUCK_VOUT,
	MAX77826_REG_BB_CFG,
	MAX77826_REG_BB_VOUT,
	/* 0x34 - 0x3F: Reserved */
	MAX77826_REG_BUCK_SS_FREQ = 0x40,
	MAX77826_REG_UVLO_FALL,
	/* 0x42 - 0xCE: Reserved */
	MAX77826_REG_DEVICE_ID = 0xCF,
};

struct max77826_dev {
	struct device *dev;
	struct mutex io_lock;
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
};

struct max77826_regulator_subdev {
	int id;
	struct regulator_init_data *initdata;
};

struct max77826_platform_data {
	char *name;
	int num_regulators;
	int enable_gpio;
	struct max77826_regulator_subdev *regulators;
};

int max77826_i2c_read(void);
#endif	/* __LINUX_REGULATOR_MAX77826_H */
