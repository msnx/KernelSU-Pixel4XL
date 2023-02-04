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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "s2mpg01-core.h"

#define DRIVER_NAME "s2mpg01-thermal"

#define S2MPG01_THERM_NUM_TRIPS 0
#define S2MPG01_THERM_TRIPS_MASK ((1 << S2MPG01_THERM_NUM_TRIPS) - 1)

struct s2mpg01_thermal {
	struct s2mpg01_core *s2mpg01;
	struct thermal_zone_device *tz_dev;
};

static int s2mpg01_thermal_get_temp(struct thermal_zone_device *tz_dev,
				    int *temp)
{
	struct s2mpg01_thermal *s2mpg01_thermal = tz_dev->devdata;
	struct s2mpg01_core *s2mpg01 = s2mpg01_thermal->s2mpg01;
	u8 chan_data = 0;
	int ret;

	ret = s2mpg01_read_adc_chan(s2mpg01, S2MPG01_ADC_PTAT, &chan_data);
	if (ret)
		return ret;

	*temp = PTAT_CODE_TO_TEMP(chan_data);
	return 0;
}

static int s2mpg01_thermal_get_mode(struct thermal_zone_device *tz_dev,
				    enum thermal_device_mode *mode)
{
	*mode = THERMAL_DEVICE_ENABLED;
	return 0;
}

static struct thermal_zone_device_ops s2mpg01_thermal_ops = {
	.get_temp = s2mpg01_thermal_get_temp,
	.get_mode = s2mpg01_thermal_get_mode,
};

static int s2mpg01_thermal_probe(struct platform_device *pdev)
{
	struct s2mpg01_core *s2mpg01 = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg01_thermal *s2mpg01_thermal;
	char tz_name[] = "s2mpg01_tz";

	s2mpg01_thermal = devm_kzalloc(&pdev->dev,
				       sizeof(struct s2mpg01_thermal),
				       GFP_KERNEL);
	if (!s2mpg01_thermal)
		return -ENOMEM;

	s2mpg01_thermal->s2mpg01 = s2mpg01;

	/* register the thermal zone device */
	s2mpg01_thermal->tz_dev =
		thermal_zone_device_register(tz_name, S2MPG01_THERM_NUM_TRIPS,
					     S2MPG01_THERM_TRIPS_MASK,
					     s2mpg01_thermal,
					     &s2mpg01_thermal_ops,
					     NULL, 0, 0);
	if (IS_ERR(s2mpg01_thermal->tz_dev)) {
		dev_err(&pdev->dev, "%s: failed to register thermal zone device (%ld)\n",
			__func__, PTR_ERR(s2mpg01_thermal->tz_dev));
		return -ENODEV;
	}

	platform_set_drvdata(pdev, s2mpg01_thermal);

	return 0;
}

static int s2mpg01_thermal_remove(struct platform_device *pdev)
{
	struct s2mpg01_thermal *s2mpg01_thermal = platform_get_drvdata(pdev);

	if (s2mpg01_thermal->tz_dev)
		thermal_zone_device_unregister(s2mpg01_thermal->tz_dev);
	return 0;
}

static const struct of_device_id s2mpg01_thermal_of_match[] = {
	{ .compatible = "samsung,s2mpg01-thermal", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg01_thermal_of_match);

static struct platform_driver s2mpg01_thermal_driver = {
	.probe = s2mpg01_thermal_probe,
	.remove = s2mpg01_thermal_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = s2mpg01_thermal_of_match,
	},
};
module_platform_driver(s2mpg01_thermal_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPG01 Thermal Sensor Driver");
MODULE_LICENSE("GPL");
