/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors: Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * Airbrush State Manager driver (DUMMY).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/airbrush-sm-ctrl.h>

static int airbrush_sm_probe(struct platform_device *pdev)
{
	return ab_sm_init(pdev);
}

static int airbrush_sm_remove(struct platform_device *pdev)
{
	ab_sm_exit(pdev);
	return 0;
}

static const struct of_device_id airbrush_sm_of_match[] = {
	{ .compatible = "abc,airbrush-sm" },
	{ },
};
MODULE_DEVICE_TABLE(of, airbrush_sm_of_match);

static struct platform_driver airbrush_sm_driver = {
	.driver = {
		.name		= "abc,airbrush-sm",
		.of_match_table	= airbrush_sm_of_match,
	},
	.probe	= airbrush_sm_probe,
	.remove	= airbrush_sm_remove,
};
module_platform_driver(airbrush_sm_driver);

MODULE_DESCRIPTION("Airbrush State Manager driver");
MODULE_LICENSE("GPL v2");
