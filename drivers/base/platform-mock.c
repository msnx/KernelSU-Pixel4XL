// SPDX-License-Identifier: GPL-2.0
/*
 * Fake platform device API for unit testing platform drivers.
 *
 * Copyright (C) 2018, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <linux/platform_device_mock.h>
#include <linux/of_platform.h>

struct device_node *of_fake_node(struct test *test, const char *name)
{
	struct device_node *node;

	node = test_kzalloc(test, sizeof(*node), GFP_KERNEL);
	if (!node)
		return NULL;

	of_node_init(node);

	return node;
}

struct platform_device *
of_fake_probe_platform(struct test *test,
		       const struct platform_driver *driver,
		       const char *node_name)
{
	struct platform_device *pdev;
	struct device_node *of_node;
	int ret;

	of_node = of_fake_node(test, node_name);
	if (!of_node)
		return ERR_PTR(-ENOMEM);

	test_info(test, "Creating device");
	pdev = of_platform_device_create(of_node, node_name, NULL);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	test_info(test, "Probing");
	ret = driver->probe(pdev);
	if (ret)
		return ERR_PTR(ret);

	return pdev;
}

struct platform_device *of_fake_probe_platform_by_name(struct test *test,
						       const char *driver_name,
						       const char *node_name)
{
	const struct device_driver *driver;

	test_info(test, "Locating driver by name");
	driver = driver_find(driver_name, &platform_bus_type);
	if (!driver)
		return ERR_PTR(-ENODEV);

	return of_fake_probe_platform(test,
				      to_platform_driver(driver),
				      node_name);
}
