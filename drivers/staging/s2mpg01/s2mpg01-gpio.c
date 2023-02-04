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
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "s2mpg01-core.h"

#define DRIVER_NAME "s2mpg01-gpio"

struct s2mpg01_gpio {
	struct gpio_chip gpio_chip;
	struct s2mpg01_core *s2mpg01_core;
	struct device *dev;
	unsigned long int pin_used;
};

static inline struct s2mpg01_gpio *to_s2mpg01_gpio
	(struct gpio_chip *_gpio_chip)
{
	return container_of(_gpio_chip, struct s2mpg01_gpio, gpio_chip);
}

static int s2mpg01_gpio_request(struct gpio_chip *gpio_chip,
				unsigned int offset)
{
	struct s2mpg01_gpio *s2mpg01_gpio = to_s2mpg01_gpio(gpio_chip);

	if (test_and_set_bit(offset, &s2mpg01_gpio->pin_used))
		return -EBUSY;
	else
		return 0;
}

static void s2mpg01_gpio_free(struct gpio_chip *gpio_chip,
			      unsigned int offset)
{
	struct s2mpg01_gpio *s2mpg01_gpio = to_s2mpg01_gpio(gpio_chip);

	clear_bit(offset, &s2mpg01_gpio->pin_used);
}

static int s2mpg01_gpio_direction_input(struct gpio_chip *gpio_chip,
					unsigned int offset)
{
	/* pins are hard-coded as outputs */
	return -EIO;
}

static int s2mpg01_gpio_get(struct gpio_chip *gpio_chip, unsigned int offset)
{
	struct s2mpg01_gpio *s2mpg01_gpio = to_s2mpg01_gpio(gpio_chip);
	struct s2mpg01_core *s2mpg01_core = s2mpg01_gpio->s2mpg01_core;
	u8 byte;

	s2mpg01_read_byte(s2mpg01_core, S2MPG01_REG_GPIO_Y, &byte);
	return byte & (1 << offset);
}

static void s2mpg01_gpio_set(struct gpio_chip *gpio_chip, unsigned int offset,
			     int value)
{
	struct s2mpg01_gpio *s2mpg01_gpio = to_s2mpg01_gpio(gpio_chip);
	struct s2mpg01_core *s2mpg01_core = s2mpg01_gpio->s2mpg01_core;

	/* Set output value */
	s2mpg01_update_bits(s2mpg01_core, S2MPG01_REG_GPIO_A,
			    (1 << offset), ((value ? 1 : 0) << offset));
	/* Enable output */
	s2mpg01_update_bits(s2mpg01_core, S2MPG01_REG_GPIO_CTRL,
			    (0x10 << offset), (0x10 << offset));
}

static int s2mpg01_gpio_direction_output(struct gpio_chip *gpio_chip,
					 unsigned int offset, int value)
{
	struct s2mpg01_gpio *s2mpg01_gpio = to_s2mpg01_gpio(gpio_chip);
	struct s2mpg01_core *s2mpg01_core = s2mpg01_gpio->s2mpg01_core;
	/* pins are already configured as outputs */
	s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_GPIO_CTRL, 0xF0);
	return 0;
}

static const struct gpio_chip s2mpg01_gpio_chip = {
	.label			= DRIVER_NAME,
	.owner			= THIS_MODULE,
	.request		= s2mpg01_gpio_request,
	.free			= s2mpg01_gpio_free,
	.direction_input	= s2mpg01_gpio_direction_input,
	.get			= s2mpg01_gpio_get,
	.direction_output	= s2mpg01_gpio_direction_output,
	.set			= s2mpg01_gpio_set,
	.base			= -1,
	.ngpio			= S2MPG01_NUM_GPIOS,
	.can_sleep		= 1,
};

static int s2mpg01_gpio_probe(struct platform_device *pdev)
{
	struct s2mpg01_core *s2mpg01_core = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg01_gpio *s2mpg01_gpio;

	s2mpg01_gpio = devm_kzalloc(&pdev->dev, sizeof(*s2mpg01_gpio),
				    GFP_KERNEL);
	if (!s2mpg01_gpio)
		return -ENOMEM;

	s2mpg01_gpio->s2mpg01_core = s2mpg01_core;
	s2mpg01_gpio->gpio_chip = s2mpg01_gpio_chip;
	s2mpg01_gpio->gpio_chip.parent = &pdev->dev;
	s2mpg01_gpio->dev = &pdev->dev;

	platform_set_drvdata(pdev, s2mpg01_gpio);

	/* enable both GPIOs for output with no-pull */
	s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_GPIO_A, 0x00);
	s2mpg01_write_byte(s2mpg01_core, S2MPG01_REG_GPIO_CTRL, 0xC0);

	return gpiochip_add(&s2mpg01_gpio->gpio_chip);
}

static int s2mpg01_gpio_remove(struct platform_device *pdev)
{
	struct s2mpg01_gpio *s2mpg01_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&s2mpg01_gpio->gpio_chip);
	return 0;
}

static const struct of_device_id s2mpg01_gpio_of_match[] = {
	{ .compatible = "samsung,s2mpg01-gpio", },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg01_gpio_of_match);

static struct platform_driver s2mpg01_gpio_driver = {
	.probe = s2mpg01_gpio_probe,
	.remove = s2mpg01_gpio_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = s2mpg01_gpio_of_match,
	},
};
module_platform_driver(s2mpg01_gpio_driver);

MODULE_AUTHOR("Trevor Bunker <trevorbunker@google.com>");
MODULE_DESCRIPTION("S2MPG01 GPIO Driver");
MODULE_LICENSE("GPL");
