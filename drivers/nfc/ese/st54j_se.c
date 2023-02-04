// SPDX-License-Identifier: GPL-2.0
/* drivers/nfc/ese/st54j_se.c
 * Copyright (C) 2018 ST Microelectronics S.A.
 * Copyright 2019 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

#include <linux/types.h>
#include <linux/version.h>

#include <linux/semaphore.h>
#include <linux/completion.h>

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi-geni-qcom.h>
#include <uapi/linux/st54j_se.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define DRIVER_VERSION "1.1.4"
#define ST54_MAX_BUF 258U

struct st54j_se_dev {
	struct spi_device	*spi;
	struct mutex		mutex;
	struct	miscdevice	device;
	bool device_open;
	/* GPIO for SE Reset pin (output) */
	struct gpio_desc *gpiod_se_reset;
	char *kbuf;
};

static long st54j_se_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int r = 0;
	struct st54j_se_dev *ese_dev = filp->private_data;
	mutex_lock(&ese_dev->mutex);
	dev_dbg(&ese_dev->spi->dev, "%s: enter, cmd=%u\n", __func__, cmd);

	switch (cmd) {
	case ST54J_SE_RESET:
		dev_info(&ese_dev->spi->dev, "%s  Reset Request received!!\n",
			 __func__);
		if (!IS_ERR(ese_dev->gpiod_se_reset)) {
			/* pulse low for 5 millisecs */
			gpiod_set_value(ese_dev->gpiod_se_reset, 0);
			usleep_range(5000, 5500);
			gpiod_set_value(ese_dev->gpiod_se_reset, 1);
			dev_info(&ese_dev->spi->dev,
				 "%s sent Reset request on eSE\n", __func__);
		}
		break;
	}
	mutex_unlock(&ese_dev->mutex);
	return r;
}

static int st54j_se_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st54j_se_dev *ese_dev = container_of(filp->private_data,
				struct st54j_se_dev, device);
	mutex_lock(&ese_dev->mutex);
	if (ese_dev->device_open) {
		ret = -EBUSY;
		dev_info(&ese_dev->spi->dev, "%s: device already opened\n",
			 __func__);
	} else {
		ese_dev->device_open = true;
		filp->private_data = ese_dev;
		dev_info(&ese_dev->spi->dev, "%s: device_open = %d", __func__,
			 ese_dev->device_open);
	}
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static int st54j_se_release(struct inode *ino, struct file *filp)
{
	struct st54j_se_dev *ese_dev = filp->private_data;

	mutex_lock(&ese_dev->mutex);
	ese_dev->device_open = false;
	mutex_unlock(&ese_dev->mutex);
	dev_dbg(&ese_dev->spi->dev, "%s : device_open  = %d\n",
		 __func__, ese_dev->device_open);
	return 0;
}

static ssize_t st54j_se_write(struct file *filp, const char __user *ubuf,
			      size_t len, loff_t *offset)
{
	struct st54j_se_dev *ese_dev = filp->private_data;
	int ret = -EFAULT;
	size_t bytes = len;
	char *tx_buf = NULL;

	if (len > INT_MAX)
		return -EINVAL;
	dev_dbg(&ese_dev->spi->dev, "%s : writing %zu bytes.\n", __func__,
		bytes);
	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < ST54_MAX_BUF ? bytes : ST54_MAX_BUF;

		tx_buf = ese_dev->kbuf;
		if (!tx_buf) {
			dev_err(&ese_dev->spi->dev, "kbuf NULL\n");
			ret = -ENOMEM;
			goto err;
		}
		if (copy_from_user(tx_buf, ubuf, block)) {
			dev_dbg(&ese_dev->spi->dev,
				"failed to copy from user\n");
			goto err;
		}

		ret = spi_write(ese_dev->spi, tx_buf, block);

		if (ret < 0) {
			dev_dbg(&ese_dev->spi->dev, "failed to write to SPI\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static ssize_t st54j_se_read(struct file *filp, char __user *ubuf, size_t len,
			     loff_t *offset)
{
	struct st54j_se_dev *ese_dev = filp->private_data;
	ssize_t ret = -EFAULT;
	size_t bytes = len;
	char *rx_buf = NULL;

	if (len > INT_MAX)
		return -EINVAL;
	dev_dbg(&ese_dev->spi->dev, "%s : reading %zu bytes.\n", __func__,
		bytes);
	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < ST54_MAX_BUF ? bytes : ST54_MAX_BUF;

		rx_buf = ese_dev->kbuf;
		if (!rx_buf) {
			dev_err(&ese_dev->spi->dev, "kbuf NULL\n");
			ret = -ENOMEM;
			goto err;
		}

		memset(rx_buf, 0, ST54_MAX_BUF);
		ret = spi_read(ese_dev->spi, rx_buf, block);
		if (ret < 0) {
			dev_err(&ese_dev->spi->dev,
				"failed to read from SPI\n");
			goto err;
		}
		if (copy_to_user(ubuf, rx_buf, block)) {
			dev_err(&ese_dev->spi->dev,
				"failed to copy from user\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static const struct file_operations st54j_se_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st54j_se_read,
	.write = st54j_se_write,
	.open = st54j_se_open,
	.release = st54j_se_release,
	.unlocked_ioctl = st54j_se_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = st54j_se_ioctl
#endif
};

static int st54j_se_probe(struct spi_device *spi)
{
	struct st54j_se_dev *ese_dev;
	struct spi_geni_qcom_ctrl_data *spi_param;
	struct device *dev = &spi->dev;
	struct device_node *np = dev_of_node(&spi->dev);
	int ret;

	dev_dbg(dev, "%s entry\n", __func__);

	if (!np) {
		dev_err(dev, "%s: device tree data missing\n", __func__);
		return -EINVAL;
	}

	ese_dev = devm_kzalloc(dev, sizeof(*ese_dev), GFP_KERNEL);
	if (ese_dev == NULL)
		return -ENOMEM;

	spi_param = devm_kzalloc(dev, sizeof(spi_param), GFP_KERNEL);
	if (spi_param == NULL) {
		return -ENOMEM;
	}

	ese_dev->kbuf = devm_kzalloc(dev, ST54_MAX_BUF, GFP_KERNEL|GFP_DMA);
	if (ese_dev->kbuf == NULL)
		return -ENOMEM;

	ese_dev->spi = spi;
	ese_dev->device.minor = MISC_DYNAMIC_MINOR;
	ese_dev->device.name = "st54j_se";
	ese_dev->device.fops = &st54j_se_dev_fops;

	spi->bits_per_word = 8;
	spi_param->spi_cs_clk_delay = 90;
	spi->controller_data = spi_param;

	ese_dev->gpiod_se_reset = devm_gpiod_get(dev, "esereset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(ese_dev->gpiod_se_reset)) {
		dev_err(dev,
			"%s : Unable to request esereset %d \n",
			__func__, IS_ERR(ese_dev->gpiod_se_reset));
		return -ENODEV;
	}

	mutex_init(&ese_dev->mutex);
	ret = misc_register(&ese_dev->device);
	if (ret) {
		dev_err(dev, "%s: misc_register failed\n", __func__);
		goto err;
	}
	dev_dbg(dev, "%s: eSE is configured\n", __func__);
	spi_set_drvdata(spi, ese_dev);

	return 0;
err:
	mutex_destroy(&ese_dev->mutex);
	return ret;
}

static int st54j_se_remove(struct spi_device *spi)
{
	struct st54j_se_dev *ese_dev = spi_get_drvdata(spi);
	struct spi_geni_qcom_ctrl_data *spi_param;
	int ret = 0;
	spi_param = ese_dev->spi->controller_data;

	if (!ese_dev) {
		dev_err(&spi->dev, "%s: device doesn't exist anymore\n",
			__func__);
		ret = -ENODEV;
		goto err;
	}
	misc_deregister(&ese_dev->device);
	mutex_destroy(&ese_dev->mutex);
	kfree(spi_param);
	kfree(ese_dev);
err:
	return ret;
}

static const struct of_device_id st54j_se_match_table[] = {
	{ .compatible = "st,st54j_se" },
	{}
};
MODULE_DEVICE_TABLE(of, st54j_se_match_table);

static struct spi_driver st54j_se_driver = {
	.driver = {
		.name = "st54j_se",
		.of_match_table = st54j_se_match_table,
	},
	.probe = st54j_se_probe,
	.remove = st54j_se_remove,
};
module_spi_driver(st54j_se_driver);

MODULE_DESCRIPTION("ST54J eSE driver");
MODULE_ALIAS("spi:st54j_se");
MODULE_AUTHOR("ST Microelectronics");
MODULE_LICENSE("GPL");
