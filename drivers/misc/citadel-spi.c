/*
 * Citadel transport driver
 *
 * Copyright (C) 2017 Fernando Lugo <flugo@google.com>
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/citadel.h>
#include <linux/wait.h>

#define CITADEL_TPM_READ	0x80000000

#define MAX_DATA_SIZE		2044
#define CITADEL_MAX_DEVICES	4
#define CITADEL_TPM_TIMEOUT_MS	10

struct citadel_data {
	dev_t			devt;
	struct cdev		cdev;
	struct spi_device	*spi;
	int			ctdl_ap_irq;
	wait_queue_head_t	waitq;
	int			ctdl_rst;
	atomic_t		users;
	void			*tx_buf;
	void			*rx_buf;
};

static struct class *citadel_class;
static dev_t citadel_devt;

static int citadel_wait_cmd_done(struct citadel_data *citadel)
{
	struct spi_device *spi = citadel->spi;
	struct spi_message m;
	int ret;
	unsigned long to = jiffies + 1 +	/* at least one jiffy */
		msecs_to_jiffies(CITADEL_TPM_TIMEOUT_MS);
	struct spi_transfer spi_xfer = {
		.rx_buf = citadel->rx_buf,
		.len = 1,
		.cs_change = 1,
	};
	uint8_t *val = citadel->rx_buf;

	/*
	 * We have sent the initial four-byte command to Citadel on MOSI, and
	 * now we're waiting for bit0 of the MISO byte to be set, indicating
	 * that we can continue with the rest of the transaction. If that bit
	 * is not immediately set, we'll keep sending one more don't-care byte
	 * on MOSI just to read MISO until it is. If Citadel is awake and
	 * functioning correctly its hardware implementation should always
	 * return 0x00 while it's thinking and return 0x01 when ready to
	 * continue. However, there are a couple of ways this can fail. First,
	 * if Citadel is in deep sleep, it should send 0x5A on MISO until it
	 * wakes up. This may take ~40ms or so but it's not unexpected, so if
	 * we time out we just give up and try again. Second, if Citadel
	 * unexpectedly reboots, it will probably return 0xFF or 0xDF, which
	 * will exit the loop but we shouldn't continue the transaction because
	 * Citadel won't know what's going on.
	 */
	do {
		if (time_after(jiffies, to)) {
			dev_warn(&spi->dev, "Citadel SPI timed out\n");
			return -EBUSY;
		}
		spi_message_init(&m);
		spi_message_add_tail(&spi_xfer, &m);
		ret = spi_sync_locked(spi, &m);
		if (ret)
			return ret;
	} while (!*val);

	/* Return EAGAIN if unexpected bytes were received. */
	return *val & 0x01 ? 0 : -EAGAIN;
}

static int citadel_tpm_datagram(struct citadel_data *citadel,
			       struct citadel_ioc_tpm_datagram *dg)
{
	int is_read;
	int citadel_is_awake;
	int ret;
	int ignore_result = 0;
	struct spi_device *spi = citadel->spi;
	struct spi_message m;
	struct spi_transfer spi_xfer = {
		.tx_buf = citadel->tx_buf,
		.rx_buf = citadel->rx_buf,
		.len = 4,
		.cs_change = 1,
	};
	uint32_t *command = citadel->tx_buf;
	uint32_t *response = citadel->rx_buf;

	/* Read == from SPI, to userland. */
	is_read = dg->command & CITADEL_TPM_READ;

	/* Lock the SPI bus until we're completely done */
	spi_bus_lock(spi->master);

	/* The command must be big-endian */
	*command = cpu_to_be32(dg->command);

	/* Prepare to send the command */
	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);

	/* Send the command out */
	ret = spi_sync_locked(spi, &m);
	if (ret)
		goto exit;

	/* Verify that citadel is idle. If it isn't, deassert CS and return
	 * -EAGAIN. 0xdf is what citadel sends when the SPI FIFO is empty,
	 * and it is in the TPM wait mode. Once a command is sent to Citadel,
	 * the last bit shows whether or not Citadel is ready to send the
	 * response. This typically happens by a 0x01 byte, but may be
	 * preceded by several 0x00 bytes while Citadel does any necessary
	 * work. */
	citadel_is_awake = *response == be32_to_cpu(0xdfdfdfde);

	if (citadel_is_awake) {
		/* Wait for the reply bit to go high */
		ret = citadel_wait_cmd_done(citadel);
		/* Reset CS in the case of unexpected bytes. */
		if (ret)
			citadel_is_awake = 0;
	}

	/* TODO: If there's no data, how do we disable the CS? */
	if (!dg->len || !citadel_is_awake) {
		/* For now, just transfer one more byte and throw it away */
		is_read = 1;
		dg->len = 1;
		ignore_result = 1;
	}

	/* Now we can transfer the data */
	spi_xfer.cs_change = 0;
	spi_xfer.len = dg->len;
	if (is_read) {
		spi_xfer.rx_buf = citadel->rx_buf;
		spi_xfer.tx_buf = NULL;
	} else {
		spi_xfer.rx_buf = NULL;
		spi_xfer.tx_buf = citadel->tx_buf;

		if (copy_from_user(citadel->tx_buf,
				   (void __user *)dg->buf, dg->len)) {
			ret = -EFAULT;
			goto exit;
		}
	}

	spi_message_init(&m);
	spi_message_add_tail(&spi_xfer, &m);
	ret = spi_sync_locked(spi, &m);
	if (ret)
		goto exit;

	/* This condition typically happens when citadel is asleep. Toggling
	 * CS should be sufficient to wake up citadel, but some time needs to
	 * pass before it will be ready. */
	if (!citadel_is_awake)
		ret = -EAGAIN;

	if (ignore_result)
		goto exit;

	if (is_read && copy_to_user((void __user *)dg->buf,
				    citadel->rx_buf, dg->len))
		ret = -EFAULT;

exit:
	spi_bus_unlock(spi->master);

	return ret;
}

static int citadel_reset(struct citadel_data *citadel)
{
	/* Synchronize with the datagrams by locking the SPI bus */
	struct spi_device *spi = citadel->spi;
	spi_bus_lock(spi->master);

	/* Assert reset for at least 3ms after VDDIOM is stable; 10ms is safe */
	gpio_set_value(citadel->ctdl_rst, 1);
	msleep(10);

	/* Clear reset and wait for Citadel to become functional */
	gpio_set_value(citadel->ctdl_rst, 0);
	msleep(100);

	spi_bus_unlock(spi->master);
	return 0;
}

static long
citadel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	u32 tmp;
	struct citadel_data *citadel = filp->private_data;
	struct citadel_ioc_tpm_datagram dg;

	/* check magic */
	if (_IOC_TYPE(cmd) != CITADEL_IOC_MAGIC)
		return -ENOTTY;
	switch (cmd) {
	case CITADEL_IOC_TPM_DATAGRAM:
		tmp = _IOC_SIZE(cmd);
		if (tmp != sizeof(dg))
			return -EINVAL;

		if (copy_from_user(&dg, (void __user *)arg, sizeof(dg)))
			return -EFAULT;

		if (dg.len > MAX_DATA_SIZE)
			return -E2BIG;

		/* translate to spi_message, execute */
		return citadel_tpm_datagram(citadel, &dg);
	case CITADEL_IOC_RESET:
		return citadel_reset(citadel);
	}
	return -EINVAL;
}

static ssize_t
citadel_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int ret;
	size_t c = 0;
	size_t len;
	struct citadel_data *citadel = filp->private_data;

	while (count) {
		len = count > MAX_DATA_SIZE ? MAX_DATA_SIZE : count;
		ret = spi_read(citadel->spi, citadel->rx_buf, len);
		if (ret)
			return ret;
		ret = copy_to_user(buf + c, citadel->rx_buf, len);
		if (ret)
			return -EFAULT;
		c += len;
		count -= len;
	}

	return c;
}

static ssize_t
citadel_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	int ret;
	size_t c = 0;
	size_t len;
	struct citadel_data *citadel = filp->private_data;

	while (count) {
		len = count > MAX_DATA_SIZE ? MAX_DATA_SIZE : count;
		ret = copy_from_user(citadel->tx_buf, buf + c, len);
		if (ret)
			return -EFAULT;
		ret = spi_write(citadel->spi, citadel->tx_buf, len);
		if (ret)
			return ret;
		c += len;
		count -= len;
	}

	return c;
}

static int citadel_open(struct inode *inode, struct file *filp)
{
	struct citadel_data *citadel;

	citadel = container_of(inode->i_cdev, struct citadel_data, cdev);

	/* we only support 1 user at the same time */
	if (!atomic_add_unless(&citadel->users, 1, 1))
		return -EBUSY;

	filp->private_data = citadel;
	nonseekable_open(inode, filp);

	return 0;
}

static unsigned int citadel_poll(struct file *filp, poll_table *wait)
{
	struct citadel_data *citadel = filp->private_data;
	poll_wait(filp, &citadel->waitq, wait);
	return gpio_get_value(citadel->ctdl_ap_irq) ? POLLIN : 0;
}

static int citadel_release(struct inode *inode, struct file *filp)
{
	struct citadel_data *citadel = filp->private_data;

	atomic_dec(&citadel->users);

	return 0;
}

static const struct file_operations citadel_fops = {
	.owner =		THIS_MODULE,
	.write =		citadel_write,
	.read =			citadel_read,
	.open =			citadel_open,
	.poll =			citadel_poll,
	.release =		citadel_release,
	.unlocked_ioctl =	citadel_ioctl,
	.llseek =		no_llseek,
};

#ifdef CONFIG_OF
static const struct of_device_id citadel_dt_ids[] = {
	{ .compatible = "google,citadel" },
	{},
};
MODULE_DEVICE_TABLE(of, citadel_dt_ids);
#endif

static irqreturn_t citadel_irq_handler(int irq, void *handle)
{
	struct citadel_data *citadel = (struct citadel_data *)handle;
	wake_up_interruptible(&citadel->waitq);
	return IRQ_HANDLED;
}

static int citadel_request_named_gpio(struct citadel_data *citadel,
				      const char *label, int *gpio)
{
	struct device *dev = &citadel->spi->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);

	return 0;
}

static int citadel_probe(struct spi_device *spi)
{
	struct device *dev;
	struct cdev cdev;
	struct citadel_data *citadel;
	int ret;
	dev_t devt;
	u32 minor;

	/* use chip select as minor */
	minor = (u32)spi->chip_select;
	if (minor >= CITADEL_MAX_DEVICES) {
		dev_err(&spi->dev, "minor %u out of boundaries\n", minor);
		return -ENXIO;
	}

	citadel = kmalloc(sizeof(*citadel), GFP_KERNEL);
	if (!citadel)
		return -ENOMEM;

	/* init citadel structure */
	citadel->tx_buf = (void *)__get_free_pages(GFP_KERNEL,
						   get_order(MAX_DATA_SIZE));
	citadel->rx_buf = (void *)__get_free_pages(GFP_KERNEL,
						   get_order(MAX_DATA_SIZE));
	if (!citadel->tx_buf || !citadel->rx_buf) {
		ret = -ENOMEM;
		goto free_citadel;
	}
	init_waitqueue_head(&citadel->waitq);
	atomic_set(&citadel->users, 0);
	citadel->spi = spi;
	devt = MKDEV(MAJOR(citadel_devt), minor);
	citadel->devt = devt;
	spi_set_drvdata(spi, citadel);

	/* setup ctdl_ap_irq  */
	ret = citadel_request_named_gpio(citadel, "citadel,ctdl_ap_irq",
					 &citadel->ctdl_ap_irq);
	if (ret) {
		dev_err(&spi->dev,
			"citadel_request_named_gpio "
			"citadel,ctdl_ap_irq failed.\n");
		goto free_citadel;
	}

	ret = devm_request_irq(&citadel->spi->dev,
			       gpio_to_irq(citadel->ctdl_ap_irq),
			       citadel_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			       dev_name(&spi->dev),
			       citadel);
	if (ret) {
		dev_err(&spi->dev,
			"devm_request_irq  citadel,ctdl_ap_irq failed.\n");
		goto free_citadel;
	}

	enable_irq_wake(gpio_to_irq(citadel->ctdl_ap_irq));

	/* setup ctdl_rst */
	ret = citadel_request_named_gpio(citadel, "citadel,ctdl_rst",
					 &citadel->ctdl_rst);
	if (ret) {
		dev_err(&spi->dev,
			"citadel_request_named_gpio "
			"citadel,ctdl_rst failed.\n");
		goto free_citadel;
	}

	gpio_direction_output(citadel->ctdl_rst, 0);

	/* create the device */
	dev = device_create(citadel_class, &spi->dev, devt, NULL,
			    "citadel%u", minor);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto free_citadel;
	}

	cdev_init(&citadel->cdev, &citadel_fops);
	cdev.owner = THIS_MODULE;
	ret = cdev_add(&citadel->cdev, citadel_devt, 1);
	if (ret)
		goto destroy_device;

	return 0;

destroy_device:
	device_destroy(citadel_class, devt);
free_citadel:
	free_pages((unsigned long)citadel->rx_buf, get_order(MAX_DATA_SIZE));
	free_pages((unsigned long)citadel->tx_buf, get_order(MAX_DATA_SIZE));
	kfree(citadel);
	return ret;
}

static int citadel_remove(struct spi_device *spi)
{
	struct citadel_data *citadel = spi_get_drvdata(spi);

	cdev_del(&citadel->cdev);
	device_destroy(citadel_class, citadel->devt);
	free_pages((unsigned long)citadel->rx_buf, get_order(MAX_DATA_SIZE));
	free_pages((unsigned long)citadel->tx_buf, get_order(MAX_DATA_SIZE));
	kfree(citadel);

	return 0;
}

static struct spi_driver citadel_spi_driver = {
	.driver = {
		.name =		"citadel",
		.of_match_table = of_match_ptr(citadel_dt_ids),
	},
	.probe =	citadel_probe,
	.remove =	citadel_remove,
};

static int __init citadel_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&citadel_devt, 0, CITADEL_MAX_DEVICES,
				  "citadel");
	if (ret) {
		pr_err("%s: failed to alloc cdev region %d\n", __func__, ret);
		return ret;
	}

	citadel_class = class_create(THIS_MODULE, "citadel");
	if (IS_ERR(citadel_class)) {
		unregister_chrdev_region(citadel_devt, 1);
		return PTR_ERR(citadel_class);
	}

	ret = spi_register_driver(&citadel_spi_driver);
	if (ret < 0) {
		class_destroy(citadel_class);
		unregister_chrdev_region(citadel_devt, 1);
	}
	return ret;
}

static void __exit citadel_exit(void)
{
	spi_unregister_driver(&citadel_spi_driver);
	class_destroy(citadel_class);
	unregister_chrdev_region(citadel_devt, 1);
}

module_init(citadel_init);
module_exit(citadel_exit);

MODULE_AUTHOR("Fernando Lugo, <flugo@google.com>");
MODULE_DESCRIPTION("CITADEL TPM driver");
MODULE_LICENSE("GPL");
