/*
 *  vd6281_module.c - Linux kernel module for rainbow sensor
 *
 *  Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <media/cam_sensor.h>
#include "cam_sensor_dev.h"

#define VD6281_DEV_NAME	"vd6281"

#define VIO_VOLTAGE_MIN 1800000
#define VIO_VOLTAGE_MAX 1800000
#define VDD_VOlTAGE_MIN 1900000
#define VDD_VOlTAGE_MAX 1900000

enum RAINBOW_POWER {
	REGULATOR_VIO,
	REGULATOR_VDD,
	POWER_MAX
};

struct rainbow_ctrl_t {
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	struct camera_io_master io_master_info;
	struct regulator *vdd;
	struct regulator *vio;
	dev_t dev;
	bool is_cci_init;
	bool is_power_up[POWER_MAX];
	bool is_probed;
	uint32_t read_addr;
	uint32_t read_data;
	struct cdev c_dev;
	struct class *cl;
};

static struct rainbow_ctrl_t *ctrl;
static const struct file_operations vd6281_fops;

static int vd6281_write_data(
	struct rainbow_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array i2c_reg_array;

	i2c_reg_array.reg_addr = addr;
	i2c_reg_array.reg_data = data;
	i2c_reg_array.delay = 1;
	write_setting.reg_setting = &i2c_reg_array;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&(ctrl->io_master_info), &write_setting);

	if (rc != 0)
		dev_err(ctrl->soc_info.dev, "%s failed rc = %d", __func__, rc);
	else
		dev_dbg(ctrl->soc_info.dev,
			"%s: set data 0x%x to 0x%x rc %d",
			__func__,
			data,
			addr,
			rc);

	return rc;
}

static int vd6281_read_data(
	struct rainbow_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t *data)
{
	int rc;
	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		addr,
		data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc != 0)
		dev_err(ctrl->soc_info.dev, "%s failed addr 0x%x rc = %d",
		 __func__, addr, rc);
	else
		dev_dbg(ctrl->soc_info.dev,
			"%s: got data 0x%x from 0x%x rc %d",
			__func__,
			*data,
			addr,
			rc);

	return rc;
}

static int vd6281_power_up(struct rainbow_ctrl_t *ctrl)
{
	int rc;

	if (!ctrl->is_power_up[REGULATOR_VDD]) {
		rc = regulator_set_voltage(ctrl->vdd,
			VDD_VOlTAGE_MIN, VDD_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set vdd voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->vdd);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"%s vdd regulator_enable failed: rc: %d",
				__func__, rc);
			return rc;
		}
		ctrl->is_power_up[REGULATOR_VDD] = true;
	}

	if (!ctrl->is_power_up[REGULATOR_VIO]) {
		rc = regulator_set_voltage(ctrl->vio,
			VIO_VOLTAGE_MIN, VIO_VOLTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set vio voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->vio);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"%s vio regulator_enable failed: rc: %d",
				__func__, rc);
			return rc;
		}
		ctrl->is_power_up[REGULATOR_VIO] = true;
	}

	/* At least 1 ms delay after power up */
	usleep_range(1000, 3000);

	if (!ctrl->is_cci_init) {
		rc = camera_io_init(&(ctrl->io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"%s cci_init failed: rc: %d", __func__, rc);
			return rc;
		}
		ctrl->is_cci_init = true;
	}

	return rc;
}

static int vd6281_power_down(struct rainbow_ctrl_t *ctrl)
{
	int rc = 0, is_error;

	if (ctrl->is_cci_init) {
		is_error = camera_io_release(&(ctrl->io_master_info));
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"%s cci_release failed: rc: %d", __func__, rc);
		} else
			ctrl->is_cci_init = false;
	}

	if (ctrl->is_power_up[REGULATOR_VDD]) {
		is_error = regulator_disable(ctrl->vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"%s vdd regulator_disable failed: rc: %d",
				__func__, rc);
		} else
			ctrl->is_power_up[REGULATOR_VDD] = false;
	}

	if (ctrl->is_power_up[REGULATOR_VIO]) {
		is_error = regulator_disable(ctrl->vio);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"%s vio regulator_disable failed: rc: %d",
				__func__, rc);
		} else
			ctrl->is_power_up[REGULATOR_VIO] = false;
	}

	return rc;
}

static ssize_t rainbow_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int is_enabled;

	mutex_lock(&ctrl->cam_sensor_mutex);
	is_enabled = (ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_cci_init);
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", is_enabled);
}

static ssize_t rainbow_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0;
	bool value = 0;

	rc = kstrtobool(buf, &value);
	if (rc)
		return rc;

	mutex_lock(&ctrl->cam_sensor_mutex);

	if (value == true)
		rc = vd6281_power_up(ctrl);
	else
		rc = vd6281_power_down(ctrl);

	mutex_unlock(&ctrl->cam_sensor_mutex);

	return rc < 0 ? rc : count;
}


static ssize_t rainbow_read_byte_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%x\n", ctrl->read_data);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t rainbow_read_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t read_data = 0;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!(ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_power_up[REGULATOR_VIO] &&
		ctrl->is_cci_init)) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);
	if (rc)
		goto error_out;

	value &= 0xFF;

	rc = vd6281_read_data(ctrl, value, &read_data);

	if (rc)
		goto error_out;
	else {
		ctrl->read_addr = value;
		ctrl->read_data = read_data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t rainbow_write_byte_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!(ctrl->is_power_up[REGULATOR_VDD] &&
		ctrl->is_power_up[REGULATOR_VIO] &&
		ctrl->is_cci_init)) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);
	if (rc)
		goto error_out;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = vd6281_write_data(ctrl, addr, data);
	if (rc < 0)
		goto error_out;
	else {
		if (addr == ctrl->read_addr)
			ctrl->read_data = data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static DEVICE_ATTR_RW(rainbow_enable);
static DEVICE_ATTR_RW(rainbow_read_byte);
static DEVICE_ATTR_WO(rainbow_write_byte);

static struct attribute *rainbow_dev_attrs[] = {
	&dev_attr_rainbow_enable.attr,
	&dev_attr_rainbow_read_byte.attr,
	&dev_attr_rainbow_write_byte.attr,
	NULL
};

ATTRIBUTE_GROUPS(rainbow_dev);

int32_t vd6281_update_i2c_info(struct device *dev)
{
	int32_t value = 0;
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (of_property_read_u32(dev->of_node, "cci-master", &value)) {
		dev_err(dev, "cci master not specified in dt");
		return -EINVAL;
	} else
		ctrl->io_master_info.cci_client->cci_i2c_master = value;

	if (of_property_read_u32(dev->of_node, "reg", &value)) {
		dev_err(dev, "slave address is not specified in dt");
		return -EINVAL;

	} else
		ctrl->io_master_info.cci_client->sid = value;

	if (of_property_read_u32(dev->of_node, "cci-device", &value) ||
		value >= CCI_DEVICE_MAX) {
		dev_err(dev, "cci device is not specified in dt");
		return -EINVAL;

	} else
		ctrl->io_master_info.cci_client->cci_device = value;

	ctrl->io_master_info.cci_client->retries = 3;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;

	return 0;
}

static int vd6281_parse_dt(struct device *dev)
{
	struct rainbow_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc = 0;

	ctrl->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ctrl->vdd)) {
		ctrl->vdd = NULL;
		dev_err(dev, "unable to get vdd\n");
		rc = -ENOENT;
	}

	ctrl->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(ctrl->vio)) {
		ctrl->vio = NULL;
		dev_err(dev, "unable to get vio\n");
		rc = -ENOENT;
	}

	return rc;
}

static int32_t vd6281_platform_remove(struct platform_device *pdev)
{
	struct rainbow_ctrl_t  *ctrl;
	int32_t rc = 0;

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		pr_err("rainbow device is NULL");
		return 0;
	}
	if (!ctrl->is_probed)
		return 0;

	device_destroy(ctrl->cl, ctrl->dev);
	class_destroy(ctrl->cl);
	cdev_del(&ctrl->c_dev);
	unregister_chrdev_region(ctrl->dev, 1);
	sysfs_remove_groups(&pdev->dev.kobj, rainbow_dev_groups);
	vd6281_power_down(ctrl);
	mutex_destroy(&ctrl->cam_sensor_mutex);

	return rc;
}

static int32_t vd6281_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc, device_id = 0;
	struct device *dev_ret;

	if (cam_cci_get_subdev(CCI_DEVICE_0) == NULL ||
		cam_cci_get_subdev(CCI_DEVICE_1) == NULL) {
		dev_warn(&pdev->dev, "wait for cci driver probe");
		return -EPROBE_DEFER;
	}

	/* Create sensor control structure */
	ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct rainbow_ctrl_t), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	/*fill in platform device*/
	ctrl->soc_info.pdev = pdev;
	ctrl->soc_info.dev = &pdev->dev;
	ctrl->soc_info.dev_name = pdev->name;
	ctrl->io_master_info.master_type = CCI_MASTER;
	ctrl->is_power_up[REGULATOR_VIO] = false;
	ctrl->is_power_up[REGULATOR_VDD] = false;
	ctrl->is_cci_init = false;
	ctrl->is_probed = false;

	ctrl->io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->io_master_info.cci_client))
		return -ENOMEM;

	platform_set_drvdata(pdev, ctrl);
	dev_set_drvdata(&pdev->dev, ctrl);

	rc = vd6281_parse_dt(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "paring rainbow dt failed rc %d", rc);
		return rc;
	}

	rc = vd6281_update_i2c_info(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "update rainbow i2c info failed rc %d",
			rc);
		return rc;
	}

	/* Fill platform device id*/
	pdev->id = ctrl->soc_info.index;

	mutex_init(&ctrl->cam_sensor_mutex);

	rc = sysfs_create_groups(&pdev->dev.kobj, rainbow_dev_groups);
	if (rc) {
		dev_err(&pdev->dev, "failed to create sysfs files");
		goto error_destroy_mutex;
	}

	rc = alloc_chrdev_region(&ctrl->dev, 0, 1, "vd6281_ioctl");
	if (rc)
		goto error_remove_sysfs;

	cdev_init(&ctrl->c_dev, &vd6281_fops);

	rc = cdev_add(&ctrl->c_dev, ctrl->dev, 1);
	if (rc)
		goto error_unregister_chrdev;

	ctrl->cl = class_create(THIS_MODULE, "char");
	rc = IS_ERR(ctrl->cl);
	if (rc)
		goto error_del_cdev;

	dev_ret = device_create(ctrl->cl, NULL, ctrl->dev, NULL, "vd6281");
	rc = IS_ERR(dev_ret);
	if (rc)
		goto error_destroy_class;

	/* Read device id */
	rc = vd6281_power_up(ctrl);
	if (rc != 0) {
		vd6281_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = vd6281_read_data(ctrl, 0x00, &device_id);
	if (rc != 0) {
		vd6281_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = vd6281_power_down(ctrl);
	if (rc != 0)
		goto error_destroy_device;

	_dev_info(&pdev->dev, "probe success, device id 0x%x rc = %d",
		device_id, rc);
	ctrl->is_probed = true;

	return rc;

error_destroy_device:
	device_destroy(ctrl->cl, ctrl->dev);
error_destroy_class:
	class_destroy(ctrl->cl);
error_del_cdev:
	cdev_del(&ctrl->c_dev);
error_unregister_chrdev:
	unregister_chrdev_region(ctrl->dev, 1);
error_remove_sysfs:
	sysfs_remove_groups(&pdev->dev.kobj, rainbow_dev_groups);
error_destroy_mutex:
	mutex_destroy(&ctrl->cam_sensor_mutex);
	return rc;
}

static const struct of_device_id vd6281_driver_dt_match[] = {
	{.compatible = "st,rainbow"},
	{}
};

MODULE_DEVICE_TABLE(of, vd6281_driver_dt_match);

static struct platform_driver vd6281_platform_driver = {
	.probe = vd6281_driver_platform_probe,
	.driver = {
		.name = VD6281_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vd6281_driver_dt_match,
	},
	.remove = vd6281_platform_remove,
};

static int vd6281_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int vd6281_release(struct inode *inode, struct file *file)
{
	vd6281_power_down(ctrl);
	return 0;
}

static long vd6281_ioctl_handler(struct file *file, unsigned int cmd,
				 unsigned long arg, void __user *p)
{
	int rc = -EINVAL;
	int i;
	struct rainbow_config config;

	mutex_lock(&ctrl->cam_sensor_mutex);

	switch (cmd) {
	case RAINBOW_CONFIG:
		if (copy_from_user(&config, p, sizeof(config))) {
			rc = -EFAULT;
			goto out;
		}

		if (config.size == 0 ||
			config.size > MAX_RAINBOW_CONFIG_SIZE) {
			rc = -EFAULT;
			goto out;
		}

		if (config.operation == RAINBOW_RANDOM_READ) {
			for (i = 0; i < config.size; i++) {
				rc = camera_io_dev_read(
					&(ctrl->io_master_info),
					config.reg_addr[i] & 0xFF,
					&config.reg_data[i],
					CAMERA_SENSOR_I2C_TYPE_BYTE,
					CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc) {
					dev_err(ctrl->soc_info.dev,
						"%s i2c random read failed,"
						"  addr 0x%x rc %d\n",
						__func__,
						config.reg_addr[i] & 0xFF,
						rc);
					goto out;
				}
			}
			rc = copy_to_user(p, &config, sizeof(config));
		} else if (config.operation == RAINBOW_SEQ_READ) {
			uint8_t data[MAX_RAINBOW_CONFIG_SIZE];

			rc = camera_io_dev_read_seq(
				&(ctrl->io_master_info),
				config.reg_addr[0] & 0xFF,
				data,
				CAMERA_SENSOR_I2C_TYPE_BYTE,
				CAMERA_SENSOR_I2C_TYPE_BYTE,
				config.size);
			if (rc) {
				dev_err(ctrl->soc_info.dev,
					"%s i2c cont read failed,"
					" addr 0x%x rc %d\n",
					__func__,
					config.reg_addr[0] & 0xFF,
					rc);
				goto out;
			}
			for (i = 0; i < config.size; i++)
				config.reg_data[i] = data[i];

			rc = copy_to_user(p, &config, sizeof(config));
		} else if (config.operation == RAINBOW_RANDOM_WRITE ||
				   config.operation == RAINBOW_SEQ_WRITE) {
			struct cam_sensor_i2c_reg_setting write_setting;
			struct cam_sensor_i2c_reg_array
				reg_setting[MAX_RAINBOW_CONFIG_SIZE] = { {0} };

			write_setting.reg_setting = reg_setting;
			write_setting.size = config.size;
			write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			write_setting.delay = 0;

			for (i = 0; i < config.size; i++) {
				reg_setting[i].reg_addr =
					config.reg_addr[0] & 0xFF;
				reg_setting[i].reg_data =
					config.reg_data[0] & 0xFF;
				reg_setting[i].delay = 0;
				reg_setting[i].data_mask = 0;
			}

			if (config.operation == RAINBOW_RANDOM_WRITE)
				rc = camera_io_dev_write(
					&(ctrl->io_master_info),
					&write_setting);
			else
				rc = camera_io_dev_write_continuous(
					&(ctrl->io_master_info),
					&write_setting, 0);

		} else
			dev_err(ctrl->soc_info.dev,
				"%s: Unsupported opertion type %d\n",
				__func__, config.operation);

		break;
	default:
		dev_err(ctrl->soc_info.dev,
			"%s: Unsupported ioctl command %u\n", __func__, cmd);
		break;
	}

out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static long vd6281_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return vd6281_ioctl_handler(file, cmd, arg, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long vd6281_compat_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return vd6281_ioctl_handler(file, cmd, arg, compat_ptr(arg));
}
#endif

static const struct file_operations vd6281_fops = {
	.owner		= THIS_MODULE,
	.open		= vd6281_open,
	.release	= vd6281_release,
	.unlocked_ioctl	= vd6281_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= vd6281_compat_ioctl,
#endif
};

static int __init vd6281_init(void)
{
	return platform_driver_register(&vd6281_platform_driver);
}

static void __exit vd6281_exit(void)
{
	platform_driver_unregister(&vd6281_platform_driver);
}

MODULE_DESCRIPTION("ST VD6281 rainbow sensor");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xu Han <xuhanyz@google.com>");
MODULE_AUTHOR("Speth Chang <spethchang@google.com>");

module_init(vd6281_init);
module_exit(vd6281_exit);