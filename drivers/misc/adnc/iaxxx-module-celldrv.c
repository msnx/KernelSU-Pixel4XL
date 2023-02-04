/*
 * iaxxx-module-celldrv.c -- IAxxx module cell driver
 *
 * Copyright 2017 Knowles Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */
#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/adnc/iaxxx-module.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-static-mem.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/module.h>

#include "iaxxx-module-celldrv.h"

static struct module_cell_params module_cell_priv;
/* 16KB block is reserved at boot-time for odsp ioctls' use
 */
#define IAXXX_MODULE_STATIC_MEM_BLK_SIZE (16 * 1024)

static long module_dev_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct module_device_priv *module_dev_priv = file->private_data;
	struct iaxxx_priv *priv = to_iaxxx_priv(module_dev_priv->parent);
	struct iaxxx_sensor_info sensor_info;
	struct iaxxx_script_info script_info;
	struct iaxxx_sensor_param param_info;
	struct iaxxx_sensor_param_blk param_blk_info;
	struct iaxxx_pwr_stats pwr_stats_count;
	struct iaxxx_osc_trim_period osc_trim_period;
	struct iaxxx_sensor_mode_stats sensor_stats[SENSOR_NUM_MODE];
	uint16_t script_id;
	void __user *blk_buff = NULL;
	uint32_t *set_param_blk_buf = NULL;
	int ret = -EINVAL;

	pr_debug("%s() cmd %d\n", __func__, cmd);

	if (!priv)
		return -EINVAL;

	if (!test_bit(IAXXX_FLG_FW_READY, &priv->flags)) {
		dev_err(priv->dev, "%s FW  is not in App mode\n", __func__);
		return -EIO;
	}

	if (!priv->iaxxx_state) {
		dev_err(priv->dev, "Chip state NULL\n");
		return -EINVAL;
	}

	memset(&sensor_stats, 0, sizeof(sensor_stats));

	switch (cmd) {
	case SCRIPT_LOAD:
		if (copy_from_user(&script_info, (void __user *)arg,
					sizeof(script_info)))
			return -EFAULT;

		/* validate the scrip load parameters */
		if (!iaxxx_core_sensor_is_valid_script_id(
						script_info.script_id)) {
			pr_err("invalid scrip parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_script_load(module_dev_priv->parent,
				script_info.script_name, script_info.script_id);
		mutex_unlock(&priv->module_lock);
		break;
	case SCRIPT_UNLOAD:
		if (copy_from_user(&script_id, (void __user *)arg,
					sizeof(uint16_t)))
			return -EFAULT;

		/* validate the scrip parameters */
		if (!iaxxx_core_sensor_is_valid_script_id(script_id)) {
			pr_err("invalid scrip parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_script_unload(
				module_dev_priv->parent, script_id);
		mutex_unlock(&priv->module_lock);
		break;
	case SCRIPT_TRIGGER:
		if (copy_from_user(&script_id, (void __user *)arg,
					sizeof(uint16_t)))
			return -EFAULT;

		/* validate the scrip parameters */
		if (!iaxxx_core_sensor_is_valid_script_id(script_id)) {
			pr_err("invalid scrip parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_script_trigger(
				module_dev_priv->parent, script_id);
		mutex_unlock(&priv->module_lock);
		break;
	case MODULE_SENSOR_ENABLE:
		if (copy_from_user(&sensor_info, (void __user *)arg,
				   sizeof(sensor_info)))
			return -EFAULT;

		/* validate the sensor parameters */
		if (!(iaxxx_core_sensor_is_valid_inst_id(sensor_info.inst_id)
			&& iaxxx_core_sensor_is_valid_block_id(
						sensor_info.block_id))
			) {
			pr_err("invalid sensor parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_sensor_change_state(
				module_dev_priv->parent,
				sensor_info.inst_id, 1, sensor_info.block_id);
		mutex_unlock(&priv->module_lock);
		break;
	case MODULE_SENSOR_DISABLE:
		if (copy_from_user(&sensor_info, (void __user *)arg,
				   sizeof(sensor_info)))
			return -EFAULT;

		/* validate the sensor parameters */
		if (!(iaxxx_core_sensor_is_valid_inst_id(sensor_info.inst_id)
			&& iaxxx_core_sensor_is_valid_block_id(
						sensor_info.block_id))
			) {
			pr_err("invalid sensor parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_sensor_change_state(
				module_dev_priv->parent,
				sensor_info.inst_id, 0, sensor_info.block_id);
		mutex_unlock(&priv->module_lock);
		break;
	case MODULE_SENSOR_SET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
				   sizeof(param_info)))
			return -EFAULT;
		/* validate the sensor parameters */
		if (!(iaxxx_core_sensor_is_valid_block_id(
							param_info.block_id)
			&& iaxxx_core_sensor_is_valid_inst_id(
							param_info.inst_id)
			&& iaxxx_core_sensor_is_valid_param_id(
							param_info.param_id)
			&& iaxxx_core_sensor_is_valid_param_val(
							param_info.param_val))
			) {
			pr_err("invalid sensor parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_sensor_set_param_by_inst(
				module_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				param_info.param_val, param_info.block_id);
		mutex_unlock(&priv->module_lock);
		if (ret) {
			pr_err("%s() Sensor set param fail\n", __func__);
			pr_err("Param info inst 0x%x p_id 0x%x p_val 0x%x\n",
					param_info.inst_id, param_info.param_id,
					param_info.param_val);
			return ret;
		}
		break;
	case MODULE_SENSOR_GET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
				   sizeof(param_info)))
			return -EFAULT;
		/* validate the sensor parameters */
		if (!(iaxxx_core_sensor_is_valid_block_id(
							param_info.block_id)
			&& iaxxx_core_sensor_is_valid_inst_id(
							param_info.inst_id)
			&& iaxxx_core_sensor_is_valid_param_id(
							param_info.param_id))
			) {
			pr_err("invalid sensor parameter received\n");
			return -EINVAL;
		}
		mutex_lock(&priv->module_lock);
		ret = iaxxx_core_sensor_get_param_by_inst(
				module_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				&param_info.param_val, param_info.block_id);
		mutex_unlock(&priv->module_lock);
		if (ret) {
			pr_err("%s() Sensor get param fail\n", __func__);
			pr_err("Param info inst 0x%x p_id 0x%x\n",
				param_info.inst_id, param_info.param_id);
			return ret;
		}
		/* After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &param_info,
				 sizeof(param_info)))
			return -EFAULT;

		break;
	case MODULE_SENSOR_WRITE_PARAM_BLK:
		if (copy_from_user(&param_blk_info, (void __user *)arg,
				sizeof(struct iaxxx_sensor_param_blk)))
			return -EFAULT;

		/* validate the sensor parameters */
		if (!(iaxxx_core_sensor_is_valid_inst_id(
				param_blk_info.inst_id) &&
			iaxxx_core_sensor_is_valid_block_id(
				param_blk_info.block_id) &&
			iaxxx_core_sensor_is_valid_param_blk_id(
				param_blk_info.param_blk_id) &&
			iaxxx_core_sensor_is_valid_param_blk_size(
				param_blk_info.blk_size))) {
			pr_err("invalid sensor parameter received\n");
			return -EINVAL;
		}

		if (param_blk_info.blk_size > 0) {
			mutex_lock(&priv->module_lock);

			blk_buff = (void __user *)
					(uintptr_t)param_blk_info.blk_data;

			set_param_blk_buf = iaxxx_alloc_from_static_mem_blk(
					priv,
					module_dev_priv->static_mem_blk_id,
					param_blk_info.blk_size);

			if (!set_param_blk_buf) {
				mutex_unlock(&priv->module_lock);
				return -ENOMEM;
			}

			if (copy_from_user(set_param_blk_buf, blk_buff,
					param_blk_info.blk_size)) {
				mutex_unlock(&priv->module_lock);
				return -EFAULT;
			}
			ret = iaxxx_core_sensor_write_param_blk_by_inst(
					module_dev_priv->parent,
					param_blk_info.inst_id,
					param_blk_info.param_blk_id,
					set_param_blk_buf,
					param_blk_info.blk_size,
					param_blk_info.block_id);
			mutex_unlock(&priv->module_lock);
		} else {
			pr_err("invalid block-size received\n");
			return -EINVAL;
		}
		break;
	case IAXXX_POWER_STATS_COUNT:
		if (copy_from_user(&pwr_stats_count, (void __user *)arg,
					sizeof(pwr_stats_count)))
			return -EFAULT;

		mutex_lock(&priv->module_lock);
		/* Get Power Statistics */
		if (iaxxx_core_get_pwr_stats(module_dev_priv->parent,
					&pwr_stats_count) < 0) {
			pr_err("Error in reading power statistics\n");
			mutex_unlock(&priv->module_lock);
			return -EINVAL;
		}
		mutex_unlock(&priv->module_lock);

		if (copy_to_user((void __user *)arg, &pwr_stats_count,
					sizeof(pwr_stats_count)))
			return -EFAULT;

		return 0;

	case IAXXX_SET_OSC_TRIM_PERIOD:
		if  (copy_from_user(&osc_trim_period, (void __user *)arg,
					sizeof(struct iaxxx_osc_trim_period)))
			return -EFAULT;
		if (osc_trim_period.period < 0)
			return -EINVAL;

		mutex_lock(&priv->module_lock);
		ret = iaxxx_set_osc_trim_period(priv,
				osc_trim_period.period);
		mutex_unlock(&priv->module_lock);
		if (ret) {
			pr_err("%s() Failed to set Osc trim period\n",
				__func__);
			return ret;
		}

		break;

	case IAXXX_SENSOR_MODE_STATS:
		mutex_lock(&priv->module_lock);
		/* Get Power Statistics */
		if (iaxxx_core_get_sensor_mode_stats(module_dev_priv->parent,
							sensor_stats) < 0) {
			pr_err("Error in reading sensor mode stats\n");
			mutex_unlock(&priv->module_lock);
			return -EINVAL;
		}
		mutex_unlock(&priv->module_lock);

		if (copy_to_user((void __user *)arg, sensor_stats,
				sizeof(sensor_stats)))
			return -EFAULT;
		return 0;

	default:
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long module_dev_compat_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	return module_dev_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int module_dev_open(struct inode *inode, struct file *file)
{
	struct module_device_priv *module_dev_priv = container_of(
		inode->i_cdev, struct module_device_priv, cdev);
	file->private_data = module_dev_priv;

	return 0;
}

static int module_dev_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations module_dev_fops = {
	.open = module_dev_open,
	.release = module_dev_release,
	.unlocked_ioctl	= module_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= module_dev_compat_ioctl,
#endif
};


static int iaxxx_module_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct module_device_priv *module_dev_priv;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	int ret;

	dev_dbg(dev, "Enter :%s\n", __func__);
	if (module_cell_priv.cdev_minor == MODULE_CDEV_MAX_DEVICES) {
		dev_err(dev,
			"Minor nos exhausted. Increase CVQ_CDEV_MAX_DEVICES\n");
		return -ENOBUFS;
	}

	module_dev_priv = kvzalloc(sizeof(struct module_device_priv),
					GFP_KERNEL);
	if (!module_dev_priv) {
		ret = -ENOMEM;
		goto out_err;
	}

	module_dev_priv->regmap = priv->regmap;
	module_dev_priv->parent = dev->parent;
	module_dev_priv->cdev.owner = THIS_MODULE;
	module_dev_priv->dev_num = MKDEV(MAJOR(module_cell_priv.dev_num),
					module_cell_priv.cdev_minor);
	cdev_init(&module_dev_priv->cdev, &module_dev_fops);
	ret = cdev_add(&module_dev_priv->cdev, module_dev_priv->dev_num, 1);
	if (ret) {
		dev_err(dev, "failed to add cdev error: %d", ret);
		goto free_module;
	}

	module_dev_priv->dev = device_create(module_cell_priv.cdev_class,
					NULL, module_dev_priv->dev_num,
					module_dev_priv, dev_name(dev));
	if (IS_ERR(module_dev_priv->dev)) {
		ret = PTR_ERR(module_dev_priv->dev);
		dev_dbg(dev, "Failed (%d) to create cdev device\n", ret);
		goto err_device_create;
	}

	module_dev_priv->static_mem_blk_id = iaxxx_create_static_mem_blk(
			priv, IAXXX_MODULE_STATIC_MEM_BLK_SIZE);
	if (module_dev_priv->static_mem_blk_id < 0) {
		ret = module_dev_priv->static_mem_blk_id;
		dev_err(dev, "Failed (%d) to allocate static memory\n", ret);
		goto err_device_create;
	}

	dev_set_drvdata(dev, module_dev_priv);
	pr_debug("MODULE device cdev initialized.\n");

	return 0;

err_device_create:
	cdev_del(&module_dev_priv->cdev);
free_module:
	kvfree(module_dev_priv);
out_err:
	dev_err(dev, "cdev setup failure: no cdevs available!\n");

	return ret;

}
static int iaxxx_module_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct module_device_priv *module_dev_priv = dev_get_drvdata(dev);
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);

	iaxxx_destroy_static_mem_blk(priv, module_dev_priv->static_mem_blk_id);
	device_destroy(module_cell_priv.cdev_class, module_dev_priv->dev_num);
	cdev_del(&module_dev_priv->cdev);

	kvfree(module_dev_priv);

	return 0;
}

static int iaxxx_module_celldrv_suspend_rt(struct device *dev)
{
		return 0;
}

static int iaxxx_module_celldrv_resume_rt(struct device *dev)
{
		return 0;
}

static const struct dev_pm_ops iaxxx_module_celldrv_pm_ops = {
		SET_RUNTIME_PM_OPS(iaxxx_module_celldrv_suspend_rt,
			iaxxx_module_celldrv_resume_rt, NULL)

};

static const struct of_device_id iaxxx_module_dt_match[] = {
	{.compatible = "knowles,iaxxx-module-celldrv"},
	{}
};

static struct platform_driver iaxxx_module_driver = {
	.probe  = iaxxx_module_dev_probe,
	.remove = iaxxx_module_dev_remove,
	.driver = {
		.name = "iaxxx-module-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_module_dt_match,
		.pm = &iaxxx_module_celldrv_pm_ops,
	},
};

static int __init iaxxx_module_init(void)
{
	int ret;

	module_cell_priv.cdev_minor  = 0;
	module_cell_priv.cdev_class = class_create(THIS_MODULE,
						"iaxxx-module-celldrv");
	if (IS_ERR(module_cell_priv.cdev_class)) {
		ret = PTR_ERR(module_cell_priv.cdev_class);
		pr_err("Err (%d) creating iaxxx-module-celldrv class.\n", ret);
		goto out_err;
	}

	ret = alloc_chrdev_region(&module_cell_priv.dev_num, 0,
			MODULE_CDEV_MAX_DEVICES, "iaxxx-module-celldrv");
	if (ret) {
		pr_err("Failed to alloc chardev region (%d)", ret);
		goto err_alloc_cdev;
	}

	pr_info("Allocated Major device number %d\n",
				MAJOR(module_cell_priv.dev_num));

	ret = platform_driver_register(&iaxxx_module_driver);
	if (ret)
		goto err_plat_register;

	return 0;

err_plat_register:
	unregister_chrdev_region(module_cell_priv.dev_num,
				MODULE_CDEV_MAX_DEVICES);
err_alloc_cdev:
	class_destroy(module_cell_priv.cdev_class);
out_err:
	pr_err("MODULE Cell init failure (%d). No CVQ cdevs available!\n", ret);
	return ret;
}


static void __exit iaxxx_module_exit(void)
{
	platform_driver_unregister(&iaxxx_module_driver);

	unregister_chrdev_region(module_cell_priv.dev_num,
						MODULE_CDEV_MAX_DEVICES);
	class_destroy(module_cell_priv.cdev_class);
}
module_init(iaxxx_module_init);
module_exit(iaxxx_module_exit);
