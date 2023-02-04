/*
 * iaxxx-odsp-celldrv.c -- IAxxx OpenDSP cell driver
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
#include <linux/mfd/adnc/iaxxx-odsp.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-debug.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mfd/adnc/iaxxx-plugin-common.h>
#include <linux/mfd/adnc/iaxxx-static-mem.h>
#include "iaxxx-odsp-celldrv.h"

static struct odsp_cell_params odsp_cell_priv;

/* 1MB block is reserved at boot-time for odsp ioctls' use
 */
#define IAXXX_ODSP_STATIC_MEM_BLK_SIZE (1 * 1024 * 1024)

static int get_execution_status(struct device *dev,
					uint8_t proc_id, uint32_t *status)
{
	int ret;
	uint8_t block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);

	ret = regmap_read(to_iaxxx_priv(dev)->regmap,
			IAXXX_DEBUG_BLOCK_0_EXEC_STATUS_ADDR
			+ (block_id * sizeof(uint32_t)), status);
	if (ret)
		dev_err(dev, "%s() read EXEC_STATUS failed %d\n",
							__func__, ret);

	return ret;
}

static long odsp_dev_ioctl_common(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct odsp_device_priv *odsp_dev_priv = file->private_data;
	struct iaxxx_priv *priv = to_iaxxx_priv(odsp_dev_priv->parent);
	struct iaxxx_plugin_info plg_info;
	struct iaxxx_plugin_param param_info;
	struct iaxxx_plugin_param_blk param_blk_info;
#ifdef CONFIG_MFD_IAXXX_CUSTOM_CONFIG_ALGO
	struct iaxxx_plugin_custom_cfg custom_cfg_info;
#endif
	struct iaxxx_plugin_create_cfg cfg_info;
	struct iaxxx_set_event set_event;
	struct iaxxx_get_event get_event;
	struct iaxxx_evt_info event_info;
	struct iaxxx_pkg_mgmt_info pkg_info;
	struct iaxxx_plugin_error_info plugin_error_info;
	struct iaxxx_plugin_set_param_blk_with_ack_info param_blk_with_ack;
	struct iaxxx_plugin_status_info plugin_status_info;
	struct iaxxx_plugin_status_data plugin_status_data;
	struct iaxxx_plugin_endpoint_status_info plugin_ep_status_info;
	struct iaxxx_plugin_endpoint_status_data plugin_ep_status_data;

	uint32_t *get_param_blk_buf = NULL;
	uint32_t *set_param_blk_buf = NULL;

	void __user *blk_buff = NULL;
	int ret = -EINVAL;
	uint32_t size_in_words;

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

	switch (cmd) {
	case ODSP_PLG_CREATE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(plg_info.inst_id)
			&& iaxxx_core_plg_is_valid_pkg_id(plg_info.pkg_id)
			&& iaxxx_core_plg_is_valid_priority(plg_info.priority)
			&& iaxxx_core_plg_is_valid_plg_idx(plg_info.plg_idx)
			&& iaxxx_core_plg_is_valid_block_id(plg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_create_plg(odsp_dev_priv->parent,
					plg_info.inst_id,
					plg_info.priority, plg_info.pkg_id,
					plg_info.plg_idx, plg_info.block_id,
					plg_info.config_id);
		if (ret) {
			pr_err("%s() Plugin create fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_PLG_CREATE_STATIC_PACKAGE:
		if (copy_from_user(&plg_info, (void __user *)arg,
				sizeof(plg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(plg_info.inst_id)
			&& iaxxx_core_plg_is_valid_pkg_id(plg_info.pkg_id)
			&& iaxxx_core_plg_is_valid_priority(plg_info.priority)
			&& iaxxx_core_plg_is_valid_plg_idx(plg_info.plg_idx)
			&& iaxxx_core_plg_is_valid_block_id(plg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_create_plg_static_package(
				odsp_dev_priv->parent,
				plg_info.inst_id,
				plg_info.priority, plg_info.pkg_id,
				plg_info.plg_idx, plg_info.block_id,
				plg_info.config_id);
		if (ret) {
			pr_err("%s() Plugin create fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_PLG_DESTROY:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(plg_info.inst_id)
			&& iaxxx_core_plg_is_valid_block_id(plg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		iaxxx_core_destroy_plg(odsp_dev_priv->parent, plg_info.inst_id,
						plg_info.block_id);
		break;

	case ODSP_PLG_ENABLE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(plg_info.inst_id)
			&& iaxxx_core_plg_is_valid_block_id(plg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		iaxxx_core_change_plg_state(odsp_dev_priv->parent,
				plg_info.inst_id, 1, plg_info.block_id);
		break;

	case ODSP_PLG_DISABLE:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(plg_info.inst_id)
			&& iaxxx_core_plg_is_valid_block_id(plg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		iaxxx_core_change_plg_state(odsp_dev_priv->parent,
				plg_info.inst_id, 0, plg_info.block_id);
		break;

	case ODSP_PLG_RESET:
		if (copy_from_user(&plg_info, (void __user *)arg,
						sizeof(plg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(plg_info.inst_id)
			&& iaxxx_core_plg_is_valid_block_id(plg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_reset_plg(odsp_dev_priv->parent,
						plg_info.inst_id,
						plg_info.block_id);
		if (ret) {
			pr_err("%s() Plugin reset fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_PLG_SET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
						sizeof(param_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(param_info.inst_id)
			&& iaxxx_core_plg_is_valid_param_id(param_info.param_id)
			&& iaxxx_core_plg_is_valid_param_val(
							param_info.param_val)
			&& iaxxx_core_plg_is_valid_block_id(
							param_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_plg_set_param_by_inst(odsp_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				param_info.param_val, param_info.block_id);
		if (ret) {
			pr_err("%s() Plugin set param fail\n", __func__);
			pr_err("Param info inst 0x%x p_id 0x%x p_val 0x%x\n",
					param_info.inst_id, param_info.param_id,
					param_info.param_val);
			return ret;
		}
		break;

	case ODSP_PLG_GET_PARAM:
		if (copy_from_user(&param_info, (void __user *)arg,
						sizeof(param_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(param_info.inst_id)
			&& iaxxx_core_plg_is_valid_param_id(
							param_info.param_id)
			&& iaxxx_core_plg_is_valid_block_id(
							param_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_plg_get_param_by_inst(odsp_dev_priv->parent,
				param_info.inst_id, param_info.param_id,
				&param_info.param_val, param_info.block_id);
		if (ret) {
			pr_err("%s() Plugin get param fail\n", __func__);
			pr_err("Param info inst 0x%x p_id 0x%x\n",
				param_info.inst_id, param_info.param_id);
			return ret;
		}
		/* After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &param_info,
						sizeof(param_info)))
			return -EFAULT;

		break;

	case ODSP_PLG_SET_PARAM_BLK:
		if (copy_from_user(&param_blk_info, (void __user *)arg,
				sizeof(struct iaxxx_plugin_param_blk)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(param_blk_info.inst_id)
			&& iaxxx_core_plg_is_valid_block_id(
							param_blk_info.block_id)
			&& iaxxx_core_plg_is_valid_param_blk_size(
						param_blk_info.param_size)
			&& iaxxx_core_plg_is_valid_param_blk_id(
							param_blk_info.id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		param_blk_info.file_name[sizeof(param_blk_info.file_name) - 1]
			 = '\0';

		if (param_blk_info.param_size > 0) {
			blk_buff = (void __user *)
					(uintptr_t)param_blk_info.param_blk;
			set_param_blk_buf = iaxxx_alloc_from_static_mem_blk(
					priv,
					odsp_dev_priv->static_mem_blk_id,
					param_blk_info.param_size);

			if (!set_param_blk_buf)
				return -ENOMEM;

			if (copy_from_user(set_param_blk_buf, blk_buff,
					param_blk_info.param_size))
				return -EFAULT;

			ret = iaxxx_core_set_param_blk(odsp_dev_priv->parent,
				param_blk_info.inst_id,
				param_blk_info.param_size,
				set_param_blk_buf, param_blk_info.block_id,
				param_blk_info.id);

		} else {
			ret = iaxxx_core_set_param_blk_from_file(
				odsp_dev_priv->parent,
				param_blk_info.inst_id,
				param_blk_info.block_id,
				param_blk_info.id,
				param_blk_info.file_name);
		}
		if (ret) {
			pr_err("%s() Set param blk fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_PLG_GET_PARAM_BLK:
		if (copy_from_user(&param_blk_info, (void __user *)arg,
				sizeof(struct iaxxx_plugin_param_blk)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(param_blk_info.inst_id)
			&& iaxxx_core_plg_is_valid_param_id(param_blk_info.id)
			&& iaxxx_core_plg_is_valid_param_blk_size(
						param_blk_info.param_size)
			&& iaxxx_core_plg_is_valid_block_id(
						param_blk_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		size_in_words = (param_blk_info.param_size >> 2) +
			(!!(param_blk_info.param_size & 0x3));
		get_param_blk_buf = iaxxx_alloc_from_static_mem_blk(priv,
				odsp_dev_priv->static_mem_blk_id,
				size_in_words << 2);
		if (!get_param_blk_buf)
			return -ENOMEM;

		ret = iaxxx_core_get_param_blk(odsp_dev_priv->parent,
			param_blk_info.inst_id, param_blk_info.block_id,
			param_blk_info.id,
			get_param_blk_buf,
			size_in_words);
		if (ret) {
			pr_err("%s() Get param blk fail\n", __func__);
			goto get_param_blk_err;
		}

		if (copy_to_user((void __user *) param_blk_info.param_blk,
				get_param_blk_buf,
				param_blk_info.param_size)) {
			pr_err("%s() copy to user fail\n", __func__);
			ret = -EFAULT;
		}
get_param_blk_err:
		get_param_blk_buf = NULL;
		if (ret)
			return ret;
		break;

	case ODSP_PLG_SET_CUSTOM_CFG:
#ifdef CONFIG_MFD_IAXXX_CUSTOM_CONFIG_ALGO
		if (copy_from_user(&custom_cfg_info, (void __user *)arg,
				sizeof(custom_cfg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(custom_cfg_info.inst_id)
			&& iaxxx_core_plg_is_valid_param_blk_id(
						custom_cfg_info.param_blk_id)
			&& iaxxx_core_plg_is_valid_block_id(
						custom_cfg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}
		ret = iaxxx_core_set_custom_cfg(odsp_dev_priv->parent,
				custom_cfg_info.inst_id,
				custom_cfg_info.block_id,
				custom_cfg_info.param_blk_id,
				custom_cfg_info.custom_config_id,
				custom_cfg_info.file_name);
		if (ret) {
			pr_err("%s() Plugin custom config setup failed\n",
					__func__);
			return ret;
		}
#else
		pr_err("%s() Custom Config IOCTL not Supported!\n",
			__func__);
		ret = -EFAULT;
#endif
		break;

	case ODSP_PLG_SET_PARAM_BLK_WITH_ACK:
		if (copy_from_user(&param_blk_with_ack, (void __user *)arg,
				sizeof(param_blk_with_ack)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(
						param_blk_with_ack.inst_id)
			&& iaxxx_core_plg_is_valid_param_blk_id(
						param_blk_with_ack.param_blk_id)
			&& iaxxx_core_plg_is_valid_param_blk_size(
					param_blk_with_ack.response_buf_size)
			&& iaxxx_core_plg_is_valid_block_id(
						param_blk_with_ack.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		blk_buff = (void __user *)
			(uintptr_t)param_blk_with_ack.set_param_blk_buffer;
		blk_buff = memdup_user(blk_buff,
				param_blk_with_ack.set_param_blk_size);
		param_blk_with_ack.set_param_blk_buffer
			= (uintptr_t)blk_buff;
		if (IS_ERR(blk_buff)) {
			ret = PTR_ERR(blk_buff);
			pr_err("%s memdup failed %d\n", __func__, ret);
			return ret;
		}

		get_param_blk_buf = kvzalloc(
					param_blk_with_ack.response_buf_size *
					sizeof(uint32_t),
					GFP_KERNEL);
		if (!get_param_blk_buf) {
			ret = -ENOMEM;
			goto set_param_blk_err;
		}

		ret = iaxxx_core_set_param_blk_with_ack(
			odsp_dev_priv->parent,
			param_blk_with_ack.inst_id,
			param_blk_with_ack.param_blk_id,
			param_blk_with_ack.block_id,
			blk_buff,
			param_blk_with_ack.set_param_blk_size,
			get_param_blk_buf,
			param_blk_with_ack.response_buf_size,
			param_blk_with_ack.max_retries);
		if (ret) {
			pr_err("%s() Set param blk with ack fail\n", __func__);
			goto err;
		}
		if (copy_to_user((void __user *)
				param_blk_with_ack.response_buffer,
				get_param_blk_buf,
				param_blk_with_ack.response_buf_size*
				sizeof(uint32_t))) {
			pr_err("%s() copy to user fail\n", __func__);
			ret = -EFAULT;
		}
err:
		kvfree(get_param_blk_buf);
		get_param_blk_buf = NULL;
set_param_blk_err:
		kfree(blk_buff);
		blk_buff = NULL;
		if (ret)
			return ret;
		break;

	case ODSP_PLG_SET_CREATE_CFG:
		if (copy_from_user(&cfg_info, (void __user *)arg,
						sizeof(cfg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(cfg_info.inst_id)
			&& iaxxx_core_plg_is_valid_cfg_size(cfg_info.cfg_size)
			&& iaxxx_core_plg_is_valid_block_id(cfg_info.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		cfg_info.file_name[sizeof(cfg_info.file_name) - 1] = '\0';
		ret = iaxxx_core_set_create_cfg(odsp_dev_priv->parent,
				cfg_info.inst_id, cfg_info.cfg_size,
				cfg_info.cfg_val, cfg_info.block_id,
				cfg_info.file_name);
		if (ret) {
			pr_err("%s() Plugin create cfg fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_PLG_SET_EVENT:
		if (copy_from_user(&set_event, (void __user *)arg,
						sizeof(set_event)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_plg_is_valid_inst_id(set_event.inst_id)
			&& iaxxx_core_plg_is_valid_block_id(set_event.block_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_set_event(odsp_dev_priv->parent,
					set_event.inst_id,
					set_event.event_enable_mask,
					set_event.block_id);
		if (ret) {
			pr_err("%s() Set event fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_EVENT_TRIGGER: {
		struct iaxxx_evt_trigger et;

		if (copy_from_user(&et, (void __user *)arg, sizeof(et)))
			return -EFAULT;

		ret = iaxxx_core_evt_trigger(odsp_dev_priv->parent,
					et.src_id, et.evt_id, et.src_opaque);
		if (ret) {
			pr_err("%s() iaxxx_core_evt_trigger fail\n", __func__);
			return ret;
		}
		break;
	}

	case ODSP_EVENT_READ_SUBSCRIPTION: {
		struct iaxxx_evt_read_subscription ers;

		ret = iaxxx_core_evt_read_subscription(odsp_dev_priv->parent,
						&ers.src_id, &ers.evt_id,
						&ers.dst_id, &ers.dst_opaque);
		if (ret) {
			pr_err("%s() iaxxx_core_evt_read_subscription fail\n",
								__func__);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &ers, sizeof(ers)))
			return -EFAULT;
		break;
	}

	case ODSP_EVENT_RETRIEVE_NOTIFICATION: {
		struct iaxxx_evt_retrieve_notification ern;

		ret = iaxxx_core_evt_retrieve_notification(
					odsp_dev_priv->parent,
					&ern.src_id, &ern.evt_id,
					&ern.src_opaque, &ern.dst_opaque);
		if (ret) {
			pr_err(
			"%s() iaxxx_core_evt_retrieve_notification fail\n",
								__func__);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &ern, sizeof(ern)))
			return -EFAULT;
		break;
	}

	case ODSP_EVENT_RESET_READ_INDEX:
		ret = iaxxx_core_evt_reset_read_index(odsp_dev_priv->parent);
		if (ret) {
			pr_err("%s() iaxxx_core_evt_reset_read_index fail\n",
								__func__);
			return ret;
		}
		break;

	case ODSP_EVENT_SUBSCRIBE:
		if (copy_from_user(&event_info, (void __user *)arg,
						sizeof(event_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_evt_is_valid_src_id(event_info.src_id)
			&& iaxxx_core_evt_is_valid_dst_id(event_info.dst_id)
			&& iaxxx_core_evt_is_valid_event_id(
							event_info.event_id)
			&& iaxxx_core_evt_is_valid_dst_opaque(
							event_info.dst_opaque))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_evt_subscribe(odsp_dev_priv->parent,
					event_info.src_id,
					event_info.event_id,
					event_info.dst_id,
					event_info.dst_opaque);
		if (ret) {
			pr_err("%s() Event subscribe fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_GET_EVENT:
		if (copy_from_user(&get_event, (void __user *)arg,
						sizeof(get_event)))
			return -EFAULT;
		ret = iaxxx_core_retrieve_event(odsp_dev_priv->parent,
				&get_event.event_id,
				&get_event.data);
		if (ret) {
			pr_err("%s() Get event fail\n", __func__);
			return ret;
		}
		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &get_event,
						sizeof(get_event)))
			return -EFAULT;
		break;

	case ODSP_EVENT_UNSUBSCRIBE:
		if (copy_from_user(&event_info, (void __user *)arg,
						sizeof(event_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!(iaxxx_core_evt_is_valid_src_id(event_info.src_id)
			&& iaxxx_core_evt_is_valid_dst_id(event_info.dst_id)
			&& iaxxx_core_evt_is_valid_event_id(
							event_info.event_id))
			) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_evt_unsubscribe(odsp_dev_priv->parent,
					event_info.src_id,
					event_info.event_id,
					event_info.dst_id);
		if (ret) {
			pr_err("%s() Event unsubscribe fail\n", __func__);
			return ret;
		}
		break;

	case ODSP_LOAD_PACKAGE:
		if (copy_from_user(&pkg_info, (void __user *)arg,
						sizeof(pkg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!iaxxx_core_plg_is_valid_pkg_id(pkg_info.pkg_id)) {
			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_package_load(odsp_dev_priv->parent,
			pkg_info.pkg_name, pkg_info.pkg_id,
			&pkg_info.proc_id);
		pr_info("%s()Pkg name %s id %d\n", __func__,
				pkg_info.pkg_name, pkg_info.pkg_id);
		if (ret) {
			pr_err("%s() Load package failed\n", __func__);
			if (ret == -EEXIST) {
				if (copy_to_user((void __user *)arg, &pkg_info,
							sizeof(pkg_info)))
					return -EFAULT;
			}
			return ret;
		}
		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &pkg_info,
						sizeof(pkg_info)))
			return -EFAULT;
		break;

	case ODSP_UNLOAD_PACKAGE:
		if (copy_from_user(&pkg_info, (void __user *)arg,
						sizeof(pkg_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!iaxxx_core_plg_is_valid_pkg_id(pkg_info.pkg_id)) {
			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_package_unload(odsp_dev_priv->parent,
					pkg_info.pkg_id);
		pr_info("%s()Pkg id 0x%x\n", __func__, pkg_info.pkg_id);
		if (ret) {
			pr_err("%s() Unload package failed\n", __func__);
			return ret;
		}
		break;

	case ODSP_PLG_GET_PACKAGE_VERSION: {
		struct iaxxx_plugin_get_package_version v;

		if (copy_from_user(&v, (void __user *)arg, sizeof(v)))
			return -EFAULT;
		if (!iaxxx_core_plg_is_valid_inst_id(v.inst_id)) {
			pr_err("%s() Invalid plugin parameter received\n",
								__func__);
			return -EINVAL;
		}
		if (v.len > sizeof(v.version)) {
			pr_warn(
			"%s() Too large len %u when get package version\n",
			__func__, v.len);
			v.len = sizeof(v.version);
		}

		ret = iaxxx_core_plg_get_package_version(odsp_dev_priv->parent,
						v.inst_id, v.version, v.len);
		if (ret) {
			pr_err("%s() Get package version failed\n", __func__);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &v, sizeof(v)))
			return -EFAULT;
		break;
	}

	case ODSP_PLG_GET_PLUGIN_VERSION: {
		struct iaxxx_plugin_get_plugin_version v;

		if (copy_from_user(&v, (void __user *)arg, sizeof(v)))
			return -EFAULT;
		if (!iaxxx_core_plg_is_valid_inst_id(v.inst_id)) {
			pr_err("%s() Invalid plugin parameter received\n",
								__func__);
			return -EINVAL;
		}
		if (v.len > sizeof(v.version)) {
			pr_warn(
			"%s() Too large len %u when get plugin version\n",
			__func__, v.len);
			v.len = sizeof(v.version);
		}

		ret = iaxxx_core_plg_get_plugin_version(odsp_dev_priv->parent,
						v.inst_id, v.version, v.len);
		if (ret) {
			pr_err("%s() Get plugin version failed\n", __func__);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &v, sizeof(v)))
			return -EFAULT;
		break;
	}

	case ODSP_PLG_READ_PLUGIN_ERROR:
		if (copy_from_user(&plugin_error_info, (void __user *)arg,
				sizeof(plugin_error_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!iaxxx_core_plg_is_valid_block_id(
					plugin_error_info.block_id)) {
			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_read_plugin_error(odsp_dev_priv->parent,
				plugin_error_info.block_id,
				&plugin_error_info.error_code,
				&plugin_error_info.error_instance);
		if (ret) {
			pr_err("%s() Read plugin error fail\n", __func__);
			return ret;
		}
		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &plugin_error_info,
				sizeof(plugin_error_info)))
			return -EFAULT;
		break;

	case ODSP_PLG_GET_STATUS_INFO:
		if (copy_from_user(&plugin_status_info, (void __user *)arg,
				sizeof(plugin_status_info)))
			return -EFAULT;

		/* validate the plugin parameters */
		if (!iaxxx_core_plg_is_valid_inst_id(
						plugin_status_info.inst_id)) {

			pr_err("invalid plugin parameter received\n");
			return -EINVAL;
		}

		ret = iaxxx_core_plg_get_status_info(odsp_dev_priv->parent,
						plugin_status_info.inst_id,
						&plugin_status_data);
		if (ret) {
			pr_err("%s() Plugin status info fail\n", __func__);
			return ret;
		}
		plugin_status_info.block_id = plugin_status_data.block_id;
		plugin_status_info.create_status =
				plugin_status_data.create_status;
		plugin_status_info.enable_status =
				plugin_status_data.enable_status;
		plugin_status_info.in_frames_consumed =
				plugin_status_data.in_frames_consumed;
		plugin_status_info.out_frames_produced =
				plugin_status_data.out_frames_produced;
		plugin_status_info.process_err_count =
				plugin_status_data.process_err_count;
		plugin_status_info.process_count =
				plugin_status_data.process_count;
		plugin_status_info.private_memsize =
				plugin_status_data.private_memsize;
		plugin_status_info.frame_notification_mode =
				plugin_status_data.frame_notification_mode;
		plugin_status_info.state_management_mode =
				plugin_status_data.state_management_mode;

		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &plugin_status_info,
				sizeof(plugin_status_info)))
			return -EFAULT;

		break;

	case ODSP_PLG_GET_ENDPOINT_STATUS:
		if (copy_from_user(&plugin_ep_status_info, (void __user *)arg,
				sizeof(plugin_ep_status_info)))
			return -EFAULT;
		ret = iaxxx_core_plg_get_endpoint_status(
				odsp_dev_priv->parent,
				plugin_ep_status_info.inst_id,
				plugin_ep_status_info.ep_index,
				plugin_ep_status_info.direction,
				&plugin_ep_status_data);
		if (ret) {
			pr_err("%s() Plugin endpoint status fail\n", __func__);
			return ret;
		}

		plugin_ep_status_info.status
				= plugin_ep_status_data.status;
		plugin_ep_status_info.frame_status
				= plugin_ep_status_data.frame_status;
		plugin_ep_status_info.endpoint_status
				= plugin_ep_status_data.endpoint_status;
		plugin_ep_status_info.usage
				= plugin_ep_status_data.usage;
		plugin_ep_status_info.mandatory
				= plugin_ep_status_data.mandatory;
		plugin_ep_status_info.counter
				= plugin_ep_status_data.counter;
		plugin_ep_status_info.op_encoding
				= plugin_ep_status_data.op_encoding;
		plugin_ep_status_info.op_sample_rate
				= plugin_ep_status_data.op_sample_rate;
		plugin_ep_status_info.op_frame_length
				= plugin_ep_status_data.op_frame_length;

		/*After read copy back the data to user space */
		if (copy_to_user((void __user *)arg, &plugin_ep_status_info,
				sizeof(plugin_ep_status_info)))
			return -EFAULT;

		break;

	case ODSP_PLG_GET_ENDPOINT_TIMESTAMPS: {
		struct iaxxx_plugin_endpoint_timestamps t;

		if (copy_from_user(&t, (void __user *)arg, sizeof(t)))
			return -EFAULT;
		ret = iaxxx_core_plg_get_endpoint_timestamps(
				odsp_dev_priv->parent, t.timestamps,
				IAXXX_MAX_PLUGIN_ENDPOINTS, t.proc_id);
		if (ret) {
			pr_err("%s() Plugin endpoint timestamps fail\n",
								__func__);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &t, sizeof(t)))
			return -EFAULT;
		break;
	}

	case ODSP_GET_PROC_EXECUTION_STATUS: {
		struct iaxxx_proc_execution_status s;

		if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
			return -EFAULT;
		ret = get_execution_status(odsp_dev_priv->parent,
							s.proc_id, &s.status);
		if (ret) {
			pr_err("%s() get_execution_status failed %d\n",
								__func__, ret);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &s, sizeof(s)))
			return -EFAULT;
		break;
	}

	case ODSP_GET_SYS_VERSIONS: {
		struct iaxxx_sys_versions v;
		int err;

		if (copy_from_user(&v, (void __user *)arg, sizeof(v)))
			return -EFAULT;

		if (v.app_ver_str_len > sizeof(v.app_ver_str)) {
			pr_warn("Too large app_ver_str_len %u\n",
						v.app_ver_str_len);
			v.app_ver_str_len = sizeof(v.app_ver_str);
		}
		err = iaxxx_get_firmware_version(odsp_dev_priv->parent,
					v.app_ver_str, v.app_ver_str_len);
		if (err) {
			pr_err("Failed to iaxxx_get_firmware_version err=%d\n",
									err);
			v.app_ver_str[0] = '\0';
			v.app_ver_str_len = 1;
		}
		err = iaxxx_get_application_ver_num(odsp_dev_priv->parent,
							&v.app_ver_num);
		if (err) {
			pr_err(
		"Failed to iaxxx_get_application_ver_num err=%d\n", err);
			ret = -EINVAL;
		}

		if (v.rom_ver_str_len > sizeof(v.rom_ver_str)) {
			pr_warn("Too large rom_ver_str_len %u\n",
						v.app_ver_str_len);
			v.rom_ver_str_len = sizeof(v.rom_ver_str);
		}
		err = iaxxx_get_rom_version(odsp_dev_priv->parent,
					v.rom_ver_str, v.rom_ver_str_len);
		if (err) {
			pr_err("Failed to iaxxx_get_rom_version err=%d\n",
									err);
			v.rom_ver_str[0] = '\0';
			v.rom_ver_str_len = 1;
		}
		err = iaxxx_get_rom_ver_num(odsp_dev_priv->parent,
							&v.rom_ver_num);
		if (err) {
			pr_err("Failed to iaxxx_get_rom_ver_num err=%d\n",
									err);
			ret = -EINVAL;
		}

		if (copy_to_user((void __user *)arg, &v, sizeof(v)))
			ret = -EFAULT;
		break;
	}

	case ODSP_GET_SYS_DEVICE_ID: {
		uint32_t device_id = 0;

		ret = iaxxx_get_device_id(odsp_dev_priv->parent, &device_id);
		if (ret) {
			pr_err("Failed to iaxxx_get_device_id ret=%d\n", ret);
		} else {
			if (copy_to_user((void __user *)arg, &device_id,
							sizeof(device_id)))
				ret = -EFAULT;
		}
		break;
	}

	case ODSP_GET_FW_STATUS: {

		uint32_t fw_status;

		if (test_bit(IAXXX_FLG_FW_CRASH, &priv->flags))
			fw_status = IAXXX_FW_CRASH;
		else if (!iaxxx_core_plg_list_empty(priv) ||
			iaxxx_core_get_route_status(priv) ||
			atomic_read(&priv->fli_route_status) ||
			!iaxxx_core_pkg_list_empty(priv))
			fw_status = IAXXX_FW_ACTIVE;
		else
			fw_status = IAXXX_FW_IDLE;

		if (copy_to_user((void __user *)arg, &fw_status,
				sizeof(fw_status)))
			ret = -EFAULT;

		break;
	}

	case ODSP_RESET_FW: {
		ret = iaxxx_fw_reset(priv);
		if (ret) {
			pr_err("%s() iaxxx_reset_fw() failed\n", __func__);
			return ret;
		}
		break;
	}
	default:
		break;
	}
	return 0;
}

static long odsp_dev_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct odsp_device_priv *odsp_dev_priv = file->private_data;
	int ret;

	/* Lock all ioctls so only one gets executed at a time */
	mutex_lock(&odsp_dev_priv->ioctl_lock);
	ret = odsp_dev_ioctl_common(file, cmd, (unsigned long)compat_ptr(arg));
	mutex_unlock(&odsp_dev_priv->ioctl_lock);
	return ret;
}

#ifdef CONFIG_COMPAT
static long odsp_dev_compat_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	return odsp_dev_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int odsp_dev_open(struct inode *inode, struct file *file)
{
	struct odsp_device_priv *odsp_dev_priv = container_of(inode->i_cdev,
						struct odsp_device_priv, cdev);
	file->private_data = odsp_dev_priv;

	return 0;
}

static int odsp_dev_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations odsp_dev_fops = {
	.open = odsp_dev_open,
	.release = odsp_dev_release,
	.unlocked_ioctl	= odsp_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= odsp_dev_compat_ioctl,
#endif
};


static int iaxxx_odsp_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct odsp_device_priv *odsp_dev_priv;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	int ret;

	dev_info(dev, "Enter :%s\n", __func__);
	if (odsp_cell_priv.cdev_minor == ODSP_CDEV_MAX_DEVICES) {
		dev_err(dev,
			"Minor nos exhausted. Increase CVQ_CDEV_MAX_DEVICES\n");
		return -ENOBUFS;
	}

	odsp_dev_priv = kvzalloc(sizeof(*odsp_dev_priv), GFP_KERNEL);
	if (!odsp_dev_priv) {
		ret = -ENOMEM;
		goto out_err;
	}

	odsp_dev_priv->regmap = priv->regmap;
	odsp_dev_priv->parent = dev->parent;
	odsp_dev_priv->cdev.owner = THIS_MODULE;
	odsp_dev_priv->dev_num =
	MKDEV(MAJOR(odsp_cell_priv.dev_num), odsp_cell_priv.cdev_minor);
	mutex_init(&odsp_dev_priv->ioctl_lock);
	cdev_init(&odsp_dev_priv->cdev, &odsp_dev_fops);
	ret = cdev_add(&odsp_dev_priv->cdev, odsp_dev_priv->dev_num, 1);
	if (ret) {
		dev_err(dev, "failed to add cdev error: %d", ret);
		goto free_odsp;
	}

	odsp_dev_priv->dev = device_create(odsp_cell_priv.cdev_class, dev,
			odsp_dev_priv->dev_num, odsp_dev_priv, dev_name(dev));
	if (IS_ERR(odsp_dev_priv->dev)) {
		ret = PTR_ERR(odsp_dev_priv->dev);
		dev_err(dev, "Failed (%d) to create cdev device\n", ret);
		goto err_device_create;
	}

	odsp_dev_priv->static_mem_blk_id = iaxxx_create_static_mem_blk(
			priv, IAXXX_ODSP_STATIC_MEM_BLK_SIZE);
	if (odsp_dev_priv->static_mem_blk_id < 0) {
		ret = odsp_dev_priv->static_mem_blk_id;
		dev_err(dev, "Failed (%d) to allocate static memory\n", ret);
		goto err_device_create;
	}

	dev_set_drvdata(dev, odsp_dev_priv);
	dev_info(dev, "ODSP device cdev initialized.\n");

	odsp_dev_priv->dev = dev;

	return 0;

err_device_create:
	cdev_del(&odsp_dev_priv->cdev);
free_odsp:
	kvfree(odsp_dev_priv);
out_err:
	dev_err(dev, "cdev setup failure: no cdevs available!\n");

	return ret;

}
static int iaxxx_odsp_dev_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev->parent);
	struct odsp_device_priv *odsp_dev_priv = dev_get_drvdata(dev);

	iaxxx_destroy_static_mem_blk(priv, odsp_dev_priv->static_mem_blk_id);
	mutex_destroy(&odsp_dev_priv->ioctl_lock);
	device_destroy(odsp_cell_priv.cdev_class, odsp_dev_priv->dev_num);
	cdev_del(&odsp_dev_priv->cdev);

	kvfree(odsp_dev_priv);

	return 0;
}

static int iaxxx_odsp_rt_suspend(struct device *dev)
{
	return 0;
}

static int iaxxx_odsp_rt_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops iaxxx_odsp_pm_ops = {
	SET_RUNTIME_PM_OPS(iaxxx_odsp_rt_suspend,
			iaxxx_odsp_rt_resume, NULL)
};

static const struct of_device_id iaxxx_odsp_dt_match[] = {
	{.compatible = "knowles,iaxxx-odsp-celldrv"},
	{}
};

static struct platform_driver iaxxx_odsp_driver = {
	.probe  = iaxxx_odsp_dev_probe,
	.remove = iaxxx_odsp_dev_remove,
	.driver = {
		.name = "iaxxx-odsp-celldrv",
		.owner = THIS_MODULE,
		.of_match_table = iaxxx_odsp_dt_match,
		.pm = &iaxxx_odsp_pm_ops,
	},
};

static int __init iaxxx_odsp_init(void)
{
	int ret;

	odsp_cell_priv.cdev_minor  = 0;
	odsp_cell_priv.cdev_class = class_create(THIS_MODULE,
						"iaxxx-odsp-celldrv");
	if (IS_ERR(odsp_cell_priv.cdev_class)) {
		ret = PTR_ERR(odsp_cell_priv.cdev_class);
		pr_err("Err (%d) creating iaxxx-odsp-celldrv class.\n", ret);
		goto out_err;
	}

	ret = alloc_chrdev_region(&odsp_cell_priv.dev_num, 0,
				ODSP_CDEV_MAX_DEVICES, "iaxxx-odsp-celldrv");
	if (ret) {
		pr_err("Failed to alloc chardev region (%d)", ret);
		goto err_alloc_cdev;
	}

	pr_info("Allocated Major device number %d\n",
						MAJOR(odsp_cell_priv.dev_num));

	ret =  platform_driver_register(&iaxxx_odsp_driver);
	if (ret)
		goto err_plat_register;

	return 0;

err_plat_register:
	unregister_chrdev_region(odsp_cell_priv.dev_num, ODSP_CDEV_MAX_DEVICES);
err_alloc_cdev:
	class_destroy(odsp_cell_priv.cdev_class);
out_err:
	pr_err("ODSP Cell init failure (%d). No CVQ cdevs available!\n", ret);
	return ret;
}


static void __exit iaxxx_odsp_exit(void)
{

	platform_driver_unregister(&iaxxx_odsp_driver);

	unregister_chrdev_region(odsp_cell_priv.dev_num,
						ODSP_CDEV_MAX_DEVICES);
	class_destroy(odsp_cell_priv.cdev_class);
}
module_init(iaxxx_odsp_init);
module_exit(iaxxx_odsp_exit);
