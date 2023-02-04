/*
 * iaxxx-plugin-common.c -- Common functions shared between plugin-related
 *				modules
 *
 * Copyright 2018 Knowles Corporation
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
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-plugin-common.h>
#include "iaxxx.h"
#include "iaxxx-btp.h"

/* Set Param Block Common code without mutex protection
 * to be shared by code using plugin mutex
 */
int iaxxx_core_set_param_blk_common(
			struct device *dev,
			uint32_t inst_id, uint32_t blk_size,
			const void *ptr_blk, uint32_t block_id,
			uint32_t param_blk_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	int rc;
	uint32_t reg_addr, reg_addr2;
	uint32_t reg_val;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;
	dev_dbg(dev, "%s() inst_id=%u blk_size=%u blk_id=%u id=%u\n",
		__func__, inst_id, blk_size, block_id, param_blk_id);
	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
			inst_id, __func__);
		goto set_param_blk_err;
	}

	/* Write the PluginHdrParamBlkCtrl register */
	/* The block size is divided by 4 here because this function gets it
	 * as block size in bytes but firmware expects in 32bit words.
	 */
	reg_val = (((blk_size >> 2) <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK);
	reg_val |= ((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK);
	reg_val |= IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_REQ_MASK;
	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id, host_id),
		reg_val);
	if (ret) {
		dev_err(dev, "write ctrl block failed %s()\n", __func__);
		goto set_param_blk_err;
	}

	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_HDR_BLOCK_ADDR(block_id,
		host_id), param_blk_id);
	if (ret) {
		dev_err(dev, "write HDR block failed %s()\n", __func__);
		goto set_param_blk_err;
	}

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);

	reg_val = IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_REQ_MASK;

	if (ret) {
		dev_err(dev, "Update blk failed after id (%u) config %s()\n",
				param_blk_id, __func__);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(
			block_id, host_id),
			reg_val,
			0);
			if (rc) {
				dev_err(dev, "clear failed %s() %d\n",
					__func__, rc);
				goto set_param_blk_err;
			}
		}
		goto set_param_blk_err;
	}
	ret = regmap_read(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_ADDR_BLOCK_ADDR(
		block_id, host_id), &reg_addr);
	if (ret) {
		dev_err(dev, "read failed %s()\n", __func__);
		goto set_param_blk_err;
	}

	ret = iaxxx_btp_write(priv, reg_addr, ptr_blk,
			blk_size / sizeof(uint32_t), host_id);

	if (ret) {
		dev_err(dev, "Raw blk write failed %s()\n", __func__);
		goto set_param_blk_err;
	}

	/* The block size is divided by 4 here because this function gets it
	 * as block size in bytes but firmware expects in 32bit words.
	 */
	reg_val = (((blk_size >> 2) <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK);
	reg_val |= ((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK);
	reg_val |= IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_DONE_MASK;
	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id,
		host_id), reg_val);

	reg_addr2 = IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id,
		host_id);
	reg_val = IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_DONE_MASK;

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev,
		"Update blk failed after plugin ctrl block config %s()\n",
		__func__);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
				reg_addr2,
				reg_val,
				0);
			if (rc)
				dev_err(dev, "clear failed %s() %d\n",
					__func__, rc);
		}
	}
set_param_blk_err:
	return ret;
}


/* Get Param Block Common code without mutex protection
 * to be shared by code using plugin mutex
 */
int iaxxx_core_get_param_blk_common(
		struct device *dev,
		uint32_t  inst_id,
		uint32_t  block_id,
		uint32_t  param_blk_id,
		uint32_t *getparam_block_data,
		uint32_t  getparam_block_size_in_words)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t write_val, read_val, getparam_block_size;
	uint32_t status = 0;
	bool host_id = find_host_id(priv, inst_id);

	if (!priv || !getparam_block_data || !getparam_block_size_in_words)
		return -EINVAL;

	dev_dbg(dev, "%s() inst_id=%u blk_size=%u blk_id=%u param_blk_id=%u\n",
		__func__, inst_id, getparam_block_size_in_words, block_id,
		param_blk_id);

	inst_id &= IAXXX_PLGIN_ID_MASK;

	/* Check if plugin exists */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x does not exist! %s()\n",
				inst_id, __func__);
		ret = -EEXIST;
		goto iaxxx_core_get_param_error;
	}

	/* Write the PluginHdrParamBlkCtrl register
	 * to request the parameter block.
	 */
	write_val = ((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK);
	write_val |= IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_GET_BLK_REQ_MASK;

	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(
		block_id, host_id),
		write_val);

	if (ret) {
		dev_err(dev, "getparamblk request failed %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_HDR_BLOCK_ADDR(
		block_id, host_id), param_blk_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed(%x) after GET_BLK_REQ %s()\n",
				status, __func__);
		goto iaxxx_core_get_param_error;
	}

	/* Get block size of parameter block to read and validate it */
	ret = regmap_read(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(
		block_id, host_id), &read_val);
	if (ret) {
		dev_err(dev, "getparamblk blksize failed %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	getparam_block_size = (read_val &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK)
		>> IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS;

	if (!getparam_block_size ||
		getparam_block_size > getparam_block_size_in_words) {
		dev_err(dev, "invalid getparam blocksize %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	/* Get parameter block address to read */
	ret = regmap_read(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_ADDR_BLOCK_ADDR(
		block_id, host_id), &read_val);
	if (ret) {
		dev_err(dev, "getparamblk addr failed %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	/* read the block from the address */
	ret = iaxxx_btp_read(priv,
		read_val, getparam_block_data,
		getparam_block_size_in_words, host_id);

	if (ret < 0) {
		dev_err(dev, "getparamblk read failed %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	/* Write the param block done */
	write_val = ((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK);
	write_val |=
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_GET_BLK_DONE_MASK;

	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id, host_id),
		write_val);

	if (ret) {
		dev_err(dev, "getparamblk done failed %s()\n", __func__);
		goto iaxxx_core_get_param_error;
	}

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed(%x) after GET_BLK_DONE %s()\n",
			status, __func__);
		goto iaxxx_core_get_param_error;
	}

iaxxx_core_get_param_error:
	return ret;

}

/* Read Plugin Error Common code without mutex protection
 * to be shared by code using plugin mutex
 */
int iaxxx_core_read_plugin_error_common(
	struct device  *dev,
	const uint32_t  block_id,
	uint32_t *error,
	uint8_t  *error_instance)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t reg_val;

	ret = regmap_read(priv->regmap,
		IAXXX_PLUGIN_HDR_ERROR_BLOCK_ADDR(block_id, 0),
		&reg_val);
	if (ret) {
		dev_err(dev, "read plugin error failed %s()\n", __func__);
		goto read_plugin_error_err;
	}

	*error = reg_val;

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_ERROR_INS_ID_BLOCK_ADDR(block_id, 0),
			&reg_val);
	if (ret) {
		dev_err(dev, "read plugin error instance failed %s()\n",
		    __func__);
		goto read_plugin_error_err;
	}
	*error_instance = (uint8_t)reg_val;

read_plugin_error_err:
	return ret;

}

/* Set Param Block with ack Common code without mutex protection
 * to be shared by code using plugin mutex
 */
int iaxxx_core_set_param_blk_with_ack_common(
					struct device *dev,
					const uint32_t inst_id,
					const uint32_t param_blk_id,
					const uint32_t block_id,
					const void *set_param_buf,
					const uint32_t set_param_buf_sz,
					uint32_t  *response_data_buf,
					const uint32_t response_data_sz,
					const uint32_t max_no_retries)
{
	int ret;
	int max_status_retries = max_no_retries;
	uint32_t error;
	uint8_t error_instance;


	ret = iaxxx_core_set_param_blk_common(dev, inst_id,
			set_param_buf_sz,
			set_param_buf, block_id, param_blk_id);
	if (ret) {
		dev_err(dev, "Error sending set_param_buf! %s()\n",
			__func__);
		return ret;
	}

	while (max_status_retries--) {
		memset(response_data_buf, 0,
			sizeof(uint32_t)*response_data_sz);
		ret = iaxxx_core_get_param_blk_common(dev, inst_id,
				block_id, param_blk_id, response_data_buf,
				response_data_sz);
		if (ret) {
			ret = iaxxx_core_read_plugin_error_common(dev,
					block_id, &error, &error_instance);

			if (!ret && error == SYSRC_ERR_BUSY) {

				dev_err(dev,
				"Getparamblk busy..retry after delay\n");
				usleep_range(10000, 10005);
			} else {
				dev_err(dev, "Getparamblk error\n");
				return -EINVAL;
			}
		} else
			break;
	}
	return ret;
}
