/*
 * iaxxx-module.c -- IAxxx module interface for managers
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
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/mm.h>
#include <linux/mfd/adnc/iaxxx-core.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-sensor-header.h>
#include <linux/mfd/adnc/iaxxx-register-defs-script-mgmt.h>
#include <linux/mfd/adnc/iaxxx-sensor-registers.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-module.h>
#include "iaxxx.h"
#include "iaxxx-btp.h"

#define IAXXX_MODULE_WORD_SIZE_SHIFT	(2)

/*****************************************************************************
 * iaxxx_core_sensor_is_valid_script_id()
 * @brief validate the sensor script id
 *
 * @id              sensor script id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_script_id(uint32_t script_id)
{
	bool ret = true;

	if (script_id > IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK
				>> IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS)
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_script_id);

/*****************************************************************************
 * iaxxx_core_sensor_is_valid_inst_id()
 * @brief validate the sensor instance id
 *
 * @id              Sensor Instance Id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_inst_id(uint32_t inst_id)
{
	bool ret = true;

	if (inst_id > IAXXX_SENSR_ID_MASK)
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_inst_id);

/*****************************************************************************
 * iaxxx_core_sensor_is_valid_block_id()
 * @brief validate the sensor block id
 *
 * @id              Sensor block id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_block_id(uint32_t block_id)
{
	bool ret = true;
	uint32_t proc_id;

	proc_id = IAXXX_BLOCK_ID_TO_PROC_ID(block_id);
	if (proc_id <= IAXXX_NO_PROC || proc_id >= IAXXX_PROC_ID_NUM)
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_block_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_id()
 * @brief validate the sensor param id
 *
 * @id              sensor param id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_param_id(uint32_t param_id)
{
	bool ret = true;

	if (param_id > (IAXXX_SENSOR_GRP_PARAM_ID_REG_MASK
				>> IAXXX_SENSOR_GRP_PARAM_ID_REG_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_param_id);

/*****************************************************************************
 * iaxxx_core_sensor_is_valid_param_val()
 * @brief validate the sensor param value
 *
 * @id              sensor param value
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_param_val(uint32_t param_val)
{
	bool ret = true;

	if (param_val > (IAXXX_SENSOR_GRP_PARAM_VAL_MASK
					>> IAXXX_SENSOR_GRP_PARAM_VAL_POS))
		ret = false;
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_param_val);


/*****************************************************************************
 * iaxxx_core_sensor_is_valid_param_blk_id()
 * @brief validate the sensor param blk id
 *
 * @id	Sensor param blk id
 * @ret	true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_param_blk_id(uint32_t param_id)
{
	bool ret = true;

	if (param_id > (IAXXX_SENSOR_HDR_PARAM_BLK_ID_REG_MASK >>
		IAXXX_SENSOR_HDR_PARAM_BLK_ID_REG_POS)) {
		pr_err("%s Invalid param block id %d\n", __func__, param_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_param_blk_id);

/*****************************************************************************
 * iaxxx_core_sensor_is_valid_param_blk_size()
 * @brief validate the sensor param blk size
 *
 * @id	Sensor param blk size
 * @ret	true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_sensor_is_valid_param_blk_size(uint32_t param_size)
{
	bool ret = true;

	if (param_size > (IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_BLK_SIZE_MASK >>
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_BLK_SIZE_POS)) {
		pr_err("%s Invalid param blk size %d\n", __func__, param_size);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_is_valid_param_blk_size);

/*****************************************************************************
 * iaxxx_core_change_sensor_state()
 * @brief Change sensor state to enable/disable
 *
 * @inst_id	Sensor Instance Id
 * @is_enable	0 - Disable, 1 - Enable
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_sensor_change_state(struct device *dev, uint32_t inst_id,
			uint8_t is_enable, uint8_t block_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s()\n", __func__);
	/* Set enable bit in sensor inst enable header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_SENSOR_HDR_SENSOR_ENABLE_ADDR,
		1 << inst_id,
		is_enable << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_change_sensor_state_err;
	}
	/* update on sensor enable and disable */
	priv->sensor_en[inst_id] = is_enable;

core_change_sensor_state_err:
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_change_state);

/*****************************************************************************
 * iaxxx_core_sensor_set_param_by_inst()
 * @brief Set a param in a sensor instance
 *
 * @id		Sensor Instance Id
 * @param_id	Param Id
 * @param_val	Param value
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_sensor_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_SENSR_ID_MASK;

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_ID_REG(inst_id), param_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_REG(inst_id), param_val);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
			IAXXX_SENSOR_HDR_SET_PARAM_REQ_ADDR,
			1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "update bit failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto sensor_set_param_inst_err;
	}

sensor_set_param_inst_err:
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_set_param_by_inst);

/*****************************************************************************
 * iaxxx_core_sensor_get_param_by_inst()
 * @brief get a param in a sensor instance
 *
 * @id		Sensor Instance Id
 * @param_id	Param Id
 * @param_val	return param value
 * @block_id	Update block id
 *
 * @ret 0 in case of success, -EINVAL in case of error.
 ****************************************************************************/
int iaxxx_core_sensor_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	inst_id &= IAXXX_SENSR_ID_MASK;
	dev_dbg(dev, "%s()\n", __func__);

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_ID_REG(inst_id), param_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
			IAXXX_SENSOR_HDR_GET_PARAM_REQ_ADDR,
			1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

	ret = regmap_read(priv->regmap,
			IAXXX_SENSOR_GRP_PARAM_REG(inst_id), param_val);
	if (ret) {
		dev_err(dev, "read failed %s()\n", __func__);
		goto sensor_get_param_inst_err;
	}

sensor_get_param_inst_err:
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_get_param_by_inst);

/*****************************************************************************
 * iaxxx_core_sensor_write_param_blk_by_inst()
 * @brief Writes data to sensor param block
 *
 * @inst_id		Sensor Instance Id
 * @param_blk_id	param block ID
 * @blk_data		pointer to data buffer
 * @blk_size		pointer to size of data buffer passed
 * @block_id		Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_sensor_write_param_blk_by_inst(struct device *dev,
				uint32_t inst_id, uint32_t param_blk_id,
				const void *ptr_blk, uint32_t blk_size,
				uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t reg_val = 0;
	uint32_t blk_addr;

	if (!priv)
		return ret;

	dev_dbg(dev, "%s() inst_id:%d, param_blk_id:%d, blk_size:%d\n",
		__func__, inst_id, param_blk_id, blk_size);

	inst_id &= IAXXX_SENSR_ID_MASK;

	/* The block size is divided by 4 here because this function gets it
	 * as block size in bytes but firmware expects in 32bit words.
	 */
	reg_val = (((blk_size >> IAXXX_MODULE_WORD_SIZE_SHIFT) <<
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_BLK_SIZE_POS) &
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_BLK_SIZE_MASK);
	reg_val |= ((inst_id <<
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_INSTANCE_ID_POS) &
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_INSTANCE_ID_MASK);
	reg_val |= IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_SET_BLK_REQ_MASK;
	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_ADDR, reg_val);
	if (ret) {
		dev_err(dev, "write SENSOR_HDR_PARAM_BLK_CTRL failed %s()\n",
			__func__);
		goto sensor_write_param_blk_err;
	}

	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_HDR_PARAM_BLK_ID_ADDR, param_blk_id);
	if (ret) {
		dev_err(dev, "write SENSOR_HDR_PARAM_BLK_ID_ADDR failed %s()\n",
			__func__);
		goto sensor_write_param_blk_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto sensor_write_param_blk_err;
	}

	ret = regmap_read(priv->regmap,
			IAXXX_SENSOR_HDR_PARAM_BLK_ADDR_ADDR, &blk_addr);
	if (ret) {
		dev_err(dev, "%s() blk addr req read failed: %d\n",
			__func__, ret);
		goto sensor_write_param_blk_err;
	}

	ret = iaxxx_btp_write(priv, blk_addr, ptr_blk,
			blk_size / sizeof(uint32_t), IAXXX_HOST_0);

	if (ret) {
		dev_err(dev, "%s() Raw blk write failed\n", __func__);
		goto sensor_write_param_blk_err;
	}

	/* Write the param block done */
	/* The block size is divided by 4 here because this function gets it
	 * as block size in bytes but firmware expects in 32bit words.
	 */
	reg_val = (((blk_size >> IAXXX_MODULE_WORD_SIZE_SHIFT) <<
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_BLK_SIZE_POS) &
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_BLK_SIZE_MASK);
	reg_val |= ((inst_id <<
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_INSTANCE_ID_POS) &
		IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_INSTANCE_ID_MASK);
	reg_val |= IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_SET_BLK_DONE_MASK;
	ret = regmap_write(priv->regmap,
			IAXXX_SENSOR_HDR_PARAM_BLK_CTRL_ADDR, reg_val);
	if (ret) {
		dev_err(dev, "write SENSOR_HDR_PARAM_BLK_CTRL failed %s()\n",
			__func__);
		goto sensor_write_param_blk_err;
	}

	ret = iaxxx_send_update_block_request(dev, &status, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto sensor_write_param_blk_err;
	}

sensor_write_param_blk_err:
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_sensor_write_param_blk_by_inst);

/*****************************************************************************
 * iaxxx_core_get_sensor_mode_stats()
 * Get mode transition stats for each sensor mdoe.
 *
 * @sensor_stats sensor mode transition stats
 *
 * @ret n number of words read, ret in case of error
 ****************************************************************************/
int iaxxx_core_get_sensor_mode_stats(struct device *dev,
			struct iaxxx_sensor_mode_stats *sensor_stats)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t sensor_stats_addr, sensor_stats_size;
	int ret = 0;

	ret = regmap_read(priv->regmap,
			IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_ADDR_ADDR,
			&sensor_stats_addr);
	if (ret) {
		dev_err(dev, "Read addr failed %d\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap,
			IAXXX_SENSOR_GRP_SENSOR_MODE_STATS_SIZE_ADDR,
			&sensor_stats_size);
	if (ret) {
		dev_err(dev, "Read size failed %d\n", ret);
		return ret;
	}

	if (sensor_stats_size != ((sizeof(struct iaxxx_sensor_mode_stats) *
					SENSOR_NUM_MODE) >> 2)) {
		dev_err(dev,
			"sensor_stats_size %d != struct size %lu\n",
			sensor_stats_size,
			sizeof(struct iaxxx_sensor_mode_stats) *
			SENSOR_NUM_MODE);
		return -EINVAL;
	}

	ret = priv->bulk_read(priv->dev, sensor_stats_addr, sensor_stats,
				sensor_stats_size);
	if (ret < 0) {
		dev_err(priv->dev, "Not able to read sensor stats %d\n", ret);
		return ret;
	}

	dev_dbg(priv->dev, "read sensor stats successfully in words %u\n",
		sensor_stats_size);

	return ret;
}
EXPORT_SYMBOL(iaxxx_core_get_sensor_mode_stats);

static int iaxxx_download_script(struct iaxxx_priv *priv,
				const struct firmware *fw, uint16_t script_id)
{
	struct device *dev = priv->dev;
	size_t script_size = 0;
	uint32_t script_addr;
	const uint8_t *data;
	/* Checksum variable */
	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;
	unsigned int finalsum1 = 0xffff;
	unsigned int finalsum2 = 0xffff;
	uint32_t *word_data;
	int i;
	int rc = 0;
	uint32_t value = 0;
	uint8_t *buf_data;
	uint32_t status;
	uint32_t checksum;

	dev_dbg(dev, "%s()\n", __func__);

	/* fw->size = script_size + checksum_size (sizeof(uint32_t))
	 * so it should be more than 4 bytes
	 */
	if (fw->size <= 4) {
		dev_err(dev, "%s() bad script binary file size: %zu\n",
			__func__, fw->size);
		return -EINVAL;
	}
	script_size = fw->size - sizeof(uint32_t);

	buf_data = kvzalloc(fw->size, GFP_KERNEL);
	if (!buf_data)
		return -ENOMEM;

	data = fw->data;

	iaxxx_copy_le32_to_cpu(buf_data, data, fw->size);
	word_data = (uint32_t *)buf_data;

	/* Include file header fields as part of the checksum */
	for (i = 0 ; i < (script_size >> 2); i++)
		CALC_FLETCHER16(word_data[i], sum1, sum2);

	checksum = (sum2 << 16) | sum1;

	dev_dbg(dev, "%s() calculated binary checksum = 0x%.08X\n",
		__func__, checksum);
	if (checksum != word_data[(script_size >> 2)]) {
		dev_err(dev, "%s(): script bin mismatch 0x%.08X != 0x%.08X\n",
			__func__, checksum, word_data[(script_size >> 2)]);
		goto out;
	}

	value = IAXXX_SCRIPT_MGMT_SCRIPT_REQ_LOAD_MASK |
		(IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK &
		(script_id << IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS));
	rc = regmap_write(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_REQ_ADDR, value);
	if (rc) {
		dev_err(dev,
			"%s() write to script req (%u) register failed: %d\n",
			__func__, script_id, rc);
		goto out;
	}

	rc = regmap_write(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_SIZE_ADDR, script_size);
	if (rc) {
		dev_err(dev,
			"%s() write to script size (%zu) register failed: %d\n",
			__func__, script_size, rc);
		goto out;
	}

	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		/* Read script error */
		rc = regmap_read(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_ERROR_ADDR, &status);
		if (rc) {
			dev_err(dev, "%s() script error read failed: %d\n",
				__func__, rc);
			goto out;
		}
		rc = -EIO;
		dev_err(dev, "%s() script error reg status: %d rc: %d\n",
			__func__, status, rc);
		goto out;
	}

	rc = regmap_read(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_ADDR_ADDR, &script_addr);
	if (rc) {
		dev_err(dev, "%s() script addr req read failed: %d\n",
			__func__, rc);
		goto out;
	}
	dev_dbg(dev, "%s() script write physical addr:0x%08x\n",
		__func__, script_addr);

	/* Write Package Binary information */
	rc = iaxxx_btp_write(priv, script_addr,
			(uint32_t *)buf_data, script_size >> 2, IAXXX_HOST_0);
	if (rc) {
		dev_err(dev, "%s() script download failed addr: 0x%08x\n",
			__func__, script_addr);
		goto out;
	}

	/* Include checksum for this section */
	rc = iaxxx_checksum_request(priv,
				script_addr, (script_size >> 2),
				&finalsum1, &finalsum2, priv->regmap);
	if (rc) {
		dev_err(dev, "%s() script Checksum request error: %d\n",
			__func__, rc);
		goto out;
	}

	dev_dbg(dev, "final calculated binary checksum = 0x%04X-0x%04X\n",
		finalsum1, finalsum2);
	if (finalsum1 != sum1 || finalsum2 != sum2) {
		dev_err(dev, "script after download Checksum failed\n");
		dev_err(dev,
			"checksum mismatch 0x%04X-0x%04X != 0x%04X-0x%04X\n",
			sum2, sum1, finalsum2, finalsum1);
		rc = -EINVAL;
		goto out;
	}

	dev_dbg(dev, "%s() script download successful", __func__);
out:
	kvfree(buf_data);
	return rc;
}

/*****************************************************************************
 * iaxxx_core_script_load()
 * @brief Load the script
 *
 * @script_name	Script binary name
 * @script_id	Script Id
 *
 * @ret 0 on success, error in case of failure
 ****************************************************************************/
int iaxxx_core_script_load(struct device *dev, const char *script_name,
			uint32_t script_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	const struct firmware *fw = NULL;
	int rc = -EINVAL;

	dev_dbg(dev, "%s()\n", __func__);

	if (!script_name) {
		dev_err(dev, "%s() Script name is NULL: %s\n",
			__func__, script_name);
		return -EINVAL;
	}
	dev_dbg(dev, "%s() Download Script %s\n",
				__func__, script_name);

	rc = request_firmware(&fw, script_name, priv->dev);
	if (rc || !fw) {
		dev_err(dev,
			"%s() request firmware %s image failed rc = %d\n",
			__func__, script_name, rc);
		goto out;
	}

	rc = iaxxx_download_script(priv, fw, script_id);
	if (rc) {
		dev_err(dev, "%s() script load fail %d\n", __func__, rc);
		goto out;
	}
	dev_info(dev, "%s() script load success\n", __func__);
out:
	release_firmware(fw);
	return rc;
}
EXPORT_SYMBOL(iaxxx_core_script_load);

/*****************************************************************************
 * iaxxx_core_script_unload()
 * @brief UnLoad the script
 *
 * @script_id	Script Id
 *
 * @ret 0 on success, error in case of failure
 ****************************************************************************/
int iaxxx_core_script_unload(struct device *dev, uint32_t script_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t value = 0;
	uint32_t status = 0;
	int rc = -EINVAL;

	dev_dbg(dev, "%s()\n", __func__);

	value = IAXXX_SCRIPT_MGMT_SCRIPT_REQ_UNLOAD_MASK |
		(IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK &
		(script_id << IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS));
	rc = regmap_write(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_REQ_ADDR, value);
	if (rc) {
		dev_err(dev,
			"%s() Write to script req (%d) register failed: %d\n",
			__func__, script_id, rc);
		goto out;
	}

	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		/* Read script error */
		rc = regmap_read(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_ERROR_ADDR, &status);
		if (rc) {
			dev_err(dev, "%s() script error read failed: %d\n",
				__func__, rc);
			goto out;
		}
		rc = -EIO;
		dev_err(dev, "%s() script error reg status: %d rc: %d\n",
			__func__, status, rc);
		goto out;
	}
	dev_info(dev, "%s() script unload success\n", __func__);
out:
	return rc;
}
EXPORT_SYMBOL(iaxxx_core_script_unload);

/*****************************************************************************
 * iaxxx_core_script_trigger()
 * @brief trigger the script
 *
 * @script_id	Script Id
 *
 * @ret 0 on success, error in case of failure
 ****************************************************************************/
int iaxxx_core_script_trigger(struct device *dev, uint32_t script_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t value = 0;
	uint32_t status = 0;
	int rc = -EINVAL;

	dev_dbg(dev, "%s()\n", __func__);

	value = IAXXX_SCRIPT_MGMT_SCRIPT_REQ_EXECUTE_MASK |
		(IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_MASK &
		(script_id << IAXXX_SCRIPT_MGMT_SCRIPT_REQ_SCRIPT_ID_POS));
	rc = regmap_write(priv->regmap,
			IAXXX_SCRIPT_MGMT_SCRIPT_REQ_ADDR, value);
	if (rc) {
		dev_err(dev,
			"%s() Write to script req (%d) register failed: %d\n",
			__func__, script_id, rc);
		goto out;
	}

	rc = iaxxx_send_update_block_request(dev, &status, IAXXX_BLOCK_0);
	if (rc) {
		/* Read script error */
		rc = regmap_read(priv->regmap,
				IAXXX_SCRIPT_MGMT_SCRIPT_ERROR_ADDR, &status);
		if (rc) {
			dev_err(dev, "%s() script error read failed: %d\n",
				__func__, rc);
			goto out;
		}
		rc = -EIO;
		dev_err(dev, "%s() script error reg status: %d rc: %d\n",
			__func__, status, rc);
		goto out;
	}
	dev_info(dev, "%s() script execute success\n", __func__);
out:
	return rc;
}
EXPORT_SYMBOL(iaxxx_core_script_trigger);
