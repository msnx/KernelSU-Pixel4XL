/*
 * iaxxx-plugin.c -- IAxxx plugin interface for Plugins
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
#include <linux/mfd/adnc/iaxxx-plugin-registers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include <linux/mfd/adnc/iaxxx-register-defs-pkg-mgmt.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-plugin-common.h>
#include <linux/mfd/adnc/iaxxx-pwr-mgmt.h>
#include "iaxxx.h"
#include "iaxxx-btp.h"
#include "ia8508a-memory-map.h"

#define IAXXX_BITS_SWAP	32
#define IAXXX_BLK_HEADER_SIZE 4
#define IAXXX_BIN_INFO_SEC_ADDR	0xF1F00000
#define IAXXX_INVALID_FILE ('\0')

/*
 * Generate package id with 'i' package id and 'p' processor id
 */
#define GEN_PKG_ID(i, p) \
	((i & IAXXX_PKG_MGMT_PKG_PROC_ID_PACKAGE_ID_MASK) | \
	((p << IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_POS) & \
	IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_MASK))

#define GET_PROC_ID_FROM_PKG_ID(pkg_id) \
	((pkg_id & IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_MASK) \
	>> IAXXX_PKG_MGMT_PKG_PROC_ID_PROC_ID_POS)

struct pkg_bin_info {
	uint32_t    version;
	uint32_t    entry_point;
	uint32_t    core_id;
	uint32_t    vendor_id;
	uint32_t    text_start_addr;
	uint32_t    text_end_addr;
	uint32_t    ro_data_start_addr;
	uint32_t    ro_data_end_addr;
	uint32_t    data_start_addr;
	uint32_t    data_end_addr;
	uint32_t    bss_start_addr;
	uint32_t    bss_end_addr;
};

struct pkg_mgmt_info {
	uint32_t req;
	uint32_t proc_id;
	uint32_t info;
	uint32_t p_text_addr;
	uint32_t v_text_addr;
	uint32_t text_size;
	uint32_t p_data_addr;
	uint32_t v_data_addr;
	uint32_t data_size;
	uint32_t entry_pt;
	uint32_t error;
};

/*****************************************************************************
 * iaxxx_core_plg_is_valid_inst_id()
 * @brief validate the plugin instance id
 *
 * @id              Plugin Instance Id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_inst_id(uint32_t inst_id)
{
	bool ret = true;

	if (inst_id > IAXXX_PLGIN_ID_MASK) {
		pr_err("%s Invalid inst id %d\n", __func__, inst_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_inst_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_pkg_id()
 * @brief validate the plugin package id
 *
 * @id              Plugin package Id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_pkg_id(uint32_t pkg_id)
{
	bool ret = true;

	if (pkg_id > IAXXX_PKG_ID_MASK) {
		pr_err("%s Invalid pkg id %d\n", __func__, pkg_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_pkg_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_priority()
 * @brief validate the plugin priority
 *
 * @id              Plugin priority
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_priority(uint32_t priority)
{
	bool ret = true;

	if (priority > (IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_MASK
				>> IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS)) {
		pr_err("%s Invalid priority %d\n", __func__, priority);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_priority);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_block_id()
 * @brief validate the plugin block id
 *
 * @id              Plugin block id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_block_id(uint32_t block_id)
{
	bool ret = true;
	uint32_t proc_id;

	proc_id = IAXXX_BLOCK_ID_TO_PROC_ID(block_id);
	if (proc_id <= IAXXX_NO_PROC || proc_id >= IAXXX_PROC_ID_NUM) {
		pr_err("%s Invalid block id %d\n", __func__, block_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_block_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_plg_idx()
 * @brief validate the plugin idx
 *
 * @id              Plugin index
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_plg_idx(uint32_t plg_idx)
{
	bool ret = true;

	if (plg_idx > (IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_MASK
			>> IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS)) {
		pr_err("%s Invalid plugin idx %d\n", __func__, plg_idx);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_plg_idx);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_id()
 * @brief validate the plugin param id
 *
 * @id              Plugin param id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_id(uint32_t param_id)
{
	bool ret = true;

	if (param_id > (IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_MASK
			>> IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_POS)) {
		pr_err("%s Invalid param id %d\n", __func__, param_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_val()
 * @brief validate the plugin idx
 *
 * @id              Plugin param value
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_val(uint32_t param_val)
{
	bool ret = true;

	if (param_val > (IAXXX_PLUGIN_INS_GRP_PARAM_VAL_MASK
			>> IAXXX_PLUGIN_INS_GRP_PARAM_VAL_POS)) {
		pr_err("%s Invalid param val %d\n", __func__, param_val);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_val);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_blk_id()
 * @brief validate the plugin param blk id
 *
 * @id              Plugin param blk id
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_blk_id(uint32_t param_id)
{
	bool ret = true;

	if (param_id > (IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_MASK
			>> IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG_POS)) {
		pr_err("%s Invalid param block id %d\n", __func__, param_id);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_blk_id);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_param_blk_size()
 * @brief validate the plugin param blk size
 *
 * @id              Plugin param blk size
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_param_blk_size(uint32_t param_size)
{
	bool ret = true;

	if (param_size > (IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK
		>> IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS)) {
		pr_err("%s Invalid param blk size %d\n", __func__, param_size);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_param_blk_size);

/*****************************************************************************
 * iaxxx_core_plg_is_valid_cfg_size()
 * @brief validate the plugin create cfg size
 *
 * @id              Plugin cfg size
 * @ret true on success, false in case of error
 ****************************************************************************/
bool iaxxx_core_plg_is_valid_cfg_size(uint32_t cfg_size)
{
	bool ret = true;

	if (cfg_size > (IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK
		>> IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS)) {
		pr_err("%s Invalid config size %d\n", __func__, cfg_size);
		ret = false;
	}
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_is_valid_cfg_size);

/*****************************************************************************
 * iaxxx_core_plugin_exist()
 * @brief check if plugin exists or not
 *
 * @priv  Pointer to iaxxx privata data structure
 * @inst_id  Instance id of a plugin
 * @ret pointer to plugin if exists, NULL otherwise
 ****************************************************************************/
struct iaxxx_plugin_data *iaxxx_core_plugin_exist(struct iaxxx_priv *priv,
							uint32_t inst_id)
{
	struct iaxxx_plugin_data *plugin_data = NULL;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	if (!list_empty_careful(&priv->iaxxx_state->plugin_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->plugin_head_list) {
			plugin_data = list_entry(node,
					struct iaxxx_plugin_data,
					plugin_node);
			if (plugin_data->inst_id == inst_id)
				goto exit;
			else
				plugin_data = NULL;
		}
	}
exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return plugin_data;
}
EXPORT_SYMBOL(iaxxx_core_plugin_exist);

/*****************************************************************************
 * iaxxx_core_pkg_exist()
 * @brief check if pkg exists or not
 *
 * @priv  Pointer to iaxxx privata data structure
 * @pkg_id  Package id of package
 * @ret pointer to package if exists, NULL otherwise
 ****************************************************************************/
struct iaxxx_pkg_data *iaxxx_core_pkg_exist(struct iaxxx_priv *priv,
						uint32_t pkg_id)
{
	struct iaxxx_pkg_data *pkg_data = NULL;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	if (!list_empty_careful(&priv->iaxxx_state->pkg_head_list)) {
		list_for_each_safe(node, tmp,
			&priv->iaxxx_state->pkg_head_list) {
			pkg_data = list_entry(node,
				struct iaxxx_pkg_data, pkg_node);
			if (pkg_data->pkg_id == pkg_id)
				goto exit;
			else
				pkg_data = NULL;
		}
	}

exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
	return pkg_data;
}
EXPORT_SYMBOL(iaxxx_core_pkg_exist);

/*****************************************************************************
 * iaxxx_core_plg_list_empty()
 * @brief check if plugin list is empty
 *
 * @priv  Pointer to iaxxx privata data structure
 * @ret true if plugin list is empty false otherwise.
 ****************************************************************************/
bool iaxxx_core_plg_list_empty(struct iaxxx_priv *priv)
{
	bool list_empty;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	list_empty = list_empty_careful(&priv->iaxxx_state->plugin_head_list);
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);

	return list_empty;

}
EXPORT_SYMBOL(iaxxx_core_plg_list_empty);

/*****************************************************************************
 * iaxxx_core_pkg_list_empty()
 * @brief check if package list is empty
 *
 * @priv  Pointer to iaxxx privata data structure
 * @ret true if package list is empty false otherwise.
 ****************************************************************************/
bool iaxxx_core_pkg_list_empty(struct iaxxx_priv *priv)
{
	bool list_empty;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	list_empty = list_empty_careful(&priv->iaxxx_state->pkg_head_list);
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);

	return list_empty;

}
EXPORT_SYMBOL(iaxxx_core_pkg_list_empty);

/*****************************************************************************
 * iaxxx_clr_pkg_plg_list()
 * @brief del, destroy and unload all plugins and packages from list.
 *
 * @priv  Pointer to iaxxx privata data structure
 *
 * @ret  SUCCESS or FAIL
 ****************************************************************************/
int iaxxx_clr_pkg_plg_list(struct iaxxx_priv *priv)
{
	struct iaxxx_pkg_data *pkg_data;
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);

	list_for_each_safe(node, tmp, &priv->iaxxx_state->plugin_head_list) {
		plugin_data = list_entry(node, struct iaxxx_plugin_data,
							plugin_node);
		list_del(&plugin_data->plugin_node);
		kvfree(plugin_data);
	}

	list_for_each_safe(node, tmp, &priv->iaxxx_state->pkg_head_list) {
		pkg_data = list_entry(node, struct iaxxx_pkg_data, pkg_node);
		list_del(&pkg_data->pkg_node);
		kvfree(pkg_data);
	}
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);

	return 0;
}
EXPORT_SYMBOL(iaxxx_clr_pkg_plg_list);

static void iaxxx_add_plugin_to_list(struct iaxxx_priv *priv,
		struct iaxxx_plugin_data *plugin_data)
{
	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	list_add_tail(&plugin_data->plugin_node,
		&priv->iaxxx_state->plugin_head_list);
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
}

static void iaxxx_del_plugin_from_list(struct iaxxx_priv *priv,
		uint32_t inst_id)
{
	struct iaxxx_plugin_data *plugin_data;
	struct list_head *node, *tmp;

	/* Search and delete the node with mutex protection
	 * to avoid the case where plugin could be cleared
	 * simultaneously (ex: by crash recovery)
	 */
	mutex_lock(&priv->iaxxx_state->plg_pkg_list_lock);
	if (!list_empty_careful(&priv->iaxxx_state->plugin_head_list)) {
		list_for_each_safe(node, tmp,
		    &priv->iaxxx_state->plugin_head_list) {
			plugin_data = list_entry(node,
					struct iaxxx_plugin_data,
					plugin_node);
			if (plugin_data->inst_id == inst_id) {
				list_del(&plugin_data->plugin_node);
				kvfree(plugin_data);
				goto exit;
			}
		}
	}
exit:
	mutex_unlock(&priv->iaxxx_state->plg_pkg_list_lock);
}

/*****************************************************************************
 * iaxxx_core_create_plg_common()
 * @brief Create plugin instance
 *
 * @id              Plugin Instance Id
 * @param_id        Param Id
 * @param_val       Param value
 * @block_id        Update block id
 * @static_package  True if the plugin is part of static package
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
static int iaxxx_core_create_plg_common(
		struct device *dev, uint32_t inst_id,
		uint32_t priority, uint32_t pkg_id,
		uint32_t plg_idx, uint8_t block_id,
		uint32_t config_id,
		bool static_package)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t package;
	uint8_t  proc_id;
	struct iaxxx_plugin_data *plugin_data;
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	dev_dbg(dev,
		"%s() inst_id=%u prio=%u pkg_id=%u plg_idx=%u blk_id=%u cfg_id=%u\n",
		__func__, inst_id, priority, pkg_id, plg_idx,
		block_id, config_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	inst_id &= IAXXX_PLGIN_ID_MASK;

	package = pkg_id & IAXXX_PKG_ID_MASK;

	/* Check Package is loaded. DO NOT check for
	 * statically loaded packages
	 */
	if (!static_package) {
		if (!iaxxx_core_pkg_exist(priv, package)) {
			dev_err(dev, "Package 0x%x is not created %s()\n",
				pkg_id, __func__);
			goto core_create_plugin_err;
		}
	}
	/* Check if plugin exists */
	if (iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x exist %s()\n",
			inst_id, __func__);
		ret = -EEXIST;
		goto core_create_plugin_err;
	}

	proc_id = IAXXX_BLOCK_ID_TO_PROC_ID(block_id);

	/* Create SysID of Package ID using Package Index
	 * and Proc ID
	 */
	pkg_id  = GEN_PKG_ID(package, proc_id);

	/* Update Package ID of plugin to be created */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(inst_id),
		IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_MASK,
		pkg_id << IAXXX_PLUGIN_INS_GRP_ORIGIN_PKG_ID_POS);

	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_create_plugin_err;
	}
	/* Update Plugin priority */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_INS_GRP_CTRL_REG(inst_id),
		IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_MASK,
		priority << IAXXX_PLUGIN_INS_GRP_CTRL_PRIORITY_POS);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_create_plugin_err;
	}

	/* Update Plugin index */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(inst_id),
		IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_MASK,
		plg_idx << IAXXX_PLUGIN_INS_GRP_ORIGIN_PLUGIN_INDEX_POS);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_create_plugin_err;
	}

	/* Update Config_id of plugin to be created */
	ret = regmap_update_bits(priv->regmap,
	IAXXX_PLUGIN_INS_GRP_ORIGIN_REG(inst_id),
	IAXXX_PLUGIN_INS_GRP_ORIGIN_CONFIG_ID_MASK,
	config_id << IAXXX_PLUGIN_INS_GRP_ORIGIN_CONFIG_ID_POS);

	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_create_plugin_err;
	}

	/* Update Plugin instance id in plg inst header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_CREATE_BLOCK_ADDR(block_id, host_id),
		1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_create_plugin_err;
	}
	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto core_create_plugin_err;
	}

	/* Insert plugin node to the list */
	plugin_data = kvzalloc(sizeof(*plugin_data), GFP_KERNEL);
	if (!plugin_data) {
		ret = -ENOMEM;
		goto core_create_plugin_err;
	}
	plugin_data->plugin_state = IAXXX_PLUGIN_LOADED;
	plugin_data->inst_id = inst_id;
	plugin_data->proc_id = proc_id;

	iaxxx_add_plugin_to_list(priv, plugin_data);

core_create_plugin_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}


/*****************************************************************************
 * iaxxx_core_create_plg()
 * @brief Create plugin instance
 *
 * @id		Plugin Instance Id
 * @param_id	Param Id
 * @param_val	Param value
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_create_plg(struct device *dev, uint32_t inst_id,
			uint32_t priority, uint32_t pkg_id,
			uint32_t plg_idx, uint8_t block_id,
			uint32_t config_id)
{
	return iaxxx_core_create_plg_common(dev, inst_id, priority,
			pkg_id, plg_idx, block_id, config_id, false);
}
EXPORT_SYMBOL(iaxxx_core_create_plg);

/*****************************************************************************
 * iaxxx_core_create_plg_static_package()
 * @brief Create plugin instance from statically loaded package
 *
 * @id      Plugin Instance Id
 * @param_id    Param Id
 * @param_val   Param value
 * @block_id    Update block id
 * @config_id   Config_id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_create_plg_static_package(
		struct device *dev, uint32_t inst_id,
		uint32_t priority, uint32_t pkg_id,
		uint32_t plg_idx, uint8_t block_id,
		uint32_t config_id)
{
	/* Generate package id using package index and
	 * block_id
	 */
	uint32_t proc_id = IAXXX_BLOCK_ID_TO_PROC_ID(block_id);

	pkg_id = GEN_PKG_ID(pkg_id, proc_id);
	return iaxxx_core_create_plg_common(dev, inst_id, priority,
			pkg_id, plg_idx, block_id, config_id, true);
}
EXPORT_SYMBOL(iaxxx_core_create_plg_static_package);


/*****************************************************************************
 * iaxxx_core_change_plg_state()
 * @brief Change plugin state to enable/disable
 *
 * @inst_id	Plugin Instance Id
 * @is_enable	0 - Disable, 1 - Enable
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_change_plg_state(struct device *dev, uint32_t inst_id,
			uint8_t is_enable, uint8_t block_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s() inst_id:%u block_id:%u enable:%u\n", __func__,
			inst_id, block_id, is_enable);
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check plugin instance is created */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
				inst_id, __func__);
		ret = -EEXIST;
		goto core_change_plg_state_err;
	}

	/* Set enable bit in plugin inst enable header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_ENABLE_BLOCK_ADDR(block_id,
		host_id), 1 << inst_id, is_enable << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_change_plg_state_err;
	}
	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret)
		dev_err(dev, "Update blk failed %s()\n", __func__);

core_change_plg_state_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_change_plg_state);

/*****************************************************************************
 * iaxxx_core_destroy_plg()
 * @brief Destroy plugin instance
 *
 * @inst_id	Plugin Instance Id
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_destroy_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	struct iaxxx_plugin_data *plugin_data;
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s() inst_id:%u block_id:%u\n", __func__,
			inst_id, block_id);
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check plugin instance is created */
	plugin_data = iaxxx_core_plugin_exist(priv, inst_id);
	if (!plugin_data) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
				inst_id, __func__);
		ret = -ENOENT;
		goto core_destroy_plg_err;
	}

	/* Clear bit in plugin instance header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_CREATE_BLOCK_ADDR(block_id, host_id),
		1 << inst_id, 0 << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_destroy_plg_err;
	}

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		goto core_destroy_plg_err;
	}

	/* Remove plugin node from list */
	iaxxx_del_plugin_from_list(priv, inst_id);

core_destroy_plg_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_destroy_plg);

/*****************************************************************************
 * iaxxx_core_reset_plg()
 * @brief Reset plugin instance
 *
 * @inst_id	Plugin Instance Id
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_reset_plg(struct device *dev, uint32_t inst_id,
				uint8_t block_id)
{
	int ret = -EINVAL;
	int rc;
	int status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;
	dev_dbg(dev, "%s() inst_id:%u block_id:%u\n", __func__,
			inst_id, block_id);
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check plugin instance is created */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
				inst_id, __func__);
		goto core_reset_plg_err;
	}

	/* Clear bit in plugin instance header */
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_RESET_BLOCK_ADDR(block_id, host_id),
		1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto core_reset_plg_err;
	}
	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		if (status) {
			/* Clear bit in plugin instance header */
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_RESET_BLOCK_ADDR(block_id,
				host_id), 1 << inst_id, 0);
			if (rc) {
				dev_err(dev, "clear failed %s() %d\n",
						__func__, rc);
				goto core_reset_plg_err;
			}
		}
		goto core_reset_plg_err;
	}

core_reset_plg_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_reset_plg);

/*****************************************************************************
 * iaxxx_core_plg_set_param_by_inst()
 * @brief Set a param in a plugin instance
 *
 * @id		Plugin Instance Id
 * @param_id	Param Id
 * @param_val	Param value
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc;
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	dev_dbg(dev, "%s() inst_id=%u param_id=%u blk_id=%u param_val=%u\n",
		__func__, inst_id, param_id, block_id, param_val);

	inst_id &= IAXXX_PLGIN_ID_MASK;
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
				inst_id, __func__);
		goto plg_set_param_inst_err;
	}
	ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(inst_id), param_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto plg_set_param_inst_err;
	}

	ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_PARAM_REG(inst_id), param_val);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto plg_set_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_SET_PARAM_REQ_BLOCK_ADDR(block_id,
		host_id), 1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "update bit failed %s()\n", __func__);
		goto plg_set_param_inst_err;
	}

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_SET_PARAM_REQ_BLOCK_ADDR(
				block_id, host_id), 1 << inst_id, 0);
			if (rc)
				dev_err(dev, "clear bit failed %s() %d\n",
						__func__, rc);
		}
		goto plg_set_param_inst_err;
	}

plg_set_param_inst_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_set_param_by_inst);


/*****************************************************************************
 * iaxxx_core_plg_get_param_by_inst()
 * @brief get a param in a plugin instance
 *
 * @id		Plugin Instance Id
 * @param_id	Param Id
 * @param_val	return param value
 * @block_id	Update block id
 *
 * @ret 0 in case of success, -EINVAL in case of error.
 ****************************************************************************/
int iaxxx_core_plg_get_param_by_inst(struct device *dev, uint32_t inst_id,
				uint32_t param_id,
				uint32_t *param_val, uint32_t block_id)
{
	int ret = -EINVAL;
	int rc;
	uint32_t status = 0;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;

	dev_dbg(dev, "%s() inst_id=%u param_id=%u blk_id=%u\n",
		__func__, inst_id, param_id, block_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
				inst_id, __func__);
		goto plg_get_param_inst_err;
	}
	ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_PARAM_ID_REG(inst_id), param_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto plg_get_param_inst_err;
	}

	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_GET_PARAM_REQ_BLOCK_ADDR(
		block_id, host_id), 1 << inst_id, 1 << inst_id);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto plg_get_param_inst_err;
	}

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
				IAXXX_PLUGIN_HDR_GET_PARAM_REQ_BLOCK_ADDR(
				block_id, host_id), 1 << inst_id, 0);
			if (rc)
				dev_err(dev, "clear bit failed %s() %d\n",
						__func__, rc);
		}
		goto plg_get_param_inst_err;
	}

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_PARAM_REG(inst_id), param_val);
	if (ret) {
		dev_err(dev, "read failed %s()\n", __func__);
		goto plg_get_param_inst_err;
	}

plg_get_param_inst_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_get_param_by_inst);

/*****************************************************************************
 * iaxxx_core_set_create_cfg()
 * @brief Set a param in a plugin instance
 *
 * @inst_id	Plugin Instance Id
 * @cfg_size	Create cfg size
 * @cfg_val	Config val
 * @block_id	Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_set_create_cfg(struct device *dev, uint32_t inst_id,
			uint32_t cfg_size, uint64_t cfg_val, uint32_t block_id,
			char *file)
{
	int ret = -EINVAL;
	uint32_t reg_addr;
	uint32_t val;
	uint32_t reg_val;
	const struct firmware *fw = NULL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint8_t *data = NULL;
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;

	dev_dbg(dev, "%s() inst_id=%u cfg_size=%u blk_id=%u\n",
		__func__, inst_id, cfg_size, block_id);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);
	/* If plugin instance already exist */
	if (iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x already exist %s()\n",
				inst_id, __func__);
		ret = -EEXIST;
		goto set_create_cfg_err;
	}

	if (file[0] != IAXXX_INVALID_FILE) {
		dev_dbg(dev, "%s() %s\n", __func__, file);
		ret = request_firmware(&fw, file, priv->dev);
		if (ret) {
			dev_err(dev, "Firmware file not found = %d\n", ret);
			ret = -EINVAL;
			goto set_create_cfg_err;
		}
		cfg_size = fw->size;
		dev_dbg(dev, "%s() cfg_size %d\n", __func__, cfg_size);
	}
	if (cfg_size > sizeof(uint32_t)) {
		if (file[0] == IAXXX_INVALID_FILE) {
			dev_dbg(dev, "%s() %llx\n", __func__, cfg_val);
			/* MSB word should be the first word to be written */
			cfg_val = (cfg_val >> IAXXX_BITS_SWAP) |
				(cfg_val << IAXXX_BITS_SWAP);
			dev_dbg(dev, "%s() cfg_val 0x%llx\n",
					__func__, cfg_val);
		} else {
			data = kvmalloc(cfg_size, GFP_KERNEL);
			if (!data) {
				ret = -ENOMEM;
				goto set_create_cfg_err;
			}
			iaxxx_copy_le32_to_cpu(data, fw->data, cfg_size);
		}

		/* Write to the ParamBlkCtrl register */
		val = (((cfg_size >> 2) <<
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_POS) &
			IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_BLK_SIZE_MASK) |
			((inst_id <<
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_POS) &
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_INSTANCE_ID_MASK) |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_SET_BLK_REQ_MASK |
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_0_IS_CREATE_CFG_MASK;

		ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_CTRL_BLOCK_ADDR(block_id, host_id),
		val);
		if (ret) {
			dev_err(dev, "write failed %s()\n", __func__);
			goto set_create_cfg_err;
		}

		ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
		if (ret) {
			dev_err(dev, "Update blk failed %s()\n", __func__);
			goto set_create_cfg_err;
		}

		ret = regmap_read(priv->regmap,
		IAXXX_PLUGIN_HDR_PARAM_BLK_ADDR_BLOCK_ADDR(block_id, host_id),
		&reg_addr);
		if (ret || !reg_addr) {
			dev_err(dev, "read failed %s()\n", __func__);
			goto set_create_cfg_err;
		}
		dev_dbg(dev, "%s() Configuration address %x\n", __func__,
			reg_addr);

		if (file[0] == IAXXX_INVALID_FILE)
			ret = iaxxx_btp_write(priv, reg_addr, &cfg_val,
						sizeof(cfg_val) /
						sizeof(uint32_t), host_id);
		else {
			ret = iaxxx_btp_write(priv, reg_addr, data,
						cfg_size / sizeof(uint32_t),
						host_id);
		}
		if (ret) {
			dev_err(dev, "Blk write failed %s()\n",
					__func__);
			goto set_create_cfg_err;
		}
	} else {
		if (file[0] == IAXXX_INVALID_FILE)
			reg_val = (uint32_t)cfg_val;
		else {
			reg_val = 0;
			iaxxx_copy_le32_to_cpu(&reg_val, fw->data, cfg_size);
		}
		pr_debug("%s() reg_val 0x%x\n", __func__, reg_val);

		ret = regmap_write(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_CREATION_CFG_REG(inst_id),
			reg_val);
		if (ret) {
			dev_err(dev, "write failed %s()\n", __func__);
			goto set_create_cfg_err;
		}
	}

set_create_cfg_err:
	kvfree(data);
	if (fw)
		release_firmware(fw);
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_set_create_cfg);

/**************************************************************************
 * iaxxx_core_set_param_blk
 * @brief Set parameter block on a plugin from given buffer
 *
 * @dev			device structure
 * @inst_id		Instance ID
 * @block_id		Block ID
 * @param_blk_id	Parameter block id
 * @blk_size		Size of buffer data in bytes
 * @ptr_blk		Pointer to buffer data
 * @ret - 0 on success, on failure < 0
 **************************************************************************/
int iaxxx_core_set_param_blk(
			struct device *dev,
			uint32_t inst_id, uint32_t blk_size,
			const void *ptr_blk, uint32_t block_id,
			uint32_t param_blk_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);
	ret = iaxxx_core_set_param_blk_common(dev, inst_id, blk_size,
				ptr_blk, block_id, param_blk_id);
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_set_param_blk);

/**************************************************************************
 *  iaxxx_core_set_param_blk_from_file
 *  @brief Set parameter block on a plugin from a file
 *
 * @dev			device structure
 * @inst_id		Instance ID
 * @block_id		Block ID
 * @param_blk_id	Parameter block id
 * @file		File in firmware directory with setparamblk data
 * @ret - 0 on success, on failure < 0
 **************************************************************************/
int iaxxx_core_set_param_blk_from_file(
			struct device *dev,
			uint32_t inst_id,
			uint32_t block_id,
			uint32_t param_blk_id,
			const char *file)
{
	int ret = -EINVAL;
	uint8_t *data = NULL;
	const struct firmware *fw = NULL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv)
		return ret;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	if (file && IAXXX_INVALID_FILE != file[0]) {
		ret = request_firmware(&fw, file, priv->dev);
		if (ret) {
			dev_err(dev, "Firmware file not found = %d\n", ret);
			ret = -EINVAL;
			goto iaxxx_core_set_param_blk_from_file_err;
		}
		data = kvmalloc(fw->size, GFP_KERNEL);
		if (!data)
			goto iaxxx_core_set_param_blk_from_file_err;
		iaxxx_copy_le32_to_cpu(data, fw->data, fw->size);
		ret = iaxxx_core_set_param_blk_common(
				dev, inst_id, fw->size, fw->data,
				block_id, param_blk_id);
	}

iaxxx_core_set_param_blk_from_file_err:
	kvfree(data);
	if (fw)
		release_firmware(fw);
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_set_param_blk_from_file);

/*****************************************************************************
 * iaxxx_core_set_event()
 * @brief Write the event enable mask to a plugin instance
 *
 * @inst_id		Plugin Instance Id
 * @event_enable_mask	Event enable mask
 * @block_id		Update block id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_set_event(struct device *dev, uint8_t inst_id,
			uint32_t event_enable_mask, uint32_t block_id)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t status = 0;
	bool host_id = find_host_id(priv, inst_id);
	int rc;

	if (!priv)
		return ret;

	inst_id &= IAXXX_PLGIN_ID_MASK;
	dev_dbg(dev, "%s() inst_id:%u block_id:%u event_en_mask:%x\n",
			__func__, inst_id, block_id, event_enable_mask);
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);
	/* Plugin instance exists or not */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		dev_err(dev, "Plugin instance 0x%x is not created %s()\n",
				inst_id, __func__);
		goto set_event_err;
	}
	ret = regmap_write(priv->regmap,
		IAXXX_PLUGIN_INS_GRP_EVT_EN_REG(inst_id),
		event_enable_mask);
	if (ret) {
		dev_err(dev, "write failed %s()\n", __func__);
		goto set_event_err;
	}
	ret = regmap_update_bits(priv->regmap,
		IAXXX_PLUGIN_HDR_EVT_UPDATE_BLOCK_ADDR(block_id,
		host_id), 1 << inst_id, 1 << inst_id);
	if (ret)
		dev_err(dev, "update bit failed %s()\n", __func__);

	ret = iaxxx_send_update_block_hostid(dev, host_id, block_id);
	if (ret) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		if (status) {
			rc = regmap_update_bits(priv->regmap,
			IAXXX_PLUGIN_HDR_EVT_UPDATE_BLOCK_ADDR(block_id,
			host_id), 1 << inst_id, 0);
			if (rc)
				dev_err(dev, "clear failed %s() %d\n",
						__func__, rc);
		}
	}

set_event_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;

}
EXPORT_SYMBOL(iaxxx_core_set_event);

/* Function to write multiple registers from a buffer pointer */
static int iaxxx_regmap_multireg_write(struct device *dev, struct regmap *map,
				unsigned int base_reg, const void *val,
				size_t word_count)
{
	int i, rc = 0;
	uint32_t *buffer = (uint32_t *) val;

	for (i = 0; i < word_count; i++) {
		rc = regmap_write(map, base_reg + i * sizeof(uint32_t),
				buffer[i]);
		if (rc) {
			dev_err(dev, "%s() regmap write error\n", __func__);
			return rc;
		}
	}
	return rc;
}

static int write_pkg_info(bool update, struct iaxxx_priv *priv, uint32_t pkg_id,
		struct pkg_bin_info bin_info, struct pkg_mgmt_info *pkg)
{
	int rc;
	struct device *dev = priv->dev;
	uint32_t status;
	uint32_t block_id;

	pkg_id &= IAXXX_PKG_ID_MASK;
	dev_dbg(dev, "Text:start:0x%pK end:0x%pK RO data:start 0x%pK end:0x%pK",
		bin_info.text_start_addr, bin_info.text_end_addr,
		bin_info.ro_data_start_addr, bin_info.ro_data_end_addr);
	dev_dbg(dev, "Data:start 0x%pK end 0x%pK BSS:start 0x%pK end 0x%pK",
		bin_info.data_start_addr, bin_info.data_end_addr,
		bin_info.bss_start_addr, bin_info.bss_end_addr);
	if (update) {
		pkg->req = 1 << IAXXX_PKG_MGMT_PKG_REQ_LOAD_POS;
		pkg->proc_id = GEN_PKG_ID(pkg_id, bin_info.core_id);
		pkg->info = bin_info.core_id |
			(bin_info.vendor_id <<
			 IAXXX_PKG_MGMT_PKG_INFO_VENDOR_ID_POS);
		pkg->v_text_addr = bin_info.text_start_addr;
		pkg->text_size = bin_info.text_end_addr -
			bin_info.text_start_addr;
		pkg->v_data_addr = bin_info.ro_data_start_addr;
		pkg->data_size = bin_info.bss_end_addr -
			bin_info.ro_data_start_addr;
		pkg->entry_pt = bin_info.entry_point;
		/* When writing this block of registers, the
		 * IAXXX_PKG_MGMT_PKG_REQ_ADDR with the guard-bit need to
		 * be written as last after writing all the info.
		 * So the register write is done here from skipping
		 * the first register.
		 * Also the last register in Package Management Block is
		 * a Read-only Error status register which doesn't need to
		 * be written.
		 * Note: The advantage of iaxxx_regmap_multireg_write allow
		 * every regmap_write could be logged in regdump.
		 */
		rc = iaxxx_regmap_multireg_write(dev, priv->regmap,
			IAXXX_PKG_MGMT_PKG_PROC_ID_ADDR, &pkg->proc_id,
			(sizeof(struct pkg_mgmt_info) / sizeof(uint32_t)) - 2);
		if (rc) {
			dev_err(dev, "Pkg info write fail %s()\n", __func__);
			return rc;
		}
	} else {
		/* Only Package-Id/Proc-id needs to be written
		 * when registering the Package
		 */
		rc = regmap_write(priv->regmap,
				IAXXX_PKG_MGMT_PKG_PROC_ID_ADDR,
				pkg->proc_id);
		if (rc) {
			dev_err(dev, "Pkg-id/Proc_id write fail %s()\n",
				__func__);
			return rc;
		}
		pkg->req = 1 << IAXXX_PKG_MGMT_PKG_REQ_REGISTER_POS;
	}

	/* This guard bit should be set to trigger the intended operation
	 * in the subsequent Update block call.
	 */
	rc = regmap_write(priv->regmap, IAXXX_PKG_MGMT_PKG_REQ_ADDR,
			pkg->req);
	if (rc) {
		dev_err(dev, "Pkg info req-bit write fail %s()\n", __func__);
		return rc;
	}
	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(bin_info.core_id);
	rc = iaxxx_send_update_block_request(dev, &status, block_id);
	if (rc) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		return rc;
	}
	return rc;
}

/* Based bin-info check if the address falls within text section */
static bool is_text_section_addr(uint32_t addr, struct pkg_bin_info *bin_info)
{
	return ((addr >= bin_info->text_start_addr) &&
		(addr <= bin_info->text_end_addr));
}

static uint32_t get_physical_address(uint32_t addr, uint32_t text,
				uint32_t data, struct pkg_bin_info *bin_info)
{
	/* Calculate the physical address to write to */
	if (is_text_section_addr(addr, bin_info))
		return (text + (addr - bin_info->text_start_addr));
	else
		return (data + (addr - bin_info->ro_data_start_addr));
}

/**
 * iaxxx_download_section_with_rom_hole_handling -
 * downloads a firmware text or data section using the function
 * iaxxx_download_section. Additionally, it checks if the physical
 * address where the section is written to falls in ROM-Hole.
 * If so, handle the downloading by skipping ROM-Hole.
 *
 * @priv    : iaxxx private data
 * @data    : the section data to be downloaded
 * @section : pointer to the section data (section address, length, etc).
 * @regmap  : regmap to use
 * @rom_hole_skipped: return a boolean flag set if ROM-Hole is skipped
 */
static int iaxxx_download_section_with_rom_hole_handling(
				struct iaxxx_priv *priv, const uint8_t *data,
				const struct firmware_section_header *section,
				struct regmap *regmap, bool btp,
				bool *rom_hole_skipped)
{
	uint32_t phy_addr_range1, phy_size_range1;
	uint32_t phy_addr_range2, phy_size_range2;
	struct firmware_section_header file_section = *section;
	int rc = 0;

	*rom_hole_skipped = false;

	/* Check for ROM-Hole in this section
	 * If found split and download the section
	 */
	phy_addr_range1 = file_section.start_address;

	rom_phy_address_range_check_and_update(
		&phy_addr_range1, file_section.length * sizeof(uint32_t),
		&phy_size_range1, &phy_addr_range2, &phy_size_range2);

	dev_dbg(priv->dev, "%s ## addr1=%pK size1=%u addr2=%pK size2=%u\n",
		__func__, phy_addr_range1, phy_size_range1, phy_addr_range2,
		phy_size_range2);

	/* If the start address has changed then
	 * it means it was moved due to ROM-Hole.
	 */
	if (phy_addr_range1 != file_section.start_address)
		*rom_hole_skipped = true;

	file_section.start_address = phy_addr_range1;
	file_section.length = phy_size_range1 / sizeof(uint32_t);

	rc = iaxxx_download_section(priv, data, &file_section, regmap, btp);

	/* If address has ROM-hole write data
	 * after the hole
	 */
	if (phy_size_range2 != 0 && phy_addr_range2 != 0) {
		file_section.start_address = phy_addr_range2;
		file_section.length = phy_size_range2 / sizeof(uint32_t);
		rc = iaxxx_download_section(priv, data + phy_size_range1,
					&file_section, regmap, btp);
		*rom_hole_skipped = true;
	}
	return rc;
}

static int iaxxx_download_pkg(struct iaxxx_priv *priv,
		const struct firmware *fw, uint32_t pkg_id, uint32_t *proc_id)
{
	const uint8_t *data;
	struct device *dev = priv->dev;
	size_t file_section_bytes;
	struct firmware_file_header header;
	struct firmware_section_header file_section = { 0x0, 0x0 };
	/* Checksum variable */
	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;
	struct pkg_bin_info bin_info = {0};
	uint32_t *word_data;
	int i, j;
	int rc = 0;
	uint32_t text_phy_addr = 0;
	uint32_t data_phy_addr = 0;
	uint8_t *buf_data;
	struct pkg_mgmt_info pkg = {0};
	bool rom_hole_skipped;
	bool is_text_section;
	bool text_sect_skipped_rom_hole = false;
	bool data_sect_skipped_rom_hole = false;

	dev_dbg(dev, "%s()\n", __func__);
	/* File header */
	if (sizeof(header) > fw->size) {
		dev_err(dev, "Bad package binary file (too small)\n");
		return -EINVAL;
	}
	iaxxx_copy_le32_to_cpu(&header, fw->data, sizeof(header));
	data = fw->data + sizeof(header);

	/* Verify the file header */
	rc = iaxxx_verify_fw_header(dev, &header);
	if (rc) {
		dev_err(dev, "Bad Package binary file\n");
		return rc;
	}
	/* Include file header fields as part of the checksum */
	CALC_FLETCHER16(header.number_of_sections, sum1, sum2);
	CALC_FLETCHER16(header.entry_point, sum1, sum2);

	/* Find the Binary info section */
	for (i = 0; i < header.number_of_sections; i++) {
		/* Load the next data section */
		if (((data - fw->data) + sizeof(file_section)) > fw->size)
			return -EINVAL;
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);
		/* Check for the magic number for the start of info section */
		if (file_section.start_address == IAXXX_BIN_INFO_SEC_ADDR) {
			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);
			if (((data - fw->data) + sizeof(bin_info))
				> fw->size)
				return -EINVAL;
			iaxxx_copy_le32_to_cpu
				(&bin_info, data, sizeof(bin_info));
			word_data = (uint32_t *)&bin_info;
			for (j = 0 ; j < file_section.length; j++)
				CALC_FLETCHER16(word_data[j], sum1, sum2);
			data += sizeof(bin_info);

			rc = iaxxx_check_and_powerup_core(priv,
					bin_info.core_id);
			if (rc) {
				dev_err(dev, "%s Failed to bring up core %d\n",
						__func__, bin_info.core_id);
				return rc;
			}

			rc = write_pkg_info(true, priv, pkg_id, bin_info, &pkg);
			if (rc) {
				dev_err(dev, "%s() Pkg info error\n", __func__);
				return rc;
			}
			break;
		} else if (file_section.length > 0)
			data += file_section.length * sizeof(uint32_t);
	}
	/* Read text and data physical address */
	rc = regmap_read(priv->regmap,
			IAXXX_PKG_MGMT_PKG_IADDR_P_ADDR, &text_phy_addr);
	if (rc) {
		dev_err(dev, "Text physical addr read failed %s %d()\n",
								__func__, rc);
		return rc;
	}
	rc = regmap_read(priv->regmap,
			IAXXX_PKG_MGMT_PKG_DADDR_P_ADDR, &data_phy_addr);
	if (rc) {
		dev_err(dev, "Data physical addr read failed %s %d()\n",
								__func__, rc);
		return rc;
	}
	dev_dbg(dev, "%s Text physical addr:0x%pK Data physical addr 0x%pK\n",
		__func__, text_phy_addr, data_phy_addr);

	data = fw->data + sizeof(header);
	/* Download sections except binary info and checksum */
	for (i = 0; i < header.number_of_sections; i++) {
		if (((data - fw->data) + sizeof(file_section)) > fw->size)
			return -EINVAL;
		iaxxx_copy_le32_to_cpu
			(&file_section, data, sizeof(file_section));
		data += sizeof(file_section);
		dev_dbg(dev, "%s Section%d addr %pK length %x\n", __func__, i,
			file_section.start_address, file_section.length);
		if (file_section.start_address == IAXXX_BIN_INFO_SEC_ADDR)
			data += sizeof(bin_info);
		else if (file_section.length) {
			file_section_bytes = file_section.length * sizeof(u32);
			/* Include section header fields in the checksum */
			CALC_FLETCHER16(file_section.length, sum1, sum2);
			CALC_FLETCHER16(file_section.start_address, sum1, sum2);

			is_text_section = is_text_section_addr(
					file_section.start_address, &bin_info);
			file_section.start_address =
				get_physical_address(file_section.start_address,
					text_phy_addr, data_phy_addr,
					&bin_info);
			dev_dbg(dev, "%s Physical address %pK\n", __func__,
				file_section.start_address);
			buf_data = kvzalloc(file_section.length *
						sizeof(uint32_t),
						GFP_KERNEL);
			if (!buf_data)
				return -ENOMEM;
			if (((data - fw->data) + (file_section.length
				* sizeof(uint32_t))) > fw->size) {
				kvfree(buf_data);
				return -EINVAL;
			}
			iaxxx_copy_le32_to_cpu(buf_data, data,
					file_section.length * sizeof(uint32_t));
			word_data = (uint32_t *)buf_data;
			for (j = 0 ; j < file_section.length; j++)
				CALC_FLETCHER16(word_data[j], sum1, sum2);

			/* Check if ROM Hole has been skipped for the current
			 * section. If so, adjust the start addresses
			 * of subsequent sections by ROME-Hole size.
			 */
			if ((is_text_section && text_sect_skipped_rom_hole) ||
				(!is_text_section &&
						data_sect_skipped_rom_hole)) {
				file_section.start_address += MAX_ADDR_ROM_SIZE;
				rc = iaxxx_download_section(priv, data,
						&file_section, priv->regmap,
						true);
			} else {
				iaxxx_download_section_with_rom_hole_handling(
						priv, data, &file_section,
						priv->regmap, true,
						&rom_hole_skipped);
				/* Check if ROM-Hole has been skipped
				 */
				if (rom_hole_skipped) {
					/* Set the flag after Rome-Hole so
					 * next section can use it
					 */
					if (is_text_section)
						text_sect_skipped_rom_hole =
							true;
					else
						data_sect_skipped_rom_hole =
							true;
				}
			}
			data += file_section_bytes;
			kvfree(buf_data);
			buf_data = NULL;
		}
	}
	/* If the last section length is 0, then verify the checksum */
	if (file_section.length == 0) {
		uint32_t checksum = (sum2 << 16) | sum1;

		dev_info(dev, "Expected checksum = 0x%.08X\n", checksum);
		if (checksum != file_section.start_address) {
			rc = -EINVAL;
			dev_err(dev, "%s(): mismatch 0x%.08X != 0x%.08X\n",
				__func__, checksum, file_section.start_address);
		}
	}
	/* Write zeros to BSS region */
	if (bin_info.bss_start_addr != bin_info.bss_end_addr) {
		file_section.start_address = data_phy_addr +
			(bin_info.bss_start_addr - bin_info.ro_data_start_addr);
		file_section.length = (bin_info.bss_end_addr
				- bin_info.bss_start_addr) >> 2;
		buf_data = kvzalloc(file_section.length *
					sizeof(uint32_t),
					GFP_KERNEL);
		if (!buf_data)
			return -ENOMEM;

		/* If ROM-Hole has been skipped in one of the
		 * data sections written already, adjust the
		 * start address of BSS section.
		 * If not, write BSS section with ROM-Hole
		 * handling.
		 */
		if (data_sect_skipped_rom_hole) {
			file_section.start_address += MAX_ADDR_ROM_SIZE;
			rc = iaxxx_download_section(priv, buf_data,
					&file_section, priv->regmap, true);
		} else
			iaxxx_download_section_with_rom_hole_handling(
					priv, buf_data, &file_section,
					priv->regmap, true, &rom_hole_skipped);
		kvfree(buf_data);
		buf_data = NULL;
	}
	/* Write to Package Management ARB */
	rc = write_pkg_info(false, priv, pkg_id, bin_info, &pkg);
	if (rc) {
		dev_err(dev, "%s() Pkg info error\n", __func__);
		return rc;
	}
	*proc_id = pkg.proc_id;
	return 0;
}

static int iaxxx_unload_pkg(struct iaxxx_priv *priv, uint32_t pkg_id,
			uint32_t proc_id)
{
	struct device *dev = priv->dev;
	uint32_t status;
	uint32_t block_id;
	int rc = 0;

	uint32_t proc_pkg_id = GEN_PKG_ID(pkg_id, proc_id);

	/* Write the package id and proc id */
	rc = regmap_write(priv->regmap, IAXXX_PKG_MGMT_PKG_PROC_ID_ADDR,
				proc_pkg_id);
	if (rc) {
		dev_err(dev,
			"%s() Write to package id (%d) register failed\n",
			__func__, pkg_id);
		return rc;
	}

	/* Write the request to unload */
	rc = regmap_write(priv->regmap, IAXXX_PKG_MGMT_PKG_REQ_ADDR,
				IAXXX_PKG_MGMT_PKG_REQ_UNLOAD_MASK);
	if (rc) {
		dev_err(dev,
			"%s() Write to package (%d) request register failed\n",
			__func__, pkg_id);
		return rc;
	}

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);
	rc = iaxxx_send_update_block_request(dev, &status, block_id);
	if (rc) {
		dev_err(dev, "Update blk failed %s()\n", __func__);
		return rc;
	}
	return 0;
}

/*****************************************************************************
 * iaxxx_package_load()
 * @brief Load the package
 *
 * @pkg_name	Package binary name
 * @pkg_id	Package Id
 * @proc_id	Package Id and Core Id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_package_load(struct device *dev, const char *pkg_name,
					uint32_t pkg_id, uint32_t *proc_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	const struct firmware *fw = NULL;
	struct iaxxx_pkg_data *pkg_data;
	int rc = -EINVAL;

	dev_info(dev, "%s() pkg_id:%u\n", __func__, pkg_id);

	if (!pkg_name) {
		dev_err(dev, "%s() Package name is NULL\n", __func__);
		return -EINVAL;
	}
	dev_info(dev, "Download Package %s\n", pkg_name);

	pkg_id &= IAXXX_PKG_ID_MASK;
	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* If package already exist */
	pkg_data = iaxxx_core_pkg_exist(priv, pkg_id);
	if (pkg_data) {
		dev_err(dev, "Package 0x%x already exist %s()\n",
				pkg_id, __func__);
		*proc_id = pkg_data->proc_id;
		rc = -EEXIST;
		goto out;
	}

	rc = request_firmware(&fw, pkg_name, priv->dev);
	if (rc) {
		dev_err(dev, "Firmware file %s not found rc = %d\n",
							pkg_name, rc);
		goto out;
	}
	rc = iaxxx_download_pkg(priv, fw, pkg_id, proc_id);
	if (rc) {
		dev_err(dev, "%s() pkg load fail %d\n", __func__, rc);
		rc = -EINVAL;
		goto out;
	}

	/* Insert package node to the list */
	pkg_data = kvzalloc(sizeof(*pkg_data), GFP_KERNEL);
	if (!pkg_data) {
		rc = -ENOMEM;
		goto out;
	}
	pkg_data->proc_id = *proc_id;
	pkg_data->pkg_id = pkg_id;
	pkg_data->pkg_state = IAXXX_PKG_LOADED;
	list_add_tail(&pkg_data->pkg_node, &priv->iaxxx_state->pkg_head_list);

out:
	release_firmware(fw);
	mutex_unlock(&priv->plugin_lock);
	return rc;
}
EXPORT_SYMBOL(iaxxx_package_load);

/*****************************************************************************
 * iaxxx_package_unload()
 * @brief Load the package
 *
 * @pkg_id	Package Id
 *
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_package_unload(struct device *dev,
			int32_t pkg_id)
{
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	int rc = -EINVAL;
	uint32_t proc_id;
	struct iaxxx_pkg_data *pkg_data;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	pkg_id &= IAXXX_PKG_ID_MASK;

	pkg_data = iaxxx_core_pkg_exist(priv, pkg_id);
	if (!pkg_data) {
		dev_err(dev, "%s() pkg not loaded already %d\n",
			__func__, pkg_id);
		rc = -ENOENT;
		goto out;
	}
	proc_id = GET_PROC_ID_FROM_PKG_ID(pkg_data->proc_id);

	rc = iaxxx_unload_pkg(priv, pkg_id, proc_id);
	if (rc) {
		dev_err(dev, "%s() pkg unload fail %d\n", __func__, rc);
		goto out;
	}

	/* Remove package node from the list */
	dev_info(dev, "%s() pkg_id:0x%x proc_id:%u\n", __func__,
							pkg_id, proc_id);
	list_del(&pkg_data->pkg_node);
	kvfree(pkg_data);

out:
	mutex_unlock(&priv->plugin_lock);
	return rc;
}
EXPORT_SYMBOL(iaxxx_package_unload);

int iaxxx_core_plg_get_package_version(struct device *dev,
			uint8_t inst_id, char *ver, uint32_t len)
{
	return iaxxx_get_version_str(to_iaxxx_priv(dev),
		IAXXX_PLUGIN_INS_GRP_PACKAGE_VER_STR_REG(inst_id),
		ver, len);
}
EXPORT_SYMBOL(iaxxx_core_plg_get_package_version);

int iaxxx_core_plg_get_plugin_version(struct device *dev,
			uint8_t inst_id, char *ver, uint32_t len)
{
	return iaxxx_get_version_str(to_iaxxx_priv(dev),
		IAXXX_PLUGIN_INS_GRP_PLUGIN_VER_STR_REG(inst_id),
		ver, len);
}
EXPORT_SYMBOL(iaxxx_core_plg_get_plugin_version);

/**************************************************************************
 *  iaxxx_core_get_param_blk
 *  @brief Get a parameter block from a plugin
 *
 * @dev					device structure
 * @inst_id				Instance ID
 * @block_id				Block ID
 * @param_blk_id			Parameter block id
 * @getparam_block_data			Pointer to buffer to copy data.
 * @getparam_block_size_in_words	Size of buffer in words.
 * @ret - 0 on success, on failure < 0
 **************************************************************************/
int iaxxx_core_get_param_blk(
		struct device *dev,
		uint32_t  inst_id,
		uint32_t  block_id,
		uint32_t  param_blk_id,
		uint32_t *getparam_block_data,
		uint32_t  getparam_block_size_in_words)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	if (!priv || !getparam_block_data || !getparam_block_size_in_words)
		return -EINVAL;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);
	ret = iaxxx_core_get_param_blk_common(dev, inst_id, block_id,
			param_blk_id,
			getparam_block_data, getparam_block_size_in_words);
	mutex_unlock(&priv->plugin_lock);
	return ret;

}
EXPORT_SYMBOL(iaxxx_core_get_param_blk);

/**************************************************************************
 *  iaxxx_core_set_param_blk_with_ack
 *  @brief Set a parameter block on a plugin and get ack to
 *  ensure the data has been sent and retry if not.
 *
 * @dev			device structure
 * @inst_id		Instance ID
 * @param_blk_id	Parameter block id
 * @block_id		Block ID
 * @set_param_buf	Pointer to the parameter block
 * @set_param_buf_sz	Parameter block size in bytes
 * @response_data_buf	Buffer for response data from plugin
 * @response_data_sz	Size of Buffer in uint32 words for
 *			response data from plugin
 * @max_no_retries	Max number of retries in case of busy
 *			response from plugin.
 * @ret - 0 on success, on failure < 0
 **************************************************************************/
int iaxxx_core_set_param_blk_with_ack(struct device *dev,
					const uint32_t inst_id,
					const uint32_t param_blk_id,
					const uint32_t block_id,
					const void *set_param_buf,
					const uint32_t set_param_buf_sz,
					uint32_t *response_data_buf,
					const uint32_t response_data_sz,
					const uint32_t max_no_retries)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t plugin_instance_id = inst_id & IAXXX_PLGIN_ID_MASK;

	if (!priv)
		return ret;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check if plugin exists */
	if (!iaxxx_core_plugin_exist(priv, inst_id)) {
		mutex_unlock(&priv->plugin_lock);
		dev_err(dev, "Plugin instance 0x%x does not exist! %s()\n",
			inst_id, __func__);
		ret = -EEXIST;
		goto set_param_blk_with_ack_err;
	}
	ret = iaxxx_core_set_param_blk_with_ack_common(dev,
			plugin_instance_id, param_blk_id, block_id,
			set_param_buf, set_param_buf_sz,
			response_data_buf,
			response_data_sz,
			max_no_retries);

set_param_blk_with_ack_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;

}
EXPORT_SYMBOL(iaxxx_core_set_param_blk_with_ack);

/*****************************************************************************
 * iaxxx_core_read_plugin_error()
 * @brief Read info on plugin error occurred.
 *
 * @dev               Core device pointer
 * @block_id	      Update block id
 * @error             Error code of error occurred
 * @error_instance    Instance of plugin where error occurred
 * @ret 0 on success, -EINVAL in case of error
 ****************************************************************************/
int iaxxx_core_read_plugin_error(
		struct device *dev,
		const uint32_t block_id,
		uint32_t *error,
		uint8_t *error_instance)
{
	int ret;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);
	ret = iaxxx_core_read_plugin_error_common(dev, block_id,
			error, error_instance);
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_read_plugin_error);

int iaxxx_core_plg_get_status_info(struct device *dev, uint32_t inst_id,
			struct iaxxx_plugin_status_data *plugin_status_data)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t plugin_inst_id = inst_id & IAXXX_PLGIN_ID_MASK;
	uint32_t block_id;
	uint32_t reg_val;
	uint8_t proc_id;
	struct iaxxx_plugin_data *plugin_data;
	bool host_id = find_host_id(priv, inst_id);

	if (!priv)
		return ret;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check if plugin exists */
	plugin_data = iaxxx_core_plugin_exist(priv, plugin_inst_id);
	if (!plugin_data) {
		dev_err(dev, "Plugin instance 0x%x does not exist! %s()\n",
			inst_id, __func__);
		ret = -EEXIST;
		goto get_plugin_status_info_err;
	}

	proc_id = plugin_data->proc_id;
	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);

	plugin_status_data->block_id = block_id;
	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_CREATE_BLOCK_ADDR(
			block_id, host_id), &reg_val);
	if (ret) {
		dev_err(dev, "plugin create status read failed %s()\n",
			__func__);
		goto get_plugin_status_info_err;
	}
	plugin_status_data->create_status = (reg_val & (1 << inst_id)) ? 1 : 0;

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_HDR_ENABLE_BLOCK_ADDR(
			block_id, host_id), &reg_val);
	if (ret) {
		dev_err(dev, "plugin enable status read failed %s()\n",
			__func__);
		goto get_plugin_status_info_err;
	}
	plugin_status_data->enable_status = (reg_val & (1 << inst_id)) ? 1 : 0;

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_PROCESS_COUNTS_REG(inst_id),
			&reg_val);
	if (ret) {
		dev_err(dev, "plugin process count read failed %s()\n",
			__func__);
		goto get_plugin_status_info_err;
	}

	plugin_status_data->process_count = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_PROCESS_COUNTS_PROCESS_COUNT_MASK) >>
		IAXXX_PLUGIN_INS_GRP_PROCESS_COUNTS_PROCESS_COUNT_POS);

	plugin_status_data->process_err_count = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_PROCESS_COUNTS_PROCESS_ERR_COUNT_MASK) >>
		IAXXX_PLUGIN_INS_GRP_PROCESS_COUNTS_PROCESS_ERR_COUNT_POS);

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_IN_FRAMES_CONSUMED_REG(inst_id),
			&reg_val);
	if (ret) {
		dev_err(dev, "plugin in frames consumed read failed %s()\n",
			__func__);
		goto get_plugin_status_info_err;
	}

	plugin_status_data->in_frames_consumed = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_IN_FRAMES_CONSUMED_COUNT_MASK) >>
		IAXXX_PLUGIN_INS_GRP_IN_FRAMES_CONSUMED_COUNT_POS);

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_OUT_FRAMES_PRODUCED_REG(inst_id),
			&reg_val);
	if (ret) {
		dev_err(dev, "plugin out frames produced read failed %s()\n",
			__func__);
		goto get_plugin_status_info_err;
	}

	plugin_status_data->out_frames_produced = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_OUT_FRAMES_PRODUCED_COUNT_MASK) >>
		IAXXX_PLUGIN_INS_GRP_OUT_FRAMES_PRODUCED_COUNT_POS);

	ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_REG(inst_id),
			&reg_val);
	if (ret) {
		dev_err(dev, "plugin info reg read failed %s()\n",
			__func__);
		goto get_plugin_status_info_err;
	}

	plugin_status_data->private_memsize = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_PRIVATE_MEMORY_SIZE_MASK) >>
		IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_PRIVATE_MEMORY_SIZE_POS);

	plugin_status_data->frame_notification_mode = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_FRAME_NOTIFICATION_MODE_MASK)
		>>
		IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_FRAME_NOTIFICATION_MODE_POS);

	plugin_status_data->state_management_mode = ((reg_val &
		IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_STATE_MANAGEMENT_MODE_MASK) >>
		IAXXX_PLUGIN_INS_GRP_PLUGIN_INFO_STATE_MANAGEMENT_MODE_POS);

get_plugin_status_info_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_get_status_info);

int iaxxx_core_plg_get_endpoint_status(struct device *dev,
	uint32_t inst_id, uint8_t ep_index, uint8_t direction,
	struct iaxxx_plugin_endpoint_status_data *plugin_ep_status_data)
{
	int ret = -EINVAL;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);
	uint32_t plugin_inst_id = inst_id & IAXXX_PLGIN_ID_MASK;
	uint32_t reg_val;

	if (!priv)
		return ret;

	/* protect this plugin operation */
	mutex_lock(&priv->plugin_lock);

	/* Check if plugin exists */
	if (!iaxxx_core_plugin_exist(priv, plugin_inst_id)) {
		dev_err(dev, "Plugin instance 0x%x does not exist! %s()\n",
			inst_id, __func__);
		ret = -EEXIST;
		goto get_plugin_ep_status_info_err;
	}

	if (direction) {
		ret = regmap_read(priv->regmap,
				IAXXX_PLUGIN_INS_GRP_OUT_EP_STATUS_REG(
				inst_id, ep_index),
				&reg_val);
		if (ret) {
			dev_err(dev, "plugin endpoint status read failed %s()\n",
				__func__);
			goto get_plugin_ep_status_info_err;
		}

		plugin_ep_status_data->status = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_SINK_STATE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_SINK_STATE_POS;
		plugin_ep_status_data->frame_status = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_FRAME_STATE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_FRAME_STATE_POS;
		plugin_ep_status_data->endpoint_status = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_ENDPOINT_STATE_MASK)
			>>
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_ENDPOINT_STATE_POS;
		plugin_ep_status_data->usage = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_USAGE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_USAGE_POS;
		plugin_ep_status_data->mandatory = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_MANDATORY_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_MANDATORY_POS;
		plugin_ep_status_data->counter = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_COUNTER_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_STATUS_COUNTER_POS;

		ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_OUT_EP_FORMAT_REG(
			inst_id, ep_index), &reg_val);
		if (ret) {
			dev_err(dev,
				"plugin endpoint format read failed %s()\n",
				__func__);
			goto get_plugin_ep_status_info_err;
		}

		plugin_ep_status_data->op_encoding = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_ENCODING_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_ENCODING_POS;
		plugin_ep_status_data->op_sample_rate = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_SAMPLE_RATE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_SAMPLE_RATE_POS;
		plugin_ep_status_data->op_frame_length = (reg_val &
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_LENGTH_MASK) >>
			IAXXX_PLUGIN_INS_GRP_OUT_0_FORMAT_LENGTH_POS;
	} else {
		ret = regmap_read(priv->regmap,
			IAXXX_PLUGIN_INS_GRP_IN_EP_STATUS_REG(
			inst_id, ep_index), &reg_val);
		if (ret) {
			dev_err(dev,
				"plugin endpoint status read failed %s()\n",
				__func__);
			goto get_plugin_ep_status_info_err;
		}

		plugin_ep_status_data->status  = (reg_val &
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_SRC_STATE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_SRC_STATE_POS;
		plugin_ep_status_data->frame_status  = (reg_val &
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_FRAME_STATE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_FRAME_STATE_POS;
		plugin_ep_status_data->endpoint_status = (reg_val &
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_ENDPOINT_STATE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_ENDPOINT_STATE_POS;
		plugin_ep_status_data->usage = (reg_val &
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_USAGE_MASK) >>
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_USAGE_POS;
		plugin_ep_status_data->mandatory = (reg_val &
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_MANDATORY_MASK) >>
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_MANDATORY_POS;
		plugin_ep_status_data->counter = (reg_val &
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_COUNTER_MASK) >>
			IAXXX_PLUGIN_INS_GRP_IN_0_STATUS_COUNTER_POS;
	}
get_plugin_ep_status_info_err:
	mutex_unlock(&priv->plugin_lock);
	return ret;
}
EXPORT_SYMBOL(iaxxx_core_plg_get_endpoint_status);

int iaxxx_core_plg_get_endpoint_timestamps(struct device *dev,
		uint64_t *timestamps, int max_timestamps, uint8_t proc_id)
{
	uint32_t val = 0;
	uint32_t block_id;
	int ep_idx;
	struct iaxxx_priv *priv = to_iaxxx_priv(dev);

	block_id = IAXXX_PROC_ID_TO_BLOCK_ID(proc_id);

	/* Check arguments */
	if (!timestamps) {
		dev_err(dev, "%s() Error in input parameters\n", __func__);
		return -EINVAL;
	}

	for (ep_idx = 0; ep_idx < max_timestamps; ++ep_idx) {
		int ret;
		uint32_t reg =
		IAXXX_PLUGIN_HDR_TIMESTAMP_LOW_EP_BLOCK_ADDR(
					block_id, ep_idx, IAXXX_HOST_0);

		ret = regmap_read(priv->regmap, reg, &val);
		if (ret) {
			dev_err(dev,
		"Reading the Timestamp register low for endpoint %d failed\n",
								ep_idx);
			return ret;
		}
		timestamps[ep_idx] = val; /* Copy to lower 32 bit */

		ret = regmap_read(priv->regmap, reg + 4, &val);
		if (ret) {
			dev_err(dev,
		"Reading the Timestamp register high for endpoint %d failed\n",
								ep_idx);
			return ret;
		}
		/* Copy to upper 32 bit */
		timestamps[ep_idx] |= (((uint64_t)(val)) << 32);
	}

	return 0;
}
EXPORT_SYMBOL(iaxxx_core_plg_get_endpoint_timestamps);
