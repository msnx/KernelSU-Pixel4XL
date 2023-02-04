
/*
 * iaxxx-plugin-common.h  -- Common functions shared between plugin-related
 *				modules
 *
 * Copyright 2018 Knowles Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IAXXX_PLUGIN_COMMON_H__
#define __IAXXX_PLUGIN_COMMON_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/kthread.h>

int iaxxx_core_set_param_blk_common(
			struct device *dev,
			uint32_t inst_id, uint32_t blk_size,
			const void *ptr_blk, uint32_t block_id,
			uint32_t param_blk_id);
int iaxxx_core_get_param_blk_common(
		struct device *dev,
		uint32_t  inst_id,
		uint32_t  block_id,
		uint32_t  param_blk_id,
		uint32_t *getparam_block_data,
		uint32_t  getparam_block_size_in_words);

int iaxxx_core_read_plugin_error_common(
	struct device  *dev,
	const uint32_t  block_id,
	uint32_t *error,
	uint8_t  *error_instance);

int iaxxx_core_set_param_blk_with_ack_common(
					struct device *dev,
					const uint32_t inst_id,
					const uint32_t param_blk_id,
					const uint32_t block_id,
					const void *set_param_buf,
					const uint32_t set_param_buf_sz,
					uint32_t  *response_data_buf,
					const uint32_t response_data_sz,
					const uint32_t max_no_retries);
struct iaxxx_plugin_data *iaxxx_core_plugin_exist(
		struct iaxxx_priv *priv,
		uint32_t inst_id);

struct iaxxx_pkg_data *iaxxx_core_pkg_exist(struct iaxxx_priv *priv,
		uint32_t pkg_id);

bool iaxxx_core_plg_list_empty(struct iaxxx_priv *priv);
bool iaxxx_core_pkg_list_empty(struct iaxxx_priv *priv);
#endif /*__IAXXX_PLUGIN_COMMON_H__ */
