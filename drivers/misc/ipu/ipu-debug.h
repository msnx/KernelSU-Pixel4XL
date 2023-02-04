/*
 * Debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __IPU_DEBUG_H__
#define __IPU_DEBUG_H__

#include <linux/types.h>

#include "ipu-client.h"

#if IS_ENABLED(CONFIG_IPU_DEBUG)
#define REG_NAME_COLUMN_NUMBER 8
#define REG_VALUE_COLUMN_NUMBER 44
#define REG_VALUE_COL_WIDTH (REG_VALUE_COLUMN_NUMBER - REG_NAME_COLUMN_NUMBER)

int ipu_debug_vprintf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, va_list args);

int ipu_debug_printf(struct paintbox_data *pb, char *buf, int *written,
		size_t len, const char *format, ...);

int ipu_debug_dump_register(struct paintbox_data *pb, uint32_t group_base,
		uint32_t reg_offset, const char *reg_name, char *buf,
		int *written, size_t len);

int ipu_debug_dump_register_with_value(struct paintbox_data *pb,
		uint32_t group_base, uint32_t reg_offset, uint64_t reg_value,
		const char *reg_name, char *buf, int *written, size_t len);

void ipu_debug_create_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, struct dentry *debug_root,
		const char *name, unsigned int resource_id,
		register_dump_t register_dump, stats_dump_t stats_dump,
		void *arg);
void ipu_debug_free_entry(struct paintbox_debug *debug);

int ipu_debug_alloc_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, size_t reg_count);
void ipu_debug_free_reg_entries(struct paintbox_debug *debug);

int ipu_debug_create_reg_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, unsigned int index,
		const char *reg_name, uint32_t reg_offset,
		register_write_t reg_write, register_read_t reg_read);

void ipu_debug_create_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, const char **reg_names,
		size_t reg_count, register_write_t reg_write,
		register_read_t reg_read);

void ipu_debug_init(struct paintbox_data *pb);
void ipu_debug_remove(struct paintbox_data *pb);
#else
static inline int ipu_debug_vprintf(struct paintbox_data *pb, char *buf,
		int *written, size_t len, const char *format, va_list args)
{
	return 0;
}

static inline int ipu_debug_printf(struct paintbox_data *pb, char *buf,
		int *written, size_t len, const char *format, ...)
{
	return 0;
}

static inline int ipu_debug_dump_register(struct paintbox_data *pb,
		uint32_t group_base, uint32_t reg_offset, const char *reg_name,
		char *buf, int *written, size_t len)
{
	return 0;
}

static inline int ipu_debug_dump_register_with_value(struct paintbox_data *pb,
		uint32_t group_base, uint32_t reg_offset, uint64_t reg_value,
		const char *reg_name, char *buf, int *written, size_t len)
{
	return 0;
}

static inline void ipu_debug_create_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, struct dentry *debug_root,
		const char *name, unsigned int resource_id,
		register_dump_t register_dump, stats_dump_t stats_dump,
		void *arg) {}
static inline void ipu_debug_free_entry(struct paintbox_debug *debug) {}

static inline int ipu_debug_alloc_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, size_t reg_count)
{
	return 0;
}

static inline void ipu_debug_free_reg_entries(struct paintbox_debug *debug) {}

static inline int ipu_debug_create_reg_entry(struct paintbox_data *pb,
		struct paintbox_debug *debug, unsigned int index,
		const char *reg_name, uint32_t reg_offset,
		register_write_t reg_write, register_read_t reg_read)
{
	return 0;
}

static inline void ipu_debug_create_reg_entries(struct paintbox_data *pb,
		struct paintbox_debug *debug, const char **reg_names,
		size_t reg_count, register_write_t reg_write,
		register_read_t reg_read) {}

static inline void ipu_debug_init(struct paintbox_data *pb) {}
static inline void ipu_debug_remove(struct paintbox_data *pb) {}
#endif

#endif /* __IPU_DEBUG_H__ */
