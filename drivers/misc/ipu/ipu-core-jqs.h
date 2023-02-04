/*
 * Core JQS management support for the Paintbox programmable IPU
 *
 * Copyright (C) 2018 Google, Inc.
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

#ifndef __IPU_CORE_JQS_H__
#define __IPU_CORE_JQS_H__

#include <linux/ipu-core.h>
#include <linux/types.h>

/* The caller to these functions must hold bus->jqs.lock */
int ipu_core_jqs_enable_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_disable_firmware_requested(struct paintbox_bus *bus);
void ipu_core_jqs_disable_firmware_suspended(struct paintbox_bus *bus);
void ipu_core_jqs_disable_firmware_fatal_error(struct paintbox_bus *bus);

int ipu_core_jqs_load_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_unload_firmware(struct paintbox_bus *bus);
int ipu_core_jqs_stage_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_unstage_firmware(struct paintbox_bus *bus);

void ipu_core_jqs_resume_firmware(struct paintbox_bus *bus,
		uint64_t ipu_clock_rate_hz);
void ipu_core_jqs_suspend_firmware(struct paintbox_bus *bus);
void ipu_core_jqs_shutdown_firmware(struct paintbox_bus *bus);

/* The caller to these functions must hold bus->jqs.lock */
int ipu_core_jqs_send_clock_rate(struct paintbox_bus *bus,
		uint32_t clock_rate_hz);
int ipu_core_jqs_send_set_log_info(struct paintbox_bus *bus);
int ipu_core_jqs_send_shutdown_mode(struct paintbox_bus *bus,
		enum jqs_shutdown_mode shutdown_mode);

int ipu_core_jqs_init(struct paintbox_bus *bus);
void ipu_core_jqs_remove(struct paintbox_bus *bus);

#endif /* __IPU_CORE_JQS_H__ */
