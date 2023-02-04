/*
 * Core JQS debug management support for the Paintbox programmable IPU
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

#ifndef __IPU_CORE_JQS_DEBUG_H__
#define __IPU_CORE_JQS_DEBUG_H__

#include <linux/ipu-core.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_IPU_DEBUG)
void ipu_core_jqs_debug_init(struct paintbox_bus *bus);
void ipu_core_jqs_debug_remove(struct paintbox_bus *bus);
#else
static inline void ipu_core_jqs_debug_init(struct paintbox_bus *bus) {}
static inline void ipu_core_jqs_debug_remove(struct paintbox_bus *bus) {}
#endif

#endif /* __IPU_CORE_JQS_DEBUG_H__ */
