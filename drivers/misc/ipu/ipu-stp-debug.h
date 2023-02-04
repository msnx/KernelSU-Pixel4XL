/*
 * STP debug support for the Paintbox programmable IPU
 *
 * Copyright (C) 2017 Google, Inc.
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

#ifndef __IPU_STP_DEBUG_H__
#define __IPU_STP_DEBUG_H__

#include <linux/types.h>

#include "ipu-client.h"

#if IS_ENABLED(CONFIG_IPU_DEBUG)
void ipu_stp_debug_init(struct paintbox_data *pb, struct paintbox_stp *stp);
void ipu_stp_debug_remove(struct paintbox_data *pb, struct paintbox_stp *stp);
#else
static inline void ipu_stp_debug_init(struct paintbox_data *pb,
		struct paintbox_stp *stp) {}
static inline void ipu_stp_debug_remove(struct paintbox_data *pb,
		struct paintbox_stp *stp) {}
#endif

#endif /* __IPU_STP_DEBUG_H__ */
