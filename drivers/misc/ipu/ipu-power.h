/*
 * Power management support for the Paintbox programmable IPU
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

#ifndef __IPU_POWER_H__
#define __IPU_POWER_H__

#include <linux/types.h>

#include "ipu-client.h"

void ipu_power_enable_mmu_bif_idle_clock_gating(struct paintbox_data *pb);
void ipu_power_disable_mmu_bif_idle_clock_gating(struct paintbox_data *pb);

int ipu_power_enable_cores_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);
int ipu_power_disable_cores_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

/* The caller to these functions must hold pb->lock */
void ipu_power_enable_cores(struct paintbox_data *pb,
		unsigned int requested_cores);
void ipu_power_disable_cores(struct paintbox_data *pb,
		unsigned int requested_cores);
void ipu_power_core_power_walk_down(struct paintbox_data *pb);

void ipu_power_init(struct paintbox_data *pb);

#endif /* __IPU_POWER_H__ */
