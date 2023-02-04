/*
 * MMU support for the Paintbox programmable IPU
 *
 * Copyright (C) 2019 Google, Inc.
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

#ifndef __IPU_MMU_H__
#define __IPU_MMU_H__

#include <linux/kernel.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-debug.h"

#if IS_ENABLED(CONFIG_IPU_DEBUG)
int ipu_mmu_debug_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void ipu_mmu_remove(struct paintbox_data *pb);
#else
int ipu_mmu_debug_init(struct paintbox_data *pb) {return 0; }

/* All sessions must be released before remove can be called. */
void ipu_mmu_remove(struct paintbox_data *pb) {}
#endif
#endif  /* __IPU_MMU_H__ */
