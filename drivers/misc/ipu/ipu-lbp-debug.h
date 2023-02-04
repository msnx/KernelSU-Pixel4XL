/*
 * Linebuffer Pool Debug Support for Paintbox IPU
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

#ifndef __IPU_LBP_DEBUG_H__
#define __IPU_LBP_DEBUG_H__

#include <linux/types.h>

#include "ipu-client.h"

#if IS_ENABLED(CONFIG_IPU_DEBUG)
int ipu_lbp_dump_registers(struct paintbox_debug *debug, char *buf, size_t len);
int ipu_lb_dump_registers(struct paintbox_debug *debug, char *buf, size_t len);
void ipu_lbp_debug_init(struct paintbox_data *pb, struct paintbox_lbp *lbp);
void ipu_lbp_debug_remove(struct paintbox_data *pb, struct paintbox_lbp *lbp);
void ipu_lb_debug_init(struct paintbox_data *pb, struct paintbox_lbp *lbp,
		struct paintbox_lb *lb);
void ipu_lb_debug_remove(struct paintbox_data *pb, struct paintbox_lb *lb);
#else
static inline int ipu_lbp_dump_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	return 0;
}

static inline int ipu_lb_dump_registers(struct paintbox_debug *debug,
		char *buf, size_t len)
{
	return 0;
}

static inline void ipu_lbp_debug_init(struct paintbox_data *pb,
		struct paintbox_lbp *lbp) { }
static inline void ipu_lbp_debug_remove(struct paintbox_data *pb,
		struct paintbox_lbp *lbp) { }
static inline void ipu_lb_debug_init(struct paintbox_data *pb,
		struct paintbox_lbp *lbp, struct paintbox_lb *lb) { }
static inline void ipu_lb_debug_remove(struct paintbox_data *pb,
		struct paintbox_lb *lb) { }
#endif

#endif /* __IPU_LBP_DEBUG_H__ */
