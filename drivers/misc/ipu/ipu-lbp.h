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

#ifndef __IPU_LBP_H__
#define __IPU_LBP_H__

#include <linux/io.h>

#include "ipu-client.h"
#include "ipu-regs.h"

/* The caller to this function must hold pb->lock */
static inline void ipu_lbp_select(struct paintbox_data *pb, unsigned int lbp_id)
{
	ipu_writel(pb->dev, lbp_id | LBP_SEL_LB_SEL_M <<
			LBP_SEL_LB_SEL_SHIFT, IPU_CSR_LBP_OFFSET + LBP_SEL);
}

/* The caller to this function must hold pb->lock */
static inline void ipu_lb_select(struct paintbox_data *pb, unsigned int lbp_id,
		unsigned int lb_id)
{
	ipu_writel(pb->dev, lbp_id | lb_id << LBP_SEL_LB_SEL_SHIFT,
			IPU_CSR_LBP_OFFSET + LBP_SEL);
}

int ipu_lbp_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void ipu_lbp_remove(struct paintbox_data *pb);

#endif /* __IPU_LBP_H__ */
