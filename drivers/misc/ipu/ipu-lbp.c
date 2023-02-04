/*
 * Line Buffer Pool Support for Paintbox IPU
 *
 * Copyright (C) 2015 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-lbp.h"
#include "ipu-lbp-debug.h"
#include "ipu-regs.h"

static int ipu_lbp_init_line_buffer_pool(struct paintbox_data *pb,
		unsigned int lbp_index)
{
	struct paintbox_lbp *lbp;
	unsigned int i;

	lbp = &pb->lbp.lbps[lbp_index];

	/* Store pool id with object as a convenience to avoid doing a lookup
	 * later on.
	 */
	lbp->pool_id = lbp_index;

	lbp->lbs = kcalloc(pb->lbp.max_lbs, sizeof(struct paintbox_lb),
			GFP_KERNEL);
	if (!lbp->lbs)
		return -ENOMEM;

	ipu_lbp_debug_init(pb, lbp);

	for (i = 0; i < pb->lbp.max_lbs; i++) {
		struct paintbox_lb *lb = &lbp->lbs[i];

		lb->lbp = lbp;
		lb->lb_id = i;

		ipu_lb_debug_init(pb, lbp, lb);
	}

	return 0;
}

int ipu_lbp_init(struct paintbox_data *pb)
{
	unsigned int i;
	int ret;

	/* TODO(b/114734817):  This is an artifact of the old IPU driver and
	 * should be factored out.
	 */
	pb->lbp.lbps = kcalloc(pb->lbp.num_lbps, sizeof(struct paintbox_lbp),
			GFP_KERNEL);
	if (!pb->lbp.lbps)
		return -ENOMEM;

	/* TODO(b/115386014):  Remove and replace with a capabilties message to
	 * the JQS.
	 */
	pb->lbp.max_lbs = LBP_CAP0_DEF & LBP_CAP0_MAX_LB_MASK;

	for (i = 0; i < pb->lbp.num_lbps; i++) {
		ret = ipu_lbp_init_line_buffer_pool(pb, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* All sessions must be released before remove can be called. */
void ipu_lbp_remove(struct paintbox_data *pb)
{
	unsigned int lbp_id, lb_id;

	for (lbp_id = 0; lbp_id < pb->lbp.num_lbps; lbp_id++) {
		struct paintbox_lbp *lbp = &pb->lbp.lbps[lbp_id];

		for (lb_id = 0; lb_id < pb->lbp.max_lbs; lb_id++) {
			struct paintbox_lb *lb = &lbp->lbs[lb_id];

			ipu_lb_debug_remove(pb, lb);
		}

		ipu_lbp_debug_remove(pb, lbp);

		kfree(pb->lbp.lbps[lbp_id].lbs);
	}

	kfree(pb->lbp.lbps);
}
