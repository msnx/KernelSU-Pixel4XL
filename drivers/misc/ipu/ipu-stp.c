/*
 * STP support for the Paintbox programmable IPU
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-regs.h"
#include "ipu-stp.h"
#include "ipu-stp-debug.h"

/* All sessions must be released before remove can be called. */
void ipu_stp_remove(struct paintbox_data *pb)
{
	unsigned int stp_index;

	for (stp_index = 0; stp_index < pb->stp.num_stps; stp_index++) {
		struct paintbox_stp *stp = &pb->stp.stps[stp_index];

		ipu_stp_debug_remove(pb, stp);
	}

	kfree(pb->stp.stps);
}

int ipu_stp_init(struct paintbox_data *pb)
{
	unsigned int stp_index;

	pb->stp.stps = kcalloc(pb->stp.num_stps, sizeof(struct paintbox_stp),
			GFP_KERNEL);
	if (!pb->stp.stps)
		return -ENOMEM;

	for (stp_index = 0; stp_index < pb->stp.num_stps; stp_index++) {
		struct paintbox_stp *stp = &pb->stp.stps[stp_index];

		stp->stp_id = ipu_stp_index_to_id(stp_index);

		ipu_stp_debug_init(pb, stp);
	}

	return 0;
}
