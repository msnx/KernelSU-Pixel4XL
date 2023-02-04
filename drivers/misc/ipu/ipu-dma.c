/*
 * DMA support for the Paintbox programmable IPU
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-dma.h"
#include "ipu-dma-debug.h"
#include "ipu-regs.h"

int ipu_dma_init(struct paintbox_data *pb)
{
	unsigned int channel_id;

	ipu_dma_debug_init(pb);

	pb->dma.channels = kcalloc(pb->dma.num_channels,
			sizeof(struct paintbox_dma_channel), GFP_KERNEL);
	if (!pb->dma.channels)
		return -ENOMEM;

	/* Store channel id with object as a convenience to avoid doing a
	 * lookup later on.
	 */
	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];
		channel->channel_id = channel_id;

		ipu_dma_channel_debug_init(pb, channel);
	}

	return 0;
}

/* All sessions must be released before remove can be called. */
void ipu_dma_remove(struct paintbox_data *pb)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];
		ipu_dma_channel_debug_remove(pb, channel);
	}

	kfree(pb->dma.channels);

	ipu_dma_debug_remove(pb);
}
