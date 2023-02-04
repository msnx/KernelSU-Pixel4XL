/*
 * DMA debug support for the Paintbox programmable IPU
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

#ifndef __IPU_DMA_DEBUG_H__
#define __IPU_DMA_DEBUG_H__

#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-regs.h"

#define DMA_DEBUG_BUFFER_SIZE ((DMA_TOP_NUM_REGS + DMA_GRP_NUM_REGS) * \
		REG_DEBUG_BUFFER_SIZE)

#if IS_ENABLED(CONFIG_IPU_DEBUG)
void ipu_dma_debug_init(struct paintbox_data *pb);
void ipu_dma_debug_remove(struct paintbox_data *pb);
void ipu_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);
void ipu_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel);
#else
static inline void ipu_dma_debug_init(struct paintbox_data *pb) {}
static inline void ipu_dma_debug_remove(struct paintbox_data *pb) {}
static inline void ipu_dma_channel_debug_init(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel) {}
static inline void ipu_dma_channel_debug_remove(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel) {}
#endif

#endif  /* __IPU_DMA_DEBUG_H__ */
