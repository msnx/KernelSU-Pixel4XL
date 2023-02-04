/*
 * Core driver for the Paintbox programmable IPU
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

#ifndef __IPU_DMA_H__
#define __IPU_DMA_H__

#include <linux/types.h>

#include "ipu-client.h"

int ipu_dma_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void ipu_dma_remove(struct paintbox_data *pb);

#endif  /* __IPU_DMA_H__ */
