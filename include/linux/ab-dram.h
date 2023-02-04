/*
 * Airbrush DRAM Manager
 *
 * Copyright (C) 2018 Google, Inc.
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

#ifndef __AB_DRAM_H__
#define __AB_DRAM_H__

#include <linux/dma-buf.h>
#include <linux/types.h>

struct dma_buf *ab_dram_alloc_dma_buf_kernel(size_t len);
void ab_dram_free_dma_buf_kernel(struct dma_buf *dmabuf);

dma_addr_t ab_dram_get_dma_buf_paddr(struct dma_buf *dmabuf);
bool is_ab_dram_dma_buf(struct dma_buf *dmabuf);

#endif /* __AB_DRAM_H__ */
