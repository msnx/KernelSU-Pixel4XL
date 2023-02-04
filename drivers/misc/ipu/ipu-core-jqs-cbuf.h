/*
 * JQS Circular buffer support for the Paintbox programmable IPU
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

#ifndef __IPU_CORE_JQS_CBUF_H__
#define __IPU_CORE_JQS_CBUF_H__

#include <linux/dma-direction.h>
#include <linux/ipu-core.h>
#include <linux/types.h>

#include "ipu-core-internal.h"

typedef int (*jqs_cbuf_copy)(void *dst, const void *src, size_t len_byes);

int ipu_core_jqs_cbus_memcpy(void *dst, const void *src, size_t len_bytes);
int ipu_core_jqs_cbus_copy_to_user(void __user *dst, const void *src,
		size_t len_bytes);
int ipu_core_jqs_cbus_copy_from_user(void *dst, const void __user *src,
		size_t len_bytes);

void ipu_core_jqs_cbuf_init(struct host_jqs_cbuf *host_cbuf,
		struct ipu_shared_buffer *shared_buf_cbuf,
		uint32_t cbuf_offset,
		struct ipu_shared_buffer *shared_buf_data,
		uint32_t data_offset, size_t size, bool to_device);

/* On systems in which the circular buffer does not reside in memory accessible
 * by both the AP and JQS, all functions below act only on a local copy of the
 * buffer. paintbox_jqs_circular_buffer_sync is necessary to refresh the local
 * copy with remote updates and send local updates to JQS.
 */
void ipu_core_jqs_cbuf_sync(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, enum dma_data_direction dir);

ssize_t ipu_core_jqs_cbuf_write(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, const void *buf, uint32_t size,
		jqs_cbuf_copy cpy);

ssize_t ipu_core_jqs_cbuf_read(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, void *buf, uint32_t size,
		jqs_cbuf_copy cpy);

#endif  /* __IPU_CORE_JQS_CBUF_H__ */

