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

#include <linux/dma-direction.h>
#include <linux/ipu-core.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>

#include "ipu-core-internal.h"
#include "ipu-core-jqs-cbuf.h"

static inline struct jqs_cbuf *get_jqs_buffer(struct host_jqs_cbuf *host_cbuf)
{
	uint8_t *host_vaddr;

	if (WARN_ON(!host_cbuf || !host_cbuf->shared_buf_cbuf ||
			!host_cbuf->shared_buf_cbuf->host_vaddr))
		return ERR_PTR(-EINVAL);

	host_vaddr = (uint8_t *)host_cbuf->shared_buf_cbuf->host_vaddr;
	return (struct jqs_cbuf *)&host_vaddr[host_cbuf->cbuf_offset];
}

static inline uint8_t *get_host_data(struct host_jqs_cbuf *host_cbuf)
{
	uint8_t *host_vaddr;

	host_vaddr = (uint8_t *)host_cbuf->shared_buf_data->host_vaddr;
	return &host_vaddr[host_cbuf->data_offset];
}

static uint32_t write_bytes_available(struct host_jqs_cbuf *host_cbuf)
{
	struct jqs_cbuf *cbuf;

	cbuf = get_jqs_buffer(host_cbuf);

	if (IS_ERR_OR_NULL(cbuf))
		return 0;

	return cbuf->size - (cbuf->bytes_written - cbuf->bytes_read);
}

static uint32_t read_bytes_available(struct host_jqs_cbuf *host_cbuf)
{
	struct jqs_cbuf *cbuf;

	cbuf = get_jqs_buffer(host_cbuf);

	if (IS_ERR_OR_NULL(cbuf))
		return 0;

	return cbuf->bytes_written - cbuf->bytes_read;
}

int ipu_core_jqs_cbus_memcpy(void *dst, const void *src, size_t len_bytes)
{
	memcpy(dst, src, len_bytes);

	return 0;
}

int ipu_core_jqs_cbus_copy_to_user(void __user *dst, const void *src,
		size_t len_bytes)
{
	if (copy_to_user(dst, src, len_bytes))
		return -EFAULT;

	return 0;
}

int ipu_core_jqs_cbus_copy_from_user(void *dst, const void __user *src,
		size_t len_bytes)
{
	if (copy_from_user(dst, src, len_bytes))
		return -EFAULT;

	return 0;
}

void ipu_core_jqs_cbuf_init(struct host_jqs_cbuf *host_cbuf,
		struct ipu_shared_buffer *shared_buf_cbuf,
		uint32_t cbuf_offset,
		struct ipu_shared_buffer *shared_buf_data,
		uint32_t data_offset,
		size_t size, bool to_device)
{
	struct jqs_cbuf *cbuf;

	host_cbuf->shared_buf_cbuf = shared_buf_cbuf;
	host_cbuf->cbuf_offset = cbuf_offset;
	host_cbuf->shared_buf_data = shared_buf_data;
	host_cbuf->data_offset = data_offset;
	host_cbuf->last_sync = 0;
	host_cbuf->to_device = to_device;

	cbuf = get_jqs_buffer(host_cbuf);
	cbuf->data = shared_buf_data->jqs_paddr + data_offset;
	cbuf->size = (uint32_t)size;
	cbuf->bytes_read = 0;
	cbuf->bytes_written = 0;
}

static void ipu_core_jqs_cbuf_sync_data(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, enum dma_data_direction dir)
{
	struct jqs_cbuf *cbuf = get_jqs_buffer(host_cbuf);
	uint32_t idx, idx_last_sync;

	if (IS_ERR_OR_NULL(cbuf))
		return;

	if (cbuf->bytes_written == host_cbuf->last_sync)
		return;

	idx = cbuf->bytes_written % cbuf->size;
	idx_last_sync = host_cbuf->last_sync % cbuf->size;

	if (idx_last_sync > idx || (idx_last_sync == idx &&
			host_cbuf->last_sync < cbuf->bytes_written)) {
		/* sync to the end of the buffer */
		ipu_core_sync_shared_memory(bus, host_cbuf->shared_buf_data,
				host_cbuf->data_offset + idx_last_sync,
				cbuf->size - idx_last_sync, dir);
		idx_last_sync = 0;
	}

	if (idx_last_sync < idx) {
		ipu_core_sync_shared_memory(bus, host_cbuf->shared_buf_data,
				host_cbuf->data_offset + idx_last_sync,
				idx - idx_last_sync, dir);
	}

	host_cbuf->last_sync = cbuf->bytes_written;
}

void ipu_core_jqs_cbuf_sync(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, enum dma_data_direction dir)
{
	bool update_data;

	/* There are 4 cases to consider
	 * 1. (host_cbuf->to_device, DMA_TO_DEVICE)
	 *    sending the write data first, then bytes_written
	 * 2. (host_cbuf->to_device, DMA_FROM_DEVICE)
	 *    update bytes_read
	 * 3. (!host_cbuf->to_device, DMA_TO_DEVICE)
	 *    update bytes_read
	 * 4. (!host_cbuf->to_device, DMA_FROM_DEVICE)
	 *    read bytes_written first, then update data
	 */
	update_data = (dir == DMA_TO_DEVICE) == host_cbuf->to_device;

	if (!update_data) {
		/* case 2 and 3 */
		ipu_core_atomic_sync32_shared_memory(bus,
				host_cbuf->shared_buf_cbuf,
				host_cbuf->cbuf_offset +
				offsetof(struct jqs_cbuf, bytes_read), dir);
		return;
	}

	if (!host_cbuf->to_device) {
		/* case 4 */
		ipu_core_atomic_sync32_shared_memory(bus,
				host_cbuf->shared_buf_cbuf,
				host_cbuf->cbuf_offset +
				offsetof(struct jqs_cbuf, bytes_written), dir);
	}

	/* write data */
	ipu_core_jqs_cbuf_sync_data(bus, host_cbuf, dir);

	if (host_cbuf->to_device) {
		/* case 1 */
		ipu_core_atomic_sync32_shared_memory(bus,
				host_cbuf->shared_buf_cbuf,
				host_cbuf->cbuf_offset +
				offsetof(struct jqs_cbuf, bytes_written), dir);
	}
}

ssize_t ipu_core_jqs_cbuf_write(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, const void *buf, uint32_t size,
		jqs_cbuf_copy cpy)
{
	uint32_t bytes_available = write_bytes_available(host_cbuf);
	struct jqs_cbuf *cbuf = get_jqs_buffer(host_cbuf);
	uint32_t write_idx, bytes_written, bytes_write1, bytes_write2;
	int ret;

	if (IS_ERR_OR_NULL(cbuf))
		return PTR_ERR(cbuf);

	if (bytes_available < size) {
		ipu_core_jqs_cbuf_sync(bus, host_cbuf, DMA_FROM_DEVICE);
		bytes_available = write_bytes_available(host_cbuf);
	}

	write_idx = cbuf->bytes_written % cbuf->size;

	bytes_written = min(bytes_available, size);
	bytes_write1 = min(bytes_written, cbuf->size - write_idx);
	bytes_write2 = bytes_written - bytes_write1;

	ret = cpy(get_host_data(host_cbuf) + write_idx, buf, bytes_write1);
	if (ret < 0)
		return ret;

	if (bytes_write2 > 0) {

		ret = cpy(get_host_data(host_cbuf), buf +
				bytes_write1, bytes_write2);
		if (ret < 0)
			return ret;
	}

	cbuf->bytes_written += bytes_written;

	ipu_core_jqs_cbuf_sync(bus, host_cbuf, DMA_TO_DEVICE);

	return bytes_written;
}

ssize_t ipu_core_jqs_cbuf_read(struct paintbox_bus *bus,
		struct host_jqs_cbuf *host_cbuf, void *buf, uint32_t size,
		jqs_cbuf_copy cpy)
{
	uint32_t bytes_available = read_bytes_available(host_cbuf);
	struct jqs_cbuf *cbuf = get_jqs_buffer(host_cbuf);
	uint32_t read_idx, bytes_read, bytes_read1, bytes_read2;
	int ret;

	if (IS_ERR_OR_NULL(cbuf))
		return PTR_ERR(cbuf);

	if (bytes_available < size) {
		ipu_core_jqs_cbuf_sync(bus, host_cbuf, DMA_FROM_DEVICE);
		bytes_available = read_bytes_available(host_cbuf);
	}

	read_idx = cbuf->bytes_read % cbuf->size;

	bytes_read = min(bytes_available, size);
	bytes_read1 = min(bytes_read, cbuf->size - read_idx);
	bytes_read2 = bytes_read - bytes_read1;

	ret = cpy(buf, get_host_data(host_cbuf) + read_idx, bytes_read1);
	if (ret < 0)
		return ret;

	if (bytes_read2 > 0) {
		ret = cpy(buf + bytes_read1, get_host_data(host_cbuf),
				bytes_read2);
		if (ret < 0)
			return ret;
	}

	cbuf->bytes_read += bytes_read;

	ipu_core_jqs_cbuf_sync(bus, host_cbuf, DMA_TO_DEVICE);

	return bytes_read;
}
