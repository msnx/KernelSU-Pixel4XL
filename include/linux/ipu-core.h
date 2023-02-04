/*
 * Core support for the Paintbox programmable IPU
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

#ifndef __IPU_CORE_H__
#define __IPU_CORE_H__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/types.h>
#include <linux/mfd/abc-pcie.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>

#include <uapi/linux/ipu.h>

#define IPU_IOMMU_IDENTITY_MAP_SIZE	SZ_1G

struct paintbox_pdata {
	unsigned long page_size_bitmap;
	unsigned int input_address_size;
	unsigned int output_address_size;
	uint64_t dma_base;
	uint64_t dma_size;
	struct ipu_capabilities_rsp capabilities;
	bool iommu_active;
};

/* Upcall interface between the common paintbox bus layer and the paintbox
 * devices.
 */
struct paintbox_device_ops {
	void (*firmware_up)(struct device *dev);
	void (*firmware_suspended)(struct device *dev);
	void (*firmware_down)(struct device *dev);
	void (*dram_up)(struct device *dev);
	void (*dram_down)(struct device *dev);
	void (*dram_suspended)(struct device *dev);
};

struct ipu_shared_buffer {
	void *host_vaddr;
	dma_addr_t host_dma_addr;
	dma_addr_t jqs_paddr;
	size_t size;
};

struct ipu_jqs_buffer {
	dma_addr_t jqs_paddr;
	size_t size;
};

extern struct bus_type ipu_bus_type;

/* Allocate an application queue.
 *
 * The kernel queue (q_id == 0) is allocated on init. Note: The caller is
 * responsible for sending a JQS_MESSAGE_TYPE_ALLOC_QUEUE to notify JQS about
 * the new queue.
 *
 * Returns: q_id > 0 on success. < 0 on failure.
 */
int ipu_alloc_queue(struct device *dev);


void ipu_flush_queue(struct device *dev, uint32_t q_id, int queue_err);

/* Free an application queue.
 *
 * Note: The caller is responsible for first sending a sync'ed
 * JQS_MESSAGE_TYPE_FREE_QUEUE to notify JQS about queue deallocation.
 *
 * Returns: q_id > 0 on success. < 0 on failure.
 */
void ipu_free_queue(struct device *dev, uint32_t queue_id, int queue_err);

/* Write data into a user queue.
 *
 * The q_id is assumed to be valid, and that the calling session has permission
 * to access the queue.
 *
 * Returns: bytes_written > 0 on success. < 0 on failure.  Partial writes are
 *    possible if the queue is near full. This API is non-blocking, even in the
 *    case that no data can be written.
 *
 */
int ipu_user_write(struct device *dev, uint32_t queue_id,
		const void __user *buf, size_t size);

/* Read data from an application queue.
 *
 * The q_id is assumed to be valid, and that the calling session has permission
 * to access the queue.
 *
 * Returns: bytes_read > 0 on success. < 0 on failure. Partial reads as possible
 *    if the queue is near empty. This API is blocking on an empty queue.
 *
 * Multiple calls on an empty queue are not allowed.
 *
 */
ssize_t ipu_user_read(struct device *dev, uint32_t q_id, void __user *buf,
		size_t size, int nonblock);

int ipu_user_set_eventfd(struct device *dev, uint32_t queue_id, int eventfd);
int ipu_user_clear_eventfd(struct device *dev, uint32_t queue_id);

/* Write a message to JQS and do not wait for a response.
 *
 * Partial writes are not allowed, if there isn't enough room for the entire
 * message, a failure result will be returned.
 */
int ipu_kernel_write(struct device *dev, const struct jqs_message *msg);

/* Write a message to JQS and wait for a response.
 *
 * Notes: The type of the response depends on the type of message being sent.
 *        Any message type will either always generate a response, or never
 *            generate one. This is the kernel-JQS API. For example, a
 *            JQS_MESSAGE_TYPE_PING will always generate a JQS_MESSAGE_TYPE_PONG
 *            message.
 *        Only one outstanding synchronous message may occur at a time.
 */
int ipu_kernel_write_sync(struct device *dev, const struct jqs_message *msg,
		struct jqs_message *rsp, size_t rsp_size);

void ipu_writel(struct device *dev, uint32_t val, unsigned int offset);
void ipu_writeq(struct device *dev, uint64_t val, unsigned int offset);
uint32_t ipu_readl(struct device *dev, unsigned int offset);
uint64_t ipu_readq(struct device *dev, unsigned int offset);

struct ipu_shared_buffer *ipu_alloc_shared_memory(struct device *dev,
		size_t size);
void ipu_free_shared_memory(struct device *dev, struct ipu_shared_buffer *buf);

struct ipu_jqs_buffer *ipu_alloc_jqs_memory(struct device *dev, size_t size);
void ipu_free_jqs_memory(struct device *dev, struct ipu_jqs_buffer *buf);

void ipu_set_device_ops(struct device *dev,
		const struct paintbox_device_ops *dev_ops);

void ipu_frc_ipu_clock_ungate(struct device *dev);

/* return the group for the device
 * the function increments the reference on the group
 */
struct iommu_group *ipu_get_device_group(struct device *dev);

struct device *ipu_get_iommu_device(struct device *dev);
struct device *ipu_get_ipu_device(struct device *dev);
struct device *ipu_get_dma_device(struct device *dev);
struct dentry *ipu_get_debug_root(struct device *dev);

void ipu_add_client(struct device *dev);
int ipu_remove_client(struct device *dev);

/* Returns true if the JQS is ready, false if it is not. */
bool ipu_is_jqs_ready(struct device *dev);

bool ipu_is_iommu_active(struct device *dev);

/* Called by the client to request a JQS reset after a catastrophic error is
 * detected.
 */
void ipu_request_reset(struct device *dev);

int ipu_core_jqs_start(struct device *dev);
int ipu_core_jqs_power_down(struct device *dev);

#endif /* __IPU_CORE_H__ */
