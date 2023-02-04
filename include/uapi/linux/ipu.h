/*
 * Driver interface for the Paintbox Image Processing Unit
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

#ifndef __UAPI_IPU_H__
#define __UAPI_IPU_H__

#include <linux/compiler.h>
#include <linux/dma-direction.h>
#include <linux/ioctl.h>
#include <linux/types.h>

#define PAINTBOX_SESSION_ID_MAX 32

struct ipu_capabilities_rsp {
	__u32 version_major;
	__u32 version_minor;
	__u32 version_build;
	__u32 hardware_id;
	__u32 num_stps;
	__u32 num_interrupts;
	__u32 num_lbps;
	__u32 num_dma_channels;
	bool is_simulator;
	bool is_fpga;
	bool iommu_enabled;
};

struct ipu_resource_allocate_request {
	__u64 stp_mask;
	__u64 lbp_mask;
	__u64 dma_channel_mask;
	__u64 timeout_ns;
};

struct ipu_dma_buf_register_entry {
	int dma_buf_fd; /* Input Parameter */
	enum dma_data_direction dir; /* Input Parameter */
	__u32 buffer_id; /* Output Parameter */
};

struct ipu_power_core_enable_request {
	__u64 stp_mask;
	__u64 lbp_mask;
};

struct ipu_power_core_disable_request {
	__u64 stp_mask;
	__u64 lbp_mask;
};

struct ipu_dma_buf_bulk_register_req {
	unsigned int num_buffers;
	struct ipu_dma_buf_register_entry __user *bufs;
};

struct ipu_dma_buf_bulk_unregister_req {
	unsigned int num_buffers;
	__u32 __user *buf_ids;
};

/* On success will return 0, otherwise will return -1 with errno set. */
#define IPU_GET_CAPABILITIES _IOR('i', 1, struct ipu_capabilities_rsp)

/* On success will return a fd >= 0, otherwise will return -1 with errno set. */
#define IPU_ALLOCATE_CMD_QUEUE _IO('i', 2)

/* On success will return 0, otherwise will return -1 with errno set. */
#define IPU_ALLOCATE_RESOURCES _IOW('i', 3, \
		struct ipu_resource_allocate_request)

/* On success will return 0, otherwise will return -1 with errno set. */
#define IPU_RELEASE_RESOURCES _IO('i', 4)

/* On success will return 0, otherwise will return -1 with errno set. */
#define IPU_POWER_ENABLE_CORES _IOW('i', 5, \
		struct ipu_power_core_enable_request)

/* On success will return 0, otherwise will return -1 with errno set. */
#define IPU_POWER_DISABLE_CORES _IOW('i', 6, \
		struct ipu_power_core_disable_request)

/* Supports both AB Dram and ION buffers, buffers can be non-contiguous if IOMMU
 * is active. IPU DMA operations using ION buffers will be across the PCIe bus.
 * On success the return value will be zero and the buffer_id field will be set
 * to the buffer id for the buffer.  On error the return value will be set to -1
 * and errno will be set.
 */
#define IPU_BULK_REGISTER_DMA_BUF _IOWR('i', 7, \
		struct ipu_dma_buf_bulk_register_req)

/* The parameter to the ioctl is the buffer id to be unregistered.  On success
 * the return value will be zero.  On error the return value will be set to -1
 * and errno will be set.
 */
#define IPU_BULK_UNREGISTER_DMA_BUF _IOW('i', 8, \
		struct ipu_dma_buf_bulk_unregister_req)

/* Only supports ab dram input buffer. buffer must be contiguous in memory.
 * On success the return value will be zero and the buffer_id field will be set
 * to the buffer id for the buffer.  On error the return value will be set to -1
 * and errno will be set.
 */
#define IPU_BULK_REGISTER_32B_ADDRESS_DMA_BUF _IOWR('i', 9, \
		struct ipu_dma_buf_bulk_register_req)

/* Associates an eventfd with a command queue allocated using
 * IPU_ALLOCATE_CMD_QUEUE.
 * The eventfd is incremented by one for every doorbell interrupt received for
 * the queue.
 */
#define IPU_CMD_QUEUE_SET_EVENTFD _IOW('q', 1, int)
#define IPU_CMD_QUEUE_CLEAR_EVENTFD _IOW('q', 2, int)

#endif /* __UAPI_IPU_H__ */
