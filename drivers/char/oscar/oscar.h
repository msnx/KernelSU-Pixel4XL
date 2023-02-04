/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Oscar kernel-userspace interface definitions.
 *
 * Copyright (C) 2018 Google, Inc.
 */
#ifndef __OSCAR_H__
#define __OSCAR_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/* gasket fields are same as gasket.h ioctls */
struct oscar_abdram_map_ioctl {
	uint32_t fd;			/* AB-DRAM buffer dma_buf fd */
	uint32_t flags;			/* gasket mapping flags */
	uint32_t page_table_index;	/* gasket page table index */
	uint64_t device_address;	/* TPU address at which to map/unmap */
};

/* Base number for all Oscar-common IOCTLs */
#define OSCAR_IOCTL_BASE 0x7F

/* Map AB-DRAM buffer to TPU. */
#define OSCAR_IOCTL_ABC_MAP_ABDRAM                                             \
	_IOW(OSCAR_IOCTL_BASE, 3, struct oscar_abdram_map_ioctl)

/* Unmap AB-DRAM buffer from TPU. */
#define OSCAR_IOCTL_ABC_UNMAP_ABDRAM                                           \
	_IOW(OSCAR_IOCTL_BASE, 6, struct oscar_abdram_map_ioctl)

#endif /* __OSCAR_H__ */
