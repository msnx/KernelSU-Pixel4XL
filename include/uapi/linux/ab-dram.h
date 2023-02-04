/*
 * Driver interface for the Airbrush DRAM Manager
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

#ifndef __AB_DRAM_UAPI_H__
#define __AB_DRAM_UAPI_H__

#include <linux/ioctl.h>
#include <linux/types.h>

enum ab_dram_alloc_flag {
	ABD_ALLOC_NON_CONTIGUOUS,
	ABD_ALLOC_CONTIGUOUS,
};

struct ab_dram_alloc_request {
	__kernel_size_t size;
	enum ab_dram_alloc_flag flag;
};

#define AB_DRAM_ALLOCATE_MEMORY_LEGACY	_IOW('a', 1, __kernel_size_t)
#define AB_DRAM_ALLOCATE_MEMORY		_IOW('a', 2, \
		struct ab_dram_alloc_request)

#endif /* __AB_DRAM_UAPI_H__ */
