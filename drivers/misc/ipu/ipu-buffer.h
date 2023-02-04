/*
 * Buffer Management Support for the Paintbox programmable IPU
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

#ifndef __IPU_BUFFER_H__
#define __IPU_BUFFER_H__

#include <linux/types.h>

#include "ipu-client.h"

int ipu_buffer_dma_buf_bulk_register_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg,
		bool b32_address);

int ipu_buffer_dma_buf_bulk_unregister_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

int ipu_buffer_init_session(struct paintbox_data *pb,
		struct paintbox_session *session);

/* The caller to this function must hold pb->lock */
void ipu_buffer_release_session(struct paintbox_data *pb,
		struct paintbox_session *session);

#endif  /* __IPU_BUFFER_H__ */
