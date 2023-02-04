/*
 * IPU command queue header
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
#ifndef __IPU_QUEUE_H__
#define __IPU_QUEUE_H__

#include <linux/types.h>

#include "ipu-client.h"

/* The caller to this function must hold pb->lock */
int ipu_queue_session_release(struct paintbox_data *pb,
		struct paintbox_session *session);

int ipu_queue_allocate_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg);

#endif /* __IPU_QUEUE_H__ */
