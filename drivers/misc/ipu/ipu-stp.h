/*
 * STP support for the Paintbox programmable IPU
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

#ifndef __IPU_STP_H__
#define __IPU_STP_H__

#include <linux/types.h>

#include "ipu-client.h"
#include "ipu-regs.h"

static inline unsigned int ipu_stp_id_to_index(unsigned int stp_id)
{
	return stp_id - 1;
}

static inline unsigned int ipu_stp_index_to_id(unsigned int stp_index)
{
	return stp_index + 1;
}

int ipu_stp_init(struct paintbox_data *pb);

/* All sessions must be released before remove can be called. */
void ipu_stp_remove(struct paintbox_data *pb);

#endif /* __IPU_STP_H__ */
