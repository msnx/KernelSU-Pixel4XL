/*
 * include/linux/citadel.h
 *
 * Copyright (C) 2017 Google Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef CITADEL_H
#define CITADEL_H

#include <linux/types.h>

#define CITADEL_IOC_MAGIC		'c'

struct citadel_ioc_tpm_datagram {
	__u64 buf;
	__u32 len;
	__u32 command;
};

#define CITADEL_IOC_TPM_DATAGRAM	_IOW(CITADEL_IOC_MAGIC, 1, \
					     struct citadel_ioc_tpm_datagram)
#define CITADEL_IOC_RESET		_IO(CITADEL_IOC_MAGIC, 2)

#endif /* CITADEL_H */
