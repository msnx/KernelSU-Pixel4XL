/*
 * JQS preamble definitions for the Paintbox programmable IPU
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

#ifndef __IPU_CORE_JQS_PREAMBLE_H__
#define __IPU_CORE_JQS_PREAMBLE_H__

#include <linux/types.h>

#define JQS_PREAMBLE_MAGIC_WORD 0x2153514A  /* JQS! */

/* These are all little-endian values */
struct jqs_firmware_preamble {
	uint32_t size;
	uint32_t magic;
	uint32_t fw_base_address;
	uint32_t fw_and_working_set_bytes;
	uint32_t prefill_transport_offset_bytes;
	uint32_t stack_start_offset_bytes;
	uint32_t stack_size_bytes;
	uint32_t build_number;
	uint32_t message_version;
	uint32_t command_version;
};

#endif /* __IPU_CORE_JQS_PREAMBLE_H__ */
