/*
 * AP <-> JQS Shared Structures
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

#ifndef __IPU_CORE_JQS_STRUCTS_H__
#define __IPU_CORE_JQS_STRUCTS_H__

/*
 *  This CircularBuffer implementation is used for shared memory message passing
 *  between the kernel driver and the JQS firmware.
 *
 *  This is a special purpose buffer and is not meant to be reused. There are
 *  several caveats to its use. It is meant to be used in a lock-free,
 *  single-producer, single-consumer way.
 *
 *  Note: Data must but written before bytes_written is updated.
 */

/*
 *	  These data structures are duplicated from
 *	  hardware/gchips/paintbox/firmware/include/message_transport.h
 *
 *	  The implementation is taken mostly from
 *	  hardware/gchips/paintbox/firmware/circular_buffer.c
 *
 *	  The code cannot easily be shared mostly due to the fact that they
 *	  live in different repositories, but nonetheless, the two
 *	  implementations need to be kept in sync.
 */

#define JQS_CACHE_LINE_SIZE 64
#define JQS_TRANSPORT_MAX_MESSAGE_SIZE 256
#define JQS_TRANSPORT_MAX_QUEUE 31 /* 30 app queues + 1 kernel queue */

/* Bits 0 and 1 in JQS_SYS_DBL register control the two MSIs assigned to the JQS
 */
#define JQS_SYS_DBL_MSI0 (1 << 0)
#define JQS_SYS_DBL_MSI1 (1 << 1)

/* MSI0 is assigned to the kernel queue.  The kernel queue id needs to match the
 * bit position of MSI0.
 */
#define JQS_TRANSPORT_KERNEL_QUEUE_ID 0

/* Bit 1 in the JQS_SYS_DBL register is used for MSI1.  The first bit available
 * for application queues is bit 2.
 */
#define JQS_TRANSPORT_FIRST_APP_QUEUE_ID 2

struct jqs_cbuf {
	/* JQS address of the backing data */
	uint32_t data;
	/* Size of data */
	uint32_t size;
	/* Total number of bytes read */
	uint32_t bytes_read;
	uint8_t padding_read[JQS_CACHE_LINE_SIZE];
	/* Total number of bytes written */
	uint32_t bytes_written;
	uint8_t padding_write[JQS_CACHE_LINE_SIZE];
};

struct jqs_msg_transport_queue {
	/* Circular buffer for sending messages from JQS to host.
	 * This is read-only from the host and write-only from JQS.
	 */
	struct jqs_cbuf jqs_sys_cbuf;
	/* Circular buffer for sending messages from host to JQS.
	 * This is write-only from the host and read-only from JQS.
	 */
	struct jqs_cbuf sys_jqs_cbuf;
};

struct jqs_msg_transport_shared_state {
	struct jqs_msg_transport_queue queues[JQS_TRANSPORT_MAX_QUEUE];
};

#endif  /* __IPU_CORE_JQS_STRUCTS_H__ */
