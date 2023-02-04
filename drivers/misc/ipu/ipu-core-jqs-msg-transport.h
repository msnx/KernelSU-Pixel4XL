/*
 * JQS Message Transport for the Paintbox programmable IPU
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

#ifndef __IPU_JQS_MSG_TRANSPORT_H__
#define __IPU_JQS_MSG_TRANSPORT_H__

#include <linux/kernel.h>
#include <linux/ipu-core.h>
#include <linux/ipu-jqs-messages.h>
#include <linux/types.h>

#include "ipu-core-internal.h"

int ipu_core_jqs_msg_transport_init(struct paintbox_bus *bus);
void ipu_core_jqs_msg_transport_shutdown(struct paintbox_bus *bus);

/* Allocate an application queue.
 *
 * The kernel queue (q_id == 0) is allocated on init. Note: The caller is
 * responsible for sending a JQS_MESSAGE_TYPE_ALLOC_QUEUE to notify JQS about
 * the new queue.
 *
 * Returns: q_id > 0 on success. < 0 on failure.
 */
int ipu_core_jqs_msg_transport_alloc_queue(struct paintbox_bus *bus);

/* Free an application queue.
 *
 * Note: The caller is responsible for first sending a sync'ed
 * JQS_MESSAGE_TYPE_FREE_QUEUE to notify JQS about queue deallocation.
 */
void ipu_core_jqs_msg_transport_free_queue(struct paintbox_bus *bus,
		uint32_t q_id, int queue_err);

void ipu_core_jqs_msg_transport_flush_queue(struct paintbox_bus *bus,
		uint32_t q_id, int queue_err);

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
ssize_t ipu_core_jqs_msg_transport_user_write(struct paintbox_bus *bus,
		uint32_t q_id, const void __user *buf, size_t size);

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
ssize_t ipu_core_jqs_msg_transport_user_read(struct paintbox_bus *bus,
		uint32_t q_id, void __user *buf, size_t size, int nonblock);

int ipu_core_jqs_msg_transport_user_set_eventfd(struct paintbox_bus *bus,
		uint32_t q_id, int eventfd);
int ipu_core_jqs_msg_transport_user_clear_eventfd(struct paintbox_bus *bus,
		uint32_t q_id);

/* Write a message to JQS and do not wait for a response.
 *
 * Partial writes are not allowed, if there isn't enough room for the entire
 * message, a failure result will be returned.
 */
ssize_t ipu_core_jqs_msg_transport_kernel_write(struct paintbox_bus *bus,
		const struct jqs_message *msg);

/* Write a message to JQS and wait for a response.
 *
 * Returns: bytes_read > 0 from response on success. < - on failure.
 *
 * Notes: The type of the response depends on the type of message being sent.
 *        Any message type will either always generate a response, or never
 *            generate one. This is the kernel-JQS API. For example, a
 *            JQS_MESSAGE_TYPE_PING will always generate a JQS_MESSAGE_TYPE_PONG
 *            message.
 *        Only one outstanding synchronous message may occur at a time.
 */
ssize_t ipu_core_jqs_msg_transport_kernel_write_sync(struct paintbox_bus *bus,
		const struct jqs_message *msg, struct jqs_message *rsp,
		size_t rsp_size);

/* Process queues based on the bitmask q_ids.
 *
 * If a thread is already processing any queues, this will simply record the
 * request and process them later. In other words, this API is synchronous if
 * the request is made when there is no one currently processing queues, and
 * asynchronous otherwise.
 */
void ipu_core_jqs_msg_transport_process_queues(struct paintbox_bus *bus,
		uint32_t q_ids);

int ipu_core_jqs_msg_transport_alloc_kernel_queue(struct paintbox_bus *bus);
void ipu_core_jqs_msg_transport_free_kernel_queue(struct paintbox_bus *bus,
		int queue_err);
void ipu_core_jqs_msg_transport_complete_kernel_queue(struct paintbox_bus *bus,
		int queue_err);

/* Called in an interrupt context */
irqreturn_t ipu_core_jqs_msg_transport_interrupt_handler(
		struct paintbox_bus *bus);
irqreturn_t ipu_core_jqs_msg_transport_interrupt_thread(
		struct paintbox_bus *bus);

#endif  /* __IPU_JQS_MSG_TRANSPORT_H__ */
