/*
 * Jqs message definitions for the Paintbox programmable IPU
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

#ifndef __IPU_JQS_MESSAGES_H__
#define __IPU_JQS_MESSAGES_H__

#include <linux/types.h>

#define JQS_MAX_MESSAGE_SIZE 1024
#define JQS_MESSAGE_VERSION 100

enum jqs_message_type {
	/* Jqs <-> Host messages */

	/* Jqs <- Host messages  (all host -> jqs messages currently get ack'd)
	 * maybe not all are necessary? (log_info, for example)
	 */
	JQS_MESSAGE_TYPE_OPEN_SESSION       = 0x80001001,
	/* struct jqs_message_close_session -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_CLOSE_SESSION      = 0x80001002,
	/* struct jqs_message_alloc_queue -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_ALLOC_QUEUE        = 0x80001003,
	/* struct jqs_message_free_queue -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_FREE_QUEUE         = 0x80001004,
	/* struct jqs_message_register_buffer -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_REGISTER_BUFFER    = 0x80001005,
	/* struct jqs_message_unregister_buffer -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_UNREGISTER_BUFFER  = 0x80001006,
	/* struct jqs_message_alloc_resources -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_ALLOC_RESOURCES    = 0x80001007,
	/* struct jqs_message_release_resources -> struct jqs_message_ack */
	JQS_MESSAGE_TYPE_RELEASE_RESOURCES  = 0x80001008,
	/* struct jqs_message_enter_replay_mode -> n/a */
	JQS_MESSAGE_TYPE_ENTER_REPLAY_MODE  = 0x80001009,
	/* struct jqs_message_clock_rate -> n/a */
	JQS_MESSAGE_TYPE_CLOCK_RATE         = 0x8000100a,
	/* struct jqs_message_set_log_info -> n/a */
	JQS_MESSAGE_TYPE_SET_LOG_INFO       = 0x8000100b,
	/* JqsMessageIpuRegAccess      ->   JqsMessageIpuRegValues */
	JQS_MESSAGE_TYPE_IPU_REG_ACCESS     = 0x8000100c,
	/* JqsMessageRegisterBuffers    ->   JqsMessageAck */
	JQS_MESSAGE_TYPE_REGISTER_BUFFERS   = 0x8000100d,
	/* JqsMessageUnregisterBuffers  ->  JqsMessageAck */
	JQS_MESSAGE_TYPE_UNREGISTER_BUFFERS = 0x8000100e,
	/* JqsMessageShutdownMode ->   JqsMessageAck and setting
	 * JQS_SYS_GPR_SHUTDOWN
	 */
	JQS_MESSAGE_TYPE_SHUTDOWN_MODE = 0x8000100f,
	/* JqsMessageIommuActivate ->   JqsMessageAck */
	JQS_MESSAGE_TYPE_IOMMU_ACTIVATE		= 0x80001010,

	/* Jqs -> Host messages */
	JQS_MESSAGE_TYPE_ACK                = 0x80002001,
	JQS_MESSAGE_TYPE_LOG                = 0x80002002,
	JQS_MESSAGE_TYPE_ERROR              = 0x80002003,
	JQS_MESSAGE_TYPE_IPU_REG_VALUES     = 0x80002004,

	JQS_MESSAGE_TYPE_FORCE_32_BIT       = 0xFFFFFFFF,
};

enum jqs_log_level {
	JQS_LOG_LEVEL_NONE,
	JQS_LOG_LEVEL_FATAL,
	JQS_LOG_LEVEL_ERROR,
	JQS_LOG_LEVEL_WARNING,
	JQS_LOG_LEVEL_INFO,
};

enum jqs_shutdown_mode {
	JQS_SHUTDOWN_MODE_NONE,
	JQS_SHUTDOWN_MODE_FOR_RESUME,
	JQS_SHUTDOWN_MODE_HARD
};

enum jqs_boot_mode {
	JQS_BOOT_MODE_COLD = 0x0,
	JQS_BOOT_MODE_WARM = 0x1
};

#define SYS_JQS_GPR_TRANSPORT SYS_JQS_GPR_0
#define SYS_JQS_GPR_BOOT_MODE SYS_JQS_GPR_1
#define JQS_SYS_GPR_SHUTDOWN JQS_SYS_GPR_1
#define JQS_SHUTDOWN_COMPLETE 0x1

#define JQS_LOG_SINK_NONE    (0x0)
#define JQS_LOG_SINK_UART    (0x1 << 0)
#define JQS_LOG_SINK_MESSAGE (0x1 << 1)
#define JQS_LOG_SINK_MEMORY  (0x1 << 2)

#define MAX_LOG_SIZE 256

#define INIT_JQS_MSG(msg, t) \
	msg.header.type = t; \
	msg.header.size = sizeof(msg)

struct jqs_message {
	uint32_t size;
	enum jqs_message_type type;
};

/* host -> jqs */

struct jqs_message_entry_replay_mode {
	struct jqs_message header;
};

struct jqs_message_open_session {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t session_memory_addr;
	uint32_t session_memory_bytes;
};

struct jqs_message_close_session {
	struct jqs_message header;
	uint32_t session_id;
};

struct jqs_message_shutdown_mode {
	struct jqs_message header;
	enum jqs_shutdown_mode shutdown_mode;
};

struct jqs_message_iommu_activate {
	struct jqs_message header;
	uint32_t activate;
	uint64_t page_table_addr;
};

struct jqs_message_alloc_queue {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t q_id;
};

struct jqs_message_free_queue {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t q_id;
};

struct jqs_message_register_buffer {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t buffer_id;
	uint64_t buffer_addr;
	uint32_t buffer_size;
};

struct jqs_message_unregister_buffer {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t buffer_id;
};

struct buffer_registration {
	uint64_t buffer_addr;
	uint32_t buffer_id;
	uint32_t buffer_size;
};

#define MAX_BUFFER_REGISTRATION					\
	((JQS_MAX_MESSAGE_SIZE - sizeof(struct jqs_message) -	\
	  sizeof(uint32_t) - sizeof(uint32_t)) /		\
	  sizeof(struct buffer_registration))

struct jqs_message_register_buffers {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t num_buffers;
	struct buffer_registration registrations[MAX_BUFFER_REGISTRATION];
};

struct jqs_message_unregister_buffers {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t num_buffers;
	uint32_t buffer_ids[MAX_BUFFER_REGISTRATION];
};

struct jqs_message_alloc_resources {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t stp_id_mask;
	uint32_t lbp_id_mask;
	uint32_t dma_channel_id_mask;
};

struct jqs_message_release_resources {
	struct jqs_message header;
	uint32_t session_id;
	uint32_t stp_id_mask;
	uint32_t lbp_id_mask;
	uint32_t dma_channel_id_mask;
};

struct jqs_message_set_log_info {
	struct jqs_message header;
	enum jqs_log_level log_level;
	enum jqs_log_level interrupt_level; /* for kernel messages only */
	uint32_t log_sinks;
	uint32_t uart_baud_rate;
};

struct jqs_message_clock_rate {
	struct jqs_message header;
	uint32_t clock_rate;
};

#define MAX_REG_ACCESS 128

struct jqs_ipu_reg_value {
	uint32_t address;
	uint64_t value;
};

struct jqs_ipu_reg_access {
	uint32_t /*bool */ read;
	struct jqs_ipu_reg_value val;
};

struct jqs_message_ipu_reg_access {
	struct jqs_message header;
	uint32_t num_regs;
	struct jqs_ipu_reg_access regs[MAX_REG_ACCESS];
};

/* Jqs -> Host */

enum jqs_error {
	JQS_ERROR_NONE = 0,
	JQS_ERROR_BUSY,
	JQS_ERROR_ASSERTION,
};

struct jqs_message_ack {
	struct jqs_message header;
	enum jqs_message_type msg_type;
	enum jqs_error error;
};

struct jqs_message_log {
	struct jqs_message header;
	enum jqs_log_level log_level;
	uint32_t data_length;
	char data[MAX_LOG_SIZE];
};

#define JQS_FILE_MAX 64

struct jqs_message_error {
	struct jqs_message header;
	enum jqs_error error;

	union {
		struct {
			int line;
			char file[JQS_FILE_MAX];
		} assertion;
	} data;
};

struct jqs_message_ipu_reg_values {
	struct jqs_message header;
	uint32_t num_regs;
	struct jqs_ipu_reg_value regs[MAX_REG_ACCESS];
};

#endif /* __IPU_JQS_MESSAGES_H__ */
