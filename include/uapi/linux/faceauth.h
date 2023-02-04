/*
 * FaceAuth coordinator driver
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

#ifndef _UAPI_LINUX_FACEAUTH_H
#define _UAPI_LINUX_FACEAUTH_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/time.h>

/* Keep it in sync with faceauth firmware */
#define FACEAUTH_RESULT_SUCCESS 0
#define FACEAUTH_RESULT_FAILURE 1

#define FACEAUTH_ERROR_NO_ERROR 0

#define FACEAUTH_MAX_TASKS 32
#define FACEAUTH_DEBUG_REGISTER_COUNT 24
#define FACEAUTH_BUFFER_TAG_LENGTH 16
#define FACEAUTH_BUFFER_LIST_LENGTH 32

struct faceauth_init_data {
	__u64 features;
} __attribute__((packed));

#define FACEAUTH_MAX_CACHE_FLUSH_SIZE 20
#define FACEAUTH_AUX_DATA_SIZE 32

/* This struct is written by userspace and read by kernel */
struct faceauth_start_data {
	/*
	 * Operation requested by user, see FACEAUTH_OP_*
	 */
	__u8 profile_id;
	__u8 operation;

	__u32 input_time_ms;

	/* ION buffer fd */
	__u64 image_dot_left_fd;
	__u64 image_dot_right_fd;
	/* TODO: change uapi for image_flood_fd b/131321019 */
	__u64 image_flood_fd;
	__u64 image_flood_right_fd;
	__u64 calibration_fd;
	__u64 deferred_autocal_fd; /* b/134792835 */

	__u32 image_dot_left_size;
	__u32 image_dot_right_size;
	__u32 image_flood_size;
	__u32 image_flood_right_size;
	__u32 calibration_size;
	__u32 deferred_autocal_size;

	__s16 cache_flush_indexes[FACEAUTH_MAX_CACHE_FLUSH_SIZE];
	__u32 cache_flush_size;

	/* Output parameters */
	__u8 result; /* result code from AB */
	union {
		__u32 citadel_output3;
		__u32 lockout_event;
	};
	__u32 angles;
	__s32 error_code; /* ab-faceauth error code */
	__u32 ab_exception_number;
	__u32 fw_version; /* ab-faceauth firmware version */

	__u8 __user *citadel_token;
	__u32 citadel_token_size;

	__u32 citadel_input;
	__u32 citadel_input2;
	__u32 citadel_output1;
	__u32 citadel_output2;

	__u32 aux_data[FACEAUTH_AUX_DATA_SIZE];

} __attribute__((packed));

/* This struct contains a user supplied buffer that is written by kernel */
struct faceauth_debug_data {
	/* ION buffer fd */
	__u64 buffer_fd;

	/* TODO(b/123892068): remove these user-buffer fields */
	union {
		__u8 __user *debug_buffer;
		__u8 __user *print_buffer;
	};
	union {
		__u32 debug_buffer_size;
		__u32 print_buffer_size;
	};

	__u32 flags;
} __attribute__((packed));

struct faceauth_workload_control {
	__u32 workload_state;
	__u32 run_count;
	__u32 run_time_ms;
	__u32 run_time_us;
	__u32 status;
} __attribute__((packed));

struct faceauth_debug_register {
	__u64 address;
	__u64 value;
} __attribute__((packed));

enum faceauth_buffer_type {
	OUTPUT_NONE,
	OUTPUT_DEPTH_EMBEDDING,
	OUTPUT_FACENET_EMBEDDING,
	OUTPUT_QUANTIZED_EMBEDDINGS,
	OUTPUT_BINARY_BLOB,
	OUTPUT_8BIT_GRAYSCALE_320x320,
	OUTPUT_16BIT_GRAYSCALE_128x128,
	OUTPUT_16BIT_GRAYSCALE_480x640,
	OUTPUT_8BITRGB_128x128,
	OUTPUT_FACE_BUF,
	OUTPUT_NUM_FACES,

	/* used to extend enum size to 4 bytes */
	OUTPUT_INTMAX = 0xffffffff,
};

struct faceauth_buffer_descriptor {
	/* offset of the buffer from the buffer_base_address */
	__u32 offset_to_buffer;
	__u32 size;
	__u32 type; /* cast to enum faceauth_buffer_type */
	char buffer_tag[FACEAUTH_BUFFER_TAG_LENGTH];
} __attribute__((packed));

struct faceauth_buffer_list {
	/* ab stores the buffer base address, the kernel replaces this with
	 * the offset into the debug_entry
	 */
	__u32 buffer_base;
	__u32 buffer_count;
	struct faceauth_buffer_descriptor buffers[FACEAUTH_BUFFER_LIST_LENGTH];
} __attribute__((packed));

#define SHA1SUM_LEN 20

struct faceauth_model_version_list {
	__u8 fssd_version[SHA1SUM_LEN];
	__u8 facenet_version[SHA1SUM_LEN];
	__u8 gazenet_version[SHA1SUM_LEN];
	__u8 skin_version[SHA1SUM_LEN];
	__u8 ultradepth_version[SHA1SUM_LEN];
	__u8 depthid_version[SHA1SUM_LEN];
} __attribute__((packed));

struct faceauth_airbrush_state {
	__u32 faceauth_version;
	__s32 error_code;
	__u32 internal_state_size;
	__u32 command;
	__s32 rightbox_x1;
	__s32 rightbox_y1;
	__s32 rightbox_x2;
	__s32 rightbox_y2;
	__s32 leftbox_x1;
	__s32 leftbox_y1;
	__s32 leftbox_x2;
	__s32 leftbox_y2;
	__u32 num_tasks;
	__u32 register_list_length;
	__u32 saved_register_count;
	struct faceauth_workload_control control_list[FACEAUTH_MAX_TASKS];
	struct faceauth_debug_register
		debug_registers[FACEAUTH_DEBUG_REGISTER_COUNT];
	struct faceauth_buffer_list output_buffers;
	__u32 flags;
	__u32 command_id;
	__u64 citadel_input_data;
	__u64 feature_bypass_flags;
	__u32 embedding_version;
	__u32 input_time_ms;
	struct faceauth_model_version_list model_versions;
} __attribute__((packed));

struct faceauth_debug_image {
	__u32 offset_to_image;
	__u32 image_size;
};

struct faceauth_debug_entry {
	struct timeval timestamp;
	uint32_t status;
	uint32_t ab_exception_number;
	uint32_t fault_address;
	uint32_t ab_link_reg;
	struct faceauth_debug_image left_dot;
	struct faceauth_debug_image right_dot;
	struct faceauth_debug_image left_flood;
	struct faceauth_debug_image right_flood;
	struct faceauth_debug_image calibration;
	struct faceauth_airbrush_state ab_state;
} __attribute__((packed));

/*
 * Prepare AP and AB for faceauth workflow. This step might include slow
 * operations like reading firmware from filesystem and copying to AB memory.
 */
#define FACEAUTH_DEV_IOC_INIT _IOR('f', 1, struct faceauth_init_data)
#define FACEAUTH_DEV_IOC_START _IOWR('f', 2, struct faceauth_start_data)
#define FACEAUTH_DEV_IOC_CLEANUP _IO('f', 4)
#define FACEAUTH_DEV_IOC_DEBUG _IOR('f', 5, struct faceauth_debug_data)
#define FACEAUTH_DEV_IOC_DEBUG_DATA _IOR('f', 6, struct faceauth_debug_data)

/* Get debug data flags:
 *  - Data from fifo is the default option, this returns debug data in first-in
 *    first-out order. When all data has been returned the ENODATA status is
 *    returned.
 *  - Data from most recent, returns the most recent set of debug data and
 *    then clears the fifo. If the fifo is empty then ENODATA will be returned.
 *  - From AB dram clears the fifo, and then copies the data from ab dram. Data
 *    is always returned. This is intended to be primarily a debug tool.
 */
#define FACEAUTH_GET_DEBUG_DATA_FROM_FIFO (0)
#define FACEAUTH_GET_DEBUG_DATA_MOST_RECENT (1)
#define FACEAUTH_GET_DEBUG_DATA_FROM_AB_DRAM (2)

#define FACEAUTH_DEBUG_DATA_PAYLOAD_SIZE (2 * 1024 * 1024)

#endif /* _UAPI_LINUX_FACEAUTH_H */
