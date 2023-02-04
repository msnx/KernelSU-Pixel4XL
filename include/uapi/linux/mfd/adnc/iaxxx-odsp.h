
/*
 * iaxxx-odsp.h  --  IAXXX odsp header file
 *
 * Copyright 2018 Knowles, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __IAXXX_ODSP_H__
#define __IAXXX_ODSP_H__

#define IAXXX_PACKAGE_VER_STR_SIZE_MAX  (100)
#define IAXXX_PLUGIN_VER_STR_SIZE_MAX   (100)
#define IAXXX_MAX_PLUGIN_ENDPOINTS       (16)
#define IAXXX_MAX_VER_STR_SIZE           (20)

enum {
	IAXXX_FW_CRASH,
	IAXXX_FW_IDLE,
	IAXXX_FW_ACTIVE
};

struct iaxxx_plugin_info {
	uint32_t plg_idx;
	uint32_t pkg_id;
	uint32_t block_id;
	uint32_t inst_id;
	uint32_t priority;
	uint32_t config_id;
};

struct iaxxx_plugin_param {
	uint32_t inst_id;
	uint32_t param_id;
	uint32_t param_val;
	uint8_t block_id;

};

struct iaxxx_plugin_param_blk {
	uint32_t inst_id;
	uint32_t param_size;
	uint64_t param_blk;
	uint8_t block_id;
	uint32_t id;
	char file_name[256];
};

struct iaxxx_plugin_create_cfg {
	char file_name[256];
	uint32_t inst_id;
	uint32_t cfg_size;
	uint64_t cfg_val;
	uint8_t block_id;
};

struct iaxxx_set_event {
	uint8_t inst_id;
	uint32_t event_enable_mask;
	uint32_t block_id;
};

struct iaxxx_evt_trigger {
	uint16_t src_id;
	uint16_t evt_id;
	uint32_t src_opaque;
};

struct iaxxx_evt_read_subscription {
	uint16_t src_id;
	uint16_t evt_id;
	uint16_t dst_id;
	uint32_t dst_opaque;
};

struct iaxxx_evt_retrieve_notification {
	uint16_t src_id;
	uint16_t evt_id;
	uint32_t src_opaque;
	uint32_t dst_opaque;
};

struct iaxxx_evt_info {
	uint16_t src_id;
	uint16_t event_id;
	uint16_t dst_id;
	uint32_t dst_opaque;
};

struct iaxxx_get_event {
	uint16_t event_id;
	uint32_t data;
};

struct iaxxx_pkg_mgmt_info {
	char pkg_name[256];
	uint32_t pkg_id;
	uint32_t proc_id;
};

struct iaxxx_plugin_custom_cfg {
	char     file_name[256];
	uint32_t inst_id;
	uint8_t  block_id;
	uint32_t param_blk_id;
	uint32_t custom_config_id;
};

struct iaxxx_plugin_error_info {
	uint8_t  block_id;
	uint32_t error_code;
	uint8_t  error_instance;
};

struct iaxxx_plugin_set_param_blk_with_ack_info {
	uint32_t inst_id;
	uint8_t  block_id;
	uint32_t param_blk_id;
	uint32_t set_param_blk_size;
	uint64_t set_param_blk_buffer;
	uint64_t response_buffer;
	uint32_t response_buf_size;
	uint32_t max_retries;
};

struct iaxxx_plugin_status_info {
	uint32_t inst_id;
	uint32_t block_id;
	uint8_t create_status;
	uint8_t enable_status;
	uint16_t process_count;
	uint16_t process_err_count;
	uint32_t in_frames_consumed;
	uint32_t out_frames_produced;
	uint32_t private_memsize;
	uint8_t frame_notification_mode;
	uint8_t state_management_mode;
};

struct iaxxx_plugin_endpoint_status_info {
	uint32_t inst_id;
	uint8_t ep_index;
	uint8_t direction;
	uint8_t status;
	uint8_t frame_status;
	uint8_t endpoint_status;
	uint8_t usage;
	uint8_t mandatory;
	uint16_t counter;
	uint8_t op_encoding;
	uint8_t op_sample_rate;
	uint16_t op_frame_length;
};

struct iaxxx_plugin_get_package_version {
	uint8_t inst_id;
	char version[IAXXX_PACKAGE_VER_STR_SIZE_MAX];
	uint32_t len;
};

struct iaxxx_plugin_get_plugin_version {
	uint8_t inst_id;
	char version[IAXXX_PLUGIN_VER_STR_SIZE_MAX];
	uint32_t len;
};

struct iaxxx_plugin_endpoint_timestamps {
	uint8_t proc_id;
	uint64_t timestamps[IAXXX_MAX_PLUGIN_ENDPOINTS];
};

struct iaxxx_proc_execution_status {
	uint8_t proc_id;
	uint32_t status;
};

struct iaxxx_sys_versions {
	uint32_t app_ver_num; /* output */
	char app_ver_str[IAXXX_MAX_VER_STR_SIZE]; /* output */
	uint32_t app_ver_str_len; /* input */

	uint32_t rom_ver_num; /* output */
	char rom_ver_str[IAXXX_MAX_VER_STR_SIZE]; /* output */
	uint32_t rom_ver_str_len; /* input */
};

/* IOCTL Magic character */
#define IAXXX_IOCTL_MAGIC 'I'

/* Create IOCTL */
#define ODSP_PLG_CREATE _IO(IAXXX_IOCTL_MAGIC, 0x11)
#define ODSP_PLG_RESET _IO(IAXXX_IOCTL_MAGIC, 0x12)
#define ODSP_PLG_ENABLE _IO(IAXXX_IOCTL_MAGIC, 0x13)
#define ODSP_PLG_DISABLE _IO(IAXXX_IOCTL_MAGIC, 0x14)
#define ODSP_PLG_DESTROY _IO(IAXXX_IOCTL_MAGIC, 0x15)
#define ODSP_PLG_SET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x16)
#define ODSP_PLG_GET_PARAM _IO(IAXXX_IOCTL_MAGIC, 0x17)
#define ODSP_PLG_SET_PARAM_BLK _IO(IAXXX_IOCTL_MAGIC, 0x18)
#define ODSP_PLG_SET_CREATE_CFG _IO(IAXXX_IOCTL_MAGIC, 0x19)
#define ODSP_PLG_SET_EVENT _IO(IAXXX_IOCTL_MAGIC, 0x1A)
#define ODSP_EVENT_SUBSCRIBE _IO(IAXXX_IOCTL_MAGIC, 0x1B)
#define ODSP_GET_EVENT _IO(IAXXX_IOCTL_MAGIC, 0x1C)
#define ODSP_EVENT_UNSUBSCRIBE _IO(IAXXX_IOCTL_MAGIC, 0x1D)
#define ODSP_LOAD_PACKAGE _IO(IAXXX_IOCTL_MAGIC, 0x1E)
#define ODSP_UNLOAD_PACKAGE _IO(IAXXX_IOCTL_MAGIC, 0x1F)
#define ODSP_PLG_SET_CUSTOM_CFG _IO(IAXXX_IOCTL_MAGIC, 0x20)
#define ODSP_PLG_GET_PARAM_BLK _IO(IAXXX_IOCTL_MAGIC, 0x21)
#define ODSP_PLG_CREATE_STATIC_PACKAGE _IO(IAXXX_IOCTL_MAGIC, 0x22)
#define ODSP_PLG_READ_PLUGIN_ERROR _IO(IAXXX_IOCTL_MAGIC, 0x23)
#define ODSP_PLG_SET_PARAM_BLK_WITH_ACK _IO(IAXXX_IOCTL_MAGIC, 0x24)
#define ODSP_PLG_GET_STATUS_INFO _IO(IAXXX_IOCTL_MAGIC, 0x25)
#define ODSP_PLG_GET_ENDPOINT_STATUS _IO(IAXXX_IOCTL_MAGIC, 0x26)
#define ODSP_EVENT_TRIGGER _IO(IAXXX_IOCTL_MAGIC, 0x27)
#define ODSP_EVENT_READ_SUBSCRIPTION _IO(IAXXX_IOCTL_MAGIC, 0x28)
#define ODSP_EVENT_RETRIEVE_NOTIFICATION _IO(IAXXX_IOCTL_MAGIC, 0x29)
#define ODSP_PLG_GET_PACKAGE_VERSION _IO(IAXXX_IOCTL_MAGIC, 0x2A)
#define ODSP_PLG_GET_PLUGIN_VERSION _IO(IAXXX_IOCTL_MAGIC, 0x2B)
#define ODSP_EVENT_RESET_READ_INDEX _IO(IAXXX_IOCTL_MAGIC, 0x2C)
#define ODSP_PLG_GET_ENDPOINT_TIMESTAMPS _IO(IAXXX_IOCTL_MAGIC, 0x2D)
#define ODSP_GET_PROC_EXECUTION_STATUS _IO(IAXXX_IOCTL_MAGIC, 0x2E)
#define ODSP_GET_SYS_VERSIONS _IO(IAXXX_IOCTL_MAGIC, 0x2F)
#define ODSP_GET_SYS_DEVICE_ID _IO(IAXXX_IOCTL_MAGIC, 0x30)
#define ODSP_GET_SYS_MODE _IO(IAXXX_IOCTL_MAGIC, 0x31)
#define ODSP_GET_FW_STATUS _IO(IAXXX_IOCTL_MAGIC, 0x32)
#define ODSP_RESET_FW _IO(IAXXX_IOCTL_MAGIC, 0x33)
#endif
