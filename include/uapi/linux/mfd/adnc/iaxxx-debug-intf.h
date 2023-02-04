/*
 * iaxxx-debug-intf.h - iaxxx debug Interface
 *
 * Copyright 2018 Knowles Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IAXXX_DEBUG_INTF_H
#define _IAXXX_DEBUG_INTF_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define IAXXX_SRB_SZ_TO_ARB      (0x16c)
#define IAXXX_SRB_REGS_NUM    (IAXXX_SRB_SZ_TO_ARB / sizeof(uint32_t))
#define IAXXX_ARB_SZ             (0x100)
#define IAXXX_ARB_REGS_NUM    (IAXXX_ARB_SZ / sizeof(uint32_t))
#define IAXXX_ARB_BLOCK_NUM     (IAXXX_ARB_REGS_NUM / 2)
#define IAXXX_MAX_CIRC_BUFS          (3)
#define IAXXX_MAX_REGS_NUM        (0xc00)
#define IAXXX_TUNNEL_MAX       (32)
#define IAXXX_CHANNEL_MAX      (32)
#define IAXXX_STREAM_MAX       (16)

struct iaxxx_srb_info {
	uint32_t reg_start_addr;
	int reg_num;
	uint32_t reg_vals[IAXXX_SRB_REGS_NUM];
};

struct iaxxx_arb_block {
	uint32_t reg_start_addr;
	int reg_num;
	uint32_t reg_vals[IAXXX_MAX_REGS_NUM];

	/* For pretty formatted display */
	char     name[15];       /* name of this arb block */
};

struct iaxxx_arb_info {
	uint32_t reg_start_addr;
	int reg_num;
	uint32_t reg_vals[IAXXX_ARB_REGS_NUM];
	struct iaxxx_arb_block blocks[IAXXX_ARB_BLOCK_NUM];
};

struct iaxxx_circ_buffer {
	uint32_t reg_start_addr;
	int reg_num;
	uint32_t reg_vals[IAXXX_MAX_REGS_NUM];

	/* For pretty formatted display */
	char     name[15];       /* name of this circular buffer */
};

struct iaxxx_circ_buffer_info {
	int buf_num;
	struct iaxxx_circ_buffer bufs[IAXXX_MAX_CIRC_BUFS];
};

struct iaxxx_registers_dump {
	struct iaxxx_srb_info srb_info;
	struct iaxxx_arb_info arb_info;
	struct iaxxx_circ_buffer_info circ_buffer_info;
};

struct iaxxx_log_level_info {
	uint32_t module_id;
	uint32_t log_level;
};

struct iaxxx_log_mode_info {
	bool mode;
	uint8_t proc_id;
};

struct iaxxx_plgin_log_mode_info {
	bool mode;
	uint32_t inst_id;
	uint8_t block_id;
};

struct iaxxx_plgin_log_state_info {
	bool state;
	uint32_t inst_id;
	uint8_t block_id;
};

enum iaxxx_fw_debug_log_mode {
	IAXXX_FROM_MEMORY,
	IAXXX_FROM_ENDPOINT,
};

enum iaxxx_fw_debug_log_level {
	IAXXX_DBG_LOG_LVL_OFF,
	IAXXX_DBG_LOG_LVL_CUSTOM,
	IAXXX_DBG_LOG_LVL_FATAL,
	IAXXX_DBG_LOG_LVL_ERROR,
	IAXXX_DBG_LOG_LVL_WARN,
	IAXXX_DBG_LOG_LVL_INFO,
	IAXXX_DBG_LOG_LVL_DEBUG,
	IAXXX_DBG_LOG_LVL_TRACE,
};

enum iaxxx_debug_module_ids {
	IAXXX_DBG_MODULE_ID_ASSERT_LOG,
	IAXXX_DBG_MODULE_ID_ACCDETMGR_LOG,
	IAXXX_DBG_MODULE_ID_BATTERYMGR_LOG,
	IAXXX_DBG_MODULE_ID_BLUETOOTHMGR_LOG,
	IAXXX_DBG_MODULE_ID_BUTTONMGR_LOG,
	IAXXX_DBG_MODULE_ID_CODECMGR_LOG,
	IAXXX_DBG_MODULE_ID_CTRLMGR_LOG,
	IAXXX_DBG_MODULE_ID_DMAMGR_LOG,
	IAXXX_DBG_MODULE_ID_EVTMGR_LOG,
	IAXXX_DBG_MODULE_ID_FLASHMGR_LOG,
	IAXXX_DBG_MODULE_ID_LEDMGR_LOG,
	IAXXX_DBG_MODULE_ID_POWERMGR_LOG,
	IAXXX_DBG_MODULE_ID_STREAMMGR_LOG,
	IAXXX_DBG_MODULE_ID_SENSORMGR_LOG,
	IAXXX_DBG_MODULE_ID_TUNNELMGR_LOG,
	IAXXX_DBG_MODULE_ID_USBMGR_LOG,
	IAXXX_DBG_MODULE_ID_PLUGINMGR_LOG,
	IAXXX_DBG_MODULE_ID_PLUGINVM_LOG,
	IAXXX_DBG_MODULE_ID_PACKAGEUTILS_LOG,
	IAXXX_DBG_MODULE_ID_ENDPOINT_LOG,
	IAXXX_DBG_MODULE_ID_PUTMSG_LOG,
	IAXXX_DBG_MODULE_ID_CONTROLLER_LOG,
	IAXXX_DBG_MODULE_ID_MIPSPROFILER_LOG,
	IAXXX_DBG_MODULE_ID_DEBUGMONITOR_LOG,
	IAXXX_DBG_MODULE_ID_SSPDRV_LOG,
	IAXXX_DBG_MODULE_ID_AFDRV_LOG,
	IAXXX_DBG_MODULE_ID_SPIDRV_LOG,
	IAXXX_DBG_MODULE_ID_I2CDRV_LOG,
	IAXXX_DBG_MODULE_ID_A400DRV_LOG,
	IAXXX_DBG_MODULE_ID_ADAU1361DRV_LOG,
	IAXXX_DBG_MODULE_ID_MAX98090DRV_LOG,
	IAXXX_DBG_MODULE_ID_BQ27425DRV_LOG,
	IAXXX_DBG_MODULE_ID_USBDRV_LOG,
	IAXXX_DBG_MODULE_ID_CSR8811_LOG,
	IAXXX_DBG_MODULE_ID_CYW20707DRV_LOG,
	IAXXX_DBG_MODULE_ID_BUTTONDRV_LOG,
	IAXXX_DBG_MODULE_ID_LEDDRV_LOG,
	IAXXX_DBG_MODULE_ID_TIMERDRV_LOG,
	IAXXX_DBG_MODULE_ID_UARTDRV_LOG,
	IAXXX_DBG_MODULE_ID_FLASHDRV_LOG,
	IAXXX_DBG_MODULE_ID_DMADRV_LOG,
	IAXXX_DBG_MODULE_ID_GPIODRV_LOG,
	IAXXX_DBG_MODULE_ID_MACDRV_LOG,
	IAXXX_DBG_MODULE_ID_STMRDRV_LOG,
	IAXXX_DBG_MODULE_ID_STMRPTDRV_LOG,
	IAXXX_DBG_MODULE_ID_SLIMBUSDRV_LOG,
	IAXXX_DBG_MODULE_ID_SSENSORDRV_LOG,
	IAXXX_DBG_MODULE_ID_STRMDRV_LOG,
	IAXXX_DBG_MODULE_ID_CPUSTRMDRV_LOG,
	IAXXX_DBG_MODULE_ID_CLKTREEUTILS_LOG,
	IAXXX_DBG_MODULE_ID_SCRIPTMGR_LOG,
};

struct iaxxx_tunnel_info_dump {
	/* input */
	uint32_t id;  /* [0, IAXXX_TUNNEL_MAX-1] */

	/* output */
	/* header */
	uint32_t out_buf_size;
	uint32_t out_buf_addr;
	uint32_t out_buf_head;
	uint32_t out_buf_tail;
	uint32_t out_buf_threshold;
	/* tunnel header configured by host and its status */
	uint32_t en;
	uint32_t st;
	/* group */
	uint32_t nframe_drops;
	uint32_t nsent_to_host;
	uint32_t nsent;
	uint32_t nrecvd;
	/* configuration registers */
	uint32_t ctrl;
	uint32_t format;
};

struct iaxxx_channel_info_dump {
	/* input */
	uint32_t id;  /* [0, IAXXX_CHANNEL_MAX-1] */

	/* output */
	/* header */
	uint32_t st;
	uint32_t direction;
	uint32_t gain;
	/* group */
	uint32_t nsent;
	uint32_t nrecvd;
	uint32_t endpoint_state;
	uint32_t intr_cnt;
	uint32_t drop_cnt;
	uint32_t gain_ctrl;
	uint32_t gain_status;
	union {
		/* valid when channel direction is output */
		uint32_t in_connect;

		/* valid when channel direction is input */
		uint32_t out_fmt;
	};
};

struct iaxxx_stream_info_dump {
	/* input */
	uint32_t id;  /* [0, IAXXX_STREAM_MAX-1] */

	/* output */
	/* header */
	uint32_t en;
	uint32_t st;
	/* group */
	uint32_t af_error_afs_fifo_overflow_cnt;
	uint32_t af_error_afs_fifo_underflow_cnt;
	uint32_t af_error_tus_fifo_overflow_cnt;
	uint32_t af_error_tus_fifo_underflow_cnt;
	uint32_t af_error_tus_fifo_coherency_cnt;
	uint32_t af_error_deadline_cnt;
	uint32_t af_error_phy_cnt;
	uint32_t af_error_timeout_cnt;
	uint32_t af_error_access_cnt;
	uint32_t ctrl;
	uint32_t status;
	uint32_t format;
	uint32_t port;
	uint32_t channel;
	uint32_t sync;
};

#define IAXXX_BUS_CONFIG	_IO('R', 0x011)
#define IAXXX_IOCTL_GET_REGISTERS_DUMP    _IO('R', 0x012)
#define IAXXX_IOCTL_GET_TUNNEL_INFO_DUMP  _IO('R', 0x013)
#define IAXXX_IOCTL_GET_CHANNEL_INFO_DUMP _IO('R', 0x014)
#define IAXXX_IOCTL_GET_STREAM_INFO_DUMP  _IO('R', 0x015)
#define IAXXX_SET_DBG_LOG_LEVEL	_IO('R', 0x021)
#define IAXXX_GET_DBG_LOG_LEVEL	_IO('R', 0x022)
#define IAXXX_SET_DBG_LOG_MODE	_IO('R', 0x023)
#define IAXXX_GET_DBG_LOG_MODE	_IO('R', 0x024)
#define IAXXX_SET_PLUGIN_LOG_MODE	_IO('R', 0x031)
#define IAXXX_GET_PLUGIN_LOG_MODE	_IO('R', 0x032)
#define IAXXX_SET_PLUGIN_LOG_STATE	_IO('R', 0x033)
#define IAXXX_GET_PLUGIN_LOG_STATE	_IO('R', 0x034)
#endif
