
/*
 * iaxxx-core.h  --  Knowles iaxxx core header file
 *
 * Copyright 2017 Knowles Corporation.
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

#ifndef __IAXXX_CORE_H__
#define __IAXXX_CORE_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-module.h>
#include <linux/mfd/adnc/iaxxx-debug-intf.h>

typedef int (*iaxxx_cb_func_ptr_t)(struct device *dev);
typedef int (*iaxxx_cb_bc_func_ptr_t)(struct device *dev, u32 iaxxx_spi_speed);
struct workqueue_struct;
/* SRB is at the end of the memory map */
#define IAXXX_RBDT_NUM_ENTRIES	32

#define IAXXX_REGMAP_NAME  "regmap_main"
#define IAXXX_REGMAP_NO_PM_NAME  "regmap_no_pm"

#define IAXXX_BLOCK_0 0 /* Update for CM4 PROC */
#define IAXXX_BLOCK_1 1 /* Update for HMD PROC */
#define IAXXX_BLOCK_2 2 /* Update for DMX PROC */
#define IAXXX_MAX_MODELS 3

#define IAXXX_PKG_ID_MASK	0x000F
#define IAXXX_PLGIN_ID_MASK	0x001F
#define IAXXX_SENSR_ID_MASK	0x0003

#define IAXXX_HOST_0 0
#define IAXXX_HOST_1 1
/*********************/
/*   GLOBAL MEMORY   */
/*********************/
#define GLOBAL_DRAM_START             0x58000000
#define GLOBAL_DRAM_END               0x58020000
/*  GLOBAL MEMORY END  */

/*********************/
/*   SSP Processor   */
/*********************/
#define SSP_SYS_OFFSET	0x82000000
/********* SSP DATA MEMORY START *********/
#define SSP_DMEM0_LCL_START	0x20000000
#define SSP_DMEM0_SYS_START	(SSP_DMEM0_LCL_START + SSP_SYS_OFFSET)

#define SSP_DRAM1_DED_END	0x21020000
#define SSP_DMEM1_LCL_END	SSP_DRAM1_DED_END
#define SSP_DMEM1_SYS_END	(SSP_DMEM1_LCL_END + SSP_SYS_OFFSET)
/**************** SSP PROCESSOR MEMORY END *****************/

/*********************/
/*   HMD Processor   */
/*********************/
#define HMD_SYS_OFFSET	0x86000000
/********* HMD DATA MEMORY START *********/
#define HMD_DMEM_LCL_START	0x20000000
#define HMD_DMEM_SYS_START	(HMD_DMEM_LCL_START + HMD_SYS_OFFSET)

#define HMD_IMEM_LCL_END	0x20800000
#define HMD_IMEM_SYS_END	(HMD_IMEM_LCL_END + HMD_SYS_OFFSET)
/**************** HMD PROCESSOR MEMORY END *****************/

/*********************/
/*   DMX Processor   */
/*********************/
#define DMX_SYS_OFFSET	0x88000000
/********* DMX DATA MEMORY START *********/
#define DMX_DMEM_LCL_START	0x20000000
#define DMX_DMEM_SYS_START	(DMX_DMEM_LCL_START + DMX_SYS_OFFSET)

#define DMX_IMEM_LCL_END	0x20800000
#define DMX_IMEM_SYS_END	(DMX_IMEM_LCL_END + DMX_SYS_OFFSET)
/**************** DMX PROCESSOR MEMORY END *****************/

/* Max size of one-line buffer maintained for crash reason
 */
#define IAXXX_CRASH_REASON_STR_MAX_SIZE 255


/* Flags denoting various states of FW and events */
enum {
	IAXXX_FLG_STARTUP,		/* Initial state */
	IAXXX_FLG_FW_READY,		/* FW is downloaded successully and
					 * is in app mode
					 */
	IAXXX_FLG_FW_CRASH,		/* FW has crashed */
	IAXXX_FLG_PM_SUSPEND,		/* Suspend state */
	IAXXX_FLG_RESUME_BY_STARTUP,	/* System boot up */
	IAXXX_FLG_RESUME_BY_RECOVERY,	/* FW update and resume
					 * after fw crash
					 */
	IAXXX_FLG_CHIP_WAKEUP_HOST0, /* chip Wakeup event for HOST0 */
};

struct iaxxx_client {
	struct regmap *regmap;
};

/* Probed Bus */
enum iaxxx_bus {
	IAXXX_SPI = 0,
	IAXXX_I2C,
	IAXXX_UART,
};

enum iaxxx_events {
	IAXXX_EV_UNKNOWN = 0,		/* Reserve value 0 */
	IAXXX_EV_APP_MODE,		/* FW entered in application mode */
	IAXXX_EV_STARTUP,		/* First ready system startup */
	IAXXX_EV_RECOVERY,		/* Recovery complete after fw crash */
	IAXXX_EV_CRASH,			/* Notify for FW crash */
	IAXXX_EV_FW_RESET,		/* FW reset forcefully*/
	IAXXX_EV_ROUTE_ACTIVE,		/* Audio routing path is done */
	IAXXX_EV_PACKAGE,		/* Loaded plugin */
};

enum iaxxx_plugin_state {
	IAXXX_PLUGIN_UNLOADED = 0,
	IAXXX_PLUGIN_LOADED,
	IAXXX_PLUGIN_ACTIVE,
	IAXXX_PLUGIN_RESET,
};

enum iaxxx_pkg_state {
	IAXXX_PKG_UNLOADED = 0,
	IAXXX_PKG_LOADED,
};

enum iaxxx_power_state {
	IAXXX_NOCHANGE     = 0,
	IAXXX_OPTIMAL_MODE = 1,
	IAXXX_SLEEP_MODE   = 2,
	IAXXX_NORMAL_MODE  = 4,
};

struct iaxxx_plugin_status_data {
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

struct iaxxx_plugin_endpoint_status_data {
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

struct iaxxx_plugin_data {
	uint32_t proc_id;
	uint32_t inst_id;
	enum iaxxx_plugin_state plugin_state;
	struct list_head plugin_node;
};

struct iaxxx_pkg_data {
	uint32_t proc_id;
	uint32_t pkg_id;
	enum iaxxx_pkg_state pkg_state;
	struct list_head pkg_node;
};

struct iaxxx_block_err {
	uint32_t plg_inst_id[3];
	uint32_t error[3];
};

struct iaxxx_system_state {
	struct mutex plg_pkg_list_lock;
	struct list_head plugin_head_list;
	struct list_head pkg_head_list;
	enum iaxxx_power_state power_state;
	struct iaxxx_block_err err;
	ktime_t ktime_sleep;
};

enum {
	IAXXX_DBGLOG_CM4,
	IAXXX_DBGLOG_HMD,
	IAXXX_DBGLOG_DMX,
	IAXXX_CRASHLOG_CM4,
	IAXXX_CRASHLOG_HMD,
	IAXXX_CRASHLOG_DMX,
	IAXXX_MEMDUMP_SSP_RAM0,
	IAXXX_MEMDUMP_SSP_RAM1,
	IAXXX_MEMDUMP_SSP_ROM0,
	IAXXX_CRASH_REASON_LOG,
	IAXXX_MAX_LOG
};

struct iaxxx_crashlog_header {
	uint32_t log_type;
	uint32_t log_addr;
	uint32_t log_size;
};

struct iaxxx_crashlog {
	char *log_buffer;
	uint32_t log_buffer_size;
	uint32_t logs_read;

	struct iaxxx_crashlog_header
	ssp_memdumps[IAXXX_MEMDUMP_SSP_ROM0 - IAXXX_MEMDUMP_SSP_RAM0 + 1];

	char *ssp_log_buffer;
	uint32_t ssp_log_size;
};

/* iaxxx_misc hwinfo
 * Revision: EFUSE_BOOT_0 bit 11:8
 * Trim LDO_BG: EFUSE_BOOT_1 bit 7
 * If LDO_BG_TRIM is above 0, it means chip trimmed.
 */

struct hwinfo_type {
	char revision[4];
	uint32_t trim_ldo_bg;
};

struct iaxxx_static_mem_blk {
	uint32_t  blk_id;
	uint32_t  blk_size;
	void     *blk_mem;
	struct list_head blk_node;
};

/**
 * Description of driver private data
 *
 * @dev: device pointer
 * @regmap: the device register map
 * @regmap_config: regmap configuration data
 * @regmap_init: bus specific regmap init function
 * @sys_rbdt: cached copy of the Register Block Descriptor Table
 */
struct iaxxx_priv {
	struct device *dev;
	struct device *codec_dev;
	struct regmap *regmap;
	struct regmap_config *regmap_config;
	struct regmap *regmap_no_pm;
	struct regmap_config *regmap_no_pm_config;
	int (*regmap_init_bus)(struct iaxxx_priv *priv);
	int (*bulk_read)(struct device *dev, uint32_t address,
				void *buf, size_t len);
	int (*raw_write)(void *context,	uint32_t reg,
			const void *val, size_t val_len);

	uint32_t sys_rbdt[2*IAXXX_RBDT_NUM_ENTRIES];

	/* GPIOs */
	int reset_gpio;
	int event_gpio;
	int pwr_vld_gpio;
	u32 spi_app_speed;

	/* Regulators */
	struct regulator *vdd_io;
	struct regulator *vdd_core;

	/* External Clock */
	bool mclk_en;
	struct clk *ext_clk;
	int (*iaxxx_mclk_cb)(struct iaxxx_priv *, bool);

	/* Sensor port ID */
	u32 sensor_port;

	/* Update block lock */
	struct mutex update_block_lock;

	/* Register plugin locks */
	struct mutex plugin_lock;

	/* Register module locks */
	struct mutex module_lock;

	/* Processor on and off lock */
	struct mutex proc_on_off_lock;

	/* Sensor tunnel device locks */
	struct mutex sensor_tunnel_dev_lock;

	/* Event work queue */
	struct mutex event_work_lock;
	struct mutex event_queue_lock;
	struct work_struct event_work_struct;
	struct workqueue_struct *event_workq;

	/* Core kthread */
	struct task_struct *thread;
	struct kthread_worker worker;
	struct kthread_work fw_update_work;
	struct kthread_work fw_crash_work;
	struct kthread_work runtime_work;

	wait_queue_head_t boot_wq;
	wait_queue_head_t wakeup_wq;
	wait_queue_head_t irq_wake;

	void *tunnel_data;
	/* Event Manager */
	struct mutex event_lock;

	enum iaxxx_bus bus;

	/* For factory testing */
	struct work_struct dev_fw_update_test_work;
	struct work_struct dev_cmem_test_work;
	struct completion bootup_done;
	struct completion cmem_done;
	bool test_result;
	iaxxx_cb_bc_func_ptr_t spi_speed_setup;
	iaxxx_cb_func_ptr_t reset_cb;
	uint32_t fw_retry_count;
	struct mutex test_mutex;
	struct iaxxx_evt_queue *event_queue;

	/* Showing mode, boot (0) or app (1) */
	bool is_application_mode;
	struct iaxxx_raw_bus_ops *raw_ops;
	struct iaxxx_reg_dump_priv *reg_dump;
	void *intf_priv;
	bool dump_log;
	bool is_irq_enabled;
	bool boot_completed;
	bool is_chip_wokenup;

	void *dfs_node;

	uint32_t oscillator_mode;
	/* Notifiers */
	struct srcu_notifier_head core_notifier_list;
	struct notifier_block notifier_core;

	/* iaxxx core flags for atomic bit field operations */
	unsigned long flags;

	atomic_t fli_route_status;
	/* Synchronize suspend/resume on this */
	struct iaxxx_system_state *iaxxx_state;
	bool sensor_en[IAXXX_SENSR_ID_MASK + 1];
	uint32_t crash_count;
	bool cm4_crashed;
	atomic_t route_status;
	struct iaxxx_crashlog_header *crashlog_header;
	struct iaxxx_crashlog *crashlog;
	struct mutex crashdump_lock;

	/* FW Crash info */
	int fw_crash_reasons;
	uint32_t fw_crash_proc_id;
	int recovery_try_count;
	int try_count;
	bool in_suspend;
	bool in_resume;
	atomic_t pm_resume;
	struct mutex resume_mutex;
	struct wakeup_source ws;
	struct wakeup_source pm_ws;
	bool pm_ws_enable;

	/* Debug flags */
	bool debug_isr_disable;
	bool debug_fwcrash_handling_disable;
	bool debug_runtime_pm_disable;
	struct mutex debug_mutex;
	/* Max plugin instances supported */
	uint32_t plugin_inst_count;
	int int_osc_trim_period;

	struct mutex btp_lock;

	struct list_head static_mem_blks_list;

	/* Misc statistics data */
	void *misc_priv;
	/* iaxxx_misc hwinfo data */
	struct hwinfo_type hwinfo;

	/* workq to handle mclk on/off */
	bool ext_clk_off_work_initialized;
	struct delayed_work ext_clk_off_work;
};

enum update_block_options_t	{
	UPDATE_BLOCK_NO_OPTIONS = 0x0,
	UPDATE_BLOCK_FIXED_WAIT_OPTION = 0x1,
	UPDATE_BLOCK_NO_LOCK_OPTION = 0x2,
	UPDATE_BLOCK_STATUS_CHECK_AFTER_FIXED_WAIT_OPTION = 0x4,
};

static inline struct iaxxx_priv *to_iaxxx_priv(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

static inline bool find_host_id(struct iaxxx_priv *priv, uint32_t inst_id)
{
	return (inst_id >= priv->plugin_inst_count)?IAXXX_HOST_1:IAXXX_HOST_0;
}

static inline bool iaxxx_core_get_route_status(struct iaxxx_priv *priv)
{
	return atomic_read(&priv->route_status);
}

static inline void iaxxx_core_set_route_status(struct iaxxx_priv *priv,
		bool route_status)
{
	/* Set the route status only if FW is in normal mode */
	if (test_bit(IAXXX_FLG_FW_READY, &priv->flags))
		atomic_set(&priv->route_status, route_status);
}

static inline bool iaxxx_is_firmware_ready(struct iaxxx_priv *priv)
{
	return test_bit(IAXXX_FLG_FW_READY, &priv->flags);
}

int iaxxx_core_sensor_change_state(struct device *dev, uint32_t inst_id,
			uint8_t is_enable, uint8_t block_id);
int iaxxx_core_sensor_get_param_by_inst(struct device *dev, uint32_t inst_id,
			uint32_t param_id,
			uint32_t *param_val, uint32_t block_id);
int iaxxx_core_sensor_set_param_by_inst(struct device *dev, uint32_t inst_id,
			uint32_t param_id,
			uint32_t param_val, uint32_t block_id);
int iaxxx_core_sensor_write_param_blk_by_inst(struct device *dev,
			uint32_t inst_id, uint32_t param_blk_id,
			const void *ptr_blk, uint32_t blk_size,
			uint32_t block_id);
int iaxxx_core_get_sensor_mode_stats(struct device *dev,
			struct iaxxx_sensor_mode_stats *sensor_stats);
int iaxxx_send_update_block_request(struct device *dev, uint32_t *status,
			int id);
int iaxxx_send_update_block_hostid(struct device *dev,
			int host_id, int block_id);
int iaxxx_poll_update_block_req_bit_clr(struct iaxxx_priv *priv,
					struct regmap *regmap);
int iaxxx_send_update_block_request_with_options(struct device *dev,
			int block_id,
			int host_id,
			struct regmap *regmap,
			uint32_t wait_time_in_ms,
			enum update_block_options_t options,
			uint32_t *status);
int iaxxx_get_firmware_version(struct device *dev, char *ver, uint32_t len);
int iaxxx_get_application_ver_num(struct device *dev, uint32_t *app_ver_num);
int iaxxx_get_rom_ver_num(struct device *dev, uint32_t *rom_ver_num);
int iaxxx_get_rom_version(struct device *dev, char *ver, uint32_t len);
int iaxxx_get_device_id(struct device *dev, uint32_t *device_id);
int iaxxx_core_plg_get_package_version(struct device *dev,
			uint8_t inst_id, char *ver, uint32_t len);
int iaxxx_core_plg_get_plugin_version(struct device *dev,
			uint8_t inst_id, char *ver, uint32_t len);
int iaxxx_core_plg_get_param_by_inst(struct device *dev, uint32_t inst_id,
			uint32_t param_id,
			uint32_t *param_val, uint32_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
			uint32_t param_id,
			uint32_t param_val, uint32_t block_id);
int iaxxx_core_create_plg(struct device *dev, uint32_t inst_id,
			uint32_t priority, uint32_t pkg_id,
			uint32_t plg_idx, uint8_t block_id,
			uint32_t config_id);
int iaxxx_core_create_plg_static_package(
			struct device *dev, uint32_t inst_id,
			uint32_t priority, uint32_t pkg_id,
			uint32_t plg_idx, uint8_t block_id,
			uint32_t config_id);

int iaxxx_core_change_plg_state(struct device *dev, uint32_t inst_id,
			uint8_t is_enable, uint8_t block_id);
int iaxxx_core_destroy_plg(struct device *dev, uint32_t inst_id,
			uint8_t block_id);
int iaxxx_core_reset_plg(struct device *dev, uint32_t inst_id,
			uint8_t block_id);
int iaxxx_core_plg_set_param_by_inst(struct device *dev, uint32_t inst_id,
			uint32_t param_id,
			uint32_t param_val, uint32_t block_id);
int iaxxx_core_set_create_cfg(struct device *dev, uint32_t inst_id,
			uint32_t cfg_size, uint64_t cfg_val, uint32_t block_id,
			char *file);
int iaxxx_core_set_param_blk_common(
			struct device *dev,
			uint32_t inst_id, uint32_t blk_size,
			const void *ptr_blk, uint32_t block_id,
			uint32_t param_blk_id);
int iaxxx_core_set_param_blk(struct device *dev, uint32_t inst_id,
			uint32_t blk_size, const void *ptr_blk,
			uint32_t block_id,
			uint32_t param_blk_id);
int iaxxx_core_set_param_blk_from_file(struct device *dev, uint32_t inst_id,
			uint32_t block_id, uint32_t param_blk_id,
			const char *file);
int iaxxx_core_set_param_blk_with_ack(struct device *dev,
			const uint32_t inst_id,
			const uint32_t param_blk_id,
			const uint32_t block_id,
			const void *set_param_buf,
			const uint32_t set_param_buf_sz,
			uint32_t  *response_data_buf,
			const uint32_t response_data_sz,
			const uint32_t max_no_retries);
int iaxxx_core_get_param_blk_common(
			struct device *dev,
			uint32_t  inst_id,
			uint32_t  block_id,
			uint32_t  param_blk_id,
			uint32_t *getparam_block_data,
			uint32_t  getparam_block_size_in_words);
int iaxxx_core_get_param_blk(
			struct device *dev,
			uint32_t  inst_id,
			uint32_t  block_id,
			uint32_t  param_blk_id,
			uint32_t *getparam_block_data,
			uint32_t  getparam_block_size_in_words);
int iaxxx_core_set_custom_cfg(struct device *dev, uint32_t inst_id,
			uint32_t block_id, uint32_t param_blk_id,
			uint32_t custom_config_id, char *file);
int iaxxx_core_evt_subscribe(struct device *dev, uint16_t src_id,
		uint16_t event_id, uint16_t dst_id, uint32_t dst_opaque);
int iaxxx_core_evt_unsubscribe(struct device *dev, uint16_t src_id,
			uint16_t event_id, uint16_t dst_id);
int iaxxx_core_evt_read_subscription(struct device *dev,
			uint16_t *src_id,
			uint16_t *evt_id,
			uint16_t *dst_id,
			uint32_t *dst_opaque);
int iaxxx_core_evt_retrieve_notification(struct device *dev,
			uint16_t *src_id,
			uint16_t *evt_id,
			uint32_t *src_opaque,
			uint32_t *dst_opaque);
int iaxxx_core_evt_reset_read_index(struct device *dev);
int iaxxx_core_evt_trigger(struct device *dev,
			uint16_t src_id, uint16_t evt_id, uint32_t src_opaque);
int iaxxx_core_retrieve_event(struct device *dev, uint16_t *event_id,
			uint32_t *data);
int iaxxx_core_set_event(struct device *dev, uint8_t inst_id,
			uint32_t event_enable_mask, uint32_t block_id);
int iaxxx_core_plg_get_status_info(struct device *dev, uint32_t inst_id,
			struct iaxxx_plugin_status_data *plugin_status_data);

int iaxxx_core_plg_get_endpoint_status(struct device *dev,
	uint32_t inst_id, uint8_t ep_index, uint8_t direction,
	struct iaxxx_plugin_endpoint_status_data *plugin_ep_status_data);
int iaxxx_core_plg_get_endpoint_timestamps(struct device *dev,
			uint64_t *timestamps, int ep_num, uint8_t proc_id);
int iaxxx_core_resume_rt(struct device *dev);
int iaxxx_core_suspend_rt(struct device *dev);
int iaxxx_core_dev_resume(struct device *dev);
int iaxxx_core_dev_suspend(struct device *dev);

int iaxxx_wait_dev_resume(struct device *dev);

int iaxxx_package_load(struct device *dev, const char *pkg_name,
			uint32_t pkg_id, uint32_t *proc_id);
int iaxxx_package_unload(struct device *dev,
			int32_t pkg_id);
int iaxxx_clr_pkg_plg_list(struct iaxxx_priv *priv);
int iaxxx_core_read_plugin_error(
			struct device  *dev,
			const uint32_t  block_id,
			uint32_t *error_code,
			uint8_t  *error_instance);
int iaxxx_core_script_load(struct device *dev,
			const char *script_name,
			uint32_t script_id);
int iaxxx_core_script_unload(struct device *dev,
			uint32_t script_id);
int iaxxx_core_script_trigger(struct device *dev,
			uint32_t script_id);
int iaxxx_fw_notifier_register(struct device *dev, struct notifier_block *nb);
int iaxxx_fw_notifier_unregister(struct device *dev, struct notifier_block *nb);
int iaxxx_fw_notifier_call(struct device *dev, unsigned long val, void *v);
int iaxxx_get_debug_log_level(struct device *dev,
			uint32_t module_id, uint32_t *log_level);
int iaxxx_set_debug_log_level(struct device *dev,
			uint32_t module_id, uint32_t log_level);
int iaxxx_set_debug_log_mode(struct device *dev,
			bool mode, uint8_t proc_id);
int iaxxx_get_debug_log_mode(struct device *dev,
			bool *mode, uint8_t proc_id);
bool iaxxx_core_plg_is_valid_inst_id(uint32_t inst_id);
bool iaxxx_core_plg_is_valid_pkg_id(uint32_t pkg_id);
bool iaxxx_core_plg_is_valid_priority(uint32_t priority);
bool iaxxx_core_plg_is_valid_block_id(uint32_t block_id);
bool iaxxx_core_plg_is_valid_plg_idx(uint32_t plg_idx);
bool iaxxx_core_plg_is_valid_param_id(uint32_t param_id);
bool iaxxx_core_plg_is_valid_param_val(uint32_t param_val);
bool iaxxx_core_plg_is_valid_param_blk_id(uint32_t param_id);
bool iaxxx_core_plg_is_valid_param_blk_size(uint32_t param_size);
bool iaxxx_core_plg_is_valid_cfg_size(uint32_t cfg_size);
bool iaxxx_core_evt_is_valid_src_id(uint32_t src_id);
bool iaxxx_core_evt_is_valid_dst_id(uint32_t dst_id);
bool iaxxx_core_evt_is_valid_event_id(uint32_t event_id);
bool iaxxx_core_evt_is_valid_dst_opaque(uint32_t dst_opaque);
bool iaxxx_core_sensor_is_valid_script_id(uint32_t script_id);
bool iaxxx_core_sensor_is_valid_inst_id(uint32_t inst_id);
bool iaxxx_core_sensor_is_valid_block_id(uint32_t block_id);
bool iaxxx_core_sensor_is_valid_param_id(uint32_t param_id);
bool iaxxx_core_sensor_is_valid_param_val(uint32_t param_val);
bool iaxxx_core_sensor_is_valid_param_blk_id(uint32_t param_id);
bool iaxxx_core_sensor_is_valid_param_blk_size(uint32_t param_size);
int iaxxx_set_plugin_log_mode(struct device *dev,
				bool mode, uint32_t inst_id, uint8_t block_id);
int iaxxx_get_plugin_log_mode(struct device *dev,
				bool *mode, uint32_t inst_id, uint8_t block_id);
int iaxxx_set_update_plugin_log_state(struct device *dev,
				bool mode, uint32_t inst_id, uint8_t block_id);
int iaxxx_get_plugin_log_state(struct device *dev,
				bool *mode, uint32_t inst_id, uint8_t block_id);
int iaxxx_core_get_pwr_stats(struct device *dev,
			struct iaxxx_pwr_stats *pwr_stats);
/* Boots up the core based on proc id mask */
int iaxxx_boot_core(struct iaxxx_priv *priv, u32 proc_id_mask);
int iaxxx_fw_reset(struct iaxxx_priv *priv);

#endif /*__IAXXX_CORE_H__ */
