/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
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
#ifndef _CAM_OIS_DEV_H_
#define _CAM_OIS_DEV_H_

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"
#include "cam_context.h"

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)
#define READ_OUT_TIME                   5000000 /*5ms*/
#define CAM_OIS_SHIFT_DATA_BUFFER_SIZE  15
#define MAX_FAIL_CNT                    3

struct cam_ois_ctrl_t;

enum cam_ois_state {
	CAM_OIS_INIT,
	CAM_OIS_ACQUIRE,
	CAM_OIS_CONFIG,
	CAM_OIS_START,
};

enum cam_ois_timer_state_t {
	CAM_OIS_TIME_INIT,
	CAM_OIS_TIME_ACTIVE,
	CAM_OIS_TIME_INACTIVE,
	CAM_OIS_TIME_ERROR,
};

/**
 * struct cam_ois_registered_driver_t - registered driver info
 * @platform_driver      :   flag indicates if platform driver is registered
 * @i2c_driver           :   flag indicates if i2c driver is registered
 *
 */
struct cam_ois_registered_driver_t {
	bool platform_driver;
	bool i2c_driver;
};

/**
 * struct cam_ois_i2c_info_t - I2C info
 * @slave_addr      :   slave address
 * @i2c_freq_mode   :   i2c frequency mode
 *
 */
struct cam_ois_i2c_info_t {
	uint16_t slave_addr;
	uint8_t i2c_freq_mode;
};

/**
 * struct cam_ois_soc_private - ois soc private data structure
 * @ois_name        :   ois name
 * @i2c_info        :   i2c info structure
 * @power_info      :   ois power info
 *
 */
struct cam_ois_soc_private {
	const char *ois_name;
	struct cam_ois_i2c_info_t i2c_info;
	struct cam_sensor_power_ctrl_t power_info;
};

/**
 * struct cam_ois_intf_params - bridge interface params
 * @device_hdl   : Device Handle
 * @session_hdl  : Session Handle
 * @ops          : KMD operations
 * @crm_cb       : Callback API pointers
 */
struct cam_ois_intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

/**
 * struct cam_ois_shift_buffer - OIS shift data ring buffer
 * @buffer          :   array of shift data readout
 * @write_pos       :   next position to write
 * @is_full         :   flag of full buffer
 */
struct cam_ois_shift_buffer {
	struct cam_ois_shift buffer[CAM_OIS_SHIFT_DATA_BUFFER_SIZE];
	int32_t write_pos;
	bool is_full;
};

/**
 * struct cam_ois_timer_t - OIS shift reader timer
 * @hr_timer        :   timer
 * @ois_wq          :   worker queue of actual worker procedure
 * @g_work          :   actual worker
 * @ois_timer_state :   state of timer
 * @o_ctrl          :   ois control structure
 * @i2c_fail_count  :   number of i2c fails
 */
struct cam_ois_timer_t {
	struct hrtimer hr_timer;
	struct workqueue_struct *ois_wq;
	struct work_struct g_work;
	enum cam_ois_timer_state_t ois_timer_state;
	struct cam_ois_ctrl_t *o_ctrl;
	int i2c_fail_count;
};

/**
 * struct cam_ois_ctrl_t - OIS ctrl private data
 * @device_name     :   ois device_name
 * @pdev            :   platform device
 * @ois_mutex       :   ois mutex
 * @soc_info        :   ois soc related info
 * @io_master_info  :   Information about the communication master
 * @cci_i2c_master  :   I2C structure
 * @v4l2_dev_str    :   V4L2 device structure
 * @bridge_intf     :   bridge interface params
 * @i2c_init_data   :   ois i2c init settings
 * @i2c_mode_data   :   ois i2c mode settings
 * @i2c_calib_data  :   ois i2c calib settings
 * @ois_device_type :   ois device type
 * @cam_ois_state   :   ois_device_state
 * @ois_fw_flag     :   flag for firmware download
 * @is_ois_calib    :   flag for Calibration data
 * @opcode          :   ois opcode
 * @device_name     :   Device name
 * @buf             :   ois shift data buffer
 * @timer           :   ois timer
 * @ois_shift_mutex :   ois shift mutex
 */
struct cam_ois_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct platform_device *pdev;
	struct mutex ois_mutex;
	struct cam_hw_soc_info soc_info;
	struct camera_io_master io_master_info;
	enum cci_i2c_master_t cci_i2c_master;
	enum cci_device_num cci_num;
	struct cam_subdev v4l2_dev_str;
	struct cam_ois_intf_params bridge_intf;
	struct i2c_settings_array i2c_init_data;
	struct i2c_settings_array i2c_calib_data;
	struct i2c_settings_array i2c_mode_data;
	enum msm_camera_device_type_t ois_device_type;
	enum cam_ois_state cam_ois_state;
	char ois_name[32];
	uint8_t ois_fw_flag;
	uint8_t is_ois_calib;
	struct cam_ois_opcode opcode;
	struct cam_ois_shift_buffer buf;
	struct cam_ois_timer_t timer;
	struct mutex ois_shift_mutex;
#ifdef CONFIG_CAMERA_ACT_READ_LENS
	struct camera_io_master af_io_master_info;
	struct cam_sensor_cci_client af_cci_client;
#endif
};

#endif /*_CAM_OIS_DEV_H_ */
