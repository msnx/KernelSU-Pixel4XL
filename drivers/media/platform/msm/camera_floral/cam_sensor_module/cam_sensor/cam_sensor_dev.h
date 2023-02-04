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

#ifndef _CAM_SENSOR_DEV_H_
#define _CAM_SENSOR_DEV_H_

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/ion.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include <cam_sensor_io.h>
#include "cam_debug_util.h"
#include "cam_context.h"

#define NUM_MASTERS 2
#define NUM_QUEUES 2

#define TRUE  1
#define FALSE 0

/* The sensor define need to match to dtsi slot number */
#define REAR_WIDE 0
#define REAR_TELE 1
#define FRONT_CAM 2
#define IR_MASTER 3
#define IR_SLAVE 4

#define FLASH_STROBE_ADDR 0x3B81
#define FLASH_STROBE_EVEN 0xAA
#define FLASH_STROBE_ODD 0x55
#define FLASH_STROBE_DISABLE 0x00

#define REAR_WIDE_STREAMON_MASK (1 << REAR_WIDE)
#define REAR_TELE_STREAMON_MASK (1 << REAR_TELE)
#define FRONT_CAM_STREAMON_MASK (1 << FRONT_CAM)
#define IR_MASTER_STREAMON_MASK (1 << IR_MASTER)
#define IR_SLAVE_STREAMON_MASK (1 << IR_SLAVE)

#undef CDBG
#ifdef CAM_SENSOR_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define SENSOR_DRIVER_I2C "i2c_camera"
#define CAMX_SENSOR_DEV_NAME "cam-sensor-driver"

enum cam_sensor_state_t {
	CAM_SENSOR_INIT,
	CAM_SENSOR_ACQUIRE,
	CAM_SENSOR_CONFIG,
	CAM_SENSOR_START,
};

enum cam_sensor_gpio_irq {
	LASER_STATUS,
	SILEGO_FAULT,
	CAM_SENSOR_GPIO_IRQ_MAX,
};

/**
 * struct intf_params
 * @device_hdl: Device Handle
 * @session_hdl: Session Handle
 * @link_hdl: Link Handle
 * @ops: KMD operations
 * @crm_cb: Callback API pointers
 */
struct intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

/**
 * struct cam_sensor_ctrl_t: Camera control structure
 * @device_name: Sensor device name
 * @pdev: Platform device
 * @cam_sensor_mutex: Sensor mutex
 * @sensordata: Sensor board Information
 * @cci_i2c_master: I2C structure
 * @io_master_info: Information about the communication master
 * @sensor_state: Sensor states
 * @is_probe_succeed: Probe succeeded or not
 * @id: Cell Index
 * @of_node: Of node ptr
 * @v4l2_dev_str: V4L2 device structure
 * @sensor_probe_addr_type: Sensor probe address type
 * @sensor_probe_data_type: Sensor probe data type
 * @i2c_data: Sensor I2C register settings
 * @sensor_info: Sensor query cap structure
 * @bridge_intf: Bridge interface structure
 * @streamon_count: Count to hold the number of times stream on called
 * @streamoff_count: Count to hold the number of times stream off called
 * @bob_reg_index: Hold to BoB regulator index
 * @bob_pwm_switch: Boolean flag to switch into PWM mode for BoB regulator
 * @fw_update_flag: Update OIS firmware
 * @strobe_type: Strobe type that indicate how to reset strobe
 * @first_strobe_frame: indicate first strobe frame count after reset
 * @last_flush_req: Last request to flush
 * @pipeline_delay: Sensor pipeline delay
 * @ois_fw_ver: OIS firmware version
 * @vcm_fw_ver: VCM firmware version
 * @hw_sync_ctrl: HW sync control structure
 */
struct cam_sensor_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	struct cam_sensor_board_info *sensordata;
	enum cci_i2c_master_t cci_i2c_master;
	enum cci_device_num cci_num;
	struct camera_io_master io_master_info;
	struct camera_io_master peer_ir_info;
	enum cam_sensor_state_t sensor_state;
	uint8_t is_probe_succeed;
	uint32_t id;
	struct device_node *of_node;
	struct cam_subdev v4l2_dev_str;
	uint8_t sensor_probe_addr_type;
	uint8_t sensor_probe_data_type;
	struct i2c_data_settings i2c_data;
	struct  cam_sensor_query_cap sensor_info;
	struct intf_params bridge_intf;
	uint32_t streamon_count;
	uint32_t streamoff_count;
	int bob_reg_index;
	bool bob_pwm_switch;
	struct cam_sensor_override_info override_info;
	uint32_t last_flush_req;
	uint16_t pipeline_delay;
	uint8_t fw_update_flag;
	enum strobe_type strobeType;
	uint64_t first_strobe_frame;
	uint16_t ois_fw_ver;
	uint16_t vcm_fw_ver;
	uint32_t hw_version;
	struct cam_sensor_hw_sync_ctrl hw_sync_ctrl;
	uint8_t cam_sensor_irq[CAM_SENSOR_GPIO_IRQ_MAX];
	uint8_t cam_safety_gpio_idx[CAM_SENSOR_GPIO_IRQ_MAX];
};

struct sensor_status_t {
	uint8_t streamon_mask;
	bool is_strobe_disabled;
};

#endif /* _CAM_SENSOR_DEV_H_ */
