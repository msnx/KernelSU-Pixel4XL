/*
 *  lm36011_module.c - Linux kernel module for led laser
 *
 *  Copyright (C) 2018 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include "cam_sensor_dev.h"
#include "cam_req_mgr_dev.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <media/cam_sensor.h>
#include <media/cam_req_mgr.h>

#define LM36011_DEV_NAME "lm36011"

#define ENABLE_REG 0x01
#define CONFIGURATION_REG 0x02
#define LED_FLASH_BRIGHTNESS_REG 0x03
#define LED_TORCH_BRIGHTNESS_REG 0x04
#define FLAG_REG 0x05
#define DEVICE_ID_REG 0x06

#define IR_STANDBY_MODE 0x04
#define IR_ENABLE_MODE 0x05
#define IR_TORCH_MODE 0x02
#define IR_DISABLE_MODE 0x00
#define DEVICE_ID 0x01

/*
 * PHASE 1 : Dot ITO-C
 * PHASE 2 : Flood ITO-C
 * PHASE 3 : Cap sense IC temperature
 */
#define PHASE0 0
#define PHASE1 1
#define PHASE2 2
#define PHASE3 3
#define PHASENUM 4

#define MAX_RETRY_COUNT 100

#define PHASE_SELECT_REG 0x60
#define PROXAVG_REG 0x63
#define PROXOFFSET_REG 0x67

#define NUM_SKIP_SAMPLE 5

/* crack unit in aF */
#define DUMMY_CRACK_THRES 0x7FFFFFFF
#define CRACK_THRES_EVT1 (250 * 1000) // in aF
#define CRACK_THRES_FLOOD_EVT1_1 (1000 * 1000) // in aF
#define CRACK_THRES_DOT (1000 * 1000) // in aF

/* Silego define */
#define FAULT_FLAG_ADDR 0xF7
#define NO_FAULT 0xC0
#define ITOR_OPEN_CIRCUIT 0xC2
#define DOT_OCP_FAULT 0xD0
#define FLOOD_OCP_FAULT 0xC4
#define DOT_ITOR_FAULT 0xD2
#define FLOOD_ITOR_FAULT 0xC6

#define VIO_VOLTAGE_MIN 1800000
#define VIO_VOLTAGE_MAX 1800000
#define SLIEGO_VDD_VOlTAGE_MIN 2800000
#define SLIEGO_VDD_VOlTAGE_MAX 3200000
#define SX9320_VDD_VOlTAGE_MIN 1800000
#define SX9320_VDD_VOlTAGE_MAX 1800000
#define PMIC_BUCK1_VOlTAGE_MIN 1350000
#define PMIC_BUCK1_VOlTAGE_MAX 1350000
#define PMIC_BUCK2_VOlTAGE_MIN 3300000
#define PMIC_BUCK2_VOlTAGE_MAX 4100000

#define BUILD_DEV 0
/* ITO-R + Silego register map V1 */
#define BUILD_PROTO 1
/* ITO-C + Silego register map V2 */
#define BUILD_EVT1_0 2
/* ITO-C + Silego register map V3 */
#define BUILD_EVT1_1 3
/* ITO-R + Silego register map V4 */
#define BUILD_DVT 4
/* ITO-R + Silego register map V4 + Silego i2c write locked */
#define BUILD_PVT 5

#define MAX_SILEGO_GPIO_SIZE (SILEGO_GPIO_MAX_ITOC + SILEGO_GPIO_MAX_ITOR)

enum SILEGO_GPIO_ITOC {
	IR_VCSEL_FAULT_ITOC,
	IR_VCSEL_TEST_ITOC,
	SILEGO_SW_HALT_ITOC,
	SILEGO_GPIO_MAX_ITOC
};

enum SILEGO_GPIO_ITOR {
	IR_VCSEL_FAULT_ITOR,
	SILEGO_GPIO_MAX_ITOR
};

enum CAP_SENSE_GPIO {
	CSENSE_PROXAVG_READ,
	CAP_SENSE_GPIO_MAX
};

enum LASER_TYPE {
	LASER_FLOOD,
	LASER_DOT,
	LASER_TYPE_MAX
};

struct reg_setting {
	uint32_t addr;
	uint32_t data;
};

struct led_laser_ctrl_t {
	struct platform_device *pdev;
	struct cam_hw_soc_info soc_info;
	struct mutex cam_sensor_mutex;
	struct camera_io_master io_master_info;
	bool is_power_up;
	bool is_cci_init;
	bool is_probed;
	bool is_certified;
	struct regulator *vio;
	struct regulator *buck1;
	struct regulator *buck2;
	enum LASER_TYPE type;
	uint32_t read_addr;
	uint32_t read_data;
	dev_t dev;
	struct cdev c_dev;
	struct class *cl;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_active_state;
	struct pinctrl_state *gpio_suspend_state;
	int gpio_count;
	struct work_struct work;
	struct workqueue_struct *work_queue;
	uint32_t hw_version;
	struct {
		bool is_validated;
		bool is_power_up;
		bool is_vcsel_fault;
		bool is_csense_halt;
		bool self_test_result;
		uint32_t vcsel_fault_count;
		struct regulator *vdd;
		struct gpio *gpio_array;
		unsigned int irq[MAX_SILEGO_GPIO_SIZE];
		int32_t fault_flag;
		struct camera_io_master io_master_info;
		bool is_cci_init;
	} silego;
	struct {
		bool is_power_up;
		bool is_validated;
		struct regulator *vdd;
		uint32_t sid;
		struct gpio *gpio_array;
		unsigned int irq[CAP_SENSE_GPIO_MAX];
		int16_t proxavg[PHASENUM];
		uint16_t proxoffset[PHASENUM];
		struct camera_io_master io_master_info;
		bool is_cci_init;
		uint16_t calibration_data[PHASENUM];
		int32_t cap_slope[PHASENUM];
		int32_t cap_bias[PHASENUM];
		int32_t cap_raw[PHASENUM];
		int32_t cap_corrected[PHASENUM];
		uint32_t max_supported_temp[PHASENUM];
		bool is_crack_detected[LASER_TYPE_MAX];
		uint16_t sample_count;
	} cap_sense;
};

static bool read_proxoffset;
static bool crack_log_en;
static enum LASER_TYPE safety_ic_owner = LASER_TYPE_MAX;
/*
 * a global mutex is needed since there have two
 * available lm36011 ic on board
 */
static DEFINE_MUTEX(lm36011_mutex);
module_param(read_proxoffset, bool, 0644);
module_param(crack_log_en, bool, 0644);

static const struct reg_setting silego_reg_settings_ver1[] = {
	{0xc0, 0x09}, {0xc1, 0xf9}, {0xc2, 0x09}, {0xc3, 0xf9}, {0xcb, 0x93},
	{0xcc, 0x81}, {0xcd, 0x93}, {0xce, 0x83}, {0x92, 0x00}, {0x93, 0x00}
};

static const struct reg_setting silego_reg_settings_ver2[] = {
	{0xc0, 0x09}, {0xc1, 0xf9}, {0xc2, 0x09}, {0xc3, 0xf9}, {0xcb, 0x00},
	{0xcc, 0x9a}, {0xcd, 0x13}, {0xce, 0x9b}, {0x92, 0x00}, {0x93, 0x00},
	{0xa0, 0x0e}, {0xa2, 0x0e}, {0x80, 0xb0}, {0x81, 0x24}, {0x82, 0x3c},
	{0x83, 0x2c}, {0x84, 0x58}, {0x85, 0x80}, {0x86, 0x40}, {0x87, 0x40},
	{0x88, 0x3e}, {0x89, 0x60}, {0x8a, 0x40}, {0x8b, 0x30}, {0x8c, 0x7c},
	{0x8d, 0x24}, {0x8e, 0xa0}, {0x8f, 0xc0}, {0x90, 0x40}, {0x91, 0xa0},
};

static const struct reg_setting silego_reg_settings_ver3[] = {
	{0xc0, 0x01}, {0xc1, 0xbb}, {0xc2, 0x01}, {0xc3, 0xbb}, {0xcb, 0x93},
	{0xcc, 0x9a}, {0xcd, 0x00}, {0xce, 0x9b}, {0x92, 0x00}, {0x93, 0x00},
	{0xa0, 0x12}, {0xa2, 0x12}, {0x80, 0xb0}, {0x81, 0x24}, {0x82, 0x58},
	{0x83, 0x2c}, {0x84, 0x68}, {0x85, 0x80}, {0x86, 0x40}, {0x87, 0x40},
	{0x88, 0x3e}, {0x89, 0x60}, {0x8a, 0x40}, {0x8b, 0x30}, {0x8c, 0x7c},
	{0x8d, 0x24}, {0x8e, 0xa0}, {0x8f, 0xc0}, {0x90, 0x40}, {0x91, 0xa0},
};

static const struct reg_setting silego_reg_settings_ver4[] = {
	{0xc0, 0x01}, {0xc1, 0xa6}, {0xc2, 0xe0}, {0xc3, 0xa6}, {0xcb, 0x97},
	{0xcc, 0x9a}, {0xce, 0x9b}, {0x92, 0x00}, {0x93, 0x00}, {0xa0, 0x1a},
	{0xa2, 0x1a}, {0x80, 0xb0}, {0x81, 0x24}, {0x82, 0x58}, {0x83, 0x2c},
	{0x84, 0x60}, {0x85, 0x2c}, {0x86, 0x40}, {0x87, 0x40}, {0x88, 0x3e},
	{0x89, 0x60}, {0x8a, 0x00}, {0x8b, 0x30}, {0x8c, 0x7c}, {0x8d, 0x24},
	{0x8e, 0xa0}, {0x8f, 0x80}, {0x90, 0x00},
};

static const struct reg_setting cap_sense_init_reg_settings[] = {
	{0x10, 0x0F},
	{0x11, 0x2E},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x20, 0x20},
	{0x23, 0x01},
	{0x24, 0x47},
	{0x26, 0x01},
	{0x27, 0x47},
	{0x28, 0x3D},
	{0x29, 0x37},
	{0x2A, 0x1F},
	{0x2B, 0x40},
	{0x2C, 0x12},
	{0x2D, 0x0F},
	{0x30, 0x09},
	{0x31, 0x09},
	{0x32, 0x3F},
	{0x33, 0xC0},
	{0x34, 0x00},
	{0x35, 0x00},
	{0x36, 0x69},
	{0x37, 0x69},
	{0x40, 0x00},
	{0x41, 0x00},
	{0x42, 0x17},
	{0x43, 0x00},
	{0x44, 0x00},
	{0x45, 0x05},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x48, 0x00},
	{0x49, 0x00},
	{0x4A, 0x40},
	{0x4B, 0x31},
	{0x4C, 0x00},
	{0x4D, 0x00},
	{0x4E, 0x80},
	{0x4F, 0x0C},
	{0x50, 0x34},
	{0x51, 0x77},
	{0x52, 0x22},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x02, 0x02},
	{0x03, 0x0F},
	{0x05, 0x08},
	{0x06, 0x04},
	{0x07, 0x80},
	{0x08, 0x01},
	{0x00, 0x08},
};

static void convert_proxvalue_to_aF(struct led_laser_ctrl_t *ctrl)
{
	int i;

	for (i = PHASE1; i <= PHASE2; i++) {
		ctrl->cap_sense.cap_raw[i] =
			((((ctrl->cap_sense.proxoffset[i] >> 7) & 0x007F)
			* 2123) +
			((ctrl->cap_sense.proxoffset[i] & 0x007F) * 50))
			* 1000 + ctrl->cap_sense.proxavg[i] * 147;
	}
}

static void itoc_temperature_correction(struct led_laser_ctrl_t *ctrl)
{
	int i;
	int32_t temp_mC = ctrl->cap_sense.proxavg[PHASE3] * 1000 / 57;
	int32_t max_temp = min(ctrl->cap_sense.max_supported_temp[PHASE1],
		ctrl->cap_sense.max_supported_temp[PHASE2]) & 0x7FFFFFFF;
	uint32_t temp_coe;

	if (temp_mC >= max_temp || temp_mC < 0) {
		dev_err(ctrl->soc_info.dev,
			"temperature %d over threshold (0~%d), kill laser",
			temp_mC, max_temp);
		ctrl->cap_sense.cap_corrected[PHASE1] = 0;
		ctrl->cap_sense.cap_corrected[PHASE2] = 0;
		return;
	}

	for (i = PHASE1; i <= PHASE2; i++) {
		temp_coe = temp_mC * ctrl->cap_sense.cap_slope[i];
		ctrl->cap_sense.cap_corrected[i] =
			ctrl->cap_sense.cap_raw[i] -
			(temp_coe / 1000);
	}
}

static int sx9320_write_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;
	rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
		&write_setting);
	if (rc < 0)
		dev_err(ctrl->soc_info.dev,
			"write 0x%x to cap sense 0x%x failed", data, addr);

	return rc;
}

static int sx9320_cleanup_nirq(struct led_laser_ctrl_t *ctrl)
{
	int rc;
	uint32_t data;

	rc = camera_io_dev_read(
		&ctrl->cap_sense.io_master_info,
		0x00,
		&data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
		dev_err(ctrl->soc_info.dev, "clean up NIRQ failed");

	return rc;
}

static void sx9320_crack_detection(struct led_laser_ctrl_t *ctrl)
{
	int gpio_level, flood_thres, dot_thres;
	bool is_sw_halt_required;

	if (ctrl->cap_sense.calibration_data[PHASE1] == 0 ||
		ctrl->cap_sense.calibration_data[PHASE2] == 0 ||
		ctrl->cap_sense.cap_bias[PHASE1] == 0 ||
		ctrl->cap_sense.cap_bias[PHASE2] == 0 ||
		ctrl->cap_sense.cap_slope[PHASE1] == 0 ||
		ctrl->cap_sense.cap_slope[PHASE2] == 0)
		return;

	convert_proxvalue_to_aF(ctrl);
	itoc_temperature_correction(ctrl);
	gpio_level =
		gpio_get_value(
		ctrl->silego.gpio_array[SILEGO_SW_HALT_ITOC].gpio);

	flood_thres = (ctrl->hw_version == BUILD_EVT1_1) ?
		CRACK_THRES_FLOOD_EVT1_1 : CRACK_THRES_EVT1;
	dot_thres = CRACK_THRES_DOT;

	ctrl->cap_sense.is_crack_detected[LASER_FLOOD] =
		(abs(ctrl->cap_sense.cap_bias[PHASE2] -
		ctrl->cap_sense.cap_corrected[PHASE2]) > flood_thres) ?
		true : false;

	ctrl->cap_sense.is_crack_detected[LASER_DOT] =
		(abs(ctrl->cap_sense.cap_bias[PHASE1] -
		ctrl->cap_sense.cap_corrected[PHASE1]) > dot_thres) ?
		true : false;

	is_sw_halt_required =
		(ctrl->cap_sense.is_crack_detected[LASER_FLOOD] |
		ctrl->cap_sense.is_crack_detected[LASER_DOT]);

	if (!gpio_level && is_sw_halt_required) {
		dev_err(ctrl->soc_info.dev,
			"detected laser lens crack %d:%d(Flood:Dot), "
			"kill laser now",
			ctrl->cap_sense.is_crack_detected[LASER_FLOOD],
			ctrl->cap_sense.is_crack_detected[LASER_DOT]);
		gpio_set_value(
			ctrl->silego.gpio_array[SILEGO_SW_HALT_ITOC].gpio, 1);
		cam_req_mgr_update_safety_ic_status(LENS_CRACK);
	}

	if (gpio_level && !is_sw_halt_required) {
		dev_info(ctrl->soc_info.dev,
			"flood and dot crack are solved, release sw halt now");
		gpio_set_value(
			ctrl->silego.gpio_array[SILEGO_SW_HALT_ITOC].gpio, 0);
		cam_req_mgr_update_safety_ic_status(NO_ERROR);
	}

	if (crack_log_en) {
		dev_info(ctrl->soc_info.dev,
			"flood C_raw %d aF C_corrected: %d aF "
			"Cap_bias: %d aF temp: %d mC is crack: %d",
			ctrl->cap_sense.cap_raw[PHASE2],
			ctrl->cap_sense.cap_corrected[PHASE2],
			ctrl->cap_sense.cap_bias[PHASE2],
			ctrl->cap_sense.proxavg[PHASE3] * 1000 / 57,
			ctrl->cap_sense.is_crack_detected[LASER_FLOOD]);
		dev_info(ctrl->soc_info.dev,
			"dot C_raw %d aF C_corrected: %d aF "
			"Cap_bias: %d aF temp: %d mC is crack: %d",
			ctrl->cap_sense.cap_raw[PHASE1],
			ctrl->cap_sense.cap_corrected[PHASE1],
			ctrl->cap_sense.cap_bias[PHASE1],
			ctrl->cap_sense.proxavg[PHASE3] * 1000 / 57,
			ctrl->cap_sense.is_crack_detected[LASER_DOT]);
	}
}

int sx9320_init_setting(struct led_laser_ctrl_t *ctrl)
{
	int rc, i;
	size_t settings_size = ARRAY_SIZE(cap_sense_init_reg_settings);
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings[settings_size];

	for (i = 0; i < settings_size; i++) {
		reg_settings[i].reg_addr = cap_sense_init_reg_settings[i].addr;
		reg_settings[i].reg_data = cap_sense_init_reg_settings[i].data;
		reg_settings[i].delay = 0;
		reg_settings[i].data_mask = 0;
	}
	write_setting.reg_setting = reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = settings_size;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
		&write_setting);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: i2c write failed: rc: %d",
			__func__, rc);
		return rc;
	}

	sx9320_cleanup_nirq(ctrl);

	return rc;
}

static int silego_check_fault_type(struct led_laser_ctrl_t *ctrl)
{
	int rc;

	rc = camera_io_dev_read(
		&ctrl->silego.io_master_info,
		FAULT_FLAG_ADDR,
		&ctrl->silego.fault_flag,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc < 0)
		dev_err(ctrl->soc_info.dev,
			"failed to read silego fault flag");

	return rc;
}

static int silego_override_setting(
	struct led_laser_ctrl_t *ctrl, uint32_t addr, uint32_t data)
{
	int rc;
	uint32_t check_value;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->silego.io_master_info, &write_setting);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed to overwrite setting: rc: %d",
			__func__, rc);
		return rc;
	}

	rc = camera_io_dev_read(
		&ctrl->silego.io_master_info,
		reg_settings.reg_addr,
		&check_value,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed to read back setting: rc: %d",
			__func__, rc);
		return rc;
	}

	if (check_value != data) {
		dev_err(ctrl->soc_info.dev,
			"%s: failed, expected 0x%x, got: 0x%x: rc: %d",
			__func__, data, check_value, rc);
		return -EINVAL;
	}
	return rc;
};

static int32_t silego_verify_settings(struct led_laser_ctrl_t *ctrl)
{
	uint32_t data;
	int rc;
	size_t i, settings_size;
	const struct reg_setting *reg_map;

	switch (ctrl->hw_version) {
	case BUILD_PROTO:
	case BUILD_DEV:
		reg_map = silego_reg_settings_ver1;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver1);
		break;
	case BUILD_EVT1_0:
		reg_map = silego_reg_settings_ver2;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver2);
		break;
	case BUILD_EVT1_1:
		reg_map = silego_reg_settings_ver3;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver3);
		break;
	case BUILD_DVT:
	case BUILD_PVT:
	default:
		reg_map = silego_reg_settings_ver4;
		settings_size = ARRAY_SIZE(silego_reg_settings_ver4);
		break;
	}

	for (i = 0; i < settings_size; i++) {
		rc = camera_io_dev_read(
			&ctrl->silego.io_master_info,
			reg_map[i].addr,
			&data,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev, "%s failed on read 0x%x",
				__func__, reg_map[i].addr);
			goto out;
		}

		if (ctrl->hw_version == BUILD_PROTO &&
			reg_map[i].addr == 0xcd && data == 0xb3) {
			/* Some of Silego part isn't store final version value.
			 *  Need to overwrite to correct value to provide
			 *  proper functionality.
			 */
			rc = silego_override_setting(ctrl,
				reg_map[i].addr, reg_map[i].data);
			if (rc < 0)
				goto out;
		} else if (ctrl->hw_version < BUILD_EVT1_1 &&
			(reg_map[i].addr == 0xc1 || reg_map[i].addr == 0xc3)) {
			/* Update pulse width limitation to 3 ms */
			rc = silego_override_setting(ctrl,
				reg_map[i].addr, reg_map[i].data);
			if (rc < 0)
				goto out;
		} else if (data != reg_map[i].data) {
			dev_err(ctrl->soc_info.dev,
				"address 0x%x mismatch,"
				" expected 0x%x but got 0x%x",
				reg_map[i].addr,
				reg_map[i].data,
				data);
			rc = -EINVAL;
			goto out;
		}
	}

out:
	return rc;
}

static int lm36011_read_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t *data)
{
	int rc = 0;
	rc = camera_io_dev_read(
		&ctrl->io_master_info,
		addr,
		data,
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		CAMERA_SENSOR_I2C_TYPE_BYTE);

	if (rc != 0)
		pr_err("%s failed rc = %d", __func__, rc);
	else
		pr_debug("%s: got data 0x%x from 0x%x rc %d",
			__func__,
			*data,
			addr,
			rc);
	return rc;
}

static int lm36011_write_data(
	struct led_laser_ctrl_t *ctrl,
	uint32_t addr,
	uint32_t data)
{
	int rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;
	reg_settings.reg_addr = addr;
	reg_settings.reg_data = data;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	rc = camera_io_dev_write(&ctrl->io_master_info, &write_setting);

	if (rc != 0)
		pr_err("%s failed rc = %d", __func__, rc);
	else
		pr_debug("%s: set data 0x%x to 0x%x rc %d",
			__func__,
			data,
			addr,
			rc);
	return rc;
}

static int lm36011_init_pinctrl(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	ctrl->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ctrl->pinctrl)) {
		dev_err(dev, "getting pinctrl handle failed");
		return -EINVAL;
	}

	ctrl->gpio_active_state =
		pinctrl_lookup_state(ctrl->pinctrl,
				"lm36011_active");
	if (IS_ERR_OR_NULL(ctrl->gpio_active_state)) {
		dev_err(dev,
			"failed to get the gpio active state pinctrl handle");
		goto pinctrl_err;
	}

	ctrl->gpio_suspend_state =
		pinctrl_lookup_state(ctrl->pinctrl,
				"lm36011_suspend");
	if (IS_ERR_OR_NULL(ctrl->gpio_suspend_state)) {
		dev_err(dev,
			"failed to get the gpio suspend state pinctrl handle");
		goto pinctrl_err;
	}

	return 0;

pinctrl_err:
	devm_pinctrl_put(ctrl->pinctrl);
	ctrl->pinctrl = NULL;
	return -EINVAL;
}

static int lm36011_power_up(struct led_laser_ctrl_t *ctrl)
{
	int rc;

	if (!ctrl->is_power_up) {
		rc = regulator_set_voltage(ctrl->buck1,
			PMIC_BUCK1_VOlTAGE_MIN, PMIC_BUCK1_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set pmic buck1 voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->buck1);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"buck1 regulator_enable failed: rc: %d", rc);
			return rc;
		}

		rc = regulator_set_voltage(ctrl->buck2,
			PMIC_BUCK2_VOlTAGE_MIN, PMIC_BUCK2_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set pmic buck2 voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->buck2);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"buck2 regulator_enable failed: rc: %d", rc);
			return rc;
		}

		rc = regulator_set_voltage(ctrl->vio,
			VIO_VOLTAGE_MIN, VIO_VOLTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set vio voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->vio);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"vio regulator_enable failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_power_up = true;
	}

	if (ctrl->hw_version < BUILD_DVT) {
		if (!ctrl->cap_sense.is_power_up) {
			rc = regulator_set_voltage(ctrl->cap_sense.vdd,
				SX9320_VDD_VOlTAGE_MIN,
				SX9320_VDD_VOlTAGE_MAX);
			if (rc < 0) {
				dev_err(ctrl->soc_info.dev,
					"set cap sense vdd voltage failed: %d",
					rc);
				return rc;
			}
			rc = regulator_enable(ctrl->cap_sense.vdd);
			if (rc < 0) {
				dev_err(ctrl->soc_info.dev,
					"cap sense regulator_enable"
					" failed: rc: %d", rc);
				return rc;
			}
			ctrl->cap_sense.is_power_up = true;
		}
	}

	if (!ctrl->silego.is_power_up) {
		/* For PVT device, set VDD to 3.2 V now */
		rc = regulator_set_voltage(ctrl->silego.vdd,
			(ctrl->hw_version == BUILD_PVT ?
			SLIEGO_VDD_VOlTAGE_MAX : SLIEGO_VDD_VOlTAGE_MIN),
			SLIEGO_VDD_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set silego vdd voltage failed: %d", rc);
			return rc;
		}
		rc = regulator_enable(ctrl->silego.vdd);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"silego regulator_enable failed: rc: %d", rc);
			return rc;
		}
		ctrl->silego.is_power_up = true;
	}

	if (!ctrl->is_cci_init) {
		rc = camera_io_init(&(ctrl->io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"cam io init failed: rc: %d", rc);
			return rc;
		}
		ctrl->is_cci_init = true;
	}

	if (!ctrl->silego.is_cci_init) {
		rc = camera_io_init(&(ctrl->silego.io_master_info));
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"silego io init failed: rc: %d", rc);
			return rc;
		}
		ctrl->silego.is_cci_init = true;
	}

	if (ctrl->hw_version < BUILD_DVT) {
		if (!ctrl->cap_sense.is_cci_init) {
			rc = camera_io_init(&(ctrl->cap_sense.io_master_info));
			if (rc < 0) {
				dev_err(ctrl->soc_info.dev,
					"cam io init for cap sense failed:"
					" rc: %d", rc);
				return rc;
			}
			ctrl->cap_sense.is_cci_init = true;
		}
	}

	/* Silego i2c need at least 3 ms after power up */
	usleep_range(3000, 6000);

	/* Suppress ITO-R by change register 0xCB */
	if (ctrl->hw_version == BUILD_DVT) {
		rc = silego_override_setting(ctrl, 0xcb, 0x97);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"failed to update Silego REG 0xCB");
			return rc;
		}

		/* Wait 1 ms for Silego register change taking effect */
		usleep_range(1000, 3000);

		rc = regulator_set_voltage(ctrl->silego.vdd,
			SLIEGO_VDD_VOlTAGE_MAX, SLIEGO_VDD_VOlTAGE_MAX);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"set silego vdd voltage failed: %d", rc);
			return rc;
		}
		dev_info(ctrl->soc_info.dev, "updated ldo6 to 3.2V");
	}

	rc = silego_verify_settings(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev,
			"verify silego reg setting failed: rc: %d",
			rc);
		ctrl->silego.is_validated = false;
		return rc;
	}
	ctrl->silego.is_validated = true;

	/* set safety ic owner to this driver, if safety ic has no onwer
	 * also init pinctrl for safety ic
	 */
	mutex_lock(&lm36011_mutex);
	if (safety_ic_owner == LASER_TYPE_MAX) {
		safety_ic_owner = ctrl->type;
		rc = lm36011_init_pinctrl(ctrl->soc_info.dev);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"failed to init pin ctrl");
			safety_ic_owner = LASER_TYPE_MAX;
			mutex_unlock(&lm36011_mutex);
			return rc;
		}
		rc = pinctrl_select_state(ctrl->pinctrl,
			ctrl->gpio_active_state);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"failed to set pin ctrl to active");
			devm_pinctrl_put(ctrl->pinctrl);
			ctrl->pinctrl = NULL;
			safety_ic_owner = LASER_TYPE_MAX;
		}
	}
	mutex_unlock(&lm36011_mutex);

	return rc;
}

static int lm36011_power_down(struct led_laser_ctrl_t *ctrl)
{
	int rc = 0, is_error;

	lm36011_write_data(ctrl, ENABLE_REG, IR_DISABLE_MODE);

	if (ctrl->is_cci_init) {
		is_error = camera_io_release(&(ctrl->io_master_info));
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser cci_release failed: rc: %d", rc);
		} else
			ctrl->is_cci_init = false;
	}

	if (ctrl->silego.is_cci_init) {
		is_error = camera_io_release(&(ctrl->silego.io_master_info));
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"silego cci_release failed: rc: %d", rc);
		} else
			ctrl->silego.is_cci_init = false;
	}

	if (ctrl->hw_version < BUILD_DVT) {
		if (ctrl->cap_sense.is_cci_init) {
			is_error = camera_io_release(
				&(ctrl->cap_sense.io_master_info));
			if (is_error < 0) {
				rc = is_error;
				dev_err(ctrl->soc_info.dev,
					"cap sense cci_release failed: rc: %d",
					rc);
			} else
				ctrl->cap_sense.is_cci_init = false;
		}
	}

	if (ctrl->silego.is_power_up) {
		is_error = regulator_set_voltage(ctrl->silego.vdd, 0,
			PMIC_BUCK2_VOlTAGE_MAX);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"silego set voltage failed: rc: %d", rc);
		}

		is_error = regulator_disable(ctrl->silego.vdd);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"silego regulator_disable failed: rc: %d", rc);
		} else
			ctrl->silego.is_power_up = false;
	}

	if (ctrl->hw_version < BUILD_DVT) {
		if (ctrl->cap_sense.is_power_up) {
			is_error = regulator_disable(ctrl->cap_sense.vdd);
			if (is_error < 0) {
				rc = is_error;
				dev_err(ctrl->soc_info.dev,
					"cap sense regulator_disable failed:"
					" rc: %d", rc);
			} else
				ctrl->cap_sense.is_power_up = false;
		}
	}

	if (ctrl->is_power_up) {
		is_error = regulator_disable(ctrl->vio) +
			regulator_set_voltage(ctrl->buck2,
				0, PMIC_BUCK2_VOlTAGE_MAX) +
			regulator_disable(ctrl->buck2) +
			regulator_disable(ctrl->buck1);
		if (is_error < 0) {
			rc = is_error;
			dev_err(ctrl->soc_info.dev,
				"laser regulator_disable failed: rc: %d", rc);
		} else
			ctrl->is_power_up = false;
	}

	/* set pinctrl to suspend if safety onwer going to disable */
	mutex_lock(&lm36011_mutex);
	if (safety_ic_owner == ctrl->type &&
		!IS_ERR_OR_NULL(ctrl->pinctrl)) {
		rc = pinctrl_select_state(ctrl->pinctrl,
			ctrl->gpio_suspend_state);
		devm_pinctrl_put(ctrl->pinctrl);
		ctrl->pinctrl = NULL;
		safety_ic_owner = LASER_TYPE_MAX;
		if (rc < 0)
			dev_err(ctrl->soc_info.dev,
				"failed to set pin ctrl to suspend");
	}
	mutex_unlock(&lm36011_mutex);

	return rc;
}

static irqreturn_t silego_ir_vcsel_fault_handler(int irq, void *dev_id)
{
	struct device *dev = (struct device *)dev_id;
	struct led_laser_ctrl_t *ctrl;

	if (!dev)
		return IRQ_NONE;

	dev_err(dev, "Silego under fault condition, irq: %d", irq);
	ctrl = dev_get_drvdata(dev);
	ctrl->silego.is_vcsel_fault = true;
	if (!ctrl->cap_sense.is_crack_detected[ctrl->type])
		cam_req_mgr_update_safety_ic_status(LASER_OPERATION_FAULT);

	return IRQ_HANDLED;
}

static irqreturn_t silego_ir_vcsel_fault_count_handler(int irq, void *dev_id)
{
	struct device *dev = (struct device *)dev_id;
	struct led_laser_ctrl_t *ctrl;

	if (!dev)
		return IRQ_NONE;

	ctrl = dev_get_drvdata(dev);
	ctrl->silego.vcsel_fault_count++;
	dev_dbg(dev, "Silego fault count: %d, irq: %d",
		ctrl->silego.vcsel_fault_count, irq);

	return IRQ_HANDLED;
}

static irqreturn_t cap_sense_irq_handler(int irq, void *dev_id)
{
	struct device *dev = (struct device *)dev_id;
	struct led_laser_ctrl_t *ctrl;

	if (!dev)
		return IRQ_NONE;

	ctrl = dev_get_drvdata(dev);

	queue_work(ctrl->work_queue, &ctrl->work);

	return IRQ_HANDLED;
}

static void cap_sense_workq_job(struct work_struct *work)
{
	struct led_laser_ctrl_t *ctrl;
	uint32_t data;
	int rc;
	uint32_t i;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	ctrl = container_of(work, struct led_laser_ctrl_t, work);
	if (!ctrl) {
		dev_err(ctrl->soc_info.dev, "failed to get driver struct");
		return;
	}

	rc = sx9320_cleanup_nirq(ctrl);
	if (rc < 0)
		return;

	reg_settings.reg_addr = PHASE_SELECT_REG;
	reg_settings.reg_data = 0x00;
	reg_settings.delay = 0;
	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	for (i = 1; i < PHASENUM; i++) {
		reg_settings.reg_data = i;

		rc = camera_io_dev_write(&ctrl->cap_sense.io_master_info,
			&write_setting);

		if (rc < 0)
			dev_err(ctrl->soc_info.dev,
				"failed to select PH %d", i);

		rc = camera_io_dev_read(
			&ctrl->cap_sense.io_master_info,
			PROXAVG_REG,
			&data,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_WORD);

		if (rc < 0)
			dev_err(ctrl->soc_info.dev,
				"failed to read prxavg from PH %d", i);

		ctrl->cap_sense.proxavg[i] = data & 0xFFFF;

		rc = camera_io_dev_read(
			&ctrl->cap_sense.io_master_info,
			PROXOFFSET_REG,
			&data,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			CAMERA_SENSOR_I2C_TYPE_WORD);

		if (rc < 0)
			dev_err(ctrl->soc_info.dev,
				"failed to read prxoffset from PH %d", i);

		ctrl->cap_sense.proxoffset[i] = data & 0x3FFF;

		if (read_proxoffset) {
			dev_info(ctrl->soc_info.dev, "proxoffset PH%d: %d",
					i, ctrl->cap_sense.proxoffset[i]);
			dev_info(ctrl->soc_info.dev, "proxavg PH%d: %d",
					i, ctrl->cap_sense.proxavg[i]);
		}
	}

	if (ctrl->cap_sense.sample_count < NUM_SKIP_SAMPLE) {
		ctrl->cap_sense.sample_count++;
		return;
	}
	sx9320_crack_detection(ctrl);
}

static int lm36011_set_up_silego_irq(
	struct device *dev,
	unsigned int irq,
	int gpio_index)
{
	int rc;
	const char *irq_name;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	switch (gpio_index) {
	case IR_VCSEL_FAULT_ITOC:
		dev_dbg(dev, "setup irq IR_VCSEL_FAULT");
		irq_name = (ctrl->type == LASER_FLOOD) ?
			"ir_vcsel_fault_flood" : "ir_vcsel_fault_dot";
		rc = request_irq(irq, silego_ir_vcsel_fault_handler,
			IRQF_TRIGGER_FALLING, irq_name, dev);
		break;
	case IR_VCSEL_TEST_ITOC:
		dev_dbg(dev, "setup irq IR_VCSEL_TEST");
		irq_name = (ctrl->type == LASER_FLOOD) ?
			"ir_vcsel_fault_count_flood" :
			"ir_vcsel_fault_count_dot";
		rc = request_irq(irq,
			silego_ir_vcsel_fault_count_handler,
			IRQF_TRIGGER_RISING,
			irq_name, dev);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int lm36011_set_up_cap_sense_irq(
	struct device *dev,
	unsigned int irq,
	int gpio_index)
{
	int rc;
	const char *irq_name;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	switch (gpio_index) {
	case CSENSE_PROXAVG_READ:
		dev_dbg(dev, "setup irq CSENSE_PROXAVG_READ as IRQF_TRIGGER_FALLING");
		irq_name = (ctrl->type == LASER_FLOOD) ?
			"cap_sense_irq_flood" : "cap_sense_irq_dot";
		rc = request_irq(irq, cap_sense_irq_handler,
			IRQF_TRIGGER_FALLING, irq_name, dev);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static void lm36011_enable_gpio_irq(struct device *dev)
{
	int index;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int silego_num = ctrl->hw_version < BUILD_DVT ?
		SILEGO_GPIO_MAX_ITOC : SILEGO_GPIO_MAX_ITOR;

	if (ctrl->gpio_count <= 0)
		return;

	for (index = 0; index < silego_num; index++) {
		if (ctrl->silego.irq[index] <= 0) {
			dev_warn(dev, "no available irq for gpio: %d",
				ctrl->silego.gpio_array[index].gpio);
		} else if (index == SILEGO_SW_HALT_ITOC) {
			gpio_request(ctrl->silego.gpio_array[index].gpio,
				"SILEGO_SW_HALT");
		} else
			lm36011_set_up_silego_irq(
				dev, ctrl->silego.irq[index], index);
	}

	if (ctrl->hw_version < BUILD_DVT) {
		for (index = 0; index < CAP_SENSE_GPIO_MAX; index++) {
			if (ctrl->cap_sense.irq[index] <= 0) {
				dev_warn(dev, "no available irq for gpio: %d",
					ctrl->cap_sense.irq[index]);
			} else
				lm36011_set_up_cap_sense_irq(
					dev, ctrl->cap_sense.irq[index], index);
		}
	}
}

static void lm36011_disable_gpio_irq(struct device *dev)
{
	int index, gpio_level;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (ctrl->gpio_count <= 0)
		return;

	if (ctrl->hw_version < BUILD_DVT) {
		for (index = 0; index < SILEGO_GPIO_MAX_ITOC; index++) {
			if (ctrl->silego.irq[index] <= 0) {
				dev_warn(dev,
					"no available irq for gpio: %d",
					ctrl->silego.gpio_array[index].gpio);
			} else if (index == SILEGO_SW_HALT_ITOC) {
				gpio_level =
					gpio_get_value(
					ctrl->silego.gpio_array[index].gpio);
				if (gpio_level)
					gpio_set_value(
					ctrl->silego.gpio_array[index].gpio,
					0);
				gpio_free(ctrl->silego.gpio_array[index].gpio);
			} else
				free_irq(ctrl->silego.irq[index], dev);
		}

		for (index = 0; index < CAP_SENSE_GPIO_MAX; index++) {
			if (ctrl->cap_sense.irq[index] <= 0) {
				dev_warn(dev,
					"no available irq for gpio: %d",
					ctrl->cap_sense.irq[index]);
			} else
				free_irq(ctrl->cap_sense.irq[index], dev);
		}
	} else {
		for (index = 0; index < SILEGO_GPIO_MAX_ITOR; index++) {
			if (ctrl->silego.irq[index] <= 0) {
				dev_warn(dev,
					"no available irq for gpio: %d",
					ctrl->silego.gpio_array[index].gpio);
			} else
				free_irq(ctrl->silego.irq[index], dev);
		}
	}
}

static int lm36011_get_gpio_info(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int16_t gpio_array_size = 0, index;
	int16_t max_available_gpio = ctrl->hw_version < BUILD_DVT ?
		SILEGO_GPIO_MAX_ITOC + CAP_SENSE_GPIO_MAX :
		SILEGO_GPIO_MAX_ITOR;

	gpio_array_size = of_gpio_count(dev->of_node);
	ctrl->gpio_count = gpio_array_size;

	if (gpio_array_size <= 0)
		return 0;

	if (gpio_array_size > max_available_gpio) {
		dev_err(dev, "too many gpio defined, max num: %d",
			max_available_gpio);
		return -EINVAL;
	}

	if (ctrl->hw_version < BUILD_DVT) {
		ctrl->silego.gpio_array = devm_kcalloc(dev,
			SILEGO_GPIO_MAX_ITOC, sizeof(struct gpio), GFP_KERNEL);
		if (!ctrl->silego.gpio_array) {
			dev_err(dev, "no memory for silego gpio");
			return -ENOMEM;
		}

		ctrl->cap_sense.gpio_array = devm_kcalloc(dev,
			CAP_SENSE_GPIO_MAX, sizeof(struct gpio), GFP_KERNEL);
		if (!ctrl->cap_sense.gpio_array) {
			dev_err(dev, "no memory for cap sense gpio");
			return -ENOMEM;
		}
		for (index = 0; index < gpio_array_size; index++) {
			if (index < SILEGO_GPIO_MAX_ITOC) {
				ctrl->silego.gpio_array[index].gpio =
					of_get_gpio(dev->of_node, index);
				ctrl->silego.irq[index] =
					gpio_to_irq(
					ctrl->silego.gpio_array[index].gpio);
			} else {
				ctrl->cap_sense.gpio_array[0].gpio =
					of_get_gpio(dev->of_node, index);
				ctrl->cap_sense.irq[0] =
					gpio_to_irq(
					ctrl->cap_sense.gpio_array[0].gpio);
			}
		}
	} else {
		ctrl->silego.gpio_array = devm_kcalloc(dev,
			SILEGO_GPIO_MAX_ITOR, sizeof(struct gpio), GFP_KERNEL);
		if (!ctrl->silego.gpio_array) {
			dev_err(dev, "no memory for silego gpio");
			return -ENOMEM;
		}

		for (index = 0; index < gpio_array_size; index++) {
			if (index < SILEGO_GPIO_MAX_ITOR) {
				ctrl->silego.gpio_array[index].gpio =
					of_get_gpio(dev->of_node, index);
				ctrl->silego.irq[index] =
					gpio_to_irq(
					ctrl->silego.gpio_array[index].gpio);
			}
		}
	}

	return 0;
}

static int lm36011_parse_dt(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int value = 0;

	ctrl->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(ctrl->vio)) {
		ctrl->vio = NULL;
		dev_err(dev, "unable to get vio");
		return -ENOENT;
	}

	ctrl->silego.vdd = devm_regulator_get(dev, "silego_vdd");
	if (IS_ERR(ctrl->silego.vdd)) {
		ctrl->silego.vdd = NULL;
		dev_err(dev, "unable to get silego vdd");
		return -ENOENT;
	}

	ctrl->buck1 = devm_regulator_get(dev, "pmic_buck1");
	if (IS_ERR(ctrl->buck1)) {
		ctrl->buck1 = NULL;
		dev_err(dev, "unable to get pmic buck1");
		return -ENOENT;
	}

	ctrl->buck2 = devm_regulator_get(dev, "pmic_buck2");
	if (IS_ERR(ctrl->buck2)) {
		ctrl->buck2 = NULL;
		dev_err(dev, "unable to get pmic buck2");
		return -ENOENT;
	}

	if (of_property_read_u32(dev->of_node, "laser-type", &value)) {
		dev_err(dev, "laser-type not specified in dt");
		return -ENOENT;
	}
	ctrl->type = value;

	if (of_property_read_u32(dev->of_node, "hw-version", &value)) {
		dev_warn(dev, "hw version not specified in dt");
		ctrl->hw_version = 0;
	} else {
		ctrl->hw_version = value;
		dev_info(dev, "hw version: %d", ctrl->hw_version);
	}

	if (lm36011_get_gpio_info(dev)) {
		dev_err(dev, "failed to parse gpio and irq info");
		return -ENOENT;
	}

	if (ctrl->hw_version < BUILD_DVT) {
		ctrl->cap_sense.vdd = devm_regulator_get(dev, "sx9320_vdd");
		if (IS_ERR(ctrl->cap_sense.vdd)) {
			ctrl->cap_sense.vdd = NULL;
			dev_err(dev, "unable to get cap sense vdd");
			return -ENOENT;
		}

		if (of_property_read_u32(dev->of_node, "sx9320_sid", &value)) {
			dev_err(dev,
				"cap sense slave address not specified in dt");
			return -ENOENT;
		}
		ctrl->cap_sense.sid = value;
	}

	return 0;
}

static int32_t lm36011_update_i2c_info(struct device *dev)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int32_t value = 0;

	if (of_property_read_u32(dev->of_node, "cci-master", &value)) {
		dev_err(dev, "cci master not specified in dt");
		return -EINVAL;
	}
	ctrl->io_master_info.cci_client->cci_i2c_master = value;

	if (of_property_read_u32(dev->of_node, "reg", &value)) {
		dev_err(dev, "slave address is not specified in dt");
		return -EINVAL;

	}
	ctrl->io_master_info.cci_client->sid = value;

	if (of_property_read_u32(dev->of_node, "cci-device", &value) ||
		value >= CCI_DEVICE_MAX) {
		dev_err(dev, "cci device is not specified in dt");
		return -EINVAL;

	}
	ctrl->io_master_info.cci_client->cci_device = value;
	ctrl->io_master_info.cci_client->retries = 3;
	ctrl->io_master_info.cci_client->id_map = 0;
	ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_MODE;

	/* Fill up Silego io info */
	ctrl->silego.io_master_info.cci_client->cci_device = value;
	ctrl->silego.io_master_info.cci_client->retries = 3;
	ctrl->silego.io_master_info.cci_client->id_map = 0;
	ctrl->silego.io_master_info.cci_client->i2c_freq_mode = I2C_FAST_MODE;
	ctrl->silego.io_master_info.cci_client->sid = 0x08;
	ctrl->silego.io_master_info.cci_client->cci_i2c_master = 0;
	ctrl->silego.io_master_info.master_type = CCI_MASTER;

	if (ctrl->hw_version < BUILD_DVT) {
		/* Fill up cap sense io info */
		ctrl->cap_sense.io_master_info.cci_client->cci_device = value;
		ctrl->cap_sense.io_master_info.cci_client->retries = 3;
		ctrl->cap_sense.io_master_info.cci_client->id_map = 0;
		ctrl->cap_sense.io_master_info.cci_client->i2c_freq_mode =
			I2C_FAST_MODE;
		ctrl->cap_sense.io_master_info.cci_client->sid =
			ctrl->cap_sense.sid;
		ctrl->cap_sense.io_master_info.cci_client->cci_i2c_master = 0;
		ctrl->cap_sense.io_master_info.master_type = CCI_MASTER;
	}

	return 0;
}

static ssize_t led_laser_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	bool is_enabled;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	is_enabled = (ctrl->is_power_up == true && ctrl->is_cci_init == true);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", is_enabled);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_enable_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;
	bool value;

	if (!ctrl->silego.is_validated) {
		dev_err(dev, "Silego is invalid");
		return -EINVAL;
	}

	if (!ctrl->silego.self_test_result) {
		dev_err(dev,
			"Silego self test failed, please re-try by reboot");
		return -EINVAL;
	}

	if (ctrl->hw_version < BUILD_DVT && !ctrl->cap_sense.is_validated) {
		dev_err(dev, "Cap sense is invalid");
		return -EINVAL;
	}

	if (!ctrl->is_certified && !ctrl->is_power_up) {
		dev_err(dev, "Cannot enable laser due to uncertified");
		return -EINVAL;
	}

	rc = kstrtobool(buf, &value);
	if (rc != 0)
		return rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (value == true) {
		rc = lm36011_power_up(ctrl);
		if (rc != 0) {
			lm36011_power_down(ctrl);
			mutex_unlock(&ctrl->cam_sensor_mutex);
			return rc;
		}
		/* Prepare safety ic IRQ */
		cam_req_mgr_update_safety_ic_status(NO_ERROR);
		mutex_lock(&lm36011_mutex);
		if (ctrl->type == safety_ic_owner) {
			if (ctrl->hw_version >= BUILD_EVT1_0 &&
				ctrl->hw_version < BUILD_DVT) {
				ctrl->cap_sense.sample_count = 0;
				rc = sx9320_init_setting(ctrl);
				if (rc < 0) {
					dev_err(ctrl->soc_info.dev,
						"initialize cap sense failed");
					mutex_unlock(&lm36011_mutex);
					lm36011_power_down(ctrl);
					mutex_unlock(&ctrl->cam_sensor_mutex);
					return rc;
				}
				/* avoid false crack detected when device first
				 * time booting up
				 */
				if (ctrl->cap_sense.proxoffset[PHASE1] != 0 &&
					ctrl->cap_sense.proxavg[PHASE1] != 0)
					sx9320_crack_detection(ctrl);
			}
			lm36011_enable_gpio_irq(dev);
			dev_info(ctrl->soc_info.dev,
				"enable safety ic funciton, safety_ic_owner %d",
				safety_ic_owner);
		}
		mutex_unlock(&lm36011_mutex);

		/* Check ITO-R state, report to camera driver if lens crack
		 * has been detected.
		 */
		if (ctrl->hw_version >= BUILD_DVT) {
			if (silego_check_fault_type(ctrl) < 0)
				dev_err(ctrl->soc_info.dev,
					"failed to get Silego state");
			else if (ctrl->silego.fault_flag == ITOR_OPEN_CIRCUIT) {
				dev_info(ctrl->soc_info.dev,
					"ITO-R detected crack");
				cam_req_mgr_update_safety_ic_status(LENS_CRACK);
			}
			dev_info(ctrl->soc_info.dev, "Silego state: 0x%x",
				ctrl->silego.fault_flag);
		}
		/* Clean up IRQ for PROTO and DEV device */
		else if (ctrl->hw_version < BUILD_EVT1_0)
			sx9320_cleanup_nirq(ctrl);

		rc = lm36011_write_data(ctrl,
			ENABLE_REG, IR_ENABLE_MODE);
		if (rc == 0)
			dev_info(dev, "Laser enabled");
	} else {
		mutex_lock(&lm36011_mutex);
		if (ctrl->type == safety_ic_owner) {
			lm36011_disable_gpio_irq(dev);
			cam_req_mgr_update_safety_ic_status(NO_ERROR);
			dev_info(ctrl->soc_info.dev,
				"disable safety ic function, safety_ic_owner: %d",
				safety_ic_owner);
		}
		mutex_unlock(&lm36011_mutex);
		rc = lm36011_power_down(ctrl);
		if (rc == 0)
			dev_info(dev, "Laser disabled");
	}
	ctrl->silego.vcsel_fault_count = 0;
	ctrl->silego.is_vcsel_fault = false;
	ctrl->silego.is_csense_halt = false;
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return rc < 0 ? rc : count;
}

static ssize_t led_laser_read_byte_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%x\n", ctrl->read_data);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_read_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t addr = 0;
	uint32_t read_data = 0;
	int rc = 0;

	if (!ctrl->is_certified) {
		dev_err(dev, "Cannot enable laser due to uncertified");
		return -EINVAL;
	}

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &addr);
	if (rc)
		goto error_out;

	addr &= 0xFF;

	rc = lm36011_read_data(ctrl, addr, &read_data);
	if (rc < 0) {
		dev_err(dev, "i2c read failed, rc = %d", rc);
		goto error_out;
	} else {
		ctrl->read_addr = addr;
		ctrl->read_data = read_data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t led_laser_write_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	mutex_lock(&ctrl->cam_sensor_mutex);
	if (!ctrl->is_cci_init || !ctrl->is_power_up) {
		rc = -EINVAL;
		goto error_out;
	}

	rc = kstrtouint(buf, 0, &value);

	if (rc)
		goto error_out;

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = lm36011_write_data(ctrl, addr, data);
	if (rc < 0) {
		dev_err(dev, "%s i2c write failed: %d.", __func__, rc);
		goto error_out;
	} else {
		if (addr == ctrl->read_addr)
			ctrl->read_data = data;
	}
	mutex_unlock(&ctrl->cam_sensor_mutex);

	return count;

error_out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t is_silego_validated_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.is_validated);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t silego_vcsel_fault_detected_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.is_vcsel_fault);
	return rc;
}

static ssize_t silego_vcsel_fault_count_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.vcsel_fault_count);
	return rc;
}

static ssize_t silego_is_cap_sense_halt_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.is_csense_halt);
	return rc;
}

static ssize_t is_certified_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->is_certified);
}

static ssize_t is_cap_sense_validated_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->cap_sense.is_validated);
}

static ssize_t cap_sense_proxvalue_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	if (!ctrl->cap_sense.is_power_up || !ctrl->is_power_up) {
		dev_warn(dev, "try to enable laser first");
		return -EINVAL;
	}

	rc = scnprintf(buf, PAGE_SIZE,
		"proxoffset PH1: %d, PH2: %d, PH3: %d\nproxavg PH1: %d, PH2: %d, PH3: %d\n",
			ctrl->cap_sense.proxoffset[PHASE1],
			ctrl->cap_sense.proxoffset[PHASE2],
			ctrl->cap_sense.proxoffset[PHASE3],
			ctrl->cap_sense.proxavg[PHASE1],
			ctrl->cap_sense.proxavg[PHASE2],
			ctrl->cap_sense.proxavg[PHASE3]);

	return rc;
}

static ssize_t cap_sense_write_byte_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	uint32_t addr;
	uint32_t data;
	int rc;

	if (!ctrl->cap_sense.is_power_up || !ctrl->cap_sense.is_cci_init) {
		dev_warn(dev, "try to enable laser first");
		return -EINVAL;
	}

	rc = kstrtouint(buf, 0, &value);

	if (rc < 0)
		return rc;
	if ((value & 0xFFFF0000) != 0) {
		dev_err(dev, "value %x out of boundary", value);
		return -EINVAL;
	}

	addr = (value >> 8) & 0xFF;
	data = value & 0xFF;

	rc = sx9320_write_data(ctrl, addr, data);
	if (rc < 0) {
		dev_err(dev, "%s i2c write failed: %d.", __func__, rc);
		return rc;
	}

	return count;
}

static ssize_t itoc_cali_data_store_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);
	uint32_t value = 0;
	int32_t temp_value;
	int rc;
	char op;

	op = buf[0];
	switch (op) {
	case 'a':
		rc = kstrtouint((buf+1), 0, &value);
		if (rc < 0)
			return rc;
		ctrl->cap_sense.calibration_data[PHASE1] = (value >> 16);
		ctrl->cap_sense.calibration_data[PHASE2] =
			(value & 0x0000FFFF);
		dev_info(dev,
			"updated ITO-C calibration data, dot: %d flood: %d",
			ctrl->cap_sense.calibration_data[PHASE1],
			ctrl->cap_sense.calibration_data[PHASE2]);
		break;
	case 'b':
		rc = kstrtoint((buf+1), 0, &temp_value);
		if (rc < 0)
			return rc;
		ctrl->cap_sense.cap_bias[PHASE1] = temp_value;
		dev_info(dev, "updated dot bias: %d",
			ctrl->cap_sense.cap_bias[PHASE1]);
		break;
	case 'c':
		rc = kstrtoint((buf+1), 0, &temp_value);
		if (rc < 0)
			return rc;
		ctrl->cap_sense.cap_slope[PHASE1] = temp_value;
		dev_info(dev, "updated dot slope: %d",
			ctrl->cap_sense.cap_slope[PHASE1]);
		if (ctrl->cap_sense.cap_slope[PHASE1] > 0) {
			ctrl->cap_sense.max_supported_temp[PHASE1] =
				((0xFFFFFFFF) /
				ctrl->cap_sense.cap_slope[PHASE1]);
			dev_info(dev,
				"max supported temperature for dot: %d mC",
				ctrl->cap_sense.max_supported_temp[PHASE1]);
		}
		break;
	case 'd':
		rc = kstrtoint((buf+1), 0, &temp_value);
		if (rc < 0)
			return rc;
		ctrl->cap_sense.cap_bias[PHASE2] = temp_value;
		dev_info(dev, "updated flood bias: %d",
			ctrl->cap_sense.cap_bias[PHASE2]);
		break;
	case 'e':
		rc = kstrtoint((buf+1), 0, &temp_value);
		if (rc < 0)
			return rc;
		ctrl->cap_sense.cap_slope[PHASE2] = temp_value;
		dev_info(dev, "updated flood slope: %d",
			ctrl->cap_sense.cap_slope[PHASE2]);
		if (ctrl->cap_sense.cap_slope[PHASE2] > 0) {
			ctrl->cap_sense.max_supported_temp[PHASE2] =
				((0xFFFFFFFF) /
				ctrl->cap_sense.cap_slope[PHASE2]);
			dev_info(dev,
				"max supported temperature for flood: %d mC",
				ctrl->cap_sense.max_supported_temp[PHASE2]);
		}
		break;
	default:
		dev_err(dev, "unsupported operation");
		break;
	}

	return count;
}

static void silego_self_test(
	struct led_laser_ctrl_t *ctrl,
	struct silego_self_test_result *test_result)
{
	int rc, retry;
	uint32_t data;

	mutex_lock(&ctrl->cam_sensor_mutex);
	mutex_lock(&lm36011_mutex);
	test_result->is_cracked = false;
	test_result->result = SILEGO_TEST_FAILED;

	if (regulator_is_enabled(ctrl->silego.vdd)) {
		/* Bypass test when silego power is on */
		mutex_unlock(&lm36011_mutex);
		test_result->result = SILEGO_TEST_BYPASS;
		goto out;
	}
	mutex_unlock(&lm36011_mutex);

	rc = lm36011_power_up(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev, "power up failed: %d", rc);
		goto power_down;
	}

	/* Prepare safety ic IRQ */
	mutex_lock(&lm36011_mutex);
	if (ctrl->type == safety_ic_owner) {
		lm36011_enable_gpio_irq(ctrl->soc_info.dev);
		dev_info(ctrl->soc_info.dev,
			"enable safety ic funciton, safety_ic_owner %d",
			safety_ic_owner);
	}
	mutex_unlock(&lm36011_mutex);

	rc = lm36011_write_data(ctrl,
		ENABLE_REG, IR_STANDBY_MODE);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev, "i2c write failed: %d", rc);
		goto release_resource;
	}

	/* Set torch current to 300 mA for flood, 50 mA for Dot */
	rc = lm36011_write_data(ctrl,
		LED_TORCH_BRIGHTNESS_REG,
		ctrl->type == LASER_FLOOD ? 0x65 : 0x10);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev, "i2c write failed: %d", rc);
		goto release_resource;
	}

	rc = lm36011_write_data(ctrl,
		ENABLE_REG, IR_TORCH_MODE);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev, "i2c write failed: %d", rc);
		goto release_resource;
	}

	if (lm36011_read_data(ctrl, ENABLE_REG, &data) < 0)
		dev_warn(ctrl->soc_info.dev, "fail to read back reg 0x%x",
			ENABLE_REG);
	else
		dev_info(ctrl->soc_info.dev,
			"laser driver mode has been set to 0x%x", data);

	/* check ITO-R status */
	if (silego_check_fault_type(ctrl) < 0)
		dev_warn(ctrl->soc_info.dev, "failed to read silego status");
	else {
		if (ctrl->silego.fault_flag == ITOR_OPEN_CIRCUIT)
			test_result->is_cracked = true;
		dev_info(ctrl->soc_info.dev,
			"Silego status: 0x%x", ctrl->silego.fault_flag);
	}


	/* wait for torch reach to 5 ms pulse width */
	usleep_range(5000, 10000);

	for (retry = 0; retry < MAX_RETRY_COUNT; retry++) {
		if (ctrl->silego.is_vcsel_fault) {
			test_result->result = SILEGO_TEST_PASS;
			break;
		}
		/* wait 3~5 ms and retry */
		usleep_range(3000, 5000);
	}

	if (retry == MAX_RETRY_COUNT)
		dev_err(ctrl->soc_info.dev,
			"silego self test failed due to no IRQ received");

release_resource:
	mutex_lock(&lm36011_mutex);
	if (ctrl->type == safety_ic_owner) {
		lm36011_disable_gpio_irq(ctrl->soc_info.dev);
		dev_info(ctrl->soc_info.dev,
			"disable safety ic function, safety_ic_owner: %d",
			safety_ic_owner);
	}
	mutex_unlock(&lm36011_mutex);

power_down:
	rc = lm36011_power_down(ctrl);
	if (rc < 0)
		dev_err(ctrl->soc_info.dev, "power up failed: %d", rc);

out:
	mutex_unlock(&ctrl->cam_sensor_mutex);
}

static ssize_t get_silego_state_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct led_laser_ctrl_t *ctrl = dev_get_drvdata(dev);

	rc = silego_check_fault_type(ctrl);
	if (rc < 0) {
		dev_err(ctrl->soc_info.dev, "failed to read Silego fault flag");
		return rc;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->silego.fault_flag);
}

static DEVICE_ATTR_RW(led_laser_enable);
static DEVICE_ATTR_RW(led_laser_read_byte);
static DEVICE_ATTR_WO(led_laser_write_byte);
static DEVICE_ATTR_RO(is_silego_validated);
static DEVICE_ATTR_RO(silego_vcsel_fault_detected);
static DEVICE_ATTR_RO(silego_vcsel_fault_count);
static DEVICE_ATTR_RO(silego_is_cap_sense_halt);
static DEVICE_ATTR_RO(is_certified);
static DEVICE_ATTR_RO(is_cap_sense_validated);
static DEVICE_ATTR_RO(cap_sense_proxvalue);
static DEVICE_ATTR_WO(cap_sense_write_byte);
static DEVICE_ATTR_WO(itoc_cali_data_store);
static DEVICE_ATTR_RO(get_silego_state);

static struct attribute *led_laser_dev_attrs[] = {
	&dev_attr_led_laser_enable.attr,
	&dev_attr_led_laser_read_byte.attr,
	&dev_attr_led_laser_write_byte.attr,
	&dev_attr_is_silego_validated.attr,
	&dev_attr_silego_vcsel_fault_detected.attr,
	&dev_attr_silego_vcsel_fault_count.attr,
	&dev_attr_silego_is_cap_sense_halt.attr,
	&dev_attr_is_certified.attr,
	&dev_attr_is_cap_sense_validated.attr,
	&dev_attr_cap_sense_proxvalue.attr,
	&dev_attr_cap_sense_write_byte.attr,
	&dev_attr_itoc_cali_data_store.attr,
	&dev_attr_get_silego_state.attr,
	NULL
};

ATTRIBUTE_GROUPS(led_laser_dev);

static int32_t lm36011_platform_remove(struct platform_device *pdev)
{
	struct led_laser_ctrl_t *ctrl;

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		dev_err(&pdev->dev, "led laser device is NULL");
		return 0;
	}

	if (!ctrl->is_probed)
		return 0;

	flush_workqueue(ctrl->work_queue);
	destroy_workqueue(ctrl->work_queue);
	class_destroy(ctrl->cl);
	cdev_del(&ctrl->c_dev);
	unregister_chrdev_region(ctrl->dev, 1);
	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
	mutex_destroy(&ctrl->cam_sensor_mutex);
	lm36011_power_down(ctrl);
	return 0;
}

static int lm36011_open(struct inode *inode, struct file *file)
{
	struct led_laser_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct led_laser_ctrl_t, c_dev);
	get_device(ctrl->soc_info.dev);
	file->private_data = ctrl;
	return 0;
}

static int lm36011_release(struct inode *inode, struct file *file)
{
	struct led_laser_ctrl_t *ctrl = container_of(inode->i_cdev,
		struct led_laser_ctrl_t, c_dev);
	put_device(ctrl->soc_info.dev);
	return 0;
}

static long lm36011_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int rc = 0;
	struct led_laser_ctrl_t *ctrl = file->private_data;
	struct silego_self_test_result test_result;

	switch (cmd) {
	case LM36011_SET_CERTIFICATION_STATUS:
		ctrl->is_certified = (arg == 1 ? true : false);
		break;
	case LM36011_SILEGO_SELF_TEST:
		silego_self_test(ctrl, &test_result);
		dev_info(ctrl->soc_info.dev,
			"silego self test result: %d",
			test_result.result);
		if (test_result.result != SILEGO_TEST_BYPASS)
			ctrl->silego.self_test_result =
				(test_result.result == SILEGO_TEST_PASS);
		rc = copy_to_user((void __user *)arg,
			&test_result, sizeof(struct silego_self_test_result));
		break;
	default:
		dev_err(ctrl->soc_info.dev,
			"%s: Unsupported ioctl command %u", __func__, cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static const struct file_operations lm36011_fops = {
	.owner		= THIS_MODULE,
	.open		= lm36011_open,
	.release	= lm36011_release,
	.unlocked_ioctl	= lm36011_ioctl,
};

static int32_t lm36011_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc;
	uint32_t device_id = 0;
	uint32_t i;
	struct device *dev_ret;
	struct led_laser_ctrl_t *ctrl;
	char *class_name, *device_name;

	if (cam_cci_get_subdev(CCI_DEVICE_0) == NULL ||
		cam_cci_get_subdev(CCI_DEVICE_1) == NULL) {
		dev_warn(&pdev->dev, "wait for cci driver probe");
		return -EPROBE_DEFER;
	}

	/* Create sensor control structure */
	ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct led_laser_ctrl_t), GFP_KERNEL);
	if (!ctrl) {
		dev_err(&pdev->dev, "no memory for driver ctrl");
		return -ENOMEM;
	}

	/*fill in platform device*/
	ctrl->soc_info.pdev = pdev;
	ctrl->soc_info.dev = &pdev->dev;
	ctrl->soc_info.dev_name = pdev->name;
	ctrl->io_master_info.master_type = CCI_MASTER;
	ctrl->is_power_up = false;
	ctrl->is_cci_init = false;
	ctrl->is_probed = false;
	ctrl->silego.is_power_up = false;
	ctrl->silego.is_validated = false;
	ctrl->silego.is_vcsel_fault = false;
	ctrl->silego.is_csense_halt = false;
	ctrl->silego.fault_flag = 0;
	ctrl->silego.vcsel_fault_count = 0;
	ctrl->silego.is_cci_init = false;
	ctrl->silego.self_test_result = false;
	ctrl->cap_sense.is_validated = false;
	ctrl->cap_sense.sample_count = 0;

	for (i = 0; i < LASER_TYPE_MAX; i++)
		ctrl->cap_sense.is_crack_detected[i] = false;
	for (i = 0; i < PHASENUM; i++) {
		ctrl->cap_sense.calibration_data[i] = 0;
		ctrl->cap_sense.cap_bias[i] = 0;
		ctrl->cap_sense.cap_slope[i] = 0;
		ctrl->cap_sense.cap_raw[i] = 0;
		ctrl->cap_sense.cap_corrected[i] = 0;
		ctrl->cap_sense.max_supported_temp[i] = 0;
	}

	ctrl->io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for cci client");
		return -ENOMEM;
	}

	ctrl->cap_sense.io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->cap_sense.io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for cap sense cci client");
		return -ENOMEM;
	}

	ctrl->silego.io_master_info.cci_client = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(ctrl->silego.io_master_info.cci_client)) {
		dev_err(&pdev->dev, "no memory for silego cci client");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ctrl);
	dev_set_drvdata(&pdev->dev, ctrl);

	rc = lm36011_parse_dt(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "paring led laser dt failed rc %d", rc);
		return rc;
	}

	rc = lm36011_update_i2c_info(&(pdev->dev));
	if (rc) {
		dev_err(&pdev->dev, "update i2c info failed rc %d", rc);
		return rc;
	}

	/* Fill platform device id*/
	pdev->id = ctrl->soc_info.index;

	mutex_init(&ctrl->cam_sensor_mutex);

	rc = sysfs_create_groups(&pdev->dev.kobj, led_laser_dev_groups);
	if (rc != 0) {
		dev_err(&pdev->dev, "failed to create sysfs files");
		goto error_destroy_mutex;
	}

	rc = alloc_chrdev_region(&ctrl->dev, 0, 1, "lm36011_ioctl");
	if (rc)
		goto error_remove_sysfs;

	cdev_init(&ctrl->c_dev, &lm36011_fops);

	rc = cdev_add(&ctrl->c_dev, ctrl->dev, 1);
	if (rc)
		goto error_unregister_chrdev;

	class_name = (ctrl->type == LASER_FLOOD ? "char_flood" : "char_dot");
	ctrl->cl = class_create(THIS_MODULE, class_name);
	rc = IS_ERR(ctrl->cl);
	if (rc)
		goto error_del_cdev;

	device_name =
		(ctrl->type == LASER_FLOOD ? "lm36011_flood" : "lm36011_dot");
	dev_ret = device_create(ctrl->cl, NULL, ctrl->dev, NULL, device_name);
	rc = IS_ERR(dev_ret);
	if (rc)
		goto error_destroy_class;

	if (ctrl->hw_version < BUILD_DVT)
		INIT_WORK(&ctrl->work, cap_sense_workq_job);
	device_name =
		(ctrl->type == LASER_FLOOD ?
		"flood_workq" : "dot_workq");
	ctrl->work_queue = create_workqueue(device_name);

	/* Read device id */
	lm36011_power_up(ctrl);

	if (ctrl->hw_version < BUILD_DVT) {
		rc = sx9320_cleanup_nirq(ctrl);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"clean up cap sense irq failed: rc: %d", rc);
			ctrl->cap_sense.is_validated = false;
		} else
			ctrl->cap_sense.is_validated = true;
	} else {
		/* check Silego initial state */
		rc = silego_check_fault_type(ctrl);
		if (rc < 0) {
			dev_err(ctrl->soc_info.dev,
				"check silego state failed");
			ctrl->silego.is_validated = false;
		} else {
			if (ctrl->silego.fault_flag != ITOR_OPEN_CIRCUIT &&
				ctrl->silego.fault_flag != NO_FAULT &&
				ctrl->silego.fault_flag != FLOOD_ITOR_FAULT &&
				ctrl->silego.fault_flag != DOT_ITOR_FAULT) {
				dev_err(ctrl->soc_info.dev,
					"Silego not in right state: %x",
					ctrl->silego.fault_flag);
				ctrl->silego.is_validated = false;
			} else
				dev_info(ctrl->soc_info.dev,
					"Silego state: 0x%x",
					ctrl->silego.fault_flag);
		}
	}

	rc = lm36011_read_data(ctrl,
		DEVICE_ID_REG, &device_id);
	if (rc != 0) {
		lm36011_power_down(ctrl);
		goto error_destroy_device;
	}

	rc = lm36011_power_down(ctrl);
	if (rc != 0)
		goto error_destroy_device;

	if (device_id == DEVICE_ID)
		_dev_info(&pdev->dev, "probe success, device id 0x%x rc = %d",
			device_id, rc);
	else
		dev_warn(&pdev->dev, "Device id mismatch, got 0x%x,"
			" expected 0x%x rc = %d", device_id, DEVICE_ID, rc);

	ctrl->is_probed = true;
	return rc;

error_destroy_device:
	if (ctrl->hw_version < BUILD_DVT) {
		flush_workqueue(ctrl->work_queue);
		destroy_workqueue(ctrl->work_queue);
	}
	device_destroy(ctrl->cl, ctrl->dev);
error_destroy_class:
	class_destroy(ctrl->cl);
error_del_cdev:
	cdev_del(&ctrl->c_dev);
error_unregister_chrdev:
	unregister_chrdev_region(ctrl->dev, 1);
error_remove_sysfs:
	sysfs_remove_groups(&pdev->dev.kobj, led_laser_dev_groups);
error_destroy_mutex:
	mutex_destroy(&ctrl->cam_sensor_mutex);
	return rc;
}


static const struct of_device_id lm36011_driver_dt_match[] = {
	{.compatible = "qcom,cam-led-laser"},
	{}
};

MODULE_DEVICE_TABLE(of, lm36011_driver_dt_match);

static struct platform_driver lm36011_platform_driver = {
	.probe = lm36011_driver_platform_probe,
	.driver = {
		.name = LM36011_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm36011_driver_dt_match,
	},
	.remove = lm36011_platform_remove,
};

static int __init lm36011_init(void)
{
	return platform_driver_register(&lm36011_platform_driver);
}

static void __exit lm36011_exit(void)
{
	platform_driver_unregister(&lm36011_platform_driver);
}

MODULE_DESCRIPTION("Led laser driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Speth Chang <spethchang@google.com>");

module_init(lm36011_init);
module_exit(lm36011_exit);
