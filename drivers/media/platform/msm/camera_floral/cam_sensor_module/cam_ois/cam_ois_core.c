/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_debug_util.h"
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_packet_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "../cam_fw_update/fw_update.h"

static bool ois_debug;
module_param(ois_debug, bool, 0644);

#define OIS_WIDE_SID (0x76 >> 1)
#define OIS_TELE_SID (0x78 >> 1)

int cam_ois_calibration(struct cam_ois_ctrl_t *o_ctrl,
	stReCalib *cal_result)
{
	int rc;

	rc = GyroReCalib(&o_ctrl->io_master_info, cal_result);
	if (rc != 0)
		CAM_ERR(CAM_OIS,
			"[OISCali] ReCalib FAIL, rc = %d", rc);
	else {
		rc = WrGyroOffsetData();
		if (rc != 0)
			CAM_ERR(CAM_OIS,
				"[OISCali] WrGyro FAIL, rc = %d", rc);
		else
			CAM_INFO(CAM_OIS,
				"[OISCali] SUCCESS");
	}

	return rc;
}

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
}

/**
 * cam_ois_shift_data_enqueue - enqueue shift data to ring buffer
 * @o_shift_data:       ois shift data
 * @o_ctrl:             ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_shift_data_enqueue(struct cam_ois_shift *o_shift_data,
	struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;

	mutex_lock(&o_ctrl->ois_shift_mutex);
	if (o_ctrl->buf.write_pos >= CAM_OIS_SHIFT_DATA_BUFFER_SIZE ||
		o_ctrl->buf.write_pos < 0) {
		CAM_ERR(CAM_OIS,
			"invalid OIS shift buffer index: %d",
			o_ctrl->buf.write_pos);
		rc = -EFAULT;
	} else {
		struct cam_ois_shift *pb =
			&o_ctrl->buf.buffer[o_ctrl->buf.write_pos];

		pb->time_readout = o_shift_data->time_readout;
		pb->ois_shift_x = o_shift_data->ois_shift_x;
		pb->ois_shift_y = o_shift_data->ois_shift_y;
		pb->af_lop1 = o_shift_data->af_lop1;
		o_ctrl->buf.write_pos++;
		if (o_ctrl->buf.write_pos == CAM_OIS_SHIFT_DATA_BUFFER_SIZE) {
			o_ctrl->buf.write_pos = 0;
			o_ctrl->buf.is_full = true;
		}
	}
	mutex_unlock(&o_ctrl->ois_shift_mutex);
	return rc;
}

/**
 * cam_ois_read_work - worker function of read timer
 * @work:       work
 *
 * Returns success or failure
 */
static void cam_ois_read_work(struct work_struct *work)
{
	uint8_t buf[12] = { 0 };
	int32_t rc = 0;
	struct timespec ts;
	struct cam_ois_shift ois_shift_data;
	struct cam_ois_timer_t *ois_timer_in;
	int8_t y_sign;

	ois_timer_in = container_of(work, struct cam_ois_timer_t, g_work);
	get_monotonic_boottime(&ts);

	/* later ois module change the magnetic polarity,
	 * sign of y axis needs to be corrected
	 */
	y_sign = ois_timer_in->o_ctrl->ois_version == 2 ? -1 : 1;

	if (ois_debug) {
		rc = camera_io_dev_read_seq(
			&ois_timer_in->o_ctrl->io_master_info,
			0xE003, &buf[0], CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_DWORD, 8);
		CAM_INFO(CAM_OIS,
			"[0xE003] %s: buf[0-1]=%02x%02x, buf[2-3]=%02x%02x, buf[4-5]=%02x%02x, buf[6-7]=%02x%02x",
			ois_timer_in->o_ctrl->ois_name, buf[0], buf[1], buf[2],
			buf[3], buf[4], buf[5], buf[6], buf[7]);

		rc = camera_io_dev_read_seq(
			&ois_timer_in->o_ctrl->io_master_info,
			0xE005, &buf[0], CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_DWORD, 8);
		CAM_INFO(CAM_OIS,
			"[0xE005] %s: buf[0-1]=%02x%02x, buf[2-3]=%02x%02x, buf[4-5]=%02x%02x, buf[6-7]=%02x%02x",
			ois_timer_in->o_ctrl->ois_name, buf[0], buf[1], buf[2],
			buf[3], buf[4], buf[5], buf[6], buf[7]);
	}

	rc = camera_io_dev_read_seq(&ois_timer_in->o_ctrl->io_master_info,
		0xE001, &buf[0], CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_DWORD, 6);
	if (ois_debug) {
		CAM_INFO(CAM_OIS,
			"[0xE001] %s: buf[0-1]=%02x%02x, buf[2-3]=%02x%02x, buf[4-5]=%02x%02x",
			ois_timer_in->o_ctrl->ois_name, buf[0], buf[1], buf[2],
			buf[3], buf[4], buf[5]);
	}

	rc = camera_io_dev_read_seq(
		&ois_timer_in->o_ctrl->io_master_info,
		0x0764, &buf[8], CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD, 2);

	if (rc != 0) {
		ois_timer_in->i2c_fail_count++;
		CAM_ERR(CAM_OIS, "read seq fail. cnt = %d",
			ois_timer_in->i2c_fail_count);
		if (ois_timer_in->i2c_fail_count >= MAX_FAIL_CNT) {
			CAM_ERR(CAM_OIS, "Too many i2c failed. Stop timer.");
			ois_timer_in->ois_timer_state = CAM_OIS_TIME_ERROR;
		}
		return;
	}

	ois_timer_in->i2c_fail_count = 0;
	ois_shift_data.time_readout =
		(int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	ois_shift_data.ois_shift_x =
		(int16_t)(((uint16_t)buf[0] << 8) + (uint16_t)buf[1]);
	ois_shift_data.ois_shift_y =
		(int16_t)((((uint16_t)buf[2] << 8) + (uint16_t)buf[3])
		* y_sign);
	ois_shift_data.af_lop1 =
		(int16_t)(((uint16_t)buf[8] << 8) + (uint16_t)buf[9]);

	rc = cam_ois_shift_data_enqueue(&ois_shift_data, ois_timer_in->o_ctrl);
	if (rc != 0)
		CAM_ERR(CAM_OIS, "OIS shift data enqueue failed");
}

/**
 * cam_ois_shift_timer - ois shift reader timer expiracy callback func
 * @timer:        pointer to the hrtimer struct
 *
 * Returns hrtimer_restart state
 */
static enum hrtimer_restart cam_ois_shift_timer(struct hrtimer *timer)
{
	ktime_t currtime, interval;
	struct cam_ois_timer_t *ois_timer_in;

	ois_timer_in = container_of(timer, struct cam_ois_timer_t, hr_timer);
	if (ois_timer_in->ois_timer_state == CAM_OIS_TIME_ERROR) {
		CAM_ERR(CAM_OIS, "HRTIMER_NORESTART");
		return HRTIMER_NORESTART;
	}

	queue_work(ois_timer_in->ois_wq, &ois_timer_in->g_work);
	currtime = ktime_get();
	interval = ktime_set(0, READ_OUT_TIME);
	hrtimer_forward(timer, currtime, interval);

	return HRTIMER_RESTART;
}

/**
 * cam_ois_stop_offset_reader_thread - stop shift reader
 *
 * Returns success or failure
 */
static int cam_ois_stop_shift_reader(struct cam_ois_ctrl_t *o_ctrl)
{
	if (o_ctrl->timer.ois_timer_state != CAM_OIS_TIME_ACTIVE &&
		o_ctrl->timer.ois_timer_state != CAM_OIS_TIME_ERROR) {
		CAM_ERR(CAM_OIS,
			"Invalid ois timer state:%d",
			o_ctrl->timer.ois_timer_state);
		return -EFAULT;
	}
	hrtimer_cancel(&o_ctrl->timer.hr_timer);
	destroy_workqueue(o_ctrl->timer.ois_wq);
	o_ctrl->timer.ois_timer_state = CAM_OIS_TIME_INACTIVE;
	CAM_INFO(CAM_OIS, "Successfully stopped OIS shift reader.");
	return 0;
}

/**
 * cam_ois_start_shift_reader_thread - start shift reader
 * @o_ctrl:        ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_start_shift_reader(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t ktime;
	int rc = 0;

	if (o_ctrl->timer.ois_timer_state == CAM_OIS_TIME_ERROR) {
		CAM_ERR(CAM_OIS, "OIS Timer Error.");
		cam_ois_stop_shift_reader(o_ctrl);
		return -EFAULT;
	}
	o_ctrl->timer.i2c_fail_count = 0;

	if (o_ctrl->timer.ois_timer_state == CAM_OIS_TIME_ACTIVE) {
		CAM_ERR(CAM_OIS, "invalid timer state = %d",
			o_ctrl->timer.ois_timer_state);
		return -EFAULT;
	}

	o_ctrl->io_master_info.cci_client->i2c_freq_mode =
		I2C_FAST_PLUS_MODE;
	o_ctrl->timer.o_ctrl = o_ctrl;

	// set worker function and work queue
	INIT_WORK(&o_ctrl->timer.g_work, cam_ois_read_work);
	o_ctrl->timer.ois_wq = alloc_workqueue("ois_wq", WQ_HIGHPRI, 1);
	if (!o_ctrl->timer.ois_wq) {
		CAM_ERR(CAM_OIS, "ois_wq create failed.");
		return -EFAULT;
	}

	// set timer
	ktime = ktime_set(0, READ_OUT_TIME);
	hrtimer_init(&o_ctrl->timer.hr_timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL);
	o_ctrl->timer.hr_timer.function = &cam_ois_shift_timer;
	hrtimer_start(&o_ctrl->timer.hr_timer, ktime,
		HRTIMER_MODE_REL);
	o_ctrl->timer.ois_timer_state = CAM_OIS_TIME_ACTIVE;

	mutex_lock(&o_ctrl->ois_shift_mutex);
	o_ctrl->buf.write_pos = 0;
	o_ctrl->buf.is_full = false;
	mutex_unlock(&o_ctrl->ois_shift_mutex);

	CAM_INFO(CAM_OIS, "Successfully started OIS shift reader.");
	return rc;
}

/**
 * cam_ois_get_shift - write ois shift data to user space
 * @o_ctrl:             ctrl structure
 * @query_size_handle:  handle of query_size in user space
 * @shift_data_handle:  handle of shift_data in user space
 *
 * Returns success or failure
 */
static int cam_ois_get_shift(struct cam_ois_ctrl_t *o_ctrl,
	uint64_t query_size_handle, uint64_t shift_data_handle)
{
	int rc = 0;
	struct cam_ois_shift buf[CAM_OIS_SHIFT_DATA_BUFFER_SIZE];
	uint8_t query_size = 0;
	int32_t write_pos = 0;

	// copy from ring buffer to local continuous buffer
	mutex_lock(&o_ctrl->ois_shift_mutex);
	write_pos = o_ctrl->buf.write_pos;
	if (o_ctrl->buf.is_full) {
		query_size = CAM_OIS_SHIFT_DATA_BUFFER_SIZE - write_pos;
		memcpy(&buf[0], &o_ctrl->buf.buffer[write_pos],
			query_size * sizeof(struct cam_ois_shift));
		if (write_pos > 0)
			memcpy(&buf[query_size], &o_ctrl->buf.buffer[0],
				write_pos * sizeof(struct cam_ois_shift));
		query_size = CAM_OIS_SHIFT_DATA_BUFFER_SIZE;
	} else {
		query_size = write_pos;
		if (query_size != 0)
			memcpy(buf, o_ctrl->buf.buffer,
				query_size * sizeof(struct cam_ois_shift));
	}
	// reset ring buffer
	o_ctrl->buf.write_pos = 0;
	o_ctrl->buf.is_full = false;
	mutex_unlock(&o_ctrl->ois_shift_mutex);

	// copy to user space
	if (copy_to_user((void __user *) query_size_handle, &query_size,
		sizeof(uint8_t))) {
		CAM_ERR(CAM_OIS, "ois_get_shift: size copy to user failed!");
		return -EFAULT;
	}
	if (copy_to_user((void __user *) shift_data_handle, buf,
		sizeof(struct cam_ois_shift) * query_size)) {
		CAM_ERR(CAM_OIS, "ois_get_shift: buf copy to user failed!");
		return -EFAULT;
	}

	return rc;
}

/**
 * cam_ois_read_reg - read register data and copy to user space
 * @o_ctrl:           ctrl structure
 * @cmd_get_ois:      handle of shift_data in user space
 *
 * Returns success or failure
 */
static int cam_ois_read_reg(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_cmd_get_ois_data *cmd_get_ois)
{
	uint8_t buf[8] = { 0 };
	int32_t rc = 0;
	uint32_t addr = cmd_get_ois->reg_addr;
	int32_t num_bytes = cmd_get_ois->reg_data;

	if (addr <= 0 || addr > 0xFFFF) {
		CAM_ERR(CAM_OIS,
			"Invalid addr while read OIS data: %x", addr);
		return -EINVAL;
	}

	if (num_bytes <= 0 || num_bytes > 8) {
		CAM_ERR(CAM_OIS,
			"Invalid read size while read OIS data: %d", num_bytes);
		return -EINVAL;
	}

	rc = camera_io_dev_read_seq(&o_ctrl->io_master_info,
		addr, buf, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD, num_bytes);

	if (rc) {
		CAM_ERR(CAM_OIS, "camera_io_dev_read_seq failed!");
		return rc;
	}

	if (copy_to_user((void __user *) cmd_get_ois->query_data_handle,
		buf, sizeof(uint8_t) * num_bytes)) {
		CAM_ERR(CAM_OIS, "ois_read_reg: copy to user failed!");
	}

	return rc;
}

/**
 * cam_ois_util_validate_packet - validate camera packet
 * @packet:        Camera packet structure
 *
 * Returns success or failure
 */
static int cam_ois_util_validate_packet(struct cam_packet *packet)
{
	if (!packet)
		return -EINVAL;

	CAM_DBG(CAM_UTIL, "num cmd buf:%d", packet->num_cmd_buf);

	if ((!packet->header.size) ||
		(packet->cmd_buf_offset > packet->header.size) ||
		(packet->io_configs_offset > packet->header.size))  {
		CAM_ERR(CAM_UTIL, "invalid packet:%d %d %d %d",
			packet->num_cmd_buf, packet->cmd_buf_offset,
			packet->io_configs_offset, packet->header.size);
		return -EINVAL;
	}

	return 0;
}

static void cam_ois_tele_standby(struct cam_ois_ctrl_t *o_ctrl)
{
	uint32_t rc = 0;
	enum camera_sensor_i2c_type addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	enum camera_sensor_i2c_type data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	struct cam_sensor_i2c_reg_array servo_setting = {0xF010, 0x70000, 0, 0};
	struct cam_sensor_i2c_reg_array standby_setting = {0xF01F, 0x2, 0, 0};
	struct cam_sensor_i2c_reg_setting write_setting = {&servo_setting, 1,
		addr_type, data_type, 0, OIS_TELE_SID};
	uint32_t delay_ms = 3;
	uint32_t orig_sid = o_ctrl->io_master_info.cci_client->sid;
	uint32_t orig_master =
		o_ctrl->io_master_info.cci_client->cci_i2c_master;

	o_ctrl->io_master_info.cci_client->sid = OIS_TELE_SID;
	o_ctrl->io_master_info.cci_client->cci_i2c_master = 1;
	rc |= camera_io_dev_poll(&(o_ctrl->io_master_info),
		0xF100, 0, 0, addr_type, data_type, delay_ms);
	rc |= camera_io_dev_write(&(o_ctrl->io_master_info), &write_setting);
	rc |= camera_io_dev_poll(&(o_ctrl->io_master_info),
		0xF100, 0, 0, addr_type, data_type, delay_ms);
	write_setting.reg_setting = &standby_setting;
	rc |= camera_io_dev_write(&(o_ctrl->io_master_info), &write_setting);

	if (rc != 0)
		CAM_WARN(CAM_UTIL, "%s failed rc %d", __func__, rc);
	o_ctrl->io_master_info.cci_client->sid = orig_sid;
	o_ctrl->io_master_info.cci_client->cci_i2c_master = orig_master;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;
	struct cam_cmd_get_ois_data     *cmd_get_ois = NULL;
	int32_t                         *cal_rc;
	stReCalib                       *cal_result;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto rel_pkt;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		rc = -EINVAL;
		goto rel_pkt;
	}


	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				goto rel_pkt;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					goto rel_cmd_buf;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			}
			break;
			}
			if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
				CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				goto rel_pkt;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply Init settings");
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		/* OIS-tele enters standby mode on OIS-wide init */
		if (o_ctrl->io_master_info.cci_client->sid == OIS_WIDE_SID)
			cam_ois_tele_standby(o_ctrl);

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			goto rel_pkt;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto rel_pkt;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto rel_pkt;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto rel_pkt;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_SHIFT_READER_START:
		rc = cam_ois_start_shift_reader(o_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed start shift reader, rc: %d", rc);
			goto rel_pkt;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_SHIFT_READER_STOP:
		rc = cam_ois_stop_shift_reader(o_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed stop shift reader, rc: %d", rc);
			goto rel_pkt;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_SHIFT_GET:
	case CAM_OIS_PACKET_OPCODE_READ:
		rc = cam_ois_util_validate_packet(csl_packet);
		if (rc < 0)
			goto rel_pkt;
		if (csl_packet->num_cmd_buf != 1) {
			CAM_ERR(CAM_OIS,
				"More than one cmd buf found in shift_get");
			rc = -EINVAL;
			goto rel_pkt;
		}

		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		cmd_desc = (struct cam_cmd_buf_desc *)offset;
		total_cmd_buf_in_bytes = cmd_desc->length;
		if (!total_cmd_buf_in_bytes) {
			CAM_ERR(CAM_OIS,
				"Empty cmd buf found in shift_get");
			rc = -EINVAL;
			goto rel_pkt;
		}

		rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
			&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get cpu buf");
			goto rel_pkt;
		}
		if (!generic_ptr) {
			CAM_ERR(CAM_OIS, "invalid generic_ptr");
			if (cam_mem_put_cpu_buf(cmd_desc->mem_handle))
				CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
			rc = -EINVAL;
			goto rel_pkt;
		}
		offset = (uint32_t *)((uint8_t *)generic_ptr +
			cmd_desc->offset);
		cmd_get_ois = (struct cam_cmd_get_ois_data *)offset;

		if ((csl_packet->header.op_code & 0xFFFFFF) ==
			CAM_OIS_PACKET_OPCODE_SHIFT_GET) {
			rc = cam_ois_get_shift(o_ctrl,
				cmd_get_ois->query_size_handle,
				cmd_get_ois->query_data_handle);
		} else if ((csl_packet->header.op_code & 0xFFFFFF) ==
			CAM_OIS_PACKET_OPCODE_READ) {
			rc = cam_ois_read_reg(o_ctrl, cmd_get_ois);
		}

		if (cam_mem_put_cpu_buf(cmd_desc->mem_handle))
			CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
				cmd_desc[i].mem_handle);

		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get ois data");
			goto rel_pkt;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_CALIBRATION:
		rc = cam_ois_util_validate_packet(csl_packet);
		if (rc < 0)
			goto rel_pkt;
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		cmd_desc = (struct cam_cmd_buf_desc *)offset;
		rc = cam_mem_get_cpu_buf(cmd_desc[0].mem_handle,
			&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get cpu buf");
			goto rel_pkt;
		}
		if (!generic_ptr) {
			CAM_ERR(CAM_OIS, "invalid generic_ptr");
			if (cam_mem_put_cpu_buf(cmd_desc[0].mem_handle))
				CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
			rc = -EINVAL;
			goto rel_pkt;
		}
		offset = (uint32_t *)((uint8_t *)generic_ptr +
			cmd_desc->offset);
		cal_rc = (int32_t *)offset;
		cal_result = (stReCalib *)(offset + 1);
		*cal_rc = cam_ois_calibration(o_ctrl, cal_result);
		if (cam_mem_put_cpu_buf(cmd_desc[0].mem_handle))
			CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
				cmd_desc[i].mem_handle);
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto rel_pkt;
	}

	if (!rc)
		goto rel_pkt;

rel_cmd_buf:
	if (cmd_desc) {
		if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
			CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
				cmd_desc[i].mem_handle);
	}
pwr_dwn:
	cam_ois_power_down(o_ctrl);
rel_pkt:
	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_OIS, "Fail in put buffer: 0x%x",
			dev_config.packet_handle);

	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
