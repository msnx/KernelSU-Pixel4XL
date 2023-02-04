/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_sensor_hw_sync.h"

#ifdef CONFIG_CAMERA_FW_UPDATE
#include "../cam_fw_update/fw_update.h"
#endif

static struct sensor_status_t sensor_status;
static enum laser_tag_type laser_tag_indicator;

static irqreturn_t cam_dot_flood_irq_handler(int irq, void *dev_id)
{
	struct cam_sensor_ctrl_t *s_ctrl = (struct cam_sensor_ctrl_t *)dev_id;
	struct cam_soc_gpio_data *gpio_conf;
	int gpio_level;
	uint8_t gpio_count;
	uint8_t gpio_idx;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "can not get sensor control structure");
		return IRQ_HANDLED;
	}
	gpio_conf = s_ctrl->soc_info.gpio_data;
	gpio_count = gpio_conf->cam_gpio_req_tbl_size;
	gpio_idx = s_ctrl->cam_safety_gpio_idx[LASER_STATUS];
	gpio_level =
		gpio_get_value(gpio_conf->cam_gpio_req_tbl[gpio_idx].gpio);
	laser_tag_indicator = (gpio_level == 1 ?
		LASER_TAG_DOT : LASER_TAG_FLOOD);

	return IRQ_HANDLED;
}

static void cam_register_laser_irq(struct cam_sensor_ctrl_t *s_ctrl,
	enum cam_sensor_gpio_irq type)
{
	if (s_ctrl->cam_sensor_irq[type] == 0)
		return;
	switch (type) {
	case LASER_STATUS:
		request_irq(s_ctrl->cam_sensor_irq[type],
			cam_dot_flood_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"laser_tag", s_ctrl);
		break;
	default:
		CAM_WARN(CAM_SENSOR, "unsupported irq type: %d", type);
	}
}

static void cam_release_laser_irq(struct cam_sensor_ctrl_t *s_ctrl,
	enum cam_sensor_gpio_irq type)
{
	if (s_ctrl->cam_sensor_irq[type] == 0)
		return;
	free_irq(s_ctrl->cam_sensor_irq[type], s_ctrl);
}

static void cam_find_laser_irq(struct cam_sensor_ctrl_t *s_ctrl)
{
	int i;
	struct cam_soc_gpio_data *gpio_conf = s_ctrl->soc_info.gpio_data;
	uint8_t gpio_count = gpio_conf->cam_gpio_req_tbl_size;

	for (i = 0; i < gpio_count; i++) {
		if (strcmp(gpio_conf->cam_gpio_req_tbl[i].label,
			"DOT_FLOOD_STATUS") == 0) {
			s_ctrl->cam_sensor_irq[LASER_STATUS] =
				gpio_to_irq(
				gpio_conf->cam_gpio_req_tbl[i].gpio);
			s_ctrl->cam_safety_gpio_idx[LASER_STATUS] = i;
			break;
		}
	}
	if (s_ctrl->cam_sensor_irq[LASER_STATUS] == 0)
		CAM_INFO(CAM_SENSOR,
			"no valid IRQ found for laser status");
}


static ssize_t ois_fw_ver_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct cam_sensor_ctrl_t *ctrl = dev_get_drvdata(dev);

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->ois_fw_ver);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static ssize_t vcm_fw_ver_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int rc;
	struct cam_sensor_ctrl_t *ctrl = dev_get_drvdata(dev);

	mutex_lock(&ctrl->cam_sensor_mutex);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", ctrl->vcm_fw_ver);
	mutex_unlock(&ctrl->cam_sensor_mutex);
	return rc;
}

static DEVICE_ATTR_RO(ois_fw_ver);
static DEVICE_ATTR_RO(vcm_fw_ver);

static struct attribute *sensor_fw_dev_attrs[] = {
	&dev_attr_ois_fw_ver.attr,
	&dev_attr_vcm_fw_ver.attr,
	NULL
};

ATTRIBUTE_GROUPS(sensor_fw_dev);

static void cam_sensor_update_req_mgr(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_req_mgr_add_request add_req;

	add_req.link_hdl = s_ctrl->bridge_intf.link_hdl;
	add_req.req_id = csl_packet->header.request_id;
	CAM_DBG(CAM_SENSOR, " Rxed Req Id: %lld",
		csl_packet->header.request_id);
	add_req.dev_hdl = s_ctrl->bridge_intf.device_hdl;
	add_req.skip_before_applying = 0;
	if (s_ctrl->bridge_intf.crm_cb &&
		s_ctrl->bridge_intf.crm_cb->add_req)
		s_ctrl->bridge_intf.crm_cb->add_req(&add_req);

	CAM_DBG(CAM_SENSOR, " add req to req mgr: %lld",
			add_req.req_id);
}

static void cam_sensor_release_stream_rsc(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int rc;

	i2c_set = &(s_ctrl->i2c_data.streamoff_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamoff settings");
	}

	i2c_set = &(s_ctrl->i2c_data.streamon_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamon settings");
	}

	i2c_set = &(s_ctrl->i2c_data.master_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Master settings");
	}

	i2c_set = &(s_ctrl->i2c_data.slave_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Slave settings");
	}
}

static void cam_sensor_release_per_frame_resource(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int i, rc;

	if (s_ctrl->i2c_data.per_frame != NULL) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(s_ctrl->i2c_data.per_frame[i]);
			if (i2c_set->is_settings_valid == 1) {
				i2c_set->is_settings_valid = -1;
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
	}
}

static int32_t cam_sensor_read_reg(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet)
{
	int32_t rc = 0;
	uint8_t buf[8] = { 0 };
	uint32_t addr = 0;
	int32_t num_bytes = 0;
	uint32_t total_cmd_buf_in_bytes = 0;
	uintptr_t generic_ptr;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	size_t len_of_buff = 0;
	uint32_t *offset = NULL;
	struct cam_cmd_get_sensor_data *cmd_get_sensor = NULL;

	if (csl_packet->num_cmd_buf != 1) {
		CAM_ERR(CAM_SENSOR,
			"More than one cmd buf found in sensor read");
		return -EINVAL;
	}

	offset = (uint32_t *)&csl_packet->payload;
	offset += csl_packet->cmd_buf_offset / sizeof(uint32_t);
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);
	total_cmd_buf_in_bytes = cmd_desc->length;
	if (!total_cmd_buf_in_bytes) {
		CAM_ERR(CAM_SENSOR,
			"Empty cmd buf found in sensor read");
		return -EINVAL;
	}

	// To keep the pointer of reg_data
	rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
		&generic_ptr, &len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to get cpu buf");
		return rc;
	}
	if (!generic_ptr) {
		CAM_ERR(CAM_SENSOR, "invalid generic_ptr");
		return -EINVAL;
	}
	offset = (uint32_t *)((uint8_t *)generic_ptr +
		cmd_desc->offset);
	cmd_get_sensor = (struct cam_cmd_get_sensor_data *)offset;

	addr = cmd_get_sensor->reg_addr;
	num_bytes = cmd_get_sensor->reg_data;
	if (addr <= 0 || addr > 0xFFFF) {
		CAM_ERR(CAM_SENSOR,
			"Invalid addr while read Sensor data: %x", addr);
		return -EINVAL;
	}

	if (num_bytes <= 0 || num_bytes > 8) {
		CAM_ERR(CAM_SENSOR,
			"Invalid read size while read Sensor data: %d",
			num_bytes);
		return -EINVAL;
	}

	rc = camera_io_dev_read_seq(&s_ctrl->io_master_info,
		addr, buf, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD, num_bytes);

	if (rc) {
		CAM_ERR(CAM_SENSOR, "camera_io_dev_read_seq failed!");
		return rc;
	}

	if (copy_to_user((void __user *) cmd_get_sensor->query_data_handle,
		buf, num_bytes)) {
		CAM_ERR(CAM_SENSOR, "sensor_read_reg: copy to user failed!");
	}

	return rc;
}

static int32_t cam_sensor_i2c_pkt_parse(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int32_t rc = 0;
	uintptr_t generic_ptr;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	size_t len_of_buff = 0;
	size_t remain_len = 0;
	uint32_t *offset = NULL;
	struct cam_config_dev_cmd config;
	struct i2c_data_settings *i2c_data = NULL;

	ioctl_ctrl = (struct cam_control *)arg;

	if (ioctl_ctrl->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SENSOR, "Invalid Handle Type");
		return -EINVAL;
	}

	if (copy_from_user(&config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config)))
		return -EFAULT;

	rc = cam_mem_get_cpu_buf(
		config.packet_handle,
		&generic_ptr,
		&len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed in getting the packet: %d", rc);
		return rc;
	}

	remain_len = len_of_buff;
	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_SENSOR,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)(generic_ptr +
		(uint32_t)config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_SENSOR, "Invalid packet params");
		rc = -EINVAL;
		goto rel_pkt_buf;

	}

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG &&
		csl_packet->header.request_id <= s_ctrl->last_flush_req
		&& s_ctrl->last_flush_req != 0) {
		CAM_ERR(CAM_SENSOR,
			"reject request %lld, last request to flush %lld",
			csl_packet->header.request_id, s_ctrl->last_flush_req);
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	if (csl_packet->header.request_id > s_ctrl->last_flush_req)
		s_ctrl->last_flush_req = 0;

	i2c_data = &(s_ctrl->i2c_data);
	CAM_DBG(CAM_SENSOR, "Header OpCode: %d", csl_packet->header.op_code);
	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
		i2c_reg_settings = &i2c_data->init_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
		i2c_reg_settings = &i2c_data->config_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
		if (s_ctrl->streamon_count > 0)
			goto rel_pkt_buf;

		s_ctrl->streamon_count = s_ctrl->streamon_count + 1;
		i2c_reg_settings = &i2c_data->streamon_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
		if (s_ctrl->streamoff_count > 0)
			goto rel_pkt_buf;

		s_ctrl->streamoff_count = s_ctrl->streamoff_count + 1;
		i2c_reg_settings = &i2c_data->streamoff_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER: {
		i2c_reg_settings = &i2c_data->master_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}

	case CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_SLAVE: {
		i2c_reg_settings = &i2c_data->slave_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}

	case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed Update packets without linking");
			goto rel_pkt_buf;
		}

		i2c_reg_settings =
			&i2c_data->per_frame[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		CAM_DBG(CAM_SENSOR, "Received Packet: %lld req: %lld",
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY,
			csl_packet->header.request_id);
		if (i2c_reg_settings->is_settings_valid == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already some pkt in offset req : %lld",
				csl_packet->header.request_id);
			/*
			 * Update req mgr even in case of failure.
			 * This will help not to wait indefinitely
			 * and freeze. If this log is triggered then
			 * fix it.
			 */
			cam_sensor_update_req_mgr(s_ctrl, csl_packet);
			goto rel_pkt_buf;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_READREG: {
		rc = cam_sensor_read_reg(s_ctrl, csl_packet);
		return rc;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_CMD: {
		rc = cam_sensor_sync_pkt_parse(s_ctrl, csl_packet);
		return rc;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_NOP: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed NOP packets without linking");
			goto rel_pkt_buf;
		}

		cam_sensor_update_req_mgr(s_ctrl, csl_packet);
		goto rel_pkt_buf;
	}
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Packet Header");
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	offset = (uint32_t *)&csl_packet->payload;
	offset += csl_packet->cmd_buf_offset / 4;
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
			i2c_reg_settings, cmd_desc, 1);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Fail parsing I2C Pkt: %d", rc);
		goto rel_pkt_buf;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) ==
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE) {
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		cam_sensor_update_req_mgr(s_ctrl, csl_packet);
	}

rel_pkt_buf:
	if (cam_mem_put_cpu_buf(config.packet_handle))
		CAM_WARN(CAM_SENSOR, "Failed in put the buffer: 0x%x",
			config.packet_handle);

	return rc;
}

static int32_t cam_sensor_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;
	uint16_t default_sid = 0;
	struct cam_sensor_ctrl_t *s_ctrl;

	s_ctrl = container_of(io_master_info,
		struct cam_sensor_ctrl_t,
		io_master_info);
	if (s_ctrl)
		cam_sensor_sync_audit(s_ctrl, i2c_list);

	if ((io_master_info->master_type == CCI_MASTER) &&
		(i2c_list->i2c_settings.slave_addr != 0)) {
		default_sid = io_master_info->cci_client->sid;
		io_master_info->cci_client->sid =
			i2c_list->i2c_settings.slave_addr >> 1;
	}

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to random write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			0);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to seq write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to burst write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
		size = i2c_list->i2c_settings.size;
		for (i = 0; i < size; i++) {
			rc = camera_io_dev_poll(
			io_master_info,
			i2c_list->i2c_settings.reg_setting[i].reg_addr,
			i2c_list->i2c_settings.reg_setting[i].reg_data,
			i2c_list->i2c_settings.reg_setting[i].data_mask,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type,
			i2c_list->i2c_settings.reg_setting[i].delay);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	}

	if ((io_master_info->master_type == CCI_MASTER) &&
		(i2c_list->i2c_settings.slave_addr != 0)) {
		io_master_info->cci_client->sid = default_sid;
	}

	return rc;
}

int32_t cam_sensor_update_i2c_info(struct cam_cmd_i2c_info *i2c_info,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct cam_sensor_cci_client   *cci_client = NULL;

	if (s_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = s_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_SENSOR, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;

		if (s_ctrl->override_info.sensor_slave_addr) {
			CAM_INFO(CAM_SENSOR, "slave_addr > ori: 0x%x new: 0x%x",
				i2c_info->slave_addr,
				s_ctrl->override_info.sensor_slave_addr);
			i2c_info->slave_addr =
				s_ctrl->override_info.sensor_slave_addr;
		}

		cci_client->sid = i2c_info->slave_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
		CAM_DBG(CAM_SENSOR, " Master: %d sid: %d freq_mode: %d",
			cci_client->cci_i2c_master, i2c_info->slave_addr,
			i2c_info->i2c_freq_mode);
	}

	s_ctrl->sensordata->slave_info.sensor_slave_addr =
		i2c_info->slave_addr;
	return rc;
}

int32_t cam_sensor_update_slave_info(struct cam_cmd_probe *probe_info,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	s_ctrl->sensordata->slave_info.sensor_id_reg_addr =
		probe_info->reg_addr;

	if (s_ctrl->override_info.sensor_id) {
		CAM_INFO(CAM_SENSOR, "sensor_id > ori: 0x%x new: 0x%x",
			probe_info->expected_data,
			s_ctrl->override_info.sensor_id);
		probe_info->expected_data = s_ctrl->override_info.sensor_id;
	}

	s_ctrl->sensordata->slave_info.sensor_id =
		probe_info->expected_data;
	s_ctrl->sensordata->slave_info.sensor_id_mask =
		probe_info->data_mask;
	s_ctrl->fw_update_flag = probe_info->fw_update_flag;
	/* Userspace passes the pipeline delay in reserved field */
	s_ctrl->pipeline_delay =
		probe_info->reserved;

	s_ctrl->sensor_probe_addr_type =  probe_info->addr_type;
	s_ctrl->sensor_probe_data_type =  probe_info->data_type;
	CAM_DBG(CAM_SENSOR,
		"Sensor Addr: 0x%x sensor_id: 0x%x sensor_mask: 0x%x sensor_pipeline_delay:0x%x",
		s_ctrl->sensordata->slave_info.sensor_id_reg_addr,
		s_ctrl->sensordata->slave_info.sensor_id,
		s_ctrl->sensordata->slave_info.sensor_id_mask,
		s_ctrl->pipeline_delay);
	return rc;
}

int32_t cam_handle_cmd_buffers_for_probe(void *cmd_buf,
	struct cam_sensor_ctrl_t *s_ctrl,
	int32_t cmd_buf_num, uint32_t cmd_buf_length, size_t remain_len)
{
	int32_t rc = 0;

	switch (cmd_buf_num) {
	case 0: {
		struct cam_cmd_i2c_info *i2c_info = NULL;
		struct cam_cmd_probe *probe_info;

		if (remain_len <
			(sizeof(struct cam_cmd_i2c_info) +
			sizeof(struct cam_cmd_probe))) {
			CAM_ERR(CAM_SENSOR,
				"not enough buffer for cam_cmd_i2c_info");
			return -EINVAL;
		}
		i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
		rc = cam_sensor_update_i2c_info(i2c_info, s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed in Updating the i2c Info");
			return rc;
		}
		probe_info = (struct cam_cmd_probe *)
			(cmd_buf + sizeof(struct cam_cmd_i2c_info));
		rc = cam_sensor_update_slave_info(probe_info, s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Updating the slave Info");
			return rc;
		}
		cmd_buf = probe_info;
	}
		break;
	case 1: {
		rc = cam_sensor_update_power_settings(cmd_buf,
			cmd_buf_length, &s_ctrl->sensordata->power_info,
			remain_len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed in updating power settings");
			return rc;
		}
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid command buffer");
		break;
	}
	return rc;
}

int32_t cam_handle_mem_ptr(uint64_t handle, struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, i;
	uint32_t *cmd_buf;
	void *ptr;
	size_t len;
	struct cam_packet *pkt = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	uintptr_t cmd_buf1 = 0;
	uintptr_t packet = 0;
	size_t    remain_len = 0;

	rc = cam_mem_get_cpu_buf(handle,
		&packet, &len);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to get the command Buffer");
		return -EINVAL;
	}

	pkt = (struct cam_packet *)packet;
	if (pkt == NULL) {
		CAM_ERR(CAM_SENSOR, "packet pos is invalid");
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	if ((len < sizeof(struct cam_packet)) ||
		(pkt->cmd_buf_offset >= (len - sizeof(struct cam_packet)))) {
		CAM_ERR(CAM_SENSOR, "Not enough buf provided");
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *)&pkt->payload + pkt->cmd_buf_offset/4);
	if (cmd_desc == NULL) {
		CAM_ERR(CAM_SENSOR, "command descriptor pos is invalid");
		rc = -EINVAL;
		goto rel_pkt_buf;
	}
	if (pkt->num_cmd_buf != 2) {
		CAM_ERR(CAM_SENSOR, "Expected More Command Buffers : %d",
			 pkt->num_cmd_buf);
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	for (i = 0; i < pkt->num_cmd_buf; i++) {
		if (!(cmd_desc[i].length))
			continue;
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&cmd_buf1, &len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the command Buffer Header");
			goto rel_pkt_buf;
		}
		if (cmd_desc[i].offset >= len) {
			CAM_ERR(CAM_SENSOR,
				"offset past length of buffer");
			rc = -EINVAL;
			goto rel_pkt_buf;
		}
		remain_len = len - cmd_desc[i].offset;
		if (cmd_desc[i].length > remain_len) {
			CAM_ERR(CAM_SENSOR,
				"Not enough buffer provided for cmd");
			rc = -EINVAL;
			goto rel_pkt_buf;
		}
		cmd_buf = (uint32_t *)cmd_buf1;
		cmd_buf += cmd_desc[i].offset/4;
		ptr = (void *) cmd_buf;

		rc = cam_handle_cmd_buffers_for_probe(ptr, s_ctrl,
			i, cmd_desc[i].length, remain_len);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the command Buffer Header");
			goto rel_cmd_buf;
		}

		if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
			CAM_WARN(CAM_SENSOR,
				"Failed to put command Buffer : 0x%x",
				cmd_desc[i].mem_handle);
	}

	if (cam_mem_put_cpu_buf(handle))
		CAM_WARN(CAM_SENSOR, "Failed to put the command Buffer: 0x%x",
			handle);

	return rc;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_SENSOR, "Failed to put command Buffer : 0x%x",
			cmd_desc[i].mem_handle);
rel_pkt_buf:
	if (cam_mem_put_cpu_buf(handle))
		CAM_WARN(CAM_SENSOR, "Failed to put the command Buffer: 0x%x",
			handle);

	return rc;
}

void cam_sensor_query_cap(struct cam_sensor_ctrl_t *s_ctrl,
	struct  cam_sensor_query_cap *query_cap)
{
	query_cap->pos_roll = s_ctrl->sensordata->pos_roll;
	query_cap->pos_pitch = s_ctrl->sensordata->pos_pitch;
	query_cap->pos_yaw = s_ctrl->sensordata->pos_yaw;
	query_cap->secure_camera = 0;
	query_cap->actuator_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_ACTUATOR];
	query_cap->csiphy_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY];
	query_cap->eeprom_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_EEPROM];
	query_cap->flash_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_LED_FLASH];
	query_cap->ois_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_OIS];
	query_cap->slot_info =
		s_ctrl->soc_info.index;
}

static uint16_t cam_sensor_id_by_mask(struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t chipid)
{
	uint16_t sensor_id = (uint16_t)(chipid & 0xFFFF);
	int16_t sensor_id_mask = s_ctrl->sensordata->slave_info.sensor_id_mask;

	if (!sensor_id_mask)
		sensor_id_mask = ~sensor_id_mask;

	sensor_id &= sensor_id_mask;
	sensor_id_mask &= -sensor_id_mask;
	sensor_id_mask -= 1;
	while (sensor_id_mask) {
		sensor_id_mask >>= 1;
		sensor_id >>= 1;
	}
	return sensor_id;
}

void cam_sensor_shutdown(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info =
		&s_ctrl->sensordata->power_info;
	int rc = 0;

	if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) &&
		(s_ctrl->is_probe_succeed == 1))
		return;

	cam_sensor_release_stream_rsc(s_ctrl);
	cam_sensor_release_per_frame_resource(s_ctrl);

	if (s_ctrl->sensor_state != CAM_SENSOR_INIT)
		cam_sensor_power_down(s_ctrl);

	rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "dhdl already destroyed: rc = %d", rc);
	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.session_hdl = -1;
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;
	s_ctrl->streamon_count = 0;
	s_ctrl->streamoff_count = 0;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->last_flush_req = 0;
	s_ctrl->sensor_state = CAM_SENSOR_INIT;
	/* reset hw sync control */
	memset(&s_ctrl->hw_sync_ctrl, 0,
	       sizeof(struct cam_sensor_hw_sync_ctrl));
}

int cam_sensor_match_id(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t chipid = 0;
	struct cam_camera_slave_info *slave_info;

	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " failed: %pK",
			 slave_info);
		return -EINVAL;
	}

	rc = camera_io_dev_read(
		&(s_ctrl->io_master_info),
		slave_info->sensor_id_reg_addr,
		&chipid, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD);

	CAM_DBG(CAM_SENSOR, "read id: 0x%x expected id 0x%x:",
			 chipid, slave_info->sensor_id);
	if (cam_sensor_id_by_mask(s_ctrl, chipid) != slave_info->sensor_id) {
		CAM_ERR(CAM_SENSOR, "chip id %x does not match %x",
				chipid, slave_info->sensor_id);
		return -ENODEV;
	}
	return rc;
}

static int32_t cam_sensor_update_ir_cams_strobe(
	struct cam_sensor_ctrl_t *s_ctrl, bool enable)
{
	int32_t rc;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array reg_settings;

	reg_settings.reg_addr = FLASH_STROBE_ADDR;
	reg_settings.delay = 0;

	write_setting.reg_setting = &reg_settings;
	write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_setting.size = 1;
	write_setting.delay = 0;

	if (enable == true) {
		CAM_INFO(CAM_SENSOR, "re-setting storbe, strobe type: %d",
			s_ctrl->strobeType);
		write_setting.reg_setting->reg_data = FLASH_STROBE_EVEN;

		rc = camera_io_dev_write(&(s_ctrl->peer_ir_info),
			&write_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"failed on cci write, addr 0x%x, data: 0x%x",
				reg_settings.reg_addr, reg_settings.reg_data);
			goto out;
		}

		if (s_ctrl->strobeType == STROBE_ALTERNATIVE)
			write_setting.reg_setting->reg_data = FLASH_STROBE_ODD;

		rc = camera_io_dev_write(&(s_ctrl->io_master_info),
			&write_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"failed on cci write, addr 0x%x, data: 0x%x",
				reg_settings.reg_addr, reg_settings.reg_data);
		} else
			sensor_status.is_strobe_disabled = false;
	} else {
		write_setting.reg_setting->reg_data = FLASH_STROBE_DISABLE;

		rc = camera_io_dev_write(&(s_ctrl->io_master_info),
			&write_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"failed on cci write, addr 0x%x, data: 0x%x",
				reg_settings.reg_addr, reg_settings.reg_data);
			goto out;
		}

		rc = camera_io_dev_write(&(s_ctrl->peer_ir_info),
			&write_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"failed on cci write, addr 0x%x, data: 0x%x",
				reg_settings.reg_addr, reg_settings.reg_data);
		} else
			sensor_status.is_strobe_disabled = true;
	}

out:
	return rc;
}

int32_t cam_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_sensor_power_ctrl_t *power_info =
		&s_ctrl->sensordata->power_info;
	if (!s_ctrl || !arg) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
		return -EINVAL;
	}

	if (cmd->op_code != CAM_SENSOR_PROBE_CMD) {
		if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
			CAM_ERR(CAM_SENSOR, "Invalid handle type: %d",
				cmd->handle_type);
			return -EINVAL;
		}
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	switch (cmd->op_code) {
	case CAM_SENSOR_PROBE_CMD: {
		if (s_ctrl->is_probe_succeed == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already Sensor Probed in the slot");
			break;
		}

		if (cmd->handle_type ==
			CAM_HANDLE_MEM_HANDLE) {
			rc = cam_handle_mem_ptr(cmd->handle, s_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "Get Buffer Handle Failed");
				goto release_mutex;
			}
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid Command Type: %d",
				 cmd->handle_type);
			rc = -EINVAL;
			goto release_mutex;
		}

		/* Parse and fill vreg params for powerup settings */
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_setting,
			s_ctrl->sensordata->power_info.power_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PUP rc %d",
				 rc);
			goto free_power_settings;
		}

		/* Parse and fill vreg params for powerdown settings*/
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_down_setting,
			s_ctrl->sensordata->power_info.power_down_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PDOWN rc %d",
				 rc);
			goto free_power_settings;
		}

		/* Power up and probe sensor */
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "power up failed");
			goto free_power_settings;
		}

		/* Match sensor ID */
		rc = cam_sensor_match_id(s_ctrl);
		if (rc < 0) {
			cam_sensor_power_down(s_ctrl);
			msleep(20);
			goto free_power_settings;
		}

		/* Find laser status irq for EVT or later build */
		if (s_ctrl->soc_info.index == IR_MASTER &&
			s_ctrl->hw_version >= 2)
			cam_find_laser_irq(s_ctrl);

		CAM_INFO(CAM_SENSOR,
			"Probe success,slot:%d,slave_addr:0x%x,sensor_id:0x%x",
			s_ctrl->soc_info.index,
			s_ctrl->sensordata->slave_info.sensor_slave_addr,
			s_ctrl->sensordata->slave_info.sensor_id);

#ifdef CONFIG_CAMERA_FW_UPDATE
		if (s_ctrl->fw_update_flag) {
			CAM_INFO(CAM_SENSOR, "[OISFW]Check OIS FW update");
			rc = checkOISFWUpdate(s_ctrl);
		}
#endif

		if (s_ctrl->ois_fw_ver == 0 || s_ctrl->vcm_fw_ver == 0) {
			rc = getFWVersion(s_ctrl);
			if (rc >= 0) {
				dev_set_drvdata(&s_ctrl->pdev->dev, s_ctrl);
				rc = sysfs_create_groups(
					&s_ctrl->pdev->dev.kobj,
					sensor_fw_dev_groups);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"failed to create sysfs");
					cam_sensor_power_down(s_ctrl);
					msleep(20);
					goto free_power_settings;
				}
			}
		}

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "fail in Sensor Power Down");
			goto free_power_settings;
		}
		/* reset hw sync control */
		memset(&s_ctrl->hw_sync_ctrl, 0,
		       sizeof(struct cam_sensor_hw_sync_ctrl));
		/*
		 * Set probe succeeded flag to 1 so that no other camera shall
		 * probed on this slot
		 */
		s_ctrl->is_probe_succeed = 1;
		s_ctrl->sensor_state = CAM_SENSOR_INIT;
	}
		break;
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev sensor_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if ((s_ctrl->is_probe_succeed == 0) ||
			(s_ctrl->sensor_state != CAM_SENSOR_INIT)) {
			CAM_WARN(CAM_SENSOR,
				"Not in right state to aquire %d",
				s_ctrl->sensor_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_SENSOR, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&sensor_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(sensor_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed Copying from user");
			goto release_mutex;
		}

		bridge_params.session_hdl = sensor_acq_dev.session_handle;
		bridge_params.ops = &s_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = s_ctrl;

		sensor_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		s_ctrl->bridge_intf.device_hdl = sensor_acq_dev.device_handle;
		s_ctrl->bridge_intf.session_hdl = sensor_acq_dev.session_handle;

		CAM_DBG(CAM_SENSOR, "Device Handle: %d",
			sensor_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Sensor Power up failed");
			goto release_mutex;
		}
		/* reset hw sync control */
		memset(&s_ctrl->hw_sync_ctrl, 0,
		       sizeof(struct cam_sensor_hw_sync_ctrl));

		if (s_ctrl->soc_info.index == IR_MASTER &&
			s_ctrl->hw_version >= 2)
			cam_register_laser_irq(s_ctrl, LASER_STATUS);

		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		s_ctrl->last_flush_req = 0;
		CAM_INFO(CAM_SENSOR,
			"CAM_ACQUIRE_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_RELEASE_DEV: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to release : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_SENSOR,
				"Device [%d] still active on link 0x%x",
				s_ctrl->sensor_state,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Sensor Power Down failed");
			goto release_mutex;
		}

		cam_sensor_release_per_frame_resource(s_ctrl);
		cam_sensor_release_stream_rsc(s_ctrl);
		if (s_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_SENSOR,
				"Invalid Handles: link hdl: %d device hdl: %d",
				s_ctrl->bridge_intf.device_hdl,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed in destroying the device hdl");

		if (s_ctrl->soc_info.index == IR_MASTER &&
			s_ctrl->hw_version >= 2)
			cam_release_laser_irq(s_ctrl, LASER_STATUS);

		s_ctrl->bridge_intf.device_hdl = -1;
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.session_hdl = -1;

		s_ctrl->sensor_state = CAM_SENSOR_INIT;

		CAM_INFO(CAM_SENSOR,
			"CAM_RELEASE_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
		s_ctrl->streamon_count = 0;
		s_ctrl->streamoff_count = 0;
		s_ctrl->last_flush_req = 0;
		/* reset hw sync control */
		memset(&s_ctrl->hw_sync_ctrl, 0,
		       sizeof(struct cam_sensor_hw_sync_ctrl));
	}
		break;
	case CAM_QUERY_CAP: {
		struct  cam_sensor_query_cap sensor_cap;

		cam_sensor_query_cap(s_ctrl, &sensor_cap);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_cap, sizeof(struct  cam_sensor_query_cap))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_START_DEV: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to start : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		/* Disable strobe when peer IR is streaming on */
		if ((((s_ctrl->soc_info.index == IR_MASTER) &&
			((sensor_status.streamon_mask &
				IR_SLAVE_STREAMON_MASK) != 0)) ||
			((s_ctrl->soc_info.index == IR_SLAVE) &&
			((sensor_status.streamon_mask &
				IR_MASTER_STREAMON_MASK) != 0))) &&
			s_ctrl->hw_version < 2) {
			rc = cam_sensor_update_ir_cams_strobe(s_ctrl, false);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"failed to disable strobe");
			}

		}

		if (s_ctrl->i2c_data.streamon_settings.is_settings_valid &&
			(s_ctrl->i2c_data.streamon_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply streamon settings");
				goto release_mutex;
			}
		}

		sensor_status.streamon_mask |= (1 << s_ctrl->soc_info.index);
		s_ctrl->strobeType = STROBE_NONE;
		if (s_ctrl->hw_version >= 2 &&
			(s_ctrl->soc_info.index == IR_MASTER ||
			s_ctrl->soc_info.index == IR_SLAVE)) {
			/* Set up static laser for
			 * common slave address device
			 */
			s_ctrl->first_strobe_frame =
				s_ctrl->soc_info.index == IR_MASTER ? 2 : 1;
		} else
			s_ctrl->first_strobe_frame = 0;

		s_ctrl->sensor_state = CAM_SENSOR_START;
		CAM_INFO(CAM_SENSOR,
			"CAM_START_DEV Success, sensor_id:0x%x,"
			"sensor_slave_addr:0x%x index: %d",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr,
			s_ctrl->soc_info.index);
	}
		break;
	case CAM_STOP_DEV: {
		if (s_ctrl->sensor_state != CAM_SENSOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to stop : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->i2c_data.streamoff_settings.is_settings_valid &&
			(s_ctrl->i2c_data.streamoff_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
				"cannot apply streamoff settings");
			}
		}

		sensor_status.streamon_mask &= ~(1 << s_ctrl->soc_info.index);
		s_ctrl->strobeType = STROBE_NONE;
		s_ctrl->first_strobe_frame = 0;

		cam_sensor_release_per_frame_resource(s_ctrl);

		s_ctrl->last_flush_req = 0;
		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		CAM_INFO(CAM_SENSOR,
			"CAM_STOP_DEV Success, sensor_id:0x%x,"
			"sensor_slave_addr:0x%x index: %d",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr,
			s_ctrl->soc_info.index);
	}
		break;
	case CAM_CONFIG_DEV: {
		rc = cam_sensor_i2c_pkt_parse(s_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed i2c pkt parse: %d", rc);
			goto release_mutex;
		}
		if (s_ctrl->i2c_data.init_settings.is_settings_valid &&
			(s_ctrl->i2c_data.init_settings.request_id == 0)) {

			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG);

			s_ctrl->i2c_data.init_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply init settings");
				delete_request(&s_ctrl->i2c_data.init_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.init_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the Init settings");
				goto release_mutex;
			}
		}

		if (s_ctrl->i2c_data.config_settings.is_settings_valid &&
			(s_ctrl->i2c_data.config_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);

			s_ctrl->i2c_data.config_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply config settings");
				delete_request(
					&s_ctrl->i2c_data.config_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.config_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the config settings");
				goto release_mutex;
			}
			s_ctrl->sensor_state = CAM_SENSOR_CONFIG;
		}
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
		goto release_mutex;
	}

release_mutex:
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!info)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(info->dev_hdl);

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	info->dev_id = CAM_REQ_MGR_DEVICE_SENSOR;
	strlcpy(info->name, CAM_SENSOR_NAME, sizeof(info->name));
	if (s_ctrl->pipeline_delay >= 1 && s_ctrl->pipeline_delay <= 3)
		info->p_delay = s_ctrl->pipeline_delay;
	else
		info->p_delay = 2;
	info->trigger = CAM_TRIGGER_POINT_SOF;

	return rc;
}

int cam_sensor_establish_link(struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!link)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(link->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&s_ctrl->cam_sensor_mutex);
	if (link->link_enable) {
		s_ctrl->bridge_intf.link_hdl = link->link_hdl;
		s_ctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&s_ctrl->cam_sensor_mutex);

	return 0;
}

int cam_sensor_power(struct v4l2_subdev *sd, int on)
{
	struct cam_sensor_ctrl_t *s_ctrl = v4l2_get_subdevdata(sd);

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if (!on && s_ctrl->sensor_state == CAM_SENSOR_START) {
		cam_sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
	}
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return 0;
}

int cam_sensor_power_up(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_camera_slave_info *slave_info;
	struct cam_hw_soc_info *soc_info =
		&s_ctrl->soc_info;
	struct cam_sensor_i2c_reg_setting i2c_settings;
	struct cam_sensor_i2c_reg_array reg_setting[2];
	uint16_t default_sid = 0;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!power_info || !slave_info) {
		CAM_ERR(CAM_SENSOR, "failed: %pK %pK", power_info, slave_info);
		return -EINVAL;
	}

	if (s_ctrl->bob_pwm_switch) {
		rc = cam_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, true);
		if (rc) {
			CAM_WARN(CAM_SENSOR,
			"BoB PWM setup failed rc: %d", rc);
			rc = 0;
		}
	}

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "power up the core is failed:%d", rc);
		return rc;
	}

	rc = camera_io_init(&(s_ctrl->io_master_info));
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "cci_init failed: rc: %d", rc);
		cam_sensor_power_down(s_ctrl);
		return rc;
	}

	if (s_ctrl->hw_version < 2 &&
		(soc_info->index == IR_MASTER || soc_info->index == IR_SLAVE)) {
		if (!s_ctrl->peer_ir_info.cci_client) {
			CAM_ERR(CAM_SENSOR, "peer IR io info is empty");
			cam_sensor_power_down(s_ctrl);
			return -EFAULT;
		}
		rc = camera_io_init(&(s_ctrl->peer_ir_info));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"cci_init for peer ir failed: rc: %d", rc);
			cam_sensor_power_down(s_ctrl);
			return rc;
		}
	}

	if (s_ctrl->soc_info.index != IR_MASTER)
		goto done;

	default_sid = s_ctrl->io_master_info.cci_client->sid;
	s_ctrl->io_master_info.cci_client->sid = 0xC0 >> 1;
	reg_setting[0].reg_addr = 0x0103;
	reg_setting[0].reg_data = 0x01;
	reg_setting[0].delay = 0;
	reg_setting[0].data_mask = 0xFF;
	reg_setting[1].reg_addr = 0x302B;
	reg_setting[1].reg_data = 0xE4;
	reg_setting[1].delay = 0;
	reg_setting[1].data_mask = 0xFF;
	i2c_settings.reg_setting = reg_setting;
	i2c_settings.size = 2;
	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_settings.delay = 0;
	rc = camera_io_dev_write(&(s_ctrl->io_master_info),
		&i2c_settings);
	s_ctrl->io_master_info.cci_client->sid = default_sid;

	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"Reprogram IR_MASTER slave address failed rc = %d",
			rc);
		cam_sensor_power_down(s_ctrl);
		return rc;
	} else {
		CAM_DBG(CAM_SENSOR,
			"Reprogram IR_MASTER slave address success");
	}

done:
	return rc;
}

int cam_sensor_power_down(struct cam_sensor_ctrl_t *s_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info *soc_info;
	int rc = 0;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: s_ctrl %pK", s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	soc_info = &s_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_SENSOR, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "power down the core is failed:%d", rc);
		return rc;
	}

	if (s_ctrl->bob_pwm_switch) {
		rc = cam_sensor_bob_pwm_mode_switch(soc_info,
			s_ctrl->bob_reg_index, false);
		if (rc) {
			CAM_WARN(CAM_SENSOR,
				"BoB PWM setup failed rc: %d", rc);
			rc = 0;
		}
	}

	camera_io_release(&(s_ctrl->io_master_info));

	if (s_ctrl->hw_version < 2 &&
		(soc_info->index == IR_MASTER || soc_info->index == IR_SLAVE)) {
		if (!s_ctrl->peer_ir_info.cci_client)
			return rc;
		camera_io_release(&(s_ctrl->peer_ir_info));
	}

	return rc;
}

int cam_sensor_apply_settings(struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id, enum cam_sensor_packet_opcodes opcode)
{
	int rc = 0, offset, i;
	uint64_t top = 0, del_req_id = 0;
	struct i2c_settings_array *i2c_set = NULL;
	struct i2c_settings_list *i2c_list;

	if (req_id == 0) {
		switch (opcode) {
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
			i2c_set = &s_ctrl->i2c_data.streamon_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
			i2c_set = &s_ctrl->i2c_data.init_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
			i2c_set = &s_ctrl->i2c_data.config_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
			i2c_set = &s_ctrl->i2c_data.streamoff_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER: {
			i2c_set = &s_ctrl->i2c_data.master_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_SLAVE: {
			i2c_set = &s_ctrl->i2c_data.slave_settings;
			break;
		}
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE:
		case CAM_SENSOR_PACKET_OPCODE_SENSOR_PROBE:
		default:
			return 0;
		}
		if (i2c_set->is_settings_valid == 1) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head), list) {
				rc = cam_sensor_i2c_modes_util(
					&(s_ctrl->io_master_info),
					i2c_list);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"Failed to apply settings: %d",
						rc);
					return rc;
				}
			}
		}
	} else {
		offset = req_id % MAX_PER_FRAME_ARRAY;
		i2c_set = &(s_ctrl->i2c_data.per_frame[offset]);
		if (i2c_set->is_settings_valid == 1 &&
			i2c_set->request_id == req_id) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head), list) {
				rc = cam_sensor_i2c_modes_util(
					&(s_ctrl->io_master_info),
					i2c_list);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR,
						"Failed to apply settings: %d",
						rc);
					return rc;
				}
			}
		} else {
			CAM_DBG(CAM_SENSOR,
				"Invalid/NOP request to apply: %lld", req_id);
		}

		/* Change the logic dynamically */
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((req_id >=
				s_ctrl->i2c_data.per_frame[i].request_id) &&
				(top <
				s_ctrl->i2c_data.per_frame[i].request_id) &&
				(s_ctrl->i2c_data.per_frame[i].
				is_settings_valid == 1)) {
				del_req_id = top;
				top = s_ctrl->i2c_data.per_frame[i].request_id;
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			return rc;

		CAM_DBG(CAM_SENSOR, "top: %llu, del_req_id:%llu",
			top, del_req_id);

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((del_req_id >
				 s_ctrl->i2c_data.per_frame[i].request_id) && (
				 s_ctrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				s_ctrl->i2c_data.per_frame[i].request_id = 0;
				rc = delete_request(
					&(s_ctrl->i2c_data.per_frame[i]));
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"Delete request Fail:%lld rc:%d",
						del_req_id, rc);
			}
		}
	}

	return rc;
}

int32_t cam_sensor_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!apply)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}
	CAM_DBG(CAM_REQ, " Sensor update req id: %lld", apply->request_id);
	trace_cam_apply_req("Sensor", apply->request_id);
	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if (s_ctrl->soc_info.index <= REAR_TELE)
		cam_sensor_sync_trigger(s_ctrl, apply->request_id);

	rc = cam_sensor_apply_settings(s_ctrl, apply->request_id,
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int32_t cam_sensor_flush_request(struct cam_req_mgr_flush_request *flush_req)
{
	int32_t rc = 0, i;
	uint32_t cancel_req_id_found = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct i2c_settings_array *i2c_set = NULL;

	if (!flush_req)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(flush_req->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	if (s_ctrl->sensor_state != CAM_SENSOR_START ||
		s_ctrl->sensor_state != CAM_SENSOR_CONFIG) {
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return rc;
	}

	if (s_ctrl->i2c_data.per_frame == NULL) {
		CAM_ERR(CAM_SENSOR, "i2c frame data is NULL");
		mutex_unlock(&(s_ctrl->cam_sensor_mutex));
		return -EINVAL;
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		s_ctrl->last_flush_req = flush_req->req_id;
		CAM_DBG(CAM_SENSOR, "last reqest to flush is %lld",
			flush_req->req_id);
	}

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
		i2c_set = &(s_ctrl->i2c_data.per_frame[i]);

		if ((flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ)
				&& (i2c_set->request_id != flush_req->req_id))
			continue;

		if (i2c_set->is_settings_valid == 1) {
			rc = delete_request(i2c_set);
			if (rc < 0)
				CAM_ERR(CAM_SENSOR,
					"delete request: %lld rc: %d",
					i2c_set->request_id, rc);

			if (flush_req->type ==
				CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
				cancel_req_id_found = 1;
				break;
			}
		}
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ &&
		!cancel_req_id_found)
		CAM_DBG(CAM_SENSOR,
			"Flush request id:%lld not found in the pending list",
			flush_req->req_id);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

int cam_sensor_set_strobe(struct cam_req_mgr_apply_request *apply, bool enable)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	if (!apply)
		return -EINVAL;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		return -EINVAL;
	}

	if (s_ctrl->hw_version >= 2)
		return 0;

	if (s_ctrl->strobeType == STROBE_NONE)
		return 0;

	if (s_ctrl->soc_info.index == IR_SLAVE)
		rc = cam_sensor_update_ir_cams_strobe(s_ctrl, enable);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR,
			"failed to set strobe");

	switch (s_ctrl->strobeType) {
	case STROBE_ALTERNATIVE:
		if (s_ctrl->soc_info.index == IR_SLAVE) {
			s_ctrl->first_strobe_frame = apply->frame_count + 2;
		} else
			s_ctrl->first_strobe_frame = apply->frame_count + 1;
		break;
	case STROBE_SYNCHRONIZE:
		if (s_ctrl->soc_info.index == IR_SLAVE) {
			s_ctrl->first_strobe_frame = apply->frame_count + 1;
		} else
			s_ctrl->first_strobe_frame = apply->frame_count + 2;
		break;
	default:
		CAM_ERR(CAM_SENSOR,
			"Unsupport strobe type, should not be here");
	}

	s_ctrl->strobeType = STROBE_NONE;

	return rc;
}

int cam_sensor_tag_laser_type(
	struct cam_req_mgr_message *msg,
	int32_t dev_hdl)
{
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	enum laser_tag_type laserTag = LASER_TAG_NONE;
	bool is_both_ir_streamon;
	bool is_ir_camera;

	s_ctrl = (struct cam_sensor_ctrl_t *)
		cam_get_device_priv(dev_hdl);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "Device data is NULL");
		msg->u.frame_msg.laser_tag = laserTag;
		return -EINVAL;
	}

	is_both_ir_streamon =
		(((sensor_status.streamon_mask &
			IR_MASTER_STREAMON_MASK) != 0) &&
		((sensor_status.streamon_mask &
			IR_SLAVE_STREAMON_MASK) != 0));
	is_ir_camera =
		(s_ctrl->soc_info.index == IR_SLAVE ||
		s_ctrl->soc_info.index == IR_MASTER);

	if (sensor_status.is_strobe_disabled)
		goto out;

	if (is_ir_camera && is_both_ir_streamon) {
		if (s_ctrl->hw_version < 2) {
			/* Tagging laser by strobe frame count after reset */
			if (s_ctrl->soc_info.index == IR_MASTER) {
				laserTag = ((msg->u.frame_msg.frame_id -
					s_ctrl->first_strobe_frame) % 2) ?
					LASER_TAG_DOT : LASER_TAG_FLOOD;
			} else
				laserTag = ((msg->u.frame_msg.frame_id -
					s_ctrl->first_strobe_frame) % 2) ?
					LASER_TAG_FLOOD : LASER_TAG_DOT;
		} else
			/* Tagging laser by IRQ signal */
			laserTag = laser_tag_indicator;
	} else if (is_ir_camera && !is_both_ir_streamon) {
		/* Static tagging for single IR case */
		if (s_ctrl->soc_info.index == IR_MASTER) {
			laserTag = ((msg->u.frame_msg.frame_id -
			s_ctrl->first_strobe_frame) % 2) ?
				LASER_TAG_NONE : LASER_TAG_FLOOD;
		} else
			laserTag = ((msg->u.frame_msg.frame_id -
			s_ctrl->first_strobe_frame) % 2) ?
				LASER_TAG_NONE : LASER_TAG_DOT;
	}
out:
	CAM_DBG(CAM_SENSOR,
		"sensor: %d frame_count: %d requestId: %d laserTag: %d",
		s_ctrl->soc_info.index,
		msg->u.frame_msg.frame_id,
		msg->u.frame_msg.request_id,
		laserTag);
	msg->u.frame_msg.laser_tag = laserTag;

	return 0;
}
