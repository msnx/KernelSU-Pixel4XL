/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "cam_req_mgr_core.h"
#include "cam_req_mgr_dev.h"
#include "cam_sensor_core.h"
#include "cam_sensor_dev.h"
#include "cam_sensor_hw_sync.h"
#include "cam_sensor_soc.h"

#define RATIO_SCALE    1000
#define MAX_SYNC_RATIO  (5 * RATIO_SCALE)

static char *sensor_names[] = {
	"REAR_WIDE",
	"REAR_TELE",
	"FRONT_CAM",
	"IR_MASTER",
	"IR_SLAVE"
};

static int32_t cam_sensor_get_frame_length(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_array *i2c_set);

static uint32_t cam_sensor_get_frame_length_ratio(
	struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id);

static void cam_sensor_filter_frame_len(
	struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id,
	uint32_t ratio);

static int32_t cam_sensor_apply_frame_len(
	struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t ratio);

static int32_t cam_sensor_apply_exposure(
	struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t ratio);

static int32_t cam_sensor_write_regs(struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t *addr, uint32_t *data, uint32_t len);

static int32_t cam_sensor_get_fll_reg(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_sensor_i2c_reg_setting *i2c_settings)
{
	int32_t i, fll = 0;
	struct cam_sensor_i2c_reg_array *reg_setting = NULL;
	struct sensor_hw_sync_reg_info *reg_info = NULL;

	if (!s_ctrl)
		return fll;

	reg_info = &s_ctrl->hw_sync_ctrl.regs;

	for (i = 0; i < i2c_settings->size; i++) {
		reg_setting = &i2c_settings->reg_setting[i];
		if (reg_setting->reg_addr == reg_info->fll_hi_addr) {
			fll += reg_setting->reg_data << 8;
		} else if (reg_setting->reg_addr == reg_info->fll_lo_addr) {
			fll += reg_setting->reg_data;
			break;
		}
	}
	return fll;
}

static int32_t cam_sync_handle_sync_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_cmd_set_sensor_sync *cmd_set_sync)
{
	int32_t rc = 0;
	uint32_t cmd = 0;
	uint32_t flags = 0;
	struct sensor_hw_sync_reg_info *reg_info = NULL;

	cmd = cmd_set_sync->cmd_type;
	flags = cmd_set_sync->flags;
	reg_info = &s_ctrl->hw_sync_ctrl.regs;

	CAM_INFO(CAM_SENSOR, "cmd %d flags %d role %d data_handle 0x%x",
		cmd, flags,
		cmd_set_sync->role, cmd_set_sync->data_handle);

	if (cmd == SENSOR_SYNC_CMD_TYPE_INIT) {
		if (cmd_set_sync->data_handle) {
			rc = copy_from_user(reg_info,
				u64_to_user_ptr(cmd_set_sync->data_handle),
				sizeof(struct sensor_hw_sync_reg_info));
			if (rc) {
				CAM_ERR(CAM_SENSOR,
					"sensor_sync: copy from user failed!");
				goto out;
			}
		}
		s_ctrl->hw_sync_ctrl.fll = cmd_set_sync->fll;
		s_ctrl->hw_sync_ctrl.margin = cmd_set_sync->margin;

		s_ctrl->hw_sync_ctrl.is_master =
			(cmd_set_sync->role == CAM_SENSOR_SYNC_MASTER);
		s_ctrl->hw_sync_ctrl.is_hwsync =
			(cmd_set_sync->role != CAM_SENSOR_SYNC_NONE);
		s_ctrl->hw_sync_ctrl.role_switch_en =
			cmd_set_sync->flags & SENSOR_SYNC_CMD_FLAG_ROLE_SWITCH;
		s_ctrl->hw_sync_ctrl.fps_unify_en =
			cmd_set_sync->flags &
				SENSOR_SYNC_CMD_FLAG_ROLE_FPS_UNIFY;
		CAM_INFO(CAM_SENSOR,
			"%s is_hwsync %d, is_master %d, init fll %d margin %d",
			sensor_names[s_ctrl->soc_info.index],
			s_ctrl->hw_sync_ctrl.is_hwsync,
			s_ctrl->hw_sync_ctrl.is_master,
			s_ctrl->hw_sync_ctrl.fll,
			s_ctrl->hw_sync_ctrl.margin);

	} else if (cmd == SENSOR_SYNC_CMD_TYPE_UPDATE) {
		if (!(cmd_set_sync->flags & SENSOR_SYNC_CMD_FLAG_ROLE_UPDATE))
			goto out;
		if (!s_ctrl->hw_sync_ctrl.is_hwsync ||
			s_ctrl->hw_sync_ctrl.role_switch_en)
			goto out;
		s_ctrl->hw_sync_ctrl.is_master =
			(cmd_set_sync->role == CAM_SENSOR_SYNC_MASTER);
		CAM_INFO(CAM_SENSOR, "%s role change. is_master %d",
			sensor_names[s_ctrl->soc_info.index],
			s_ctrl->hw_sync_ctrl.is_master);
		if (s_ctrl->hw_sync_ctrl.is_master)
			cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER);
		else
			cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_SLAVE);
	} else {
		rc = -EINVAL;
	}

out:
	return rc;
}

struct cam_sensor_ctrl_t *cam_sensor_get_sensor_ctrl(
	struct cam_req_mgr_core_link *link)
{
	int i;
	struct cam_req_mgr_connected_device *dev = NULL;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	for (i = 0; i < link->num_devs; i++) {
		dev = &link->l_dev[i];
		if (!dev)
			continue;
		if (dev->dev_info.dev_id != CAM_REQ_MGR_DEVICE_SENSOR)
			continue;
		s_ctrl = (struct cam_sensor_ctrl_t *)
			cam_get_device_priv(dev->dev_info.dev_hdl);

		if (!s_ctrl)
			CAM_ERR(CAM_SENSOR, "Invalid sensor dev_hdl.",
				dev->dev_info.dev_hdl);
		else
			return s_ctrl;
	}
	return s_ctrl;
}

int cam_sensor_sync_pkt_parse(struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet)
{
	int32_t i, rc = 0;
	uint32_t total_cmd_buf_in_bytes = 0;
	uintptr_t generic_ptr;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	size_t len_of_buff = 0;
	uint32_t *offset = NULL;
	struct cam_cmd_set_sensor_sync *cmd_set_sync = NULL;


	offset = (uint32_t *)&csl_packet->payload;
	offset += csl_packet->cmd_buf_offset / sizeof(uint32_t);
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	CAM_DBG(CAM_SENSOR, "num_cmd_buf: %d", csl_packet->num_cmd_buf);
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		total_cmd_buf_in_bytes = cmd_desc[i].length;
		if (!total_cmd_buf_in_bytes) {
			CAM_ERR(CAM_SENSOR,
			"Empty cmd buf found in sync packet");
			return -EINVAL;
		}

		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
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
			cmd_desc[i].offset);

		cmd_set_sync = (struct cam_cmd_set_sensor_sync *)offset;
		rc = cam_sync_handle_sync_cmd(s_ctrl, cmd_set_sync);

	}

	return rc;
}

void cam_sensor_sync_init(struct cam_req_mgr_core_session *cam_session)
{
	uint32_t i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_sensor_ctrl_t *s_ctrl_sync = NULL;

	if (cam_session->num_links == 1) {
		s_ctrl = cam_sensor_get_sensor_ctrl(
			cam_session->links[0]);
		if (!s_ctrl)
			return;
		if (s_ctrl->soc_info.index > REAR_TELE)
			return;
		/* set sensor as master */
		cam_sensor_apply_settings(s_ctrl, 0,
			CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER);
		/* reset peer sensor */
		s_ctrl->hw_sync_ctrl.peer = NULL;
		return;
	}
	/* Currently only support 2 links */
	if (cam_session->num_links > 2)
		return;

	for (i = 0; i < cam_session->num_links; i++) {
		s_ctrl = cam_sensor_get_sensor_ctrl(
			cam_session->links[i]);
		if (!s_ctrl)
			continue;
		if (s_ctrl->soc_info.index > REAR_TELE)
			continue;
		s_ctrl->hw_sync_ctrl.last_sof_timestamp = 0;
		s_ctrl->hw_sync_ctrl.req_id = -1;
		s_ctrl_sync = cam_sensor_get_sensor_ctrl(
			cam_session->links[1 - i]);
		CAM_INFO(CAM_SENSOR, "%s, peer %s",
			sensor_names[s_ctrl->soc_info.index],
			sensor_names[s_ctrl_sync->soc_info.index]);
		s_ctrl->hw_sync_ctrl.peer = s_ctrl_sync;
	}
}

void cam_sensor_sync_deinit(struct cam_req_mgr_core_session *cam_session)
{
	uint32_t i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;

	/* Currently only support 2 links */
	if (cam_session->num_links != 2)
		return;

	for (i = 0; i < cam_session->num_links; i++) {
		s_ctrl = cam_sensor_get_sensor_ctrl(
			cam_session->links[i]);
		if (!s_ctrl)
			continue;
		if (s_ctrl->soc_info.index > REAR_TELE)
			continue;
		/* set sensor as master */
		cam_sensor_apply_settings(s_ctrl, 0,
			CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER);
		/* reset hw sync control */
		memset(&s_ctrl->hw_sync_ctrl, 0,
			sizeof(struct cam_sensor_hw_sync_ctrl));
	}
}

void cam_sensor_sof_notify(struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id, uint64_t sof_timestamp)
{
	struct cam_sensor_ctrl_t *s_ctrl_sync;

	if (!s_ctrl)
		return;
	s_ctrl_sync = s_ctrl->hw_sync_ctrl.peer;

	CAM_DBG(CAM_SENSOR, "%s req_id %d frame duration %d",
		sensor_names[s_ctrl->soc_info.index],
		req_id,
		sof_timestamp - s_ctrl->hw_sync_ctrl.last_sof_timestamp);
	if (s_ctrl->hw_sync_ctrl.is_stream_on) {
		s_ctrl->hw_sync_ctrl.last_sof_timestamp = sof_timestamp;
		s_ctrl->hw_sync_ctrl.req_id = req_id;
	}

	if (!s_ctrl_sync)
		return;
	/* if same req_id */
	if (req_id == s_ctrl_sync->hw_sync_ctrl.req_id) {
		CAM_DBG(CAM_SENSOR, "%s wide-tele %d",
			sensor_names[s_ctrl->soc_info.index],
			sof_timestamp -
			s_ctrl_sync->hw_sync_ctrl.last_sof_timestamp);
	}
}

void cam_sensor_sync_trigger(struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id)
{
	uint32_t ratio = 0;
	struct cam_sensor_ctrl_t *s_ctrl_sync;

	if (!s_ctrl)
		return;
	s_ctrl_sync = s_ctrl->hw_sync_ctrl.peer;

	CAM_DBG(CAM_SENSOR, "%s, is_master %d",
		sensor_names[s_ctrl->soc_info.index],
		s_ctrl->hw_sync_ctrl.is_master);
	if (!s_ctrl->hw_sync_ctrl.is_hwsync)
		return;

	ratio = cam_sensor_get_frame_length_ratio(s_ctrl, req_id);
	/* if no fll in sensor packet */
	if (!ratio)
		return;

	if (!s_ctrl_sync) {
		/* filter the frame length with ratio */
		s_ctrl->hw_sync_ctrl.ratio = ratio;
		cam_sensor_filter_frame_len(s_ctrl, req_id, ratio);
		return;
	}
	/* if sensor's SOF comes first */
	if (s_ctrl->hw_sync_ctrl.req_id > s_ctrl_sync->hw_sync_ctrl.req_id) {
		s_ctrl_sync->hw_sync_ctrl.ratio = ratio;
		/* apply the ratio on sync link */
		cam_sensor_apply_frame_len(s_ctrl_sync, ratio);
	}
	/* filter the frame length with ratio */
	cam_sensor_filter_frame_len(s_ctrl, req_id, ratio);
}

static int32_t cam_sensor_get_frame_length(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list = NULL;
	int32_t fll = 0;

	list_for_each_entry(i2c_list, &(i2c_set->list_head), list) {
		if (i2c_list->op_code != CAM_SENSOR_I2C_WRITE_RANDOM &&
			i2c_list->op_code != CAM_SENSOR_I2C_WRITE_SEQ &&
			i2c_list->op_code != CAM_SENSOR_I2C_WRITE_BURST)
			continue;
		fll = cam_sensor_get_fll_reg(s_ctrl, &i2c_list->i2c_settings);
		if (fll > 0)
			return fll;
	}
	return fll;
}

static uint32_t cam_sensor_get_frame_length_ratio(
	struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id)
{
	int offset;
	uint32_t fll_val = 0;
	struct i2c_settings_array *i2c_set = NULL;
	uint32_t ratio = 0;

	if (!s_ctrl)
		goto out;

	offset = req_id % MAX_PER_FRAME_ARRAY;
	i2c_set = &(s_ctrl->i2c_data.per_frame[offset]);
	if (i2c_set->is_settings_valid != 1)
		goto out;
	if (i2c_set->request_id != req_id)
		goto out;

	fll_val = cam_sensor_get_frame_length(s_ctrl, i2c_set);

	if (s_ctrl->hw_sync_ctrl.fll > 0 && fll_val > 0) {
		ratio = (RATIO_SCALE * fll_val) / s_ctrl->hw_sync_ctrl.fll;
		CAM_DBG(CAM_SENSOR, "fll %d (%d) ratio %d",
			fll_val, s_ctrl->hw_sync_ctrl.fll, ratio);
	}
out:
	return ratio;
}

static void cam_sensor_filter_frame_len(
	struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id,
	uint32_t ratio)
{
	int offset, i;
	uint32_t reg_addr;
	uint32_t fll;
	struct i2c_settings_array *i2c_set = NULL;
	struct i2c_settings_list *i2c_list = NULL;
	struct cam_sensor_i2c_reg_array *reg_setting = NULL;
	struct sensor_hw_sync_reg_info *reg_info = NULL;

	if (!s_ctrl || ratio < RATIO_SCALE)
		return;
	if (!s_ctrl->hw_sync_ctrl.is_hwsync)
		return;

	fll = (s_ctrl->hw_sync_ctrl.fll * ratio) / RATIO_SCALE;
	if (s_ctrl->hw_sync_ctrl.is_master)
		fll += s_ctrl->hw_sync_ctrl.margin;

	CAM_DBG(CAM_SENSOR, "fll %d (%d) ratio %d",
		fll, s_ctrl->hw_sync_ctrl.fll, ratio);

	reg_info = &s_ctrl->hw_sync_ctrl.regs;
	offset = req_id % MAX_PER_FRAME_ARRAY;
	i2c_set = &(s_ctrl->i2c_data.per_frame[offset]);
	if (i2c_set->is_settings_valid != 1 || i2c_set->request_id != req_id) {
		CAM_DBG(CAM_SENSOR, "req_id %d valid %d",
			req_id, i2c_set->is_settings_valid);
		return;
	}

	list_for_each_entry(i2c_list, &(i2c_set->list_head), list) {
		if (i2c_list->op_code != CAM_SENSOR_I2C_WRITE_RANDOM &&
			i2c_list->op_code != CAM_SENSOR_I2C_WRITE_SEQ &&
			i2c_list->op_code != CAM_SENSOR_I2C_WRITE_BURST)
			continue;
		for (i = 0; i < i2c_list->i2c_settings.size; i++) {
			reg_setting =
				&i2c_list->i2c_settings.reg_setting[i];
			reg_addr = reg_setting->reg_addr;
			if (reg_addr == reg_info->fll_lo_addr)
				reg_setting->reg_data = fll & 0xFF;
			else if (reg_addr == reg_info->fll_hi_addr)
				reg_setting->reg_data = fll >> 8;
		}
	}
}

static int32_t cam_sensor_write_regs(struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t *addr, uint32_t *data, uint32_t len)
{
	struct cam_sensor_i2c_reg_setting i2c_settings;
	struct cam_sensor_i2c_reg_array reg_setting[3];
	int32_t idx, rc = 0;

	if (!s_ctrl || len > 3)
		return -EINVAL;

	for (idx = 0; idx < len; idx++) {
		reg_setting[idx].reg_addr = addr[idx];
		reg_setting[idx].reg_data = data[idx];
		reg_setting[idx].delay = 0;
		reg_setting[idx].data_mask = 0xFF;
	}
	i2c_settings.reg_setting = reg_setting;
	i2c_settings.size = len;
	i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_settings.delay = 0;
	rc = camera_io_dev_write(&(s_ctrl->io_master_info),
		&i2c_settings);
	return rc;
}

static int32_t cam_sensor_apply_frame_len(
	struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t ratio)
{
	uint32_t fll;
	int32_t rc = 0;
	struct sensor_hw_sync_reg_info *reg_info = NULL;
	uint32_t addr[3], data[3];

	if (!s_ctrl)
		return -EINVAL;
	reg_info = &s_ctrl->hw_sync_ctrl.regs;

	fll = (s_ctrl->hw_sync_ctrl.fll * ratio) / RATIO_SCALE;
	if (s_ctrl->hw_sync_ctrl.is_hwsync && s_ctrl->hw_sync_ctrl.is_master)
		fll += s_ctrl->hw_sync_ctrl.margin;
	addr[0] = reg_info->fll_hi_addr;
	data[0] = fll >> 8;
	addr[1] = reg_info->fll_lo_addr;
	data[1] = fll & 0xFF;
	addr[2] = reg_info->gph_addr;
	data[2] = 0;
	CAM_DBG(CAM_SENSOR, "fll %d (%d) ratio %d",
		fll, s_ctrl->hw_sync_ctrl.fll, ratio);
	rc = cam_sensor_write_regs(s_ctrl, addr, data, 3);
	return rc;
}

static int32_t cam_sensor_apply_exposure(
	struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t ratio)
{
	uint32_t fll, cit;
	int32_t rc = 0;
	struct sensor_hw_sync_reg_info *reg_info = NULL;
	uint32_t addr[3],  data[3];

	if (!s_ctrl)
		return -EINVAL;
	reg_info = &s_ctrl->hw_sync_ctrl.regs;

	fll = (s_ctrl->hw_sync_ctrl.fll * ratio) / RATIO_SCALE;
	if (s_ctrl->hw_sync_ctrl.is_hwsync && s_ctrl->hw_sync_ctrl.is_master)
		fll += s_ctrl->hw_sync_ctrl.margin;
	cit = fll - 20;
	addr[0] = reg_info->cit_hi_addr;
	data[0] = cit >> 8;
	addr[1] = reg_info->cit_lo_addr;
	data[1] = cit & 0xFF;
	addr[2] = reg_info->gph_addr;
	data[2] = 0;
	CAM_DBG(CAM_SENSOR, "cit %d (%d) ratio %d",
		cit, s_ctrl->hw_sync_ctrl.fll, ratio);
	rc = cam_sensor_write_regs(s_ctrl, addr, data, 3);
	return rc;
}

static void cam_sensor_apply_role_fll(
	struct cam_sensor_ctrl_t *s_ctrl,
	uint32_t fll_ratio)
{
	if (fll_ratio > 0) {
		/* apply fll ratio of peer sensor */
		cam_sensor_apply_frame_len(s_ctrl,
			fll_ratio);
	}
	if (s_ctrl->hw_sync_ctrl.is_master) {
		/* apply role */
		cam_sensor_apply_settings(s_ctrl,
			0,
			CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER);
	} else {
		cam_sensor_apply_settings(s_ctrl,
			0,
			CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_SLAVE);
	}
}

static void cam_sensor_handle_stream_on_peer_on(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_sensor_ctrl_t *s_ctrl_peer)
{
	if (!s_ctrl || !s_ctrl_peer)
		return;
	if (s_ctrl_peer->hw_sync_ctrl.ratio > MAX_SYNC_RATIO) {
		if (!s_ctrl_peer->hw_sync_ctrl.is_master) {
			CAM_ERR(CAM_SENSOR, "%s is slave and stream on",
				sensor_names[s_ctrl_peer->soc_info.index]);
			return;
		}
		/* change to no sync */
		s_ctrl->hw_sync_ctrl.is_master = 1;
		cam_sensor_apply_role_fll(s_ctrl, s_ctrl->hw_sync_ctrl.ratio);
		return;
	}
	/* if peer is master */
	if (s_ctrl_peer->hw_sync_ctrl.is_master) {
		/* change it to slave */
		s_ctrl->hw_sync_ctrl.is_master = 0;
		s_ctrl->hw_sync_ctrl.ratio = s_ctrl_peer->hw_sync_ctrl.ratio;
		/* apply fll ratio and role */
		cam_sensor_apply_role_fll(s_ctrl, s_ctrl->hw_sync_ctrl.ratio);
		/* apply exposure */
		cam_sensor_apply_exposure(s_ctrl, s_ctrl->hw_sync_ctrl.ratio);
	} else {
		CAM_ERR(CAM_SENSOR, "%s is slave and stream on",
			sensor_names[s_ctrl_peer->soc_info.index]);
	}
}

static void cam_sensor_handle_stream_on_peer_off(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_sensor_ctrl_t *s_ctrl_peer)
{
	if (!s_ctrl)
		return;

	if (!s_ctrl->hw_sync_ctrl.is_hwsync) {
		cam_sensor_apply_settings(s_ctrl, 0,
			CAM_SENSOR_PACKET_OPCODE_SENSOR_SYNC_MASTER);
		return;
	}

	if (!s_ctrl || !s_ctrl_peer)
		return;

	if (!s_ctrl->hw_sync_ctrl.is_master &&
		s_ctrl->hw_sync_ctrl.role_switch_en) {
		CAM_INFO(CAM_SENSOR, "Change %s as master to stream on first.",
			sensor_names[s_ctrl->soc_info.index]);
		s_ctrl->hw_sync_ctrl.is_master = 1;
		s_ctrl->hw_sync_ctrl.ratio = 1 * RATIO_SCALE;
		/* apply role and fll ratio */
		cam_sensor_apply_role_fll(s_ctrl, s_ctrl->hw_sync_ctrl.ratio);
	}
}

static void cam_sensor_handle_stream_on_off(
	struct cam_sensor_ctrl_t *s_ctrl,
	int val)
{
	struct cam_sensor_ctrl_t *s_ctrl_peer = NULL;

	if (!s_ctrl)
		return;

	CAM_INFO(CAM_SENSOR, "%s : [%d=>%d] is_master %d",
		sensor_names[s_ctrl->soc_info.index],
		s_ctrl->hw_sync_ctrl.is_stream_on, val,
		s_ctrl->hw_sync_ctrl.is_master);
	if (!val) {
		/* clear sof timestamp when stream off */
		s_ctrl->hw_sync_ctrl.last_sof_timestamp = 0;
	}
	s_ctrl_peer = s_ctrl->hw_sync_ctrl.peer;
	if (!s_ctrl_peer) {
		s_ctrl->hw_sync_ctrl.is_stream_on = val;
		return;
	}

	if (s_ctrl->hw_sync_ctrl.is_stream_on == val)
		return;

	if (!val) { /* on -> off */
		if (s_ctrl->hw_sync_ctrl.is_master &&
			s_ctrl_peer->hw_sync_ctrl.is_stream_on) {
			/* ask peer to be master */
			s_ctrl_peer->hw_sync_ctrl.is_master = 1;
			cam_sensor_apply_role_fll(s_ctrl_peer, 0);
			/* wait 5ms for the role switch */
			usleep_range(5000, 6000);
		}
	} else { /* off -> on */
		/* if peer is stream on */
		if (s_ctrl_peer->hw_sync_ctrl.is_stream_on)
			cam_sensor_handle_stream_on_peer_on(
				s_ctrl, s_ctrl_peer);
		else
			cam_sensor_handle_stream_on_peer_off(
				s_ctrl, s_ctrl_peer);
	}
	s_ctrl->hw_sync_ctrl.is_stream_on = val;
}

void cam_sensor_sync_audit(struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list)
{
	int i, val = 0, fll = 0, cit = 0;
	uint32_t ratio, reg_addr, fll_lo_idx = 0, fll_hi_idx = 0;
	uint32_t cit_lo_idx = 0, cit_hi_idx = 0;
	struct cam_sensor_i2c_reg_array *reg_setting = NULL;
	struct sensor_hw_sync_reg_info *reg_info = NULL;

	if (!s_ctrl)
		return;

	if (!s_ctrl->hw_sync_ctrl.is_hwsync)
		return;

	reg_info = &s_ctrl->hw_sync_ctrl.regs;

	for (i = 0; i < i2c_list->i2c_settings.size; i++) {
		reg_setting = &i2c_list->i2c_settings.reg_setting[i];
		reg_addr = reg_setting->reg_addr;
		if (reg_addr == reg_info->fll_hi_addr) {
			fll += reg_setting->reg_data << 8;
			fll_hi_idx = i;
		}
		if (reg_addr == reg_info->fll_lo_addr) {
			fll += reg_setting->reg_data;
			fll_lo_idx = i;
		}
		if (fll_hi_idx > 0 && fll_lo_idx > 0 &&
			s_ctrl->hw_sync_ctrl.fll > 0) {
			if (s_ctrl->hw_sync_ctrl.is_master)
				val = fll - s_ctrl->hw_sync_ctrl.margin;
			else
				val = fll;
			ratio = (RATIO_SCALE * val) / s_ctrl->hw_sync_ctrl.fll;
			s_ctrl->hw_sync_ctrl.ratio = ratio;
			fll_hi_idx = 0;
			fll_lo_idx = 0;
			CAM_DBG(CAM_SENSOR, "%s fll %d (%d) ratio %d",
				sensor_names[s_ctrl->soc_info.index],
				fll, s_ctrl->hw_sync_ctrl.fll, ratio);
			continue;
		}
		if (reg_addr == reg_info->cit_hi_addr) {
			cit += reg_setting->reg_data << 8;
			cit_hi_idx = i;
		}
		if (reg_addr == reg_info->cit_lo_addr) {
			cit += reg_setting->reg_data;
			cit_lo_idx = i;
		}
		if (cit_hi_idx > 0 && cit_lo_idx > 0) {
			CAM_DBG(CAM_SENSOR, "%s cit %d",
				sensor_names[s_ctrl->soc_info.index],
				cit);
			cit_hi_idx = 0;
			cit_lo_idx = 0;
			continue;
		}
		if (reg_addr == reg_info->ms_sel_addr) {
			val = reg_setting->reg_data;
			CAM_INFO(CAM_SENSOR, "%s ms_sel %d",
				sensor_names[s_ctrl->soc_info.index],
				val);
			s_ctrl->hw_sync_ctrl.is_master = val;
		}
		if (reg_addr == reg_info->stream_on_addr) {
			val = reg_setting->reg_data;
			CAM_INFO(CAM_SENSOR, "%s stream_on %d",
				sensor_names[s_ctrl->soc_info.index],
				val);
			cam_sensor_handle_stream_on_off(s_ctrl, val);
		}
	}
}
