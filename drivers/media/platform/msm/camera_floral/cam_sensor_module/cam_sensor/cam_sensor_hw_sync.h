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

#ifndef _CAM_SENSOR_HW_SYNC_H_
#define _CAM_SENSOR_HW_SYNC_H_

#include "cam_sensor_dev.h"

void cam_sensor_sof_notify(struct cam_sensor_ctrl_t *s_ctrl,
	int64_t req_id, uint64_t sof_timestamp);

struct cam_sensor_ctrl_t *cam_sensor_get_sensor_ctrl(
	struct cam_req_mgr_core_link *link);

int cam_sensor_sync_pkt_parse(struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet);

void cam_sensor_sync_init(struct cam_req_mgr_core_session *cam_session);

void cam_sensor_sync_deinit(struct cam_req_mgr_core_session *cam_session);

void cam_sensor_sync_trigger(struct cam_sensor_ctrl_t *s_ctrl, int64_t req_id);

void cam_sensor_sync_audit(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_list *i2c_list);

#endif /* _CAM_SENSOR_HW_SYNC_H_ */
