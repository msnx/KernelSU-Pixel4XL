/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <media/cam_req_mgr.h>
#include "cam_sensor_vsync.h"
#include "cam_debug_util.h"
#include <linux/qrtr.h>
#include <linux/net.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/string.h>
#include <net/sock.h>
#include <linux/soc/qcom/qmi.h>

static uint8_t pb_encode_tag(enum pb_wire_type wire_type,
				uint32_t field_number)
{
	return (uint8_t)((field_number << 3) | wire_type);
}

static enum pb_wire_type pb_decode_tag(uint8_t data, uint32_t *field_number)
{
	enum pb_wire_type wire_type;
	*field_number = data >> 3;
	wire_type = (enum pb_wire_type)(data & 0x7);
	return wire_type;
}

static uint32_t pb_encode_fixed32(uint8_t *stream, const uint32_t val)
{
	stream[0] = (uint8_t)(val & 0xFF);
	stream[1] = (uint8_t)((val >> 8) & 0xFF);
	stream[2] = (uint8_t)((val >> 16) & 0xFF);
	stream[3] = (uint8_t)((val >> 24) & 0xFF);
	return 4;
}

static uint32_t pb_encode_fixed64(uint8_t *stream, const uint64_t val)
{
	stream[0] = (uint8_t)(val & 0xFF);
	stream[1] = (uint8_t)((val >> 8) & 0xFF);
	stream[2] = (uint8_t)((val >> 16) & 0xFF);
	stream[3] = (uint8_t)((val >> 24) & 0xFF);
	stream[4] = (uint8_t)((val >> 32) & 0xFF);
	stream[5] = (uint8_t)((val >> 40) & 0xFF);
	stream[6] = (uint8_t)((val >> 48) & 0xFF);
	stream[7] = (uint8_t)((val >> 56) & 0xFF);
	return 8;
}

static uint32_t pb_decode_fixed32(uint8_t *stream, uint32_t *val)
{
	uint32_t i, tmp;
	*val = 0;
	for (i = 0; i < 4; i++) {
		tmp = stream[i];
		tmp = tmp << (i * 8);
		*val |= tmp;
	}

	return 4;
}

static uint64_t pb_decode_fixed64(uint8_t *stream, uint64_t *val)
{
	uint32_t i;
	uint64_t tmp;
	*val = 0;
	for (i = 0; i < 8; i++) {
		tmp = stream[i];
		tmp = tmp << (i * 8);
		*val |= tmp;
	}

	return 8;
}

static uint32_t pb_encode_varint(uint8_t *stream, uint64_t value)
{
	uint32_t len = 0;

	do {
		stream[len++] = (uint8_t)((value & 0x7F) | 0x80);
		value >>= 7;
	} while (value > 0);
	stream[len - 1] &= 0x7F; /* Unset top bit on last byte */

	return len;
}

static uint32_t pb_decode_varint(uint8_t *stream, uint64_t *val)
{
	uint32_t i;
	uint64_t tmp;

	*val = 0;
	for (i = 0; i < 8; i++) {
		tmp = stream[i] & 0x7F;
		tmp = tmp << (i * 7);
		*val |= tmp;
		if (!(stream[i] & 0x80))
			break;
	}

	return i + 1;
}

static uint32_t pb_encode_suid(uint8_t *data, struct sns_std_suid *suid)
{
	uint32_t len = 0;

	data[len++] = pb_encode_tag(PB_WT_64BIT, 1);
	len += pb_encode_fixed64(data + len, suid->suid_low);
	data[len++] = pb_encode_tag(PB_WT_64BIT, 2);
	len += pb_encode_fixed64(data + len, suid->suid_high);
	return len;
}

static uint32_t pb_decode_suid(uint8_t *data, struct sns_std_suid *suid)
{
	uint32_t field_num;
	uint32_t len = 0;

	if (pb_decode_tag(data[len++], &field_num) == PB_WT_64BIT)
		len += pb_decode_fixed64(data + len, &suid->suid_low);

	if (pb_decode_tag(data[len++], &field_num) == PB_WT_64BIT)
		len += pb_decode_fixed64(data + len, &suid->suid_high);

	return len;
}

static uint32_t pb_encode_susp_config(uint8_t *data,
		struct sns_client_request_msg_suspend_config *susp_config)
{
	uint32_t len = 0;

	data[len++] = pb_encode_tag(PB_WT_VARINT, 1);
	len += pb_encode_varint(data + len, susp_config->client_proc_type);
	data[len++] = pb_encode_tag(PB_WT_VARINT, 2);
	len += pb_encode_varint(data + len, susp_config->delivery_type);
	return len;
}

static uint32_t pb_encode_sns_request(uint8_t *data,
				struct sns_std_request *sns_request)
{
	/* Skip optional batching field. */
	/* Start with encoding the payload, which is the 2nd field */
	uint32_t len = 0;

	data[len++] = pb_encode_tag(PB_WT_STRING, 2);
	len += pb_encode_varint(data + len, sns_request->data_len);
	memcpy(data + len, sns_request->data, sns_request->data_len);
	len += sns_request->data_len;
	data[len++] = pb_encode_tag(PB_WT_VARINT, 3);
	len += pb_encode_varint(data + len, sns_request->is_passive);
	return len;
}

static uint32_t pb_encode_vsync_packet(uint8_t *data,
				struct cam_sensor_vsync_packet *packet)
{
	uint32_t len = 0;
	data[len++] = pb_encode_tag(PB_WT_VARINT, 1);
	len += pb_encode_varint(data + len, packet->cam_id);
	data[len++] = pb_encode_tag(PB_WT_VARINT, 2);
	len += pb_encode_varint(data + len, packet->frame_id);
	data[len++] = pb_encode_tag(PB_WT_64BIT, 3);
	len += pb_encode_fixed64(data + len, packet->timestamp);
	return len;
}

static char *hexdump(const char *in, size_t in_len, char *out, size_t out_len)
{
	static const char chr[] = "0123456789abcdef";
	size_t i, oi = 0;

	for (i = 0; i < in_len && oi < out_len - 3; i++) {
		char c = in[i];

		out[oi++] = chr[c / 16];
		out[oi++] = chr[c % 16];
		if ((oi+1) % 33 == 0)
			out[oi++] = ' ';
	}
	out[oi++] = '\0';
	return out;
}

static uint64_t get_ns_from_qtimer_ticks(uint64_t ticks)
{
	const uint64_t clock_freq = 19200000;
	uint64_t nsec = 0;
	uint64_t seconds = 0;
	/* 19.2MHz QTimer clock */

	if (ticks >= clock_freq) {
		seconds = (ticks / clock_freq);
		ticks %= clock_freq;

		nsec = (seconds * 1000000000UL);
	}
	nsec += (ticks * 1000000000UL) / clock_freq;
	return nsec;
}

void cam_vsync_build_sensor_request(uint16_t msg_id, uint64_t cam_id,
		struct sns_client_request_msg *client_request)
{
	memset(client_request, 0, sizeof(struct sns_client_request_msg));
	// Only VSync sensor IDs 0, 1 and 2 which are supported
	if (cam_id > 2)
		return;
	client_request->suid.suid_low = CAM_VSYNC_SUID_LOW;
	client_request->suid.suid_high =
		CAM_VSYNC_SUID_HIGH | (cam_id << 56);
	client_request->msg_id = msg_id;
	client_request->susp_config.client_proc_type =
		SNS_STD_CLIENT_PROCESSOR_APSS;
	client_request->susp_config.delivery_type =
		SNS_CLIENT_DELIVERY_WAKEUP;
	client_request->request.has_passive = true;
	client_request->request.is_passive = false;

}

void cam_vsync_encode_sensor_request(
		struct sns_client_request_msg *sns_client_request,
		uint8_t *data, uint32_t *data_len)
{
	uint32_t temp;
	uint32_t len = 0;
	char buffer[256];

	// Encode SUID. Field 1 - 1st field in struct
	data[len++] = pb_encode_tag(PB_WT_STRING, 1);
	temp = pb_encode_suid(data + len + 1, &sns_client_request->suid);
	data[len] = (uint8_t)temp;
	len += (temp + 1);

	// Encode message ID. 2nd field
	data[len++] = pb_encode_tag(PB_WT_32BIT, 2);
	len += pb_encode_fixed32(data + len, sns_client_request->msg_id);

	// Encode susp_config. 3rd field
	data[len++] = pb_encode_tag(PB_WT_STRING, 3);
	temp = pb_encode_susp_config(data + len + 1,
					&sns_client_request->susp_config);
	data[len] = (uint8_t)temp;
	len += (temp + 1);

	// Encode sns_request. 4th field
	data[len++] = pb_encode_tag(PB_WT_STRING, 4);
	temp = pb_encode_sns_request(data + len + 1,
				&sns_client_request->request);
	data[len] = (uint8_t)temp;
	len += (temp + 1);

	*data_len = len;
	CAM_DBG(CAM_SENSOR, "VSync data(%d)%s", len,
		hexdump(data, len, buffer, 256));
}

void cam_vsync_make_sensor_request(struct cam_sensor_vsync_packet *packet,
		uint8_t *data, uint32_t *data_len)
{
	struct sns_client_request_msg sns_client_request;

	cam_vsync_build_sensor_request(SNS_CAMERA_VSYNC_MSGID_INTERRUPT_INFO,
				packet->cam_id, &sns_client_request);
	sns_client_request.request.data_len =
		pb_encode_vsync_packet(sns_client_request.request.data, packet);

	cam_vsync_encode_sensor_request(&sns_client_request, data, data_len);
}
EXPORT_SYMBOL(cam_vsync_make_sensor_request);

void cam_vsync_make_config_request(uint64_t cam_id,
					uint8_t *data, uint32_t *data_len)
{
	struct sns_client_request_msg sns_client_request;

	cam_vsync_build_sensor_request(
		SNS_STD_SENSOR_MSGID_SNS_STD_ON_CHANGE_CONFIG,
		cam_id, &sns_client_request);
	sns_client_request.request.data_len = 0;

	cam_vsync_encode_sensor_request(&sns_client_request, data, data_len);
}
EXPORT_SYMBOL(cam_vsync_make_config_request);

int cam_vsync_parse_ind_message(uint8_t *data, uint32_t data_len,
		struct cam_sensor_vsync_result *result)
{
	int i = 0, rc = 0;
	int len;
	struct sns_std_suid suid;
	uint32_t field_number;
	uint32_t msg_id;
	uint64_t tmp;

	if (pb_decode_tag(data[i++], &field_number) == PB_WT_STRING) {
		len = data[i++];
		if (len == CAM_VSYNC_SUID_PB_LENGTH &&
			field_number == 1) {
			//Decode SUID
			i += pb_decode_suid(data + i, &suid);
		} else
			goto fail;
	}

	if (pb_decode_tag(data[i++], &field_number) == PB_WT_STRING) {
		len = data[i++];
		if (pb_decode_tag(data[i++], &field_number)
			== PB_WT_32BIT)	{
			/* Decode msg_id */
			i += pb_decode_fixed32(data + i, &msg_id);
			if (msg_id != SNS_CAMERA_VSYNC_MSGID_INTERRUPT_INFO)
				goto fail;
		} else
			goto fail;

		if (pb_decode_tag(data[i++], &field_number) == PB_WT_64BIT) {
			/* Decode event timestamp */
			i += pb_decode_fixed64(data + i, &tmp);
			result->timestamp_vsync = get_ns_from_qtimer_ticks(tmp);
		} else
			goto fail;

		if (pb_decode_tag(data[i++], &field_number) == PB_WT_STRING)
			len = data[i++];
		else
			goto fail;

		if (pb_decode_tag(data[i++], &field_number) == PB_WT_VARINT) {
			/* Decode sensor id */
			i += pb_decode_varint(data + i, &tmp);
			result->cam_id = tmp;
		} else
			goto fail;

		if (pb_decode_tag(data[i++], &field_number) == PB_WT_VARINT) {
			/* Decode frame_id */
			i += pb_decode_varint(data + i, &tmp);
			result->frame_id = tmp;
		} else
			goto fail;

		if (pb_decode_tag(data[i++], &field_number) == PB_WT_64BIT) {
			/* Decode sof timestamp */
			i += pb_decode_fixed64(data + i, &tmp);
			result->timestamp_sof = tmp;
		} else
			goto fail;
	}
	rc = i - data_len;

	return rc;
fail:
	rc = -EINVAL;
	return rc;
}
EXPORT_SYMBOL(cam_vsync_parse_ind_message);
