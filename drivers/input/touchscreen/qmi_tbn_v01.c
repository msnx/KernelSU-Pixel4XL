/* drivers/input/touchscreen/qmi_tbn_v01.c
 *
 * QMI message interface definition for Touch Bus Negotiator.
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/input/qmi_tbn_v01.h>

struct qmi_elem_info tbn_kernel_request_bus_v01_ei[] = {
	{
		.data_type = QMI_EOTI,
		.is_array = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

struct qmi_elem_info tbn_kernel_release_bus_v01_ei[] = {
	{
		.data_type = QMI_EOTI,
		.is_array = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

struct qmi_elem_info tbn_ssc_release_bus_v01_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.is_array = NO_ARRAY,
		.tlv_type = 0x02,
		.offset = offsetof(struct tbn_ssc_release_bus_v01, resp),
		.ei_array = qmi_response_type_v01_ei,
	},
	{
		.data_type = QMI_EOTI,
		.is_array = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

struct qmi_elem_info tbn_ssc_acquire_bus_v01_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.is_array = NO_ARRAY,
		.tlv_type = 0x02,
		.offset = offsetof(struct tbn_ssc_acquire_bus_v01, resp),
		.ei_array = qmi_response_type_v01_ei,
	},
	{
		.data_type = QMI_EOTI,
		.is_array = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};
