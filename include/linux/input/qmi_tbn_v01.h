/* drivers/input/touchscreen/qmi_tbn_v01.h
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

#ifndef QMI_TBN_V01_H
#define QMI_TBN_V01_H

#include <linux/soc/qcom/qmi.h>

#define TBN_SERVICE_ID_V01 232
#define TBN_SERVICE_VERS_V01 1

#define QMI_TBN_KERNEL_REQUEST_BUS_V01  0x0020
#define QMI_TBN_SSC_RELEASE_BUS_V01     0x0020

#define QMI_TBN_KERNEL_RELEASE_BUS_V01  0X0021
#define QMI_TBN_SSC_ACQUIRE_BUS_V01     0x0021


struct tbn_kernel_request_bus_v01 {
	char placeholder;
};
#define TBN_KERNEL_REQUEST_BUS_V01_MAX_MSG_LEN 0
extern struct qmi_elem_info tbn_kernel_request_bus_v01_ei[];


struct tbn_kernel_release_bus_v01 {
	char placeholder;
};
#define TBN_KERNEL_RELEASE_BUS_V01_MAX_MSG_LEN 0
extern struct qmi_elem_info tbn_kernel_release_bus_v01_ei[];


struct tbn_ssc_release_bus_v01 {
	struct qmi_response_type_v01 resp;
};
#define TBN_SSC_RELEASE_BUS_V01_MAX_MSG_LEN 7
extern struct qmi_elem_info tbn_ssc_release_bus_v01_ei[];


struct tbn_ssc_acquire_bus_v01 {
	struct qmi_response_type_v01 resp;
};
#define TBN_SSC_ACQUIRE_BUS_V01_MAX_MSG_LEN 7
extern struct qmi_elem_info tbn_ssc_acquire_bus_v01_ei[];


#endif // QMI_TBN_V01_H
