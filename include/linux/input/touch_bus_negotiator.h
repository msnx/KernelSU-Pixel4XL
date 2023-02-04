/* include/linux/input/touch_bus_negotiator.h
 *
 * Touch Bus Negotiator for Google Pixel devices.
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

#ifndef TOUCHSCREEN_BUS_NEGOTIATOR_H
#define TOUCHSCREEN_BUS_NEGOTIATOR_H

#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include "qmi_tbn_v01.h"

#define TBN_REQUEST_BUS_TIMEOUT_MS 500
#define TBN_RELEASE_BUS_TIMEOUT_MS 500

struct tbn_context {
	struct device *dev;
	struct qmi_handle qmi_handle;
	struct mutex service_lock;
	struct work_struct request_work;
	struct work_struct release_work;
	struct workqueue_struct *qmi_wq;
	struct completion bus_requested;
	struct completion bus_released;
	bool connected;
};

struct tbn_context *tbn_init(struct device *dev);
void tbn_cleanup(struct tbn_context *tbn);
int tbn_request_bus(struct tbn_context *tbn);
int tbn_release_bus(struct tbn_context *tbn);


#endif /* TOUCHSCREEN_BUS_NEGOTIATOR_H */
