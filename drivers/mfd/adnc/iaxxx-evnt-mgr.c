/*
 * iaxxx-evnt-mgr.c -- IAxxx Event Manager Service
 *
 * Copyright 2016 Knowles Corporation
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

#define pr_fmt(fmt) "iaxxx : %s:%d, " fmt, __func__, __LINE__

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/mfd/adnc/iaxxx-evnt-mgr.h>
#include <linux/mfd/adnc/iaxxx-system-identifiers.h>
#include <linux/mfd/adnc/iaxxx-register-defs-srb.h>
#include "iaxxx.h"
#include "iaxxx-tunnel-priv.h"

struct iaxxx_event_subscription {
	u16 event_dst;
	void *userdata;
	iaxxx_evnt_mgr_callback event_handler;
};

static void iaxxx_add_event(struct iaxxx_priv *priv,
		const struct iaxxx_event *evt)
{
	struct device *dev = priv->dev;
	struct iaxxx_evt_queue *eq = priv->event_queue;
	int w_index;

	eq->w_index++;
	w_index = eq->w_index;
	if (w_index == IAXXX_MAX_EVENTS) {
		dev_info(dev, "%s Event Index reach last\n", __func__);
		w_index = 0;
		eq->w_index = w_index;
	}
	if (eq->w_index == eq->r_index) {
		dev_info(dev, "%s Event buffer is full\n", __func__);
		eq->r_index++;
	}
	eq->event_info[w_index].event_id = evt->event_id;
	eq->event_info[w_index].data = evt->src_opaque;
	if (eq->r_index == IAXXX_MAX_EVENTS)
		eq->r_index = 0;
	dev_dbg(dev, "%s written index %d\n", __func__, eq->w_index);
}

/*
 * uevent format:
 *   "ACTION=IAXXX_VQ_EVENT\0
 *    EVENT_ID=1\0
 *    EVENT_DATA=12345678\0"
 */
int iaxxx_event_handler(struct iaxxx_priv *priv, struct iaxxx_event *evt)
{
	struct device *dev = priv->dev;
	char action[] = "ACTION=IAXXX_VQ_EVENT";
	char eid[14];
	char edata[20];
	char *event[] = {action, eid, edata, NULL};
	int ret = 0;

	if (WARN_ON(!evt) || WARN_ON(!priv->tunnel_data))
		return -EINVAL;

	dev_dbg(dev, "%s:src:0x%04x id:0x%04x src_opq=0x%08x dst_opq=0x%08x\n",
			__func__, evt->event_src, evt->event_id,
			evt->src_opaque, evt->dst_opaque);

	/* Check for tunnel event and notify producer thread*/
	if (evt->event_src == IAXXX_SYSID_TUNNEL_EVENT) {
		iaxxx_tunnel_signal_event(priv);
		return ret;
	}

	if (evt->event_src == IAXXX_CM4_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_BOOT_COMPLETE_EVENT_ID) {
		dev_info(dev, "FW boot complete event %s: src:0x%.04x\n",
						__func__, evt->event_src);
		priv->boot_completed = true;
		wake_up(&priv->boot_wq);
		return ret;
	}

	if (evt->event_src == IAXXX_CM4_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_HOST0_WAKEUP_EVENT_ID) {
		dev_info(dev, "FW chip wakeup event %s: src:0x%.04x\n",
						__func__, evt->event_src);
		return ret;
	}

	if (evt->event_src == IAXXX_CM4_CTRL_MGR_SRC_ID
			&& evt->event_id == IAXXX_CRASH_EVENT_ID) {
		dev_err(dev, "FW crash %s: src:0x%.04x, proc id:0x%.04X\n",
			__func__, evt->event_src, evt->src_opaque);
		iaxxx_fw_crash(dev, IAXXX_FW_CRASH_EVENT, evt->src_opaque);
		return ret;
	}

	dev_info(dev,
		"%s:EVENT src:0x%04x id:0x%04x src_opq=0x%08x dst_opq=0x%08x\n",
		__func__, evt->event_src, evt->event_id,
		evt->src_opaque, evt->dst_opaque);

	snprintf(eid, sizeof(eid), "EVENT_ID=%hx", evt->event_id);
	snprintf(edata, sizeof(edata), "EVENT_DATA=%x", evt->src_opaque);

	mutex_lock(&priv->event_queue_lock);
	iaxxx_add_event(priv, evt);
	mutex_unlock(&priv->event_queue_lock);
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, event);
	return 0;
}

int iaxxx_evnt_mgr_unsubscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst)
{
	return -ENOTSUPP;
}
EXPORT_SYMBOL(iaxxx_evnt_mgr_unsubscribe);

int iaxxx_evnt_mgr_subscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst,
					u32 dst_opaque, void *userdata,
					iaxxx_evnt_mgr_callback cb_func_ptr)
{
	return -ENOTSUPP;
}
EXPORT_SYMBOL(iaxxx_evnt_mgr_subscribe);
