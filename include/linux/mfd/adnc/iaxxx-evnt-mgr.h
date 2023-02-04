/*
 * iaxxx-evnt-mgr.h -- IAxxx Event Manager header
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

#ifndef __IAXXX_EVNT_MGR_H__
#define __IAXXX_EVNT_MGR_H__

#define IAXXX_MAX_EVENTS	50
struct device;

/**
 * System Defined Events
 *
 * The device (System ID 0x0000) defines the following built-in events.
 */
enum iaxxx_system_events {
	SYS_EVENT_BOOT_COMPLETE = 0,
	SYS_EVENT_BLOCK_UPDATE_COMPLETE = 1,
	SYS_EVENT_TIMER_EXPIRED = 2,
	SYS_EVENT_SCRIPT_EXECUTION_COMPLETE = 3
};

/**
 * Description of event notification data
 *
 * @event_id	: The event identifier
 * @event_src   : System ID of the event source
 * @src_opaque	: Opaque data sent by the event source
 * @dst_opaque	: Opaque data specified by the destination
 */
struct iaxxx_event {
	uint16_t event_id;
	uint16_t event_src;
	uint32_t src_opaque;
	uint32_t dst_opaque;
};

struct iaxxx_get_event {
	uint16_t event_id;
	uint32_t data;
};

struct iaxxx_evt_queue {
	/* ToDo : Replace with list */
	struct iaxxx_get_event event_info[IAXXX_MAX_EVENTS];
	int r_index;
	int w_index;
};
/**
 * Event manager callback type
 *
 * @event	- The incoming event
 * @ctx		- User context (user data)
 */
typedef void (*iaxxx_evnt_mgr_callback)(const struct iaxxx_event *, void *ctx);

extern int iaxxx_evnt_mgr_unsubscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst);

extern int iaxxx_evnt_mgr_subscribe(struct device *dev, u16 event_id,
					u16 event_src, u16 event_dst,
					u32 dst_opaque, void *userdata,
					iaxxx_evnt_mgr_callback cb_func_ptr);
#endif /* __IAXXX_EVNT_MGR_H__ */
