/*
 * Trace events for airbrush
 *
 * Copyright (C) 2019 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM airbrush

#if !defined(_TRACE_AIRBRUSH_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_AIRBRUSH_H

#include <linux/airbrush-sm-ctrl.h>
#include <linux/tracepoint.h>

/* tracepoint for airbrush state changes */
TRACE_EVENT(ab_state_change,

	TP_PROTO(enum chip_state new_state),

	TP_ARGS(new_state),

	TP_STRUCT__entry(
		__field(enum chip_state, new_state)
	),

	TP_fast_assign(
		__entry->new_state = new_state;
	),

	TP_printk("Switched to state %d", __entry->new_state)
);

#endif /* _TRACE_AIRBRUSH_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

