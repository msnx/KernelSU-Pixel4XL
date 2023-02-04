/*
 * Trace events for faceauth
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
#define TRACE_SYSTEM faceauth

#if !defined(_TRACE_FACEAUTH_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_FACEAUTH_H

#include <linux/tracepoint.h>

/* tracepoint for faceauth el2 trace */
TRACE_EVENT(faceauth_el2_duration,

	TP_PROTO(int el2_command,
			 int time),

	TP_ARGS(el2_command, time),

	TP_STRUCT__entry(
		__field(int, el2_command)
		__field(int, time)
	),

	TP_fast_assign(
		__entry->el2_command = el2_command;
		__entry->time = time;
	),

	TP_printk("EL2 enter reason %d, duration %d us",
	      __entry->el2_command, __entry->time)
);

#endif /* _TRACE_FACEAUTH_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
