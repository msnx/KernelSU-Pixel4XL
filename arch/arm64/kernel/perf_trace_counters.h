/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2013-2014,2017 The Linux Foundation. All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM perf_trace_counters

#if !defined(_PERF_TRACE_COUNTERS_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _PERF_TRACE_COUNTERS_H_

#define NUM_EVENTS 3

#include <linux/cpumask.h>
#include <linux/perf_event.h>
#include <linux/sched.h>
#include <linux/tracepoint.h>

DECLARE_PER_CPU(u32[NUM_EVENTS], previous_cnts);
DECLARE_PER_CPU(struct perf_event **, perf_events);
TRACE_EVENT(sched_switch_with_ctrs,

	TP_PROTO(struct task_struct *prev),

	TP_ARGS(prev),

	TP_STRUCT__entry(
		__array(char,	prev_comm,	TASK_COMM_LEN)
		__field(pid_t,	prev_pid)
		__field(u32, cyc)
		__field(u32, inst)
		__field(u32, l2dm)
	),

	TP_fast_assign(
		u32 cpu = raw_smp_processor_id();
		u32 i;
		u32 total_cnt = 0;
		u32 delta_cnts[NUM_EVENTS];
		struct perf_event *pevent;

		memcpy(__entry->prev_comm, prev->comm, TASK_COMM_LEN);
		__entry->prev_pid	= prev->pid;

		for (i = 0; i < NUM_EVENTS; i++) {
			pevent = per_cpu(perf_events, cpu)[i];
			pevent->pmu->read(pevent);
			total_cnt = (int32_t) local64_read(&pevent->count);
			delta_cnts[i] = total_cnt -
				per_cpu(previous_cnts[i], cpu);
			per_cpu(previous_cnts[i], cpu) =
				total_cnt;
		}

		__entry->cyc = delta_cnts[0];
		__entry->inst = delta_cnts[1];
		__entry->l2dm = delta_cnts[2];
	),

	TP_printk("prev_comm=%s, prev_pid=%d, CYC=%u, INST=%u, L2DM=%u",
		__entry->prev_comm,
		__entry->prev_pid,
		__entry->cyc,
		__entry->inst,
		__entry->l2dm)
);

#endif
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../arch/arm64/kernel
#define TRACE_INCLUDE_FILE perf_trace_counters
#include <trace/define_trace.h>
