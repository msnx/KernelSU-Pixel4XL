/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM fastrpc

#if !defined(_TRACE_FASTRPC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_FASTRPC_H
#include <linux/tracepoint.h>

TRACE_EVENT(fastrpc_dma_stat,

	TP_PROTO(int cid, long len,
		unsigned long total_allocated),

	TP_ARGS(cid, len, total_allocated),

	TP_STRUCT__entry(
		__field(int, cid)
		__field(long, len)
		__field(unsigned long, total_allocated)
	),

	TP_fast_assign(
		__entry->cid = cid;
		__entry->len = len;
		__entry->total_allocated = total_allocated;
	),

	TP_printk("cid=%u len=%ldB total_allocated=%ldB",
		__entry->cid,
		__entry->len,
		__entry->total_allocated)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
