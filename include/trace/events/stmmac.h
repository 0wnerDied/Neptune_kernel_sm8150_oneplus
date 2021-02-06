/* Copyright (c)2021, The Linux Foundation. All rights reserved.
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
#define TRACE_SYSTEM stmmac

#if !defined(_TRACE_STMMAC_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_STMMAC_H_

#include <linux/tracepoint.h>
#include <linux/timekeeping.h>
/*****************************************************************************/
/* Trace events for stmmac module */
/*****************************************************************************/
DECLARE_EVENT_CLASS(stmmac_time_template,

	TP_PROTO(int queue),

	TP_ARGS(queue),

	TP_STRUCT__entry(
		__field(int, queue)
		__field(u64, qtime)

	),

	TP_fast_assign(
		__entry->queue = queue;
		__entry->qtime = ktime_get_tai_ns();
	),

	TP_printk("queue[%d] UTC time %ld",
		  __entry->queue, __entry->qtime)
);

DEFINE_EVENT
	(stmmac_time_template, stmmac_xmit_entry,

	TP_PROTO(int queue),

	TP_ARGS(queue)
);

DEFINE_EVENT
	(stmmac_time_template, stmmac_xmit_exit,

	TP_PROTO(int queue),

	TP_ARGS(queue)
);

DEFINE_EVENT
	(stmmac_time_template, stmmac_xmit_err,

	TP_PROTO(int queue),

	TP_ARGS(queue)
);

DEFINE_EVENT
	(stmmac_time_template, stmmac_rx_entry,

	TP_PROTO(int queue),

	TP_ARGS(queue)
);

DEFINE_EVENT
	(stmmac_time_template, stmmac_rx_pkt,

	TP_PROTO(int queue),

	TP_ARGS(queue)
);

DEFINE_EVENT
	(stmmac_time_template, stmmac_rx_exit,

	TP_PROTO(int queue),

	TP_ARGS(queue)
);

TRACE_EVENT(stmmac_irq_enter,

	TP_PROTO(int irq),

	TP_ARGS(irq),

	TP_STRUCT__entry(
		__field(int, irq)
		__field(u64, qtime)
	),

	TP_fast_assign(
		__entry->irq = irq;
		__entry->qtime = ktime_get_tai_ns();
	),

	TP_printk("Enter stmmac_interrupt - irq %d, UTC time %ld",
		  __entry->irq, __entry->qtime)
);

TRACE_EVENT(stmmac_irq_exit,

	TP_PROTO(int irq),

	TP_ARGS(irq),

	TP_STRUCT__entry(
		__field(int, irq)
		__field(u64, qtime)
	),

	TP_fast_assign(
		__entry->irq = irq;
		__entry->qtime = ktime_get_tai_ns();
	),

	TP_printk("Exit stmmac_interrupt - irq %d, UTC time %ld",
		  __entry->irq, __entry->qtime)
);

TRACE_EVENT(stmmac_poll_enter,

	TP_PROTO(int queue),

	TP_ARGS(queue),

	TP_STRUCT__entry(
		__field(int, queue)
		__field(u64, qtime)
	),

	TP_fast_assign(
		__entry->queue = queue;
		__entry->qtime = ktime_get_tai_ns();
	),

	TP_printk("Enter stmmac_poll - queue[%d], UTC time %ld",
		  __entry->queue, __entry->qtime)
);

TRACE_EVENT(stmmac_poll_exit,

	TP_PROTO(int queue),

	TP_ARGS(queue),

	TP_STRUCT__entry(
		__field(int, queue)
		__field(u64, qtime)
	),

	TP_fast_assign(
		__entry->queue = queue;
		__entry->qtime = ktime_get_tai_ns();
	),

	TP_printk("Exit stmmac_poll - queue[%d], UTC time %ld",
		  __entry->queue, __entry->qtime)
);
#endif /* _TRACE_STMMAC_H_ */

/* This part must be outside protection */
#include <trace/define_trace.h>
