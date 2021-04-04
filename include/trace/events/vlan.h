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
#define TRACE_SYSTEM vlan

#if !defined(_TRACE_VLAN_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VLAN_H_

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/tracepoint.h>
#include <linux/timekeeping.h>
/*****************************************************************************/
/* Trace events for vlan module */
/*****************************************************************************/
TRACE_EVENT(vlan_receive_skb,

	TP_PROTO(const struct sk_buff *skb),

	TP_ARGS(skb),

	TP_STRUCT__entry(
		__string(name, skb->dev->name)
		__field(const void *, skbaddr)
		__field(unsigned int, len)
		__field(unsigned int, data_len)
		__field(ktime_t, utctime)
	),

	TP_fast_assign(
		__assign_str(name, skb->dev->name);
		__entry->skbaddr = skb;
		__entry->len = skb->len;
		__entry->data_len = skb->data_len;
		__entry->utctime = ktime_get_tai_ns();
	),

	TP_printk("dev=%s skbaddr=%px len=%u data_len=%u UTC: %ld",
		  __get_str(name), __entry->skbaddr, __entry->len,
		  __entry->data_len, __entry->utctime)
);

#endif /* _TRACE_VLAN_H_ */

/* This part must be outside protection */
#include <trace/define_trace.h>
