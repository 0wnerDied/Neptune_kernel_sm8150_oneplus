/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include "bpf_service.h"
#include <linux/bpf.h>

/* for service lookup for eBPF */
static RADIX_TREE(service_lookup, GFP_KERNEL);

/* mutex to lock service lookup */
static DEFINE_MUTEX(service_lookup_lock);

/* variable to hold bpf filter object */
static struct sk_filter __rcu *bpf_filter;

/**
 * Add service information (service id & instance id) to lookup table
 * with key as node & port id pair
 */
void qrtr_service_add(struct qrtr_ctrl_pkt *pkt)
{
	struct service_info *info;
	unsigned long key = 0;

	key = (u64)le32_to_cpu(pkt->server.node) << 32 |
			le32_to_cpu(pkt->server.port);
	mutex_lock(&service_lookup_lock);
	info = radix_tree_lookup(&service_lookup, key);
	if (!info) {
		info = kzalloc(sizeof(*info), GFP_KERNEL);
		if (info) {
			info->service_id = le32_to_cpu(pkt->server.service);
			info->instance_id = le32_to_cpu(pkt->server.instance);
			radix_tree_insert(&service_lookup, key, info);
		} else {
			pr_err("%s svc<0x%x:0x%x> adding to lookup failed\n",
			       __func__, le32_to_cpu(pkt->server.service),
			       le32_to_cpu(pkt->server.instance));
		}
	}
	mutex_unlock(&service_lookup_lock);
}
EXPORT_SYMBOL(qrtr_service_add);

/* Get service information from service lookup table */
int qrtr_service_lookup(u32 node, u32 port, struct service_info **info)
{
	struct service_info *sinfo = NULL;
	unsigned long key = 0;
	int rc = -EINVAL;

	key = (u64)node << 32 | port;
	mutex_lock(&service_lookup_lock);
	sinfo = radix_tree_lookup(&service_lookup, key);
	mutex_unlock(&service_lookup_lock);
	if (sinfo) {
		*info = sinfo;
		rc = 0;
	}

	return rc;
}
EXPORT_SYMBOL(qrtr_service_lookup);

/* Remove service information from service lookup table */
void qrtr_service_remove(struct qrtr_ctrl_pkt *pkt)
{
	struct service_info *info;
	unsigned long key = 0;

	key = (u64)le32_to_cpu(pkt->server.node) << 32 |
			le32_to_cpu(pkt->server.port);
	mutex_lock(&service_lookup_lock);
	info = radix_tree_lookup(&service_lookup, key);
	kfree(info);
	radix_tree_delete(&service_lookup, key);
	mutex_unlock(&service_lookup_lock);
}
EXPORT_SYMBOL(qrtr_service_remove);

/* Remove all services from requested node */
void qrtr_service_node_remove(u32 src_node)
{
	struct radix_tree_iter iter;
	struct service_info *info;
	void __rcu **slot;
	unsigned long node_id;

	mutex_lock(&service_lookup_lock);
	radix_tree_for_each_slot(slot, &service_lookup, &iter, 0) {
		info = rcu_dereference(*slot);
		/**
		 * extract node id from the index key & remove service
		 * info only for matching node_id
		 */
		node_id = (iter.index & 0xFFFFFFFF00000000) >> 32;
		if (node_id != src_node)
			continue;

		kfree(info);
		radix_tree_iter_delete(&service_lookup, &iter, slot);
	}
	mutex_unlock(&service_lookup_lock);
}
EXPORT_SYMBOL(qrtr_service_node_remove);

/**
 * Get eBPF filter object from eBPF framework using filter fd
 * passed by the user space program. QRTR will allow to attach
 * eBPF filter only for privileged user space program. This
 * attached filter then executed on different qrtr events.
 */
int qrtr_bpf_filter_attach(int ufd)
{
	struct sk_filter *filter = NULL;
	struct bpf_prog *prog = NULL;
	int rc = 0;

	/* return invalid error if fd is not valid */
	if (ufd < 0)
		return -EINVAL;

	/* return -EEXIST if ebpf filter is already attached */
	if (bpf_filter)
		return -EEXIST;

	if (!uid_eq(current_euid(), GLOBAL_ROOT_UID))
		return -EPERM;

	prog = bpf_prog_get_type(ufd, BPF_PROG_TYPE_SOCKET_FILTER);
	if (prog) {
		pr_info("%s bpf filter with fd %d attached with qrtr\n",
			__func__, ufd);
		filter = kzalloc(sizeof(*bpf_filter), GFP_KERNEL);
		if (!filter)
			return -ENOMEM;
		filter->prog = prog;
		rcu_assign_pointer(bpf_filter, filter);
	} else {
		rc = -EFAULT;
	}

	return rc;
}
EXPORT_SYMBOL(qrtr_bpf_filter_attach);

/* Detach previous attached eBPF filter program */
int qrtr_bpf_filter_detach(void)
{
	struct sk_filter *filter = NULL;
	int rc = -EFAULT;

	rcu_read_lock();
	filter = rcu_dereference(bpf_filter);
	rcu_read_unlock();

	if (filter && filter->prog) {
		pr_info("%s bpf filter program detached\n",
			__func__);
		bpf_filter = NULL;
		bpf_prog_put(filter->prog);
		kfree(filter);
		rc = 0;
	}

	return rc;
}
EXPORT_SYMBOL(qrtr_bpf_filter_detach);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. QRTR filter driver");
MODULE_LICENSE("GPL v2");
