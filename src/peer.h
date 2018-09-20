/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015-2018 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved.
 */

#ifndef _WG_PEER_H
#define _WG_PEER_H

#include "device.h"
#include "noise.h"
#include "cookie.h"

#include <linux/types.h>
#include <linux/netfilter.h>
#include <linux/spinlock.h>
#include <linux/kref.h>
#include <net/dst_cache.h>

struct wireguard_device;

struct endpoint {
	union {
		struct sockaddr addr;
		struct sockaddr_in addr4;
		struct sockaddr_in6 addr6;
	};
	union {
		struct {
			struct in_addr src4;
			/* Essentially the same as addr6->scope_id */
			int src_if4;
		};
		struct in6_addr src6;
	};
};

struct wireguard_peer {
	struct wireguard_device *device;
	struct crypt_queue tx_queue, rx_queue;
	struct sk_buff_head staged_packet_queue;
	int serial_work_cpu;
	struct noise_keypairs keypairs;
	struct endpoint endpoint;
	struct dst_cache endpoint_cache;
	rwlock_t endpoint_lock;
	struct noise_handshake handshake;
	atomic64_t last_sent_handshake;
	struct work_struct transmit_handshake_work, clear_peer_work;
	struct cookie latest_cookie;
	struct hlist_node pubkey_hash;
	u64 rx_bytes, tx_bytes;
	struct timer_list timer_retransmit_handshake, timer_send_keepalive;
	struct timer_list timer_new_handshake, timer_zero_key_material;
	struct timer_list timer_persistent_keepalive;
	unsigned int timer_handshake_attempts;
	u16 persistent_keepalive_interval;
	bool timers_enabled, timer_need_another_keepalive;
	bool sent_lastminute_handshake;
	struct timespec walltime_last_handshake;
	struct kref refcount;
	struct rcu_head rcu;
	struct list_head peer_list;
	u64 internal_id;
	struct napi_struct napi;
	bool is_dead;
};

struct wireguard_peer *
peer_create(struct wireguard_device *wg,
	    const u8 public_key[NOISE_PUBLIC_KEY_LEN],
	    const u8 preshared_key[NOISE_SYMMETRIC_KEY_LEN]);

struct wireguard_peer *__must_check
peer_get_maybe_zero(struct wireguard_peer *peer);
static inline struct wireguard_peer *peer_get(struct wireguard_peer *peer)
{
	kref_get(&peer->refcount);
	return peer;
}
void peer_put(struct wireguard_peer *peer);
void peer_remove(struct wireguard_peer *peer);
void peer_remove_all(struct wireguard_device *wg);

struct wireguard_peer *peer_lookup_by_index(struct wireguard_device *wg,
					    u32 index);

#endif /* _WG_PEER_H */
