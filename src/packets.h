/* Copyright (C) 2015-2017 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved. */

#ifndef PACKETS_H
#define PACKETS_H

#include "noise.h"
#include "messages.h"
#include "socket.h"

#include <linux/types.h>
#include <linux/padata.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/ipv6.h>

struct wireguard_device;
struct wireguard_peer;
struct sk_buff;

struct packet_cb {
	u64 nonce;
	u8 ds;
};
#define PACKET_CB(skb) ((struct packet_cb *)skb->cb)

/* receive.c */
void packet_receive(struct wireguard_device *wg, struct sk_buff *skb);
void packet_process_queued_handshake_packets(struct work_struct *work);
void packet_consume_data_done(struct sk_buff *skb, struct wireguard_peer *peer, struct endpoint *endpoint, bool used_new_key);

/* send.c */
void packet_send_queue(struct wireguard_peer *peer);
void packet_send_keepalive(struct wireguard_peer *peer);
void packet_queue_handshake_initiation(struct wireguard_peer *peer, bool is_retry);
void packet_send_queued_handshakes(struct work_struct *work);
void packet_send_handshake_response(struct wireguard_peer *peer);
void packet_send_handshake_cookie(struct wireguard_device *wg, struct sk_buff *initiating_skb, __le32 sender_index);
void packet_create_data_done(struct sk_buff_head *queue, struct wireguard_peer *peer);

/* data.c */
int packet_create_data(struct sk_buff_head *queue, struct wireguard_peer *peer);
void packet_consume_data(struct sk_buff *skb, struct wireguard_device *wg);

/* Returns either the correct skb->protocol value, or 0 if invalid. */
static inline __be16 skb_examine_untrusted_ip_hdr(struct sk_buff *skb)
{
	if (skb_network_header(skb) >= skb->head && (skb_network_header(skb) + sizeof(struct iphdr)) <= skb_tail_pointer(skb) && ip_hdr(skb)->version == 4)
		return htons(ETH_P_IP);
	if (skb_network_header(skb) >= skb->head && (skb_network_header(skb) + sizeof(struct ipv6hdr)) <= skb_tail_pointer(skb) && ipv6_hdr(skb)->version == 6)
		return htons(ETH_P_IPV6);
	return 0;
}

#ifdef CONFIG_WIREGUARD_PARALLEL
int packet_init_data_caches(void);
void packet_deinit_data_caches(void);
#endif

#ifdef DEBUG
bool packet_counter_selftest(void);
#endif

#endif
