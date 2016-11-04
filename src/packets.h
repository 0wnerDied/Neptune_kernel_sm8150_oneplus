/* Copyright 2015-2016 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved. */

#ifndef PACKETS_H
#define PACKETS_H

#include "noise.h"
#include "messages.h"
#include "socket.h"

#include <linux/types.h>
#include <linux/padata.h>

enum {
	MAX_QUEUED_HANDSHAKES = 4096,
	MAX_BURST_HANDSHAKES = 16
};

/* AF41, plus 00 ECN */
#define HANDSHAKE_DSCP 0b10001000

struct wireguard_device;
struct wireguard_peer;
struct sk_buff;

/* receive.c */
void packet_receive(struct wireguard_device *wg, struct sk_buff *skb);

/* send.c */
int packet_send_queue(struct wireguard_peer *peer);
void packet_send_keepalive(struct wireguard_peer *peer);
void packet_send_handshake_initiation(struct wireguard_peer *peer);
void packet_send_handshake_response(struct wireguard_peer *peer);
void packet_send_handshake_cookie(struct wireguard_device *wg, struct sk_buff *initiating_skb, void *data, size_t data_len, __le32 sender_index);
void packet_send_handshake_initiation_ratelimited(struct wireguard_peer *peer);

void packet_queue_send_handshake_initiation(struct wireguard_peer *peer);
void packet_process_queued_handshake_packets(struct work_struct *work);
void packet_send_queued_handshakes(struct work_struct *work);


/* data.c */
int packet_create_data(struct sk_buff_head *queue, struct wireguard_peer *peer, void(*callback)(struct sk_buff_head *, struct wireguard_peer *));
void packet_consume_data(struct sk_buff *skb, size_t offset, struct wireguard_device *wg, void(*callback)(struct sk_buff *, struct wireguard_peer *, struct sockaddr_storage *, bool used_new_key, int err));
int packet_init_data_caches(void);
void packet_deinit_data_caches(void);

#define DATA_PACKET_HEAD_ROOM ALIGN(sizeof(struct message_data) + SKB_HEADER_LEN, 4)

#ifdef DEBUG
bool packet_counter_selftest(void);
#endif

#endif
