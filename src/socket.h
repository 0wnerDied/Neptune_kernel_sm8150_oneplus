/* Copyright (C) 2015-2017 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved. */

#ifndef WGSOCKET_H
#define WGSOCKET_H

#include <linux/netdevice.h>
#include <linux/udp.h>
#include <linux/if_vlan.h>
#include <linux/if_ether.h>

struct wireguard_device;
struct endpoint;

int socket_init(struct wireguard_device *wg);
void socket_uninit(struct wireguard_device *wg);
int socket_send_buffer_to_peer(struct wireguard_peer *peer, void *data, size_t len, u8 ds);
int socket_send_skb_to_peer(struct wireguard_peer *peer, struct sk_buff *skb, u8 ds);
int socket_send_buffer_as_reply_to_skb(struct wireguard_device *wg, struct sk_buff *in_skb, void *out_buffer, size_t len);

int socket_endpoint_from_skb(struct endpoint *endpoint, struct sk_buff *skb);
void socket_set_peer_endpoint(struct wireguard_peer *peer, struct endpoint *endpoint);
void socket_clear_peer_endpoint_src(struct wireguard_peer *peer);

#endif
