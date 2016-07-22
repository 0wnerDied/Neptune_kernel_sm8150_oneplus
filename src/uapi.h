/* Copyright 2015-2016 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved.
 *
 * Userspace API for WireGuard
 * ---------------------------
 *
 * ioctl(WG_GET_DEVICE, { .ifr_name: "wg0", .ifr_data: NULL }):
 *
 *     Returns the number of bytes required to hold the peers of a device (`ret_peers_size`).
 *
 * ioctl(WG_GET_DEVICE, { .ifr_name: "wg0", .ifr_data: user_pointer }):
 *
 *     Retrevies device info, peer info, and ipmask info.
 *
 *     `user_pointer` must point to a region of memory of size `sizeof(struct wgdevice) + ret_peers_size`
 *     and containing the structure `struct wgdevice { .peers_size: ret_peers_size }`.
 *
 *     Writes to `user_pointer` a succession of structs:
 *
 *         struct wgdevice { .num_peers = 3 }
 *             struct wgpeer { .num_ipmasks = 4 }
 *                 struct wgipmask
 *                 struct wgipmask
 *                 struct wgipmask
 *                 struct wgipmask
 *             struct wgpeer { .num_ipmasks = 2 }
 *                 struct wgipmask
 *                 struct wgipmask
 *             struct wgpeer { .num_ipmasks = 0 }
 *
 *     Returns 0 on success. Returns -EMSGSIZE if there is too much data for the size of passed-in
 *     memory, in which case, this should be recalculated using the call above. Returns -errno if
 *     another error occured.
 *
 * ioctl(WG_SET_DEVICE, { .ifr_name: "wg0", .ifr_data: user_pointer }):
 *
 *     Sets device info, peer info, and ipmask info.
 *
 *     `user_pointer` must point to a region of memory containing a succession of structs:
 *
 *         struct wgdevice { .num_peers = 3 }
 *             struct wgpeer { .num_ipmasks = 4 }
 *                 struct wgipmask
 *                 struct wgipmask
 *                 struct wgipmask
 *                 struct wgipmask
 *             struct wgpeer { .num_ipmasks = 2 }
 *                 struct wgipmask
 *                 struct wgipmask
 *             struct wgpeer { .num_ipmasks = 0 }
 *
 *     If `wgdevice->replace_peer_list` is true, removes all peers of device before adding new ones.
 *     If `wgpeer->remove_me` is true, the peer identified by `wgpeer->public_key` is removed.
 *     If `wgpeer->replace_ipmasks` is true, removes all ipmasks before adding new ones.
 *     If `wgdevice->private_key` is filled with zeros, no action is taken on the private key.
 *     If `wgdevice->preshared_key` is filled with zeros, no action is taken on the pre-shared key.
 *     If `wgdevice->remove_private_key` is true, the private key is removed.
 *     If `wgdevice->remove_preshared_key` is true, the pre-shared key is removed.
 *
 *     Returns 0 on success, or -errno if an error occurred.
 */


#ifndef WGUAPI_H
#define WGUAPI_H

#ifdef __linux__
#include <linux/types.h>
#else
#include <stdint.h>
typedef uint8_t __u8;
typedef uint16_t __u16;
typedef uint32_t __u32;
typedef uint64_t __u64;
typedef int32_t __s32;
#endif
#ifdef __KERNEL__
#include <linux/time.h>
#include <linux/socket.h>
#else
#include <sys/time.h>
#include <sys/socket.h>
#endif

#define WG_GET_DEVICE (SIOCDEVPRIVATE + 0)
#define WG_SET_DEVICE (SIOCDEVPRIVATE + 1)

#define WG_KEY_LEN 32

struct wgipmask {
	__s32 family;
	union {
		struct in_addr ip4;
		struct in6_addr ip6;
	};
	__u8 cidr;
};

struct wgpeer {
	__u8 public_key[WG_KEY_LEN]; /* Get/Set */

	struct sockaddr_storage endpoint; /* Get/Set */

	struct timeval last_handshake_time; /* Get */
	__u64 rx_bytes, tx_bytes; /* Get */

	__u32 remove_me : 1; /* Set */
	__u32 replace_ipmasks : 1; /* Set */

	__u16 num_ipmasks; /* Get/Set */
	__u16 persistent_keepalive_interval; /* Get/Set -- 0 = off, 0xffff = unset */
};

struct wgdevice {
	char interface[IFNAMSIZ]; /* Get */

	__u8 public_key[WG_KEY_LEN]; /* Get/Set */
	__u8 private_key[WG_KEY_LEN]; /* Get/Set */
	__u8 preshared_key[WG_KEY_LEN]; /* Get/Set */

	__u16 port; /* Get/Set */

	__u32 replace_peer_list : 1; /* Set */
	__u32 remove_private_key : 1; /* Set */
	__u32 remove_preshared_key : 1; /* Set */

	union {
		__u16 num_peers; /* Get/Set */
		__u64 peers_size; /* Get */
	};
};

/* These are simply for convenience in iterating. It allows you to write something like:
 *
 *    for_each_wgpeer(device, peer, i) {
 *        for_each_wgipmask(peer, ipmask, j) {
 *            do_something_with_ipmask(ipmask);
 *        }
 *     }
 */
#define for_each_wgpeer(__dev, __peer, __i) for ((__i) = 0, (__peer) = (typeof(__peer))((uint8_t *)(__dev) + sizeof(struct wgdevice)); \
						 (__i) < (__dev)->num_peers; \
						 ++(__i), (__peer) = (typeof(__peer))((uint8_t *)(__peer) + sizeof(struct wgpeer) + (sizeof(struct wgipmask) * (__peer)->num_ipmasks)))

#define for_each_wgipmask(__peer, __ipmask, __i) for ((__i) = 0, (__ipmask) = (typeof(__ipmask))((uint8_t *)(__peer) + sizeof(struct wgpeer)); \
						 (__i) < (__peer)->num_ipmasks; \
						 ++(__i), (__ipmask) = (typeof(__ipmask))((uint8_t *)(__ipmask) + sizeof(struct wgipmask)))

#endif
