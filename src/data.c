/* Copyright 2015-2016 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved. */

#include "wireguard.h"
#include "noise.h"
#include "messages.h"
#include "packets.h"
#include "hashtables.h"
#include <crypto/algapi.h>
#include <net/xfrm.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/bitmap.h>
#include <linux/scatterlist.h>

/* This is RFC6479, a replay detection bitmap algorithm that avoids bitshifts */
static inline bool counter_validate(union noise_counter *counter, u64 their_counter)
{
	bool ret = false;
	unsigned long index, index_current, top, i;
	spin_lock_bh(&counter->receive.lock);

	if (unlikely(counter->receive.counter >= REJECT_AFTER_MESSAGES + 1 || their_counter >= REJECT_AFTER_MESSAGES))
		goto out;

	++their_counter;

	if (unlikely((COUNTER_WINDOW_SIZE + their_counter) < counter->receive.counter))
		goto out;

	index = their_counter >> ilog2(COUNTER_REDUNDANT_BITS);

	if (likely(their_counter > counter->receive.counter)) {
		index_current = counter->receive.counter >> ilog2(COUNTER_REDUNDANT_BITS);
		top = min_t(unsigned long, index - index_current, COUNTER_BITS_TOTAL / BITS_PER_LONG);
		for (i = 1; i <= top; ++i)
			counter->receive.backtrack[(i + index_current) & ((COUNTER_BITS_TOTAL / BITS_PER_LONG) - 1)] = 0;
		counter->receive.counter = their_counter;
	}

	index &= (COUNTER_BITS_TOTAL / BITS_PER_LONG) - 1;
	ret = !test_and_set_bit(their_counter & (COUNTER_REDUNDANT_BITS - 1), &counter->receive.backtrack[index]);

out:
	spin_unlock_bh(&counter->receive.lock);
	return ret;
}
#include "selftest/counter.h"

static inline size_t skb_padding(struct sk_buff *skb)
{
	/* We do this modulo business with the MTU, just in case the networking layer
	 * gives us a packet that's bigger than the MTU. Now that we support GSO, this
	 * shouldn't be a real problem, and this can likely be removed. But, caution! */
	size_t last_unit = skb->len % skb->dev->mtu;
	size_t padded_size = (last_unit + MESSAGE_PADDING_MULTIPLE - 1) & ~(MESSAGE_PADDING_MULTIPLE - 1);
	if (padded_size > skb->dev->mtu)
		padded_size = skb->dev->mtu;
	return padded_size - last_unit;
}

static inline void skb_reset(struct sk_buff *skb)
{
	skb_scrub_packet(skb, false);
	memset(&skb->headers_start, 0, offsetof(struct sk_buff, headers_end) - offsetof(struct sk_buff, headers_start));
	skb->queue_mapping = 0;
	skb->nohdr = 0;
	skb->peeked = 0;
	skb->mac_len = 0;
	skb->dev = NULL;
	skb->hdr_len = skb_headroom(skb);
	skb->mac_header = (typeof(skb->mac_header))~0U;
	skb->transport_header = (typeof(skb->transport_header))~0U;
	skb_reset_network_header(skb);
}

static inline void skb_encrypt(struct sk_buff *skb, struct packet_data_encryption_ctx *ctx)
{
	struct scatterlist sg[ctx->num_frags]; /* This should be bound to at most 128 by the caller. */
	struct message_data *header;

	/* We have to remember to add the checksum to the innerpacket, in case the receiver forwards it. */
	if (likely(!skb_checksum_setup(skb, true)))
		skb_checksum_help(skb);

	/* Only after checksumming can we safely add on the padding at the end and the header. */
	header = (struct message_data *)skb_push(skb, sizeof(struct message_data));
	header->header.type = MESSAGE_DATA;
	header->key_idx = ctx->keypair->remote_index;
	header->counter = cpu_to_le64(ctx->nonce);
	pskb_put(skb, ctx->trailer, ctx->trailer_len);

	/* Now we can encrypt the scattergather segments */
	sg_init_table(sg, ctx->num_frags);
	skb_to_sgvec(skb, sg, sizeof(struct message_data), noise_encrypted_len(ctx->plaintext_len));
	chacha20poly1305_encrypt_sg(sg, sg, ctx->plaintext_len, NULL, 0, ctx->nonce, ctx->keypair->sending.key);

	/* When we're done, we free the reference to the key pair */
	noise_keypair_put(ctx->keypair);
}

static inline bool skb_decrypt(struct sk_buff *skb, unsigned int num_frags, uint64_t nonce, struct noise_symmetric_key *key)
{
	struct scatterlist sg[num_frags]; /* This should be bound to at most 128 by the caller. */

	if (unlikely(!key))
		return false;

	if (unlikely(!key->is_valid || time_is_before_eq_jiffies64(key->birthdate + REJECT_AFTER_TIME) || key->counter.receive.counter >= REJECT_AFTER_MESSAGES)) {
		key->is_valid = false;
		return false;
	}

	sg_init_table(sg, num_frags);
	skb_to_sgvec(skb, sg, 0, skb->len);

	if (!chacha20poly1305_decrypt_sg(sg, sg, skb->len, NULL, 0, nonce, key->key))
		return false;

	return pskb_trim(skb, skb->len - noise_encrypted_len(0)) == 0;
}

static inline bool get_encryption_nonce(uint64_t *nonce, struct noise_symmetric_key *key)
{
	if (unlikely(!key))
		return false;

	if (unlikely(!key->is_valid || time_is_before_eq_jiffies64(key->birthdate + REJECT_AFTER_TIME))) {
		key->is_valid = false;
		return false;
	}

	*nonce = atomic64_inc_return(&key->counter.counter) - 1;
	if (*nonce >= REJECT_AFTER_MESSAGES) {
		key->is_valid = false;
		return false;
	}

	return true;
}

#ifdef CONFIG_WIREGUARD_PARALLEL
static void do_encryption(struct padata_priv *padata)
{
	struct packet_data_encryption_ctx *ctx = container_of(padata, struct packet_data_encryption_ctx, padata);

	skb_encrypt(ctx->skb, ctx);
	skb_reset(ctx->skb);

	padata_do_serial(padata);
}

static void finish_encryption(struct padata_priv *padata)
{
	struct packet_data_encryption_ctx *ctx = container_of(padata, struct packet_data_encryption_ctx, padata);

	ctx->callback(ctx->skb, ctx->peer);
}

static inline int start_encryption(struct padata_instance *padata, struct padata_priv *priv, int cb_cpu)
{
	memset(priv, 0, sizeof(struct padata_priv));
	priv->parallel = do_encryption;
	priv->serial = finish_encryption;
	return padata_do_parallel(padata, priv, cb_cpu);
}

static inline unsigned int choose_cpu(__le32 key)
{
	unsigned int cpu_index, cpu, cb_cpu;

	/* This ensures that packets encrypted to the same key are sent in-order. */
	cpu_index = ((__force unsigned int)key) % cpumask_weight(cpu_online_mask);
	cb_cpu = cpumask_first(cpu_online_mask);
	for (cpu = 0; cpu < cpu_index; ++cpu)
		cb_cpu = cpumask_next(cb_cpu, cpu_online_mask);

	return cb_cpu;
}
#endif

int packet_create_data(struct sk_buff *skb, struct wireguard_peer *peer, void(*callback)(struct sk_buff *, struct wireguard_peer *), bool parallel)
{
	int ret = -ENOKEY;
	struct noise_keypair *keypair;
	struct packet_data_encryption_ctx *ctx = NULL;
	u64 nonce;
	struct sk_buff *trailer = NULL;
	size_t plaintext_len, padding_len, trailer_len;
	unsigned int num_frags;

	rcu_read_lock();
	keypair = rcu_dereference(peer->keypairs.current_keypair);
	if (unlikely(!keypair))
		goto err_rcu;
	kref_get(&keypair->refcount);
	rcu_read_unlock();

	if (unlikely(!get_encryption_nonce(&nonce, &keypair->sending)))
		goto err;

	padding_len = skb_padding(skb);
	trailer_len = padding_len + noise_encrypted_len(0);
	plaintext_len = skb->len + padding_len;

	/* Expand data section to have room for padding and auth tag */
	ret = skb_cow_data(skb, trailer_len, &trailer);
	if (unlikely(ret < 0))
		goto err;
	num_frags = ret;
	ret = -ENOMEM;
	if (unlikely(num_frags > 128))
		goto err;

	/* Set the padding to zeros, and make sure it and the auth tag are part of the skb */
	memset(skb_tail_pointer(trailer), 0, padding_len);

	/* Expand head section to have room for our header and the network stack's headers,
	 * plus our key and nonce in the head. */
	ret = skb_cow_head(skb, DATA_PACKET_HEAD_ROOM);
	if (unlikely(ret < 0))
		goto err;

	ctx = (struct packet_data_encryption_ctx *)skb->head;
	ctx->skb = skb;
	ctx->callback = callback;
	ctx->peer = peer;
	ctx->num_frags = num_frags;
	ctx->trailer_len = trailer_len;
	ctx->trailer = trailer;
	ctx->plaintext_len = plaintext_len;
	ctx->nonce = nonce;
	ctx->keypair = keypair;

#ifdef CONFIG_WIREGUARD_PARALLEL
	if (parallel && cpumask_weight(cpu_online_mask) > 1) {
		unsigned int cpu = choose_cpu(keypair->remote_index);
		ret = start_encryption(peer->device->parallel_send, &ctx->padata, cpu);
		if (unlikely(ret < 0))
			goto err;
	} else
#endif
	{
		skb_encrypt(skb, ctx);
		skb_reset(skb);
		callback(skb, peer);
	}
	return 0;

err:
	noise_keypair_put(keypair);
	return ret;
err_rcu:
	rcu_read_unlock();
	return ret;
}

struct packet_data_decryption_ctx {
	struct padata_priv padata;
	struct sk_buff *skb;
	void (*callback)(struct sk_buff *skb, struct wireguard_peer *, struct sockaddr_storage *, bool used_new_key, int err);
	struct noise_keypair *keypair;
	struct sockaddr_storage addr;
	uint64_t nonce;
	unsigned int num_frags;
	int ret;
};

static void begin_decrypt_packet(struct packet_data_decryption_ctx *ctx)
{
	if (unlikely(!skb_decrypt(ctx->skb, ctx->num_frags, ctx->nonce, &ctx->keypair->receiving)))
		goto err;

	skb_reset(ctx->skb);
	ctx->ret = 0;
	return;

err:
	ctx->ret = -ENOKEY;
	peer_put(ctx->keypair->entry.peer);
}

static void finish_decrypt_packet(struct packet_data_decryption_ctx *ctx)
{
	struct noise_keypairs *keypairs;
	bool used_new_key = false;
	int ret = ctx->ret;
	if (ret)
		goto err;

	keypairs = &ctx->keypair->entry.peer->keypairs;
	ret = counter_validate(&ctx->keypair->receiving.counter, ctx->nonce) ? 0 : -ERANGE;

	if (likely(!ret))
		used_new_key = noise_received_with_keypair(&ctx->keypair->entry.peer->keypairs, ctx->keypair);
	else {
		net_dbg_ratelimited("Packet has invalid nonce %Lu (max %Lu)\n", ctx->nonce, ctx->keypair->receiving.counter.receive.counter);
		peer_put(ctx->keypair->entry.peer);
		goto err;
	}

	noise_keypair_put(ctx->keypair);
	ctx->callback(ctx->skb, ctx->keypair->entry.peer, &ctx->addr, used_new_key, 0);
	return;

err:
	noise_keypair_put(ctx->keypair);
	ctx->callback(ctx->skb, NULL, NULL, false, ret);
}

#ifdef CONFIG_WIREGUARD_PARALLEL
static void do_decryption(struct padata_priv *padata)
{
	struct packet_data_decryption_ctx *ctx = container_of(padata, struct packet_data_decryption_ctx, padata);
	begin_decrypt_packet(ctx);
	padata_do_serial(padata);
}

static void finish_decryption(struct padata_priv *padata)
{
	struct packet_data_decryption_ctx *ctx = container_of(padata, struct packet_data_decryption_ctx, padata);
	finish_decrypt_packet(ctx);
	kfree(ctx);
}

static inline int start_decryption(struct padata_instance *padata, struct padata_priv *priv, int cb_cpu)
{
	priv->parallel = do_decryption;
	priv->serial = finish_decryption;
	return padata_do_parallel(padata, priv, cb_cpu);
}
#endif

void packet_consume_data(struct sk_buff *skb, size_t offset, struct wireguard_device *wg, void(*callback)(struct sk_buff *skb, struct wireguard_peer *, struct sockaddr_storage *, bool used_new_key, int err))
{
	int ret;
	struct sockaddr_storage addr = { 0 };
	unsigned int num_frags;
	struct sk_buff *trailer;
	struct message_data *header;
	struct noise_keypair *keypair;
	uint64_t nonce;
	__le32 idx;

	ret = socket_addr_from_skb(&addr, skb);
	if (unlikely(ret < 0))
		goto err;

	ret = -ENOMEM;
	if (unlikely(!pskb_may_pull(skb, offset + sizeof(struct message_data))))
		goto err;

	header = (struct message_data *)(skb->data + offset);
	offset += sizeof(struct message_data);
	skb_pull(skb, offset);

	idx = header->key_idx;
	nonce = le64_to_cpu(header->counter);

	ret = skb_cow_data(skb, 0, &trailer);
	if (unlikely(ret < 0))
		goto err;
	num_frags = ret;
	ret = -ENOMEM;
	if (unlikely(num_frags > 128))
		goto err;
	ret = -EINVAL;
	rcu_read_lock();
	keypair = (struct noise_keypair *)index_hashtable_lookup(&wg->index_hashtable, INDEX_HASHTABLE_KEYPAIR, idx);
	if (unlikely(!keypair)) {
		rcu_read_unlock();
		goto err;
	}
	kref_get(&keypair->refcount);
	rcu_read_unlock();
#ifdef CONFIG_WIREGUARD_PARALLEL
	if (cpumask_weight(cpu_online_mask) > 1) {
		struct packet_data_decryption_ctx *ctx;
		unsigned int cpu = choose_cpu(idx);

		ret = -ENOMEM;
		ctx = kzalloc(sizeof(struct packet_data_decryption_ctx), GFP_ATOMIC);
		if (unlikely(!ctx))
			goto err_peer;

		ctx->skb = skb;
		ctx->keypair = keypair;
		ctx->callback = callback;
		ctx->nonce = nonce;
		ctx->num_frags = num_frags;
		ctx->addr = addr;
		ret = start_decryption(wg->parallel_receive, &ctx->padata, cpu);
		if (unlikely(ret)) {
			kfree(ctx);
			goto err_peer;
		}
	} else
#endif
	{
		struct packet_data_decryption_ctx ctx = {
			.skb = skb,
			.keypair = keypair,
			.callback = callback,
			.nonce = nonce,
			.num_frags = num_frags,
			.addr = addr
		};
		begin_decrypt_packet(&ctx);
		finish_decrypt_packet(&ctx);
	}
	return;

#ifdef CONFIG_WIREGUARD_PARALLEL
err_peer:
	peer_put(keypair->entry.peer);
	noise_keypair_put(keypair);
#endif
err:
	callback(skb, NULL, NULL, false, ret);
}
