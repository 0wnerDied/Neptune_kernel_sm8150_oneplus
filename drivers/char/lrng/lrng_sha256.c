// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Backend for the LRNG providing the SHA-256 implementation that can be used
 * without the kernel crypto API available including during early boot and in
 * atomic contexts.
 *
 * Copyright (C) 2022, Stephan Mueller <smueller@chronox.de>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/lrng.h>
#include <crypto/sha.h>

#include <linux/bitops.h>
#include <linux/string.h>
#include <asm/unaligned.h>

#define SHA256_H0	0x6a09e667UL
#define SHA256_H1	0xbb67ae85UL
#define SHA256_H2	0x3c6ef372UL
#define SHA256_H3	0xa54ff53aUL
#define SHA256_H4	0x510e527fUL
#define SHA256_H5	0x9b05688cUL
#define SHA256_H6	0x1f83d9abUL
#define SHA256_H7	0x5be0cd19UL

static const u32 SHA256_K[] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
	0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
	0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
	0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
	0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
	0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
	0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
	0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
	0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static inline u32 Ch(u32 x, u32 y, u32 z)
{
	return z ^ (x & (y ^ z));
}

static inline u32 Maj(u32 x, u32 y, u32 z)
{
	return (x & y) | (z & (x | y));
}

#define e0(x)	(ror32(x, 2) ^ ror32(x, 13) ^ ror32(x, 22))
#define e1(x)	(ror32(x, 6) ^ ror32(x, 11) ^ ror32(x, 25))
#define s0(x)	(ror32(x, 7) ^ ror32(x, 18) ^ (x >> 3))
#define s1(x)	(ror32(x, 17) ^ ror32(x, 19) ^ (x >> 10))

static inline void LOAD_OP(int I, u32 *W, const u8 *input)
{
	W[I] = get_unaligned_be32((__u32 *)input + I);
}

static inline void BLEND_OP(int I, u32 *W)
{
	W[I] = s1(W[I-2]) + W[I-7] + s0(W[I-15]) + W[I-16];
}

#define SHA256_ROUND(i, a, b, c, d, e, f, g, h) do {		\
	u32 t1, t2;						\
	t1 = h + e1(e) + Ch(e, f, g) + SHA256_K[i] + W[i];	\
	t2 = e0(a) + Maj(a, b, c);				\
	d += t1;						\
	h = t1 + t2;						\
} while (0)

static inline void sha256_transform(u32 *state, const u8 *input, u32 *W)
{
	u32 a, b, c, d, e, f, g, h;
	int i;

	/* load the input */
	for (i = 0; i < 16; i += 8) {
		LOAD_OP(i + 0, W, input);
		LOAD_OP(i + 1, W, input);
		LOAD_OP(i + 2, W, input);
		LOAD_OP(i + 3, W, input);
		LOAD_OP(i + 4, W, input);
		LOAD_OP(i + 5, W, input);
		LOAD_OP(i + 6, W, input);
		LOAD_OP(i + 7, W, input);
	}

	/* now blend */
	for (i = 16; i < 64; i += 8) {
		BLEND_OP(i + 0, W);
		BLEND_OP(i + 1, W);
		BLEND_OP(i + 2, W);
		BLEND_OP(i + 3, W);
		BLEND_OP(i + 4, W);
		BLEND_OP(i + 5, W);
		BLEND_OP(i + 6, W);
		BLEND_OP(i + 7, W);
	}

	/* load the state into our registers */
	a = state[0];  b = state[1];  c = state[2];  d = state[3];
	e = state[4];  f = state[5];  g = state[6];  h = state[7];

	/* now iterate */
	for (i = 0; i < 64; i += 8) {
		SHA256_ROUND(i + 0, a, b, c, d, e, f, g, h);
		SHA256_ROUND(i + 1, h, a, b, c, d, e, f, g);
		SHA256_ROUND(i + 2, g, h, a, b, c, d, e, f);
		SHA256_ROUND(i + 3, f, g, h, a, b, c, d, e);
		SHA256_ROUND(i + 4, e, f, g, h, a, b, c, d);
		SHA256_ROUND(i + 5, d, e, f, g, h, a, b, c);
		SHA256_ROUND(i + 6, c, d, e, f, g, h, a, b);
		SHA256_ROUND(i + 7, b, c, d, e, f, g, h, a);
	}

	state[0] += a; state[1] += b; state[2] += c; state[3] += d;
	state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

static inline void sha256_init(struct sha256_state *sctx)
{
	sctx->state[0] = SHA256_H0;
	sctx->state[1] = SHA256_H1;
	sctx->state[2] = SHA256_H2;
	sctx->state[3] = SHA256_H3;
	sctx->state[4] = SHA256_H4;
	sctx->state[5] = SHA256_H5;
	sctx->state[6] = SHA256_H6;
	sctx->state[7] = SHA256_H7;
	sctx->count = 0;
}

static inline void sha256_update(struct sha256_state *sctx, const u8 *data, unsigned int len)
{
	unsigned int partial, done;
	const u8 *src;
	u32 W[64];

	partial = sctx->count & 0x3f;
	sctx->count += len;
	done = 0;
	src = data;

	if ((partial + len) > 63) {
		if (partial) {
			done = -partial;
			memcpy(sctx->buf + partial, data, done + 64);
			src = sctx->buf;
		}

		do {
			sha256_transform(sctx->state, src, W);
			done += 64;
			src = data + done;
		} while (done + 63 < len);

		memzero_explicit(W, sizeof(W));

		partial = 0;
	}
	memcpy(sctx->buf + partial, src, len - done);
}

static inline void __sha256_final(struct sha256_state *sctx, u8 *out, int digest_words)
{
	__be32 *dst = (__be32 *)out;
	__be64 bits;
	unsigned int index, pad_len;
	int i;
	static const u8 padding[64] = { 0x80, };

	/* Save number of bits */
	bits = cpu_to_be64(sctx->count << 3);

	/* Pad out to 56 mod 64. */
	index = sctx->count & 0x3f;
	pad_len = (index < 56) ? (56 - index) : ((64+56) - index);
	sha256_update(sctx, padding, pad_len);

	/* Append length (before padding) */
	sha256_update(sctx, (const u8 *)&bits, sizeof(bits));

	/* Store state in digest */
	for (i = 0; i < digest_words; i++)
		put_unaligned_be32(sctx->state[i], &dst[i]);

	/* Zeroize sensitive information. */
	memzero_explicit(sctx, sizeof(*sctx));
}

static inline void sha256_final(struct sha256_state *sctx, u8 *out)
{
	__sha256_final(sctx, out, 8);
}

#include "lrng_sha.h"

static u32 lrng_sha256_hash_digestsize(void *hash)
{
	return SHA256_DIGEST_SIZE;
}

static int lrng_sha256_hash_init(struct shash_desc *shash, void *hash)
{
	/*
	 * We do not need a TFM - we only need sufficient space for
	 * struct sha256_state on the stack.
	 */
	sha256_init(shash_desc_ctx(shash));
	return 0;
}

static int lrng_sha256_hash_update(struct shash_desc *shash,
				   const u8 *inbuf, u32 inbuflen)
{
	sha256_update(shash_desc_ctx(shash), inbuf, inbuflen);
	return 0;
}

static int lrng_sha256_hash_final(struct shash_desc *shash, u8 *digest)
{
	sha256_final(shash_desc_ctx(shash), digest);
	return 0;
}

static const char *lrng_sha256_hash_name(void)
{
	return "SHA-256";
}

static void lrng_sha256_hash_desc_zero(struct shash_desc *shash)
{
	memzero_explicit(shash_desc_ctx(shash), sizeof(struct sha256_state));
}

static void *lrng_sha256_hash_alloc(void)
{
	pr_info("Hash %s allocated\n", lrng_sha256_hash_name());
	return NULL;
}

static void lrng_sha256_hash_dealloc(void *hash) { }

const struct lrng_hash_cb lrng_sha_hash_cb = {
	.hash_name		= lrng_sha256_hash_name,
	.hash_alloc		= lrng_sha256_hash_alloc,
	.hash_dealloc		= lrng_sha256_hash_dealloc,
	.hash_digestsize	= lrng_sha256_hash_digestsize,
	.hash_init		= lrng_sha256_hash_init,
	.hash_update		= lrng_sha256_hash_update,
	.hash_final		= lrng_sha256_hash_final,
	.hash_desc_zero		= lrng_sha256_hash_desc_zero,
};
