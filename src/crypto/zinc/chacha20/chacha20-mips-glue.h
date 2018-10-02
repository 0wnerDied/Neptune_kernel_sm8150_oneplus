/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Copyright (C) 2015-2018 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved.
 */

asmlinkage void chacha20_mips(u32 state[16], u8 *out, const u8 *in,
			      const size_t len);

static void __init chacha20_fpu_init(void)
{
}

static inline bool chacha20_arch(struct chacha20_ctx *state, u8 *dst,
				 const u8 *src, const size_t len,
				 simd_context_t *simd_context)
{
	chacha20_mips((u32 *)state, dst, src, len);
	return true;
}


static inline bool hchacha20_arch(u32 derived_key[CHACHA20_KEY_WORDS],
				  const u8 nonce[HCHACHA20_NONCE_SIZE],
				  const u8 key[HCHACHA20_KEY_SIZE],
				  simd_context_t *simd_context)
{
	return false;
}
