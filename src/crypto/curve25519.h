/* Copyright 2015-2016 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved. */

#ifndef CURVE25519_H
#define CURVE25519_H

#include <linux/types.h>

enum curve25519_lengths {
	CURVE25519_POINT_SIZE = 32
};

void curve25519(uint8_t mypublic[static CURVE25519_POINT_SIZE], const uint8_t secret[static CURVE25519_POINT_SIZE], const uint8_t basepoint[static CURVE25519_POINT_SIZE]);
void curve25519_generate_secret(uint8_t secret[static CURVE25519_POINT_SIZE]);
void curve25519_generate_public(uint8_t pub[static CURVE25519_POINT_SIZE], const uint8_t secret[static CURVE25519_POINT_SIZE]);

#ifdef DEBUG
bool curve25519_selftest(void);
#endif

#endif
