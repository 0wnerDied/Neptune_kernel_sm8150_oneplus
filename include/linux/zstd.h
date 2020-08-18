/*
 * Copyright (c) 2016-present, Yann Collet, Facebook, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of https://github.com/facebook/zstd.
 * An additional grant of patent rights can be found in the PATENTS file in the
 * same directory.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation. This program is dual-licensed; you may select
 * either version 2 of the GNU General Public License ("GPL") or BSD license
 * ("BSD").
 */

#ifndef ZSTD_H
#define ZSTD_H

#include <linux/types.h>   /* size_t */

/*-*****************************************************************************
 * Introduction
 *
 * zstd, short for Zstandard, is a fast lossless compression algorithm,
 * targeting real-time compression scenarios at zlib-level and better
 * compression ratios. The zstd compression library provides in-memory
 * compression and decompression functions. The library supports compression
 * levels from 1 up to ZSTD_maxCLevel() which is 22. Levels >= 20, labeled
 * ultra, should be used with caution, as they require more memory.
 * Compression can be done in:
 *  - a single step, reusing a context (described as Explicit memory management)
 *  - unbounded multiple steps (described as Streaming compression)
 * The compression ratio achievable on small data can be highly improved using
 * compression with a dictionary in:
 *  - a single step (described as Simple dictionary API)
 *  - a single step, reusing a dictionary (described as Fast dictionary API)
 ******************************************************************************/

typedef enum {
	ZSTD_error_no_error,
	ZSTD_error_GENERIC,
	ZSTD_error_prefix_unknown,
	ZSTD_error_version_unsupported,
	ZSTD_error_parameter_unknown,
	ZSTD_error_frameParameter_unsupported,
	ZSTD_error_frameParameter_unsupportedBy32bits,
	ZSTD_error_frameParameter_windowTooLarge,
	ZSTD_error_compressionParameter_unsupported,
	ZSTD_error_init_missing,
	ZSTD_error_memory_allocation,
	ZSTD_error_stage_wrong,
	ZSTD_error_dstSize_tooSmall,
	ZSTD_error_srcSize_wrong,
	ZSTD_error_corruption_detected,
	ZSTD_error_checksum_wrong,
	ZSTD_error_tableLog_tooLarge,
	ZSTD_error_maxSymbolValue_tooLarge,
	ZSTD_error_maxSymbolValue_tooSmall,
	ZSTD_error_dictionary_corrupted,
	ZSTD_error_dictionary_wrong,
	ZSTD_error_dictionaryCreation_failed,
	ZSTD_error_maxCode
} ZSTD_ErrorCode;

static __attribute__((unused)) unsigned int ZSTD_isError(size_t code)
{
	return code > (size_t)-ZSTD_error_maxCode;
}

static __attribute__((unused)) ZSTD_ErrorCode ZSTD_getErrorCode(
	size_t functionResult)
{
	if (!ZSTD_isError(functionResult))
		return (ZSTD_ErrorCode)0;
	return (ZSTD_ErrorCode)(0 - functionResult);
}

typedef enum {
	ZSTD_fast,
	ZSTD_dfast,
	ZSTD_greedy,
	ZSTD_lazy,
	ZSTD_lazy2,
	ZSTD_btlazy2,
	ZSTD_btopt,
	ZSTD_btopt2
} ZSTD_strategy;

typedef struct {
	unsigned int windowLog;
	unsigned int chainLog;
	unsigned int hashLog;
	unsigned int searchLog;
	unsigned int searchLength;
	unsigned int targetLength;
	ZSTD_strategy strategy;
} ZSTD_compressionParameters;

typedef struct {
	unsigned int contentSizeFlag;
	unsigned int checksumFlag;
	unsigned int noDictIDFlag;
} ZSTD_frameParameters;

typedef struct {
	ZSTD_compressionParameters cParams;
	ZSTD_frameParameters fParams;
} ZSTD_parameters;

ZSTD_parameters ZSTD_getParams(int compressionLevel,
	unsigned long long estimatedSrcSize, size_t dictSize);

size_t ZSTD_CCtxWorkspaceBound(ZSTD_compressionParameters cParams);

typedef struct ZSTD_CCtx_s ZSTD_CCtx;

ZSTD_CCtx *ZSTD_initCCtx(void *workspace, size_t workspaceSize);

size_t ZSTD_compressCCtx(ZSTD_CCtx *ctx, void *dst, size_t dstCapacity,
	const void *src, size_t srcSize, ZSTD_parameters params);

size_t ZSTD_DCtxWorkspaceBound(void);

typedef struct ZSTD_DCtx_s ZSTD_DCtx;

ZSTD_DCtx *ZSTD_initDCtx(void *workspace, size_t workspaceSize);

size_t ZSTD_decompressDCtx(ZSTD_DCtx *ctx, void *dst, size_t dstCapacity,
	const void *src, size_t srcSize);

typedef struct ZSTD_CDict_s ZSTD_CDict;

typedef struct ZSTD_DDict_s ZSTD_DDict;

typedef struct ZSTD_inBuffer_s {
	const void *src;
	size_t size;
	size_t pos;
} ZSTD_inBuffer;

typedef struct ZSTD_outBuffer_s {
	void *dst;
	size_t size;
	size_t pos;
} ZSTD_outBuffer;

typedef struct ZSTD_CStream_s ZSTD_CStream;

typedef struct ZSTD_DStream_s ZSTD_DStream;

/* --- Constants ---*/
#define ZSTD_MAGICNUMBER            0xFD2FB528   /* >= v0.8.0 */
#define ZSTD_MAGIC_SKIPPABLE_START  0x184D2A50U

#define ZSTD_CONTENTSIZE_UNKNOWN (0ULL - 1)
#define ZSTD_CONTENTSIZE_ERROR   (0ULL - 2)

#define ZSTD_WINDOWLOG_MAX_32  27
#define ZSTD_WINDOWLOG_MAX_64  27
#define ZSTD_WINDOWLOG_MAX \
	((unsigned int)(sizeof(size_t) == 4 \
		? ZSTD_WINDOWLOG_MAX_32 \
		: ZSTD_WINDOWLOG_MAX_64))
#define ZSTD_WINDOWLOG_MIN 10
#define ZSTD_HASHLOG_MAX ZSTD_WINDOWLOG_MAX
#define ZSTD_HASHLOG_MIN        6
#define ZSTD_CHAINLOG_MAX     (ZSTD_WINDOWLOG_MAX+1)
#define ZSTD_CHAINLOG_MIN      ZSTD_HASHLOG_MIN
#define ZSTD_HASHLOG3_MAX      17
#define ZSTD_SEARCHLOG_MAX    (ZSTD_WINDOWLOG_MAX-1)
#define ZSTD_SEARCHLOG_MIN      1
/* only for ZSTD_fast, other strategies are limited to 6 */
#define ZSTD_SEARCHLENGTH_MAX   7
/* only for ZSTD_btopt, other strategies are limited to 4 */
#define ZSTD_SEARCHLENGTH_MIN   3
#define ZSTD_TARGETLENGTH_MIN   4
#define ZSTD_TARGETLENGTH_MAX 999

/* for static allocation */
#define ZSTD_FRAMEHEADERSIZE_MAX 18
#define ZSTD_FRAMEHEADERSIZE_MIN  6
static const size_t ZSTD_frameHeaderSize_prefix = 5;
static const size_t ZSTD_frameHeaderSize_min = ZSTD_FRAMEHEADERSIZE_MIN;
static const size_t ZSTD_frameHeaderSize_max = ZSTD_FRAMEHEADERSIZE_MAX;
/* magic number + skippable frame length */
static const size_t ZSTD_skippableHeaderSize = 8;

typedef struct {
	unsigned long long frameContentSize;
	unsigned int windowSize;
	unsigned int dictID;
	unsigned int checksumFlag;
} ZSTD_frameParams;

typedef enum {
	ZSTDnit_frameHeader,
	ZSTDnit_blockHeader,
	ZSTDnit_block,
	ZSTDnit_lastBlock,
	ZSTDnit_checksum,
	ZSTDnit_skippableFrame
} ZSTD_nextInputType_e;

/* Define for static allocation */
#define ZSTD_BLOCKSIZE_ABSOLUTEMAX (128 * 1024)

#endif  /* ZSTD_H */
