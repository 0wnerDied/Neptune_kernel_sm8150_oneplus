/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 * Copyright 2011~2018 Bosch Sensortec GmbH All Rights Reserved
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#ifndef _SMI230_LOG_H
#define _SMI230_LOG_H

#include <linux/kernel.h>

/*! @ trace functions
 @{*/
/*! ERROR LOG LEVEL */
#define LOG_LEVEL_E 3
/*! NOTICE LOG LEVEL */
#define LOG_LEVEL_N 5
/*! INFORMATION LOG LEVEL */
#define LOG_LEVEL_I 6
/*! DEBUG LOG LEVEL */
#define LOG_LEVEL_D 7
/*! DEBUG_FWDL LOG LEVEL */
#define LOG_LEVEL_DF 10
/*! DEBUG_DATA LOG LEVEL */
#define LOG_LEVEL_DA 15
/*! ALL LOG LEVEL */
#define LOG_LEVEL_A 20

#ifndef MODULE_TAG
/*! MODULE TAG DEFINATION */
#define MODULE_TAG "<BS_LOG>"
#endif

#ifndef LOG_LEVEL
/*! LOG LEVEL DEFINATION */
#define LOG_LEVEL LOG_LEVEL_A
#endif

#ifdef BOSCH_DRIVER_LOG_FUNC
	#ifdef BSLOG_VAR_DEF
		uint8_t debug_log_level = LOG_LEVEL;
	#else
		extern uint8_t debug_log_level;
	#endif

	/*! print error message */
	#define PERR(fmt, args...) do\
	{\
		if (debug_log_level >= LOG_LEVEL_E)\
			printk(KERN_INFO "\n" "[E]" KERN_ERR MODULE_TAG \
				"<%s><%d>" fmt "\n", __func__, __LINE__, ##args);\
	} while (0)

	/*! print notice message */
	#define PNOTICE(fmt, args...) do\
	{\
		if (debug_log_level >= LOG_LEVEL_N)\
			printk(KERN_INFO "\n" "[N]" KERN_NOTICE MODULE_TAG \
				"<%s><%d>" fmt "\n", __func__, __LINE__, ##args);\
	} while (0)

	/*! print information message */
	#define PINFO(fmt, args...) do\
	{\
		if (debug_log_level >= LOG_LEVEL_I)\
			printk(KERN_INFO "\n" "[I]" KERN_INFO MODULE_TAG \
				"<%s><%d>" fmt "\n", __func__, __LINE__, ##args);\
	} while (0)

	/*! print debug message */
	#define PDEBUG(fmt, args...) do\
	{\
		if (debug_log_level >= LOG_LEVEL_D)\
			printk(KERN_INFO "\n" "[D]" KERN_DEBUG MODULE_TAG \
				"<%s><%d>" fmt "\n", __func__, __LINE__, ##args);\
	} while (0)

	/*! print debug fw download message */
	#define PDEBUG_FWDL(fmt, args...) do\
	{\
		if (debug_log_level >= LOG_LEVEL_DF)\
			printk(KERN_INFO "\n" "[DF]" KERN_DEBUG MODULE_TAG \
				"<%s><%d>" fmt "\n", __func__, __LINE__, ##args);\
	} while (0)

	/*! print debug data log message */
	#define PDEBUG_DLOG(fmt, args...) do\
	{\
		if (debug_log_level >= LOG_LEVEL_DA)\
			printk(KERN_INFO "\n" "[DA]" KERN_DEBUG MODULE_TAG \
				"<%s><%d>" fmt "\n", __func__, __LINE__, ##args);\
	} while (0)

	void set_debug_log_level(uint8_t level);
	uint8_t get_debug_log_level(void);

#else

	#if (LOG_LEVEL >= LOG_LEVEL_E)
	/*! print error message */
	#define PERR(fmt, args...) \
		printk(KERN_INFO "\n" "[E]" KERN_ERR MODULE_TAG \
		"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
	#else
	/*! invalid message */
	#define PERR(fmt, args...)
	#endif

	#if (LOG_LEVEL >= LOG_LEVEL_N)
	/*! print notice message */
	#define PNOTICE(fmt, args...) \
		printk(KERN_INFO "\n" "[N]" KERN_NOTICE MODULE_TAG \
		"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
	#else
	/*! invalid message */
	#define PNOTICE(fmt, args...)
	#endif

	#if (LOG_LEVEL >= LOG_LEVEL_I)
	/*! print information message */
	#define PINFO(fmt, args...) printk(KERN_INFO "\n" "[I]" KERN_INFO MODULE_TAG \
		"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
	#else
	/*! invalid message */
	#define PINFO(fmt, args...)
	#endif

	#if (LOG_LEVEL >= LOG_LEVEL_D)
	/*! print debug message */
	#define PDEBUG(fmt, args...) printk(KERN_INFO "\n" "[D]" KERN_DEBUG MODULE_TAG \
		"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
	#else
	/*! invalid message */
	#define PDEBUG(fmt, args...)
	#endif

	#if (LOG_LEVEL >= LOG_LEVEL_DF)
	/*! print debug fw download message */
	#define PDEBUG_FWDL(fmt, args...) printk(KERN_INFO "\n" "[DF]" KERN_DEBUG MODULE_TAG \
		"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
	#else
	/*! invalid message */
	#define PDEBUG_FWDL(fmt, args...)
	#endif

	#if (LOG_LEVEL >= LOG_LEVEL_DA)
	/*! print debug data log message */
	#define PDEBUG_DLOG(fmt, args...) printk(KERN_INFO "\n" "[DA]" KERN_DEBUG MODULE_TAG \
		"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
	#else
	/*! invalid message */
	#define PDEBUG_DLOG(fmt, args...)
	#endif

	#define set_debug_log_level(level) {}
	#define get_debug_log_level() (LOG_LEVEL)

#endif

#endif/*_SMI230_LOG_H*/
/*@}*/
