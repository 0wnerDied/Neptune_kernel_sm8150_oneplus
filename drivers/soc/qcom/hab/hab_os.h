/* Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __HAB_OS_H
#define __HAB_OS_H

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "hab:%s:%d " fmt, __func__, __LINE__

#include <linux/types.h>

#include <linux/habmm.h>
#include <linux/hab_ioctl.h>

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rbtree.h>
#include <linux/idr.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/devcoredump.h>

#if IS_ENABLED(CONFIG_MSM_BOOT_TIME_MARKER)
#include <soc/qcom/boot_stats.h>
#else
static inline unsigned long long msm_timer_get_sclk_ticks(void)
{
	return 0;
}
#endif

#endif /*__HAB_OS_H*/
