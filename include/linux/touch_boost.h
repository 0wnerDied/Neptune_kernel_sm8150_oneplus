// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2021 atndko <z1281552865@gmail.com>
 */

#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/msm_drm_notify.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

#ifndef _TOUCH_BOOST_H_
#define _TOUCH_BOOST_H_

enum {
	SCREEN_ON,
	TOUCH_PHC
};

struct boost_drv {
	struct delayed_work phc_unboost;
	struct notifier_block msm_drm_notif;
	wait_queue_head_t boost_waitq;
	unsigned long state;
};


#endif /* _TOUCH_BOOST_H_ */
