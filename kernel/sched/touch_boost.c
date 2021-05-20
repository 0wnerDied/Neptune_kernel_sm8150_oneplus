// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020-2021 atndko <z1281552865@gmail.com>
 */

#define pr_fmt(fmt) "touch_boost: " fmt

#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/msm_drm_notify.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

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

static void phc_unboost_worker(struct work_struct *work);

static struct boost_drv boost_drv_g __read_mostly = {
	.phc_unboost = __DELAYED_WORK_INITIALIZER(boost_drv_g.phc_unboost,
						  phc_unboost_worker, 0),
	.boost_waitq = __WAIT_QUEUE_HEAD_INITIALIZER(boost_drv_g.boost_waitq)
};

static void __touch_boost_kick_phc(struct boost_drv *boost)
{
	if (!test_bit(SCREEN_ON, &boost->state))
		return;

	set_bit(TOUCH_PHC, &boost->state);

	if (!mod_delayed_work(system_unbound_wq, &boost->phc_unboost,
			      msecs_to_jiffies(300))) {
		set_bit(TOUCH_PHC, &boost->state);
		wake_up(&boost->boost_waitq);
	}
}

static void touch_boost_phc_event(struct boost_drv *boost)
{
	if (!test_bit(TOUCH_PHC, &boost->state))
		set_prefer_high_cap("top-app", false);
	else
		set_prefer_high_cap("top-app", true);
}

static void phc_unboost_worker(struct work_struct *work)
{
	struct boost_drv *boost = container_of(to_delayed_work(work),
					   typeof(*boost), phc_unboost);

	clear_bit(TOUCH_PHC, &boost->state);
	wake_up(&boost->boost_waitq);
}

static int touch_boost_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	struct boost_drv *boost = data;
	unsigned long state_bef = 0;

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

	while (1) {
		bool sho_stop = false;
		unsigned long state_nex;

		wait_event(boost->boost_waitq,
				(state_nex = READ_ONCE(boost->state)) != state_bef
				|| (sho_stop = kthread_should_stop()));

		if (sho_stop)
			break;

		if (state_bef != state_nex) {
			touch_boost_phc_event(boost);
			state_bef = state_nex;
		}
	}

	return 0;
}

static int msm_drm_notifier_sp(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	struct boost_drv *boost = container_of(nb, typeof(*boost), msm_drm_notif);
	struct msm_drm_notifier *evdata = data;
	int *blank = evdata->data;

	if (action != MSM_DRM_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	if (*blank == MSM_DRM_BLANK_UNBLANK_CUST) {
		set_bit(SCREEN_ON, &boost->state);
	} else if (*blank == MSM_DRM_BLANK_POWERDOWN_CUST) {
		clear_bit(SCREEN_ON, &boost->state);
		wake_up(&boost->boost_waitq);
	}

	return NOTIFY_OK;
}

static void touch_boost_input_event(struct input_handle *handle,
					unsigned int type, unsigned int code,
					int value)
{
	struct boost_drv *boost = handle->handler->private;

	__touch_boost_kick_phc(boost);
}

static int touch_boost_input_connect(struct input_handler *handler,
					 struct input_dev *dev,
					 const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "touch_boost_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void touch_boost_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id touch_boost_ids[] = {
	/* Multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) }
	},
	/* Touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) }
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) }
	},
	{ }
};

static struct input_handler touch_boost_input_handler = {
	.event		= touch_boost_input_event,
	.connect	= touch_boost_input_connect,
	.disconnect	= touch_boost_input_disconnect,
	.name		= "touch_boost_handler",
	.id_table	= touch_boost_ids
};

static int __init touch_boost_init(void)
{
	struct boost_drv *boost = &boost_drv_g;
	struct task_struct *thread;
	int ret;

	set_bit(SCREEN_ON, &boost->state);

	touch_boost_input_handler.private = boost;
	ret = input_register_handler(&touch_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		return ret;
	}

	boost->msm_drm_notif.notifier_call = msm_drm_notifier_sp;
	boost->msm_drm_notif.priority = INT_MAX;
	ret = msm_drm_register_client(&boost->msm_drm_notif);
	if (ret) {
		pr_err("Failed to register msm_drm notifier, err: %d\n", ret);
		goto unregister_handler;
	}

	thread = kthread_run_perf_critical(cpu_perf_mask,
							  touch_boost_thread,
							  boost, "touch_boostd");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		pr_err("Failed to start Touch boost thread, err: %d\n", ret);
		goto unregister_fb_notif;
	}

	return 0;

unregister_fb_notif:
	msm_drm_unregister_client(&boost->msm_drm_notif);
unregister_handler:
	input_unregister_handler(&touch_boost_input_handler);
	return ret;
}
late_initcall(touch_boost_init);
