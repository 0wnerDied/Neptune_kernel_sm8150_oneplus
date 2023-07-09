// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Vlad Adumitroaie <celtare21@gmail.com>.
 *               2021 Vwool0xE9 <z1281552865@gmail.com>
 */

#define pr_fmt(fmt) "userland_worker: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/security.h>
#include <linux/delay.h>

#include "../security/selinux/include/security.h"

#define STANDARD_SIZE 4
#define MAX_CHAR 128
#define MINI_DELAY 100
#define DELAY 500

static char** argv;

static struct delayed_work userland_work;

static void free_memory(char** argv, int size)
{
	int i;

	for (i = 0; i < size; i++)
		kfree(argv[i]);
	kfree(argv);
}

static char** alloc_memory(int size)
{
	char** argv;
	int i;

	argv = kmalloc(size * sizeof(char*), GFP_KERNEL);
	if (!argv) {
		pr_err("Couldn't allocate memory!");
		return NULL;
	}

	for (i = 0; i < size; i++) {
		argv[i] = kmalloc(MAX_CHAR * sizeof(char), GFP_KERNEL);
		if (!argv[i]) {
			pr_err("Couldn't allocate memory!");
			kfree(argv);
			return NULL;
		}
	}

	return argv;
}

static int use_userspace(char** argv)
{
	static char* envp[] = {
		"SHELL=/bin/sh",
		"HOME=/",
		"USER=shell",
		"TERM=xterm-256color",
		"PATH=/product/bin:/apex/com.android.runtime/bin:/apex/com.android.art/bin:/system_ext/bin:/system/bin:/system/xbin:/odm/bin:/vendor/bin:/vendor/xbin",
		"DISPLAY=:0",
		NULL
	};

	return call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
}

static inline int linux_write(const char* prop, const char* value, bool resetprop)
{
	int ret;

	strcpy(argv[0], resetprop ? "/data/local/tmp/resetprop_static" : "/system/bin/setprop");
	strcpy(argv[1], prop);
	strcpy(argv[2], value);
	argv[3] = NULL;

	ret = use_userspace(argv);
	if (!ret)
		pr_info("%s set succesfully!", prop);
	else
		pr_err("Couldn't set %s! %d", prop, ret);

	return ret;
}

static inline int linux_sh(const char* command)
{
	int ret;

	strcpy(argv[0], "/system/bin/sh");
	strcpy(argv[1], "-c");
	strcpy(argv[2], command);
	argv[3] = NULL;

	ret = use_userspace(argv);
	if (!ret)
		pr_info("%s called succesfully!", command);
	else
		pr_err("Couldn't call %s! %d", command, ret);

	return ret;
}

static inline int linux_test(const char* path, bool dir)
{
	strcpy(argv[0], "/system/bin/test");
	strcpy(argv[1], (dir ? "-d" : "-f"));
	strcpy(argv[2], path);
	argv[3] = NULL;

	return use_userspace(argv);
}

static void common_optimize(void)
{
	int retries = 0;
	const int max_retries = 5;

	if (strstr(saved_command_line, "project_name=18857")
		|| strstr(saved_command_line, "project_name=18865"))
		linux_sh("/system/bin/chmod 666 /dev/input/event4");

	if (IS_ENABLED(CONFIG_VBSWAP)) {
		while (retries++ < max_retries) {
			if (linux_sh("/system/bin/echo 4294967296 > /sys/devices/virtual/block/vbswap0/disksize"))
				msleep(MINI_DELAY);
			else
				break;
		} 
		linux_sh("/vendor/bin/mkswap /dev/block/vbswap0");
		linux_sh("/system/bin/swapon /dev/block/vbswap0");
	}
	
	linux_sh("/system/bin/echo 2000 > /dev/blkio/blkio.group_idle");

	linux_sh("/system/bin/echo 262144 > /proc/sys/net/core/rmem_max");
	linux_sh("/system/bin/echo 262144 > /proc/sys/net/core/wmem_max");

	if (IS_ENABLED(CONFIG_PSI)) {
		linux_write("ro.lmk.use_psi", "true", true);
		linux_write("ro.config.low_ram", "false", true);
		linux_write("ro.lmk.debug", "false", true);
		linux_write("ro.lmk.psi_partial_stall_ms", "0", true);
		linux_write("ro.lmk.psi_complete_stall_ms", "700", true);
		linux_write("ro.lmk.thrashing_limit", "100", true);
		linux_write("ro.lmk.thrashing_limit_decay", "10", true);
		linux_write("ro.lmk.swap_util_max", "100", true);
		linux_write("ro.lmk.swap_free_low_percentage", "20", true);
	}
}

static void userland_worker(struct work_struct *work)
{
	bool is_enforcing;
	int retries = 0;
  	const int max_retries = 25;

	argv = alloc_memory(STANDARD_SIZE);
	if (!argv) {
		pr_err("Couldn't allocate memory!");
		return;
	}

	do {
		is_enforcing = get_enforce_value();
		if (!is_enforcing) 
			msleep(DELAY);
	} while (!is_enforcing && (retries++ < max_retries));

	if (is_enforcing) {
		pr_info("Going permissive");
		set_selinux(0);
	}

	msleep(MINI_DELAY);

	common_optimize();

	if (is_enforcing) {
		pr_info("Going enforcing");
		set_selinux(1);
	}

	free_memory(argv, STANDARD_SIZE);
}

static int __init userland_worker_entry(void)
{
	INIT_DELAYED_WORK(&userland_work, userland_worker);
	queue_delayed_work(system_power_efficient_wq,
			&userland_work, DELAY);

	return 0;
}

module_init(userland_worker_entry);
