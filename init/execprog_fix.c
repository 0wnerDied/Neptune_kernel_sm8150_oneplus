// SPDX-License-Identifier: GPL-2.0	
/*	
 * Copyright (C) 2020 Atndko <z1281552865@gmail.com>	
 */

#include <linux/module.h>

unsigned int fake_enforce;

MODULE_PARM_DESC(fake_enforce, "Fix selinux problem for some custom ROMs");
module_param(fake_enforce, uint, 0444);
