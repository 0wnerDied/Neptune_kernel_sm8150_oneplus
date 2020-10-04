// SPDX-License-Identifier: GPL-2.0	
/*	
 * Copyright (C) 2020 Atndko <z1281552865@gmail.com>.	
 */

#include <linux/module.h>

unsigned int remove_op_capacity;

MODULE_PARM_DESC(remove_op_capacity, "Remove battery capacity limit");
module_param(remove_op_capacity, uint, 0444);
