/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 idkwhoiam322 <idkwhoiam322@raphielgang.org>
 */

/* In-kernel powerHAL to replicate some behaviours of the pixel powerHAL */

#ifndef _POWER_HAL_H
#define _POWER_HAL_H

/* UFS Boosting */
void set_ufshcd_clkgate_enable_status(u32 value);
//void set_ufshcd_hibern8_enable_status(u32 value);

#endif /* _POWER_HAL_H */