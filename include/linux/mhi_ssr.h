/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MHI_DEBUGFS_SSR_H__
#define __MHI_DEBUGFS_SSR_H__

/* Flag values used with the power_on and power_off hooks */
#define SSR_HOOK_MDM_CRASH	0x0001 /* In crash handling path */

struct ssr_client_hook {
	char *name;
	void *priv;
	int (*ssr_link_power_on)(void *priv, unsigned int flags);
	void (*ssr_link_power_off)(void *priv, unsigned int flags);
};

#ifdef CONFIG_QTI_MHI_SSR
int mhi_ssr_init(struct ssr_client_hook *client_hook);
#else
static inline int mhi_ssr_init(struct ssr_client_hook *client_hook)
{
	return -EIO;
}
#endif
#endif
