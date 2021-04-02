/* Copyright (c) 2020,2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CMDQ_HCI_CRYPTO_QTI_H
#define _CMDQ_HCI_CRYPTO_QTI_H

#include "cmdq_hci-crypto.h"

#if IS_ENABLED(CONFIG_MMC_QTI_NONCMDQ_ICE)
#define CRYPTO_CDU_SIZE 0xFF
#define CRYPTO_ICE_INDEX 3
#define LEGACY_ICE_CAP_VAL 0x50001F06
#define ICE_CRYPTO_DATA_UNIT_512B 0
#define ICE_CRYPTO_DATA_UNIT_4K 3

#define CORE_VENDOR_SPEC_ICE_CTRL_INFO_1_n      0x304
#define CORE_VENDOR_SPEC_ICE_CTRL_INFO_2_n      0x308
#define CORE_VENDOR_SPEC_ICE_CTRL_INFO_3_n      0x30C

/* SDHCI MSM ICE CTRL Info register offset */
enum {
	OFFSET_SDHCI_MSM_ICE_CTRL_INFO_BYPASS     = 0,
	OFFSET_SDHCI_MSM_ICE_CTRL_INFO_KEY_INDEX  = 1,
	OFFSET_SDHCI_MSM_ICE_CTRL_INFO_CDU        = 6,
	OFFSET_SDHCI_MSM_ICE_HCI_PARAM_CCI        = 0,
	OFFSET_SDHCI_MSM_ICE_HCI_PARAM_CE         = 8,
};

/* SDHCI MSM ICE CTRL Info register masks */
enum {
	MASK_SDHCI_MSM_ICE_CTRL_INFO_BYPASS     = 0x1,
	MASK_SDHCI_MSM_ICE_CTRL_INFO_KEY_INDEX  = 0x1F,
	MASK_SDHCI_MSM_ICE_CTRL_INFO_CDU        = 0x7,
	MASK_SDHCI_MSM_ICE_HCI_PARAM_CE         = 0x1,
	MASK_SDHCI_MSM_ICE_HCI_PARAM_CCI        = 0xff
};

void crypto_qti_enable_noncmdq(struct sdhci_host *host);

int sdhci_crypto_cfg(struct sdhci_host *host, struct mmc_request *mrq,
		     u32 slot);

void sdhci_msm_ice_update_cfg(struct sdhci_host *host, u64 lba,
			      u32 slot, unsigned int bypass,
			      short key_index, u32 cdu_sz);
#endif

void cmdq_crypto_qti_enable(struct cmdq_host *host);

void cmdq_crypto_qti_disable(struct cmdq_host *host);

#ifdef CONFIG_BLK_INLINE_ENCRYPTION
int cmdq_crypto_qti_init_crypto(struct cmdq_host *host,
				 const struct keyslot_mgmt_ll_ops *ksm_ops);
#endif

int cmdq_crypto_qti_debug(struct cmdq_host *host);

void cmdq_crypto_qti_set_vops(struct cmdq_host *host);

int cmdq_crypto_qti_resume(struct cmdq_host *host);

int cmdq_crypto_qti_prep_desc(struct cmdq_host *host,
			      struct mmc_request *mrq,
			      u64 *ice_ctx);

int cmdq_crypto_qti_reset(struct cmdq_host *host);

#endif /* _CMDQ_HCI_CRYPTO_QTI_H */
