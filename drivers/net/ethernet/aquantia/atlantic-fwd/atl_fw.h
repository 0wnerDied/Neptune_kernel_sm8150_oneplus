/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2017 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#ifndef _ATL_FW_H_
#define _ATL_FW_H_

struct atl_hw;

struct atl_mcp {
	uint32_t fw_rev;
	struct atl_fw_ops *ops;
	uint32_t fw_stat_addr;
	uint32_t fw_settings_addr;
	uint32_t fw_settings_len;
	uint32_t req_high;
	uint32_t req_high_mask;	/* Clears link rate-dependend bits */
	uint32_t caps_low;
	uint32_t caps_high;
	uint32_t caps_ex;
	struct mutex lock;
	unsigned long next_wdog;
	bool wdog_disabled;
	uint16_t phy_hbeat;
};

struct atl_link_type {
	unsigned speed;
	unsigned ethtool_idx;
	uint32_t fw_bits[2];
	const char *name;
};

extern struct atl_link_type atl_link_types[];
extern const int atl_num_rates;

struct atl_fw2_thermal_cfg {
	uint32_t msg_id;
	uint8_t shutdown_temp;
	uint8_t high_temp;
	uint8_t normal_temp;
};

#define atl_for_each_rate(idx, type)		\
	for (idx = 0, type = atl_link_types;	\
	     idx < atl_num_rates;		\
	     idx++, type++)

#define atl_define_bit(_name, _bit)		\
	_name ## _shift = (_bit),		\
	_name = BIT(_name ## _shift),

enum atl_fw2_opts {
	atl_define_bit(atl_fw2_pause, 3)
	atl_define_bit(atl_fw2_asym_pause, 4)
	atl_fw2_pause_mask = atl_fw2_pause | atl_fw2_asym_pause,
	atl_define_bit(atl_fw2_wake_on_link, 16)
	atl_define_bit(atl_fw2_wake_on_link_force, 17)
	atl_define_bit(atl_fw2_phy_temp, 18)
	atl_define_bit(atl_fw2_set_thermal, 21)
	atl_define_bit(atl_fw2_link_drop, 22)
	atl_define_bit(atl_fw2_nic_proxy, 0x17)
	atl_define_bit(atl_fw2_wol, 0x18)
	atl_define_bit(atl_fw2_thermal_alarm, 29)
	atl_define_bit(atl_fw2_statistics, 30)
};

enum atl_fw2_ex_caps {
	atl_define_bit(atl_fw2_ex_caps_wol_ex, 23)
	atl_define_bit(atl_fw2_ex_caps_mac_heartbeat, 25)
};

enum atl_fw2_wol_ex {
	atl_define_bit(atl_fw2_wol_ex_wake_on_link_keep_rate, 0)
	atl_define_bit(atl_fw2_wol_ex_wake_on_magic_keep_rate, 1)
};

enum atl_fw2_stat_offt {
	atl_fw2_stat_phy_hbeat = 0x4c,
	atl_fw2_stat_temp = 0x50,
	atl_fw2_stat_lcaps = 0x84,
	atl_fw2_stat_settings_addr = 0x10c,
	atl_fw2_stat_settings_len = 0x110,
	atl_fw2_stat_caps_ex = 0x114,
};

enum atl_fw2_settings_offt {
	atl_fw2_setings_msm_opts = 0x90,
	atl_fw2_setings_media_detect = 0x98,
	atl_fw2_setings_wol_ex = 0x9c,
};

enum atl_fw2_msm_opts {
	atl_define_bit(atl_fw2_settings_msm_opts_strip_pad, 0)
};

enum atl_fc_mode {
	atl_fc_none = 0,
	atl_define_bit(atl_fc_rx, 0)
	atl_define_bit(atl_fc_tx, 1)
	atl_fc_full = atl_fc_rx | atl_fc_tx,
};

enum atl_thermal_flags {
	atl_define_bit(atl_thermal_monitor, 0)
	atl_define_bit(atl_thermal_throttle, 1)
	atl_define_bit(atl_thermal_ignore_lims, 2)
};

struct atl_fc_state {
	enum atl_fc_mode req;
	enum atl_fc_mode prev_req;
	enum atl_fc_mode cur;
};

enum atl_wake_flags {
	atl_fw_wake_on_link = WAKE_PHY,
	atl_fw_wake_on_magic = WAKE_MAGIC,
	atl_fw_wake_on_link_rtpm = BIT(10),
};

#define ATL_EEE_BIT_OFFT 16
#define ATL_EEE_MASK ~(BIT(ATL_EEE_BIT_OFFT) - 1)

struct atl_link_state{
	/* The following three bitmaps use alt_link_types[] indices
	 * as link bit positions. Conversion to/from ethtool bits is
	 * done in atl_ethtool.c. */
	unsigned supported;
	unsigned advertized;
	unsigned lp_advertized;
	unsigned prev_advertized;
	int lp_lowest; 		/* Idx of lowest rate advertized by
				 * link partner in atl_link_types[] */
	int throttled_to;	/* Idx of the rate we're throttled to */
	bool force_off;
	bool thermal_throttled;
	bool autoneg;
	bool eee;
	bool eee_enabled;
	struct atl_link_type *link;
	struct atl_fc_state fc;
};

struct atl_fw_ops {
	void (*set_link)(struct atl_hw *hw, bool force);
	struct atl_link_type *(*check_link)(struct atl_hw *hw);
	int (*__wait_fw_init)(struct atl_hw *hw);
	int (*__get_link_caps)(struct atl_hw *hw);
	int (*restart_aneg)(struct atl_hw *hw);
	void (*set_default_link)(struct atl_hw *hw);
	int (*enable_wol)(struct atl_hw *hw, unsigned int wol_mode);
	int (*get_phy_temperature)(struct atl_hw *hw, int *temp);
	int (*dump_cfg)(struct atl_hw *hw);
	int (*restore_cfg)(struct atl_hw *hw);
	int (*set_phy_loopback)(struct atl_nic *nic, u32 mode);
	int (*set_mediadetect)(struct atl_hw *hw, bool on);
	unsigned efuse_shadow_addr_reg;
};

int atl_read_mcp_word(struct atl_hw *hw, uint32_t offt, uint32_t *val);

#endif
