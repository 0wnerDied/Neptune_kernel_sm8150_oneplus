/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_asm330lhhx sensor driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Tesi Mario <mario.tesi@st.com>
 */

#ifndef ST_ASM330LHHX_H
#define ST_ASM330LHHX_H

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>

//#define ST_ASM330LHHX_ENABLE_DEBUG

#define ST_ASM330LHHX_ODR_EXPAND(odr, uodr)		((odr * 1000000) + uodr)

#define ST_ASM330LHHX_DEV_NAME				"asm330lhhx"
#define ST_ASM330LHHX_DRV_VERSION			"1.14"

#define ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR		0x01
#define ST_ASM330LHHX_REG_SHUB_REG_MASK			BIT(6)
#define ST_ASM330LHHX_REG_FUNC_CFG_MASK			BIT(7)
#define ST_ASM330LHHX_REG_ACCESS_MASK			GENMASK(7, 6)

#define ST_ASM330LHHX_REG_FIFO_CTRL1_ADDR		0x07
#define ST_ASM330LHHX_REG_FIFO_CTRL2_ADDR		0x08
#define ST_ASM330LHHX_REG_FIFO_WTM_MASK			GENMASK(8, 0)
#define ST_ASM330LHHX_REG_FIFO_WTM8_MASK		BIT(0)
#define ST_ASM330LHHX_REG_FIFO_STATUS_DIFF		GENMASK(9, 0)

#define ST_ASM330LHHX_REG_FIFO_CTRL3_ADDR		0x09
#define ST_ASM330LHHX_REG_BDR_XL_MASK			GENMASK(3, 0)
#define ST_ASM330LHHX_REG_BDR_GY_MASK			GENMASK(7, 4)

#define ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR		0x0a
#define ST_ASM330LHHX_REG_FIFO_MODE_MASK		GENMASK(2, 0)
#define ST_ASM330LHHX_REG_ODR_T_BATCH_MASK		GENMASK(5, 4)
#define ST_ASM330LHHX_REG_DEC_TS_MASK			GENMASK(7, 6)

#define ST_ASM330LHHX_REG_INT1_CTRL_ADDR		0x0d
#define ST_ASM330LHHX_REG_INT2_CTRL_ADDR		0x0e
#define ST_ASM330LHHX_REG_FIFO_TH_MASK			BIT(3)

#define ST_ASM330LHHX_REG_WHOAMI_ADDR			0x0f
#define ST_ASM330LHHX_WHOAMI_VAL			0x6b

#define ST_ASM330LHHX_CTRL1_XL_ADDR			0x10
#define ST_ASM330LHHX_CTRL1_XL_ODR_XL_MASK		GENMASK(7, 4)
#define ST_ASM330LHHX_CTRL1_XL_FS_XL_MASK		GENMASK(3, 2)

#define ST_ASM330LHHX_CTRL2_G_ADDR			0x11
#define ST_ASM330LHHX_CTRL1_XL_ODR_G_MASK		GENMASK(7, 4)
#define ST_ASM330LHHX_CTRL1_XL_FS_G_MASK		GENMASK(3, 0)

#define ST_ASM330LHHX_REG_CTRL3_C_ADDR			0x12
#define ST_ASM330LHHX_REG_SW_RESET_MASK			BIT(0)
#define ST_ASM330LHHX_REG_PP_OD_MASK			BIT(4)
#define ST_ASM330LHHX_REG_H_LACTIVE_MASK		BIT(5)
#define ST_ASM330LHHX_REG_BDU_MASK			BIT(6)
#define ST_ASM330LHHX_REG_BOOT_MASK			BIT(7)

#define ST_ASM330LHHX_REG_CTRL4_C_ADDR			0x13
#define ST_ASM330LHHX_REG_INT2_ON_INT1_MASK		BIT(5)
#define ST_ASM330LHHX_REG_DRDY_MASK			BIT(3)

#define ST_ASM330LHHX_REG_CTRL5_C_ADDR			0x14
#define ST_ASM330LHHX_REG_ROUNDING_MASK			GENMASK(6, 5)
#define ST_ASM330LHHX_REG_ST_G_MASK			GENMASK(3, 2)
#define ST_ASM330LHHX_REG_ST_XL_MASK			GENMASK(1, 0)
#define ST_ASM330LHHX_SELFTEST_ACCEL_MIN		737
#define ST_ASM330LHHX_SELFTEST_ACCEL_MAX		13934
#define ST_ASM330LHHX_SELFTEST_GYRO_MIN			2142
#define ST_ASM330LHHX_SELFTEST_GYRO_MAX			10000

#define ST_ASM330LHHX_SELF_TEST_DISABLED_VAL		0
#define ST_ASM330LHHX_SELF_TEST_POS_SIGN_VAL		1
#define ST_ASM330LHHX_SELF_TEST_NEG_ACCEL_SIGN_VAL	2
#define ST_ASM330LHHX_SELF_TEST_NEG_GYRO_SIGN_VAL	3

#define ST_ASM330LHHX_REG_CTRL6_C_ADDR			0x15
#define ST_ASM330LHHX_REG_XL_HM_MODE_MASK		BIT(4)

#define ST_ASM330LHHX_REG_CTRL7_G_ADDR			0x16
#define ST_ASM330LHHX_REG_G_HM_MODE_MASK		BIT(7)

#define ST_ASM330LHHX_REG_CTRL10_C_ADDR			0x19
#define ST_ASM330LHHX_REG_TIMESTAMP_EN_MASK		BIT(5)

#define ST_ASM330LHHX_REG_STATUS_ADDR			0x1e
#define ST_ASM330LHHX_REG_STATUS_XLDA			BIT(0)
#define ST_ASM330LHHX_REG_STATUS_GDA			BIT(1)
#define ST_ASM330LHHX_REG_STATUS_TDA			BIT(2)

#define ST_ASM330LHHX_REG_OUT_TEMP_L_ADDR		0x20

#define ST_ASM330LHHX_REG_OUTX_L_A_ADDR			0x28
#define ST_ASM330LHHX_REG_OUTY_L_A_ADDR			0x2a
#define ST_ASM330LHHX_REG_OUTZ_L_A_ADDR			0x2c

#define ST_ASM330LHHX_REG_OUTX_L_G_ADDR			0x22
#define ST_ASM330LHHX_REG_OUTY_L_G_ADDR			0x24
#define ST_ASM330LHHX_REG_OUTZ_L_G_ADDR			0x26

#define ST_ASM330LHHX_FSM_STATUS_A_MAINPAGE		0x36
#define ST_ASM330LHHX_FSM_STATUS_B_MAINPAGE		0x37
#define ST_ASM330LHHX_MLC_STATUS_MAINPAGE		0x38

#define ST_ASM330LHHX_REG_STATUS_MASTER_MAINPAGE_ADDR	0x39
#define ST_ASM330LHHX_REG_STATUS_SENS_HUB_ENDOP_MASK	BIT(0)

#define ST_ASM330LHHX_REG_FIFO_STATUS1_ADDR		0x3a
#define ST_ASM330LHHX_REG_TIMESTAMP0_ADDR		0x40
#define ST_ASM330LHHX_REG_TIMESTAMP2_ADDR		0x42

#define ST_ASM330LHHX_REG_TAP_CFG0_ADDR			0x56
#define ST_ASM330LHHX_REG_LIR_MASK			BIT(0)

#define ST_ASM330LHHX_REG_MD1_CFG_ADDR			0x5e
#define ST_ASM330LHHX_REG_MD2_CFG_ADDR			0x5f
#define ST_ASM330LHHX_REG_INT2_TIMESTAMP_MASK		BIT(0)
#define ST_ASM330LHHX_REG_INT_EMB_FUNC_MASK		BIT(1)

#define ST_ASM330LHHX_INTERNAL_FREQ_FINE		0x63

#define ST_ASM330LHHX_REG_FIFO_DATA_OUT_TAG_ADDR	0x78

/* shub registers */
#define ST_ASM330LHHX_REG_MASTER_CONFIG_ADDR		0x14
#define ST_ASM330LHHX_REG_WRITE_ONCE_MASK		BIT(6)
#define ST_ASM330LHHX_REG_SHUB_PU_EN_MASK		BIT(3)
#define ST_ASM330LHHX_REG_MASTER_ON_MASK		BIT(2)

#define ST_ASM330LHHX_REG_SLV0_ADDR			0x15
#define ST_ASM330LHHX_REG_SLV0_CFG			0x17
#define ST_ASM330LHHX_REG_SLV1_ADDR			0x18
#define ST_ASM330LHHX_REG_SLV2_ADDR			0x1b
#define ST_ASM330LHHX_REG_SLV3_ADDR			0x1e
#define ST_ASM330LHHX_REG_DATAWRITE_SLV0_ADDR		0x21
#define ST_ASM330LHHX_REG_BATCH_EXT_SENS_EN_MASK	BIT(3)
#define ST_ASM330LHHX_REG_SLAVE_NUMOP_MASK		GENMASK(2, 0)

#define ST_ASM330LHHX_REG_STATUS_MASTER_ADDR		0x22
#define ST_ASM330LHHX_REG_SENS_HUB_ENDOP_MASK		BIT(0)

#define ST_ASM330LHHX_REG_SLV0_OUT_ADDR			0x02

/* embedded function registers */
#define ST_ASM330LHHX_EMB_FUNC_EN_B_ADDR		0x05
#define ST_ASM330LHHX_FSM_EN_MASK			BIT(0)
#define ST_ASM330LHHX_MLC_EN_MASK			BIT(4)

#define ST_ASM330LHHX_FSM_INT1_A_ADDR			0x0b
#define ST_ASM330LHHX_FSM_INT1_B_ADDR			0x0c
#define ST_ASM330LHHX_MLC_INT1_ADDR			0x0d

#define ST_ASM330LHHX_FSM_INT2_A_ADDR			0x0f
#define ST_ASM330LHHX_FSM_INT2_B_ADDR			0x10
#define ST_ASM330LHHX_MLC_INT2_ADDR			0x11

#define ST_ASM330LHHX_REG_MLC_STATUS_ADDR		0x15

#define ST_ASM330LHHX_FSM_ENABLE_A_ADDR			0x46
#define ST_ASM330LHHX_FSM_ENABLE_B_ADDR			0x47

#define ST_ASM330LHHX_FSM_OUTS1_ADDR			0x4c

#define ST_ASM330LHHX_REG_MLC0_SRC_ADDR			0x70

/* Timestamp Tick 25us/LSB */
#define ST_ASM330LHHX_TS_DELTA_NS			25000ULL

/* Temperature in uC */
#define ST_ASM330LHHX_TEMP_GAIN				256
#define ST_ASM330LHHX_TEMP_OFFSET			6400

/* FIFO simple size and depth */
#define ST_ASM330LHHX_SAMPLE_SIZE			6
#define ST_ASM330LHHX_TS_SAMPLE_SIZE			4
#define ST_ASM330LHHX_TAG_SIZE				1
#define ST_ASM330LHHX_FIFO_SAMPLE_SIZE		(ST_ASM330LHHX_SAMPLE_SIZE + \
						ST_ASM330LHHX_TAG_SIZE)
#define ST_ASM330LHHX_MAX_FIFO_DEPTH			416

/* Define Custom events for FIFO flush */
#define CUSTOM_IIO_EV_DIR_FIFO_EMPTY (IIO_EV_DIR_NONE + 1)
#define CUSTOM_IIO_EV_DIR_FIFO_DATA (IIO_EV_DIR_NONE + 2)
#define CUSTOM_IIO_EV_TYPE_FIFO_FLUSH (IIO_EV_TYPE_CHANGE + 1)

#define ST_ASM330LHHX_DATA_CHANNEL(chan_type, addr, mod, ch2, scan_idx,	\
				rb, sb, sg)				\
{									\
	.type = chan_type,						\
	.address = addr,						\
	.modified = mod,						\
	.channel2 = ch2,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = scan_idx,						\
	.scan_type = {							\
		.sign = sg,						\
		.realbits = rb,						\
		.storagebits = sb,					\
		.endianness = IIO_LE,					\
	},								\
}

static const struct iio_event_spec st_asm330lhhx_flush_event = {
	.type = CUSTOM_IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_event_spec st_asm330lhhx_thr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

#define ST_ASM330LHHX_EVENT_CHANNEL(ctype, etype)	\
{							\
	.type = ctype,					\
	.modified = 0,					\
	.scan_index = -1,				\
	.indexed = -1,					\
	.event_spec = &st_asm330lhhx_##etype##_event,	\
	.num_event_specs = 1,				\
}

#define ST_ASM330LHHX_SHIFT_VAL(val, mask)	(((val) << __ffs(mask)) & (mask))

enum st_asm330lhhx_pm_t {
	ST_ASM330LHHX_HP_MODE = 0,
	ST_ASM330LHHX_LP_MODE,
	ST_ASM330LHHX_NO_MODE,
};

#ifdef CONFIG_IIO_ST_ASM330LHHX_MLC
enum st_asm330lhhx_fsm_mlc_enable_id {
	ST_ASM330LHHX_MLC_FSM_DISABLED = 0,
	ST_ASM330LHHX_MLC_ENABLED = BIT(0),
	ST_ASM330LHHX_FSM_ENABLED = BIT(1),
};

/**
 * struct mlc_config_t -
 * @mlc_int_addr: interrupt register address
 * @mlc_int_mask: interrupt register mask
 * @fsm_enabled_mask: enable fsm register mask
 * @fsm_int_addr: interrupt register address
 * @fsm_int_mask: interrupt register mask
 * @mlc_configured: number of mlc configured
 * @fsm_configured: number of fsm configured
 * @bin_len: fw binary size
 * @mlc_int_pin: where route mlc int pin
 * @mlc_fsm_en: mlc and fsm enable bit reported by ucf file
 * @fsm_mlc_requested_odr, fsm_mlc_requested_uodr: Min required ODR by MLC/FSM.
 * @status: mlc/fsm status
 */
struct st_asm330lhhx_mlc_config_t {
	u8 mlc_int_addr;
	u8 mlc_int_mask;
	u8 fsm_enabled_mask[2];
	u8 fsm_int_addr[2];
	u8 fsm_int_mask[2];
	u8 mlc_configured;
	u8 fsm_configured;
	u16 bin_len;
	int mlc_int_pin;
	u8 mlc_fsm_en;
	u16 fsm_mlc_requested_odr;
	u32 fsm_mlc_requested_uodr;
	enum st_asm330lhhx_fsm_mlc_enable_id status;
};
#endif /* CONFIG_IIO_ST_ASM330LHHX_MLC */

/**
 * struct st_asm330lhhx_reg - Generic sensor register description (addr + mask)
 * @addr: Address of register.
 * @mask: Bitmask register for proper usage.
 */
struct st_asm330lhhx_reg {
	u8 addr;
	u8 mask;
};

enum st_asm330lhh_suspend_resume_register {
	ST_ASM330LHHX_CTRL1_XL_REG = 0,
	ST_ASM330LHHX_CTRL2_G_REG,
	ST_ASM330LHHX_REG_CTRL3_C_REG,
	ST_ASM330LHHX_REG_CTRL4_C_REG,
	ST_ASM330LHHX_REG_CTRL5_C_REG,
	ST_ASM330LHHX_REG_CTRL6_C_REG,
	ST_ASM330LHHX_REG_CTRL10_C_REG,
	ST_ASM330LHHX_REG_TAP_CFG0_REG,
	ST_ASM330LHHX_REG_INT1_CTRL_REG,
	ST_ASM330LHHX_REG_INT2_CTRL_REG,
	ST_ASM330LHHX_REG_FIFO_CTRL1_REG,
	ST_ASM330LHHX_REG_FIFO_CTRL2_REG,
	ST_ASM330LHHX_REG_FIFO_CTRL3_REG,
	ST_ASM330LHHX_REG_FIFO_CTRL4_REG,
#ifdef ST_ASM330LHHX_BACKUP_FUNC_CFG_REGS
	ST_ASM330LHHX_REG_EMB_FUNC_EN_B_REG,
	ST_ASM330LHHX_REG_FSM_INT1_A_REG,
	ST_ASM330LHHX_REG_FSM_INT1_B_REG,
	ST_ASM330LHHX_REG_MLC_INT1_REG,
	ST_ASM330LHHX_REG_FSM_INT2_A_REG,
	ST_ASM330LHHX_REG_FSM_INT2_B_REG,
	ST_ASM330LHHX_REG_MLC_INT2_REG,
#endif /* ST_ASM330LHHX_BACKUP_FUNC_CFG_REGS */
	ST_ASM330LHHX_SUSPEND_RESUME_REGS,
};

/**
 * Define embedded functions register access
 *
 * FUNC_CFG_ACCESS_0 is default bank
 * FUNC_CFG_ACCESS_SHUB_REG Enable access to the sensor hub (I2C master)
 *                          registers.
 * FUNC_CFG_ACCESS_FUNC_CFG Enable access to the embedded functions
 *                          configuration registers.
 */
enum st_asm330lhh_page_sel_register {
	FUNC_CFG_ACCESS_0 = 0,
	FUNC_CFG_ACCESS_SHUB_REG,
	FUNC_CFG_ACCESS_FUNC_CFG,
};

/**
 * struct st_asm330lhhx_suspend_resume_entry - Register value for backup/restore
 * @page: Page bank reg map.
 * @addr: Address of register.
 * @val: Register value.
 * @mask: Bitmask register for proper usage.
 */
struct st_asm330lhhx_suspend_resume_entry {
	u8 page;
	u8 addr;
	u8 val;
	u8 mask;
};

/**
 * struct st_asm330lhhx_odr - Single ODR entry
 * @hz: Most significant part of the sensor ODR (Hz).
 * @uhz: Less significant part of the sensor ODR (micro Hz).
 * @val: ODR register value.
 * @batch_val: Batching ODR register value.
 */
struct st_asm330lhhx_odr {
	u16 hz;
	u32 uhz;
	u8 val;
	u8 batch_val;
};

/**
 * struct st_asm330lhhx_odr_table_entry - Sensor ODR table
 * @size: Size of ODR table.
 * @reg: ODR register.
 * @pm: Power mode register.
 * @batching_reg: ODR register for batching on fifo.
 * @odr_avl: Array of supported ODR value.
 */
struct st_asm330lhhx_odr_table_entry {
	u8 size;
	struct st_asm330lhhx_reg reg;
	struct st_asm330lhhx_reg pm;
	struct st_asm330lhhx_reg batching_reg;
	struct st_asm330lhhx_odr odr_avl[8];
};

/**
 * struct st_asm330lhhx_fs - Full Scale sensor table entry
 * @reg: Register description for FS settings.
 * @gain: Sensor sensitivity (mdps/LSB, mg/LSB and uC/LSB).
 * @val: FS register value.
 */
struct st_asm330lhhx_fs {
	struct st_asm330lhhx_reg reg;
	u32 gain;
	u8 val;
};

/**
 * struct st_asm330lhhx_fs_table_entry - Full Scale sensor table
 * @size: Full Scale sensor table size.
 * @fs_avl: Full Scale list entries.
 */
struct st_asm330lhhx_fs_table_entry {
	u8 size;
	struct st_asm330lhhx_fs fs_avl[5];
};

enum st_asm330lhhx_sensor_id {
	ST_ASM330LHHX_ID_GYRO = 0,
	ST_ASM330LHHX_ID_ACC,
	ST_ASM330LHHX_ID_TEMP,
	ST_ASM330LHHX_ID_EXT0,
	ST_ASM330LHHX_ID_EXT1,

#ifdef CONFIG_IIO_ST_ASM330LHHX_MLC
	ST_ASM330LHHX_ID_FIFO_MLC,
	ST_ASM330LHHX_ID_MLC,
	ST_ASM330LHHX_ID_MLC_0,
	ST_ASM330LHHX_ID_MLC_1,
	ST_ASM330LHHX_ID_MLC_2,
	ST_ASM330LHHX_ID_MLC_3,
	ST_ASM330LHHX_ID_MLC_4,
	ST_ASM330LHHX_ID_MLC_5,
	ST_ASM330LHHX_ID_MLC_6,
	ST_ASM330LHHX_ID_MLC_7,
	ST_ASM330LHHX_ID_FSM_0,
	ST_ASM330LHHX_ID_FSM_1,
	ST_ASM330LHHX_ID_FSM_2,
	ST_ASM330LHHX_ID_FSM_3,
	ST_ASM330LHHX_ID_FSM_4,
	ST_ASM330LHHX_ID_FSM_5,
	ST_ASM330LHHX_ID_FSM_6,
	ST_ASM330LHHX_ID_FSM_7,
	ST_ASM330LHHX_ID_FSM_8,
	ST_ASM330LHHX_ID_FSM_9,
	ST_ASM330LHHX_ID_FSM_10,
	ST_ASM330LHHX_ID_FSM_11,
	ST_ASM330LHHX_ID_FSM_12,
	ST_ASM330LHHX_ID_FSM_13,
	ST_ASM330LHHX_ID_FSM_14,
	ST_ASM330LHHX_ID_FSM_15,
#endif /* CONFIG_IIO_ST_ASM330LHHX_MLC */

	ST_ASM330LHHX_ID_MAX,
};

static const enum st_asm330lhhx_sensor_id st_asm330lhhx_mlc_sensor_list[] = {
	 [0] = ST_ASM330LHHX_ID_MLC_0,
	 [1] = ST_ASM330LHHX_ID_MLC_1,
	 [2] = ST_ASM330LHHX_ID_MLC_2,
	 [3] = ST_ASM330LHHX_ID_MLC_3,
	 [4] = ST_ASM330LHHX_ID_MLC_4,
	 [5] = ST_ASM330LHHX_ID_MLC_5,
	 [6] = ST_ASM330LHHX_ID_MLC_6,
	 [7] = ST_ASM330LHHX_ID_MLC_7,
};

static const enum st_asm330lhhx_sensor_id st_asm330lhhx_fsm_sensor_list[] = {
	 [0] = ST_ASM330LHHX_ID_FSM_0,
	 [1] = ST_ASM330LHHX_ID_FSM_1,
	 [2] = ST_ASM330LHHX_ID_FSM_2,
	 [3] = ST_ASM330LHHX_ID_FSM_3,
	 [4] = ST_ASM330LHHX_ID_FSM_4,
	 [5] = ST_ASM330LHHX_ID_FSM_5,
	 [6] = ST_ASM330LHHX_ID_FSM_6,
	 [7] = ST_ASM330LHHX_ID_FSM_7,
	 [8] = ST_ASM330LHHX_ID_FSM_8,
	 [9] = ST_ASM330LHHX_ID_FSM_9,
	 [10] = ST_ASM330LHHX_ID_FSM_10,
	 [11] = ST_ASM330LHHX_ID_FSM_11,
	 [12] = ST_ASM330LHHX_ID_FSM_12,
	 [13] = ST_ASM330LHHX_ID_FSM_13,
	 [14] = ST_ASM330LHHX_ID_FSM_14,
	 [15] = ST_ASM330LHHX_ID_FSM_15,
};

enum st_asm330lhhx_fifo_mode {
	ST_ASM330LHHX_FIFO_BYPASS = 0x0,
	ST_ASM330LHHX_FIFO_MODE = 0x1,
	ST_ASM330LHHX_FIFO_CONT = 0x6,
};

enum {
	ST_ASM330LHHX_HW_FLUSH,
	ST_ASM330LHHX_HW_OPERATIONAL,
};

struct st_asm330lhhx_ext_dev_info {
	const struct st_asm330lhhx_ext_dev_settings *ext_dev_settings;
	u8 ext_dev_i2c_addr;
};

/**
 * struct st_asm330lhhx_sensor - ST IMU sensor instance
 * @name: Sensor name.
 * @id: Sensor identifier.
 * @hw: Pointer to instance of struct st_asm330lhhx_hw.
 * @gain: Configured sensor sensitivity.
 * @offset: Sensor data offset.
 * decimator: Sensor decimator
 * dec_counter: Sensor decimator counter
 * @odr: Output data rate of the sensor [Hz].
 * @uodr: Output data rate of the sensor [uHz].
 * @old_data: Used by Temperature sensor for data comtinuity.
 * @max_watermark: Max supported watermark level.
 * @watermark: Sensor watermark level.
 * @pm: sensor power mode (HP, LP).
 * @selftest_status: Last status of self test output.
 * @min_st, @max_st: Min/Max acc/gyro data values during self test procedure.
 */
struct st_asm330lhhx_sensor {
	char name[32];
	enum st_asm330lhhx_sensor_id id;
	struct st_asm330lhhx_hw *hw;
	struct st_asm330lhhx_ext_dev_info ext_dev_info;

	union {
		struct {
			u32 gain;
			u32 offset;
			u8 decimator;
			u8 dec_counter;
			int odr;
			int uodr;
			__le16 old_data;
			u16 max_watermark;
			u16 watermark;
			u8 pm;
			s64 last_fifo_timestamp;

			/* self test */
			s8 selftest_status;
			int min_st;
			int max_st;
		};
		struct {
			u8 status_reg;
			u8 outreg_addr;
			enum st_asm330lhhx_fsm_mlc_enable_id status;
		};
	};
};

/**
 * struct st_asm330lhhx_hw - ST IMU MEMS hw instance
 * @dev: Pointer to instance of struct device (I2C or SPI).
 * @irq: Device interrupt line (I2C or SPI).
 * @regmap: Register map of the device.
 * @fifo_lock: Mutex to prevent concurrent access to the hw FIFO.
 * @fifo_mode: FIFO operating mode supported by the device.
 * @state: hw operational state.
 * @enable_mask: Enabled sensor bitmask.
 * @requested_mask: Sensor requesting bitmask.
 * @ext_data_len: Number of i2c slave devices connected to I2C master.
 * @ts_delta_ns: Calibrated delta timestamp.
 * @ts_offset: Hw timestamp offset.
 * @hw_ts: Latest hw timestamp from the sensor.
 * @tsample: Sample timestamp.
 * @delta_ts: Delta time between two consecutive interrupts.
 * @ts: Latest timestamp from irq handler.
 * @i2c_master_pu: I2C master line Pull Up configuration.
 * @int_pin: fifo wtm interrupt pin configuration.
 * @mlc_config: mlc configuration structure.
 * @odr_table_entry: Sensors ODR table.
 * @preload_mlc: Preloaded MLC flag.
 * @resuming: System resuming flag.
 * @iio_devs: Pointers to acc/gyro iio_dev instances.
 */
struct st_asm330lhhx_hw {
	struct device *dev;
	int irq;

	struct regmap *regmap;
	struct mutex fifo_lock;
	struct mutex page_lock;

	enum st_asm330lhhx_fifo_mode fifo_mode;
	unsigned long state;
	u32 enable_mask;
	u32 requested_mask;

	s64 ts_offset;
	u8 ext_data_len;
	u64 ts_delta_ns;
	s64 hw_ts;
	u32 val_ts_old;
	u32 hw_ts_high;
	s64 tsample;
	s64 delta_ts;
	s64 ts;
	u8 i2c_master_pu;
	u8 int_pin;

	struct st_asm330lhhx_mlc_config_t *mlc_config;
	const struct st_asm330lhhx_odr_table_entry *odr_table_entry;

	bool preload_mlc;
	bool resuming;

	struct iio_dev *iio_devs[ST_ASM330LHHX_ID_MAX];
	int enable_gpio;
};

extern const struct dev_pm_ops st_asm330lhhx_pm_ops;

static inline bool st_asm330lhhx_is_fifo_enabled(struct st_asm330lhhx_hw *hw)
{
	return hw->enable_mask & (BIT(ST_ASM330LHHX_ID_GYRO) |
				  BIT(ST_ASM330LHHX_ID_ACC)  |
				  BIT(ST_ASM330LHHX_ID_EXT0) |
				  BIT(ST_ASM330LHHX_ID_EXT1));
}

static inline int __st_asm330lhhx_write_with_mask(struct st_asm330lhhx_hw *hw,
						  unsigned int addr,
						  unsigned int mask,
						  unsigned int data)
{
	int err;
	unsigned int val = ST_ASM330LHHX_SHIFT_VAL(data, mask);

	err = regmap_update_bits(hw->regmap, addr, mask, val);

	return err;
}

static inline int
st_asm330lhhx_update_bits_locked(struct st_asm330lhhx_hw *hw, unsigned int addr,
				 unsigned int mask, unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = __st_asm330lhhx_write_with_mask(hw, addr, mask, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

/* use when mask is constant */
static inline int st_asm330lhhx_write_with_mask_locked(
		struct st_asm330lhhx_hw *hw,
		unsigned int addr,
		unsigned int mask,
		unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_update_bits(hw->regmap, addr, mask, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_asm330lhhx_read_locked(struct st_asm330lhhx_hw *hw, unsigned int addr,
			  void *val, unsigned int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_bulk_read(hw->regmap, addr, val, len);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_asm330lhhx_write_locked(struct st_asm330lhhx_hw *hw, unsigned int addr,
			   unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_write(hw->regmap, addr, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int st_asm330lhhx_set_page_access(struct st_asm330lhhx_hw *hw,
						unsigned int val,
						unsigned int mask)
{
	int err;

	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR,
				 mask,
				 ST_ASM330LHHX_SHIFT_VAL(val, mask));

	return err;
}

static inline s64 st_asm330lhhx_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	return timespec_to_ns(&ts);
}

int st_asm330lhhx_probe(struct device *dev, int irq,
		struct regmap *regmap);
int st_asm330lhhx_sensor_set_enable(struct st_asm330lhhx_sensor *sensor,
				    bool enable);
int st_asm330lhhx_buffers_setup(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_get_odr_val(enum st_asm330lhhx_sensor_id id, int odr,
			      int uodr, int *podr, int *puodr, u8 *val);
int st_asm330lhhx_get_batch_val(struct st_asm330lhhx_sensor *sensor, int odr,
		int uodr, u8 *val);
int st_asm330lhhx_update_watermark(struct st_asm330lhhx_sensor *sensor,
				   u16 watermark);
ssize_t st_asm330lhhx_flush_fifo(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size);
ssize_t st_asm330lhhx_get_max_watermark(struct device *dev,
					struct device_attribute *attr,
					char *buf);
ssize_t st_asm330lhhx_get_watermark(struct device *dev,
				    struct device_attribute *attr, char *buf);
ssize_t st_asm330lhhx_set_watermark(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size);
int st_asm330lhhx_suspend_fifo(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_set_fifo_mode(struct st_asm330lhhx_hw *hw,
		enum st_asm330lhhx_fifo_mode fifo_mode);
int st_asm330lhhx_update_batching(struct iio_dev *iio_dev, bool enable);
int st_asm330lhhx_of_get_pin(struct st_asm330lhhx_hw *hw, int *pin);
int st_asm330lhhx_shub_probe(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_shub_set_enable(struct st_asm330lhhx_sensor *sensor,
				  bool enable);
void st_asm330lhhx_shutdown(struct device *dev);

#ifdef CONFIG_IIO_ST_ASM330LHHX_MLC
int st_asm330lhhx_mlc_probe(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_mlc_remove(struct device *dev);
int st_asm330lhhx_mlc_check_status(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_mlc_init_preload(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_read_fifo(struct st_asm330lhhx_hw *hw);
int st_asm330lhhx_get_odr_from_reg(enum st_asm330lhhx_sensor_id id, u8 reg_val,
				   u16 *podr, u32 *puodr);
#endif /* CONFIG_IIO_ST_ASM330LHHX_MLC */

#endif /* ST_ASM330LHHX_H */
