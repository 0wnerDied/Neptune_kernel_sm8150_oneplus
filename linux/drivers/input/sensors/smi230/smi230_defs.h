/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*! \file smi230_defs.h */

/*!
 * @defgroup SMI230 SENSOR API
 * @brief
 * @{*/
#ifndef _SMI230_DEFS_H
#define _SMI230_DEFS_H

/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#endif

/*********************************************************************/
/** \name       Common macros                   */
/*********************************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                 0
#else
#define NULL                 ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE                 UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE                UINT8_C(0)
#endif

/*************************** SMI230 Accelerometer Macros *****************************/

/** Register map */
/* Accel registers */

/**\name    Accel Chip Id register */
#define SMI230_ACCEL_CHIP_ID_REG            UINT8_C(0x00)

/**\name    Accel Error condition register */
#define SMI230_ACCEL_ERR_REG                UINT8_C(0x02)

/**\name    Accel Status flag register */
#define SMI230_ACCEL_STATUS_REG             UINT8_C(0x03)

/**\name    Accel X LSB data register */
#define SMI230_ACCEL_X_LSB_REG              UINT8_C(0x12)

/**\name    Accel X MSB data register */
#define SMI230_ACCEL_X_MSB_REG              UINT8_C(0x13)

/**\name    Accel Y LSB data register */
#define SMI230_ACCEL_Y_LSB_REG              UINT8_C(0x14)

/**\name    Accel Y MSB data register */
#define SMI230_ACCEL_Y_MSB_REG              UINT8_C(0x15)

/**\name    Accel Z LSB data register */
#define SMI230_ACCEL_Z_LSB_REG              UINT8_C(0x16)

/**\name    Accel Z MSB data register */
#define SMI230_ACCEL_Z_MSB_REG              UINT8_C(0x17)

/**\name    Sensor time byte 0 register */
#define SMI230_ACCEL_SENSORTIME_0_REG       UINT8_C(0x18)

/**\name    Sensor time byte 1 register */
#define SMI230_ACCEL_SENSORTIME_1_REG       UINT8_C(0x19)

/**\name    Sensor time byte 2 register */
#define SMI230_ACCEL_SENSORTIME_2_REG       UINT8_C(0x1A)

/**\name    Accel Interrupt status0 register */
#define SMI230_ACCEL_INT_STAT_0_REG         UINT8_C(0x1C)

/**\name    Accel Interrupt status1 register */
#define SMI230_ACCEL_INT_STAT_1_REG         UINT8_C(0x1D)

/**\name    Accel general purpose register 0*/
#define SMI230_ACCEL_GP_0_REG               UINT8_C(0x1E)

/**\name    Sensor temperature MSB data register */
#define SMI230_TEMP_MSB_REG                 UINT8_C(0x22)

/**\name    Sensor temperature LSB data register */
#define SMI230_TEMP_LSB_REG                 UINT8_C(0x23)

/**\name    Accel general purpose register 4*/
#define SMI230_ACCEL_GP_4_REG               UINT8_C(0x27)

/**\name    Accel Internal status register */
#define SMI230_ACCEL_INTERNAL_STAT_REG      UINT8_C(0x2A)

/**\name    Accel configuration register */
#define SMI230_ACCEL_CONF_REG               UINT8_C(0x40)

/**\name    Accel range setting register */
#define SMI230_ACCEL_RANGE_REG              UINT8_C(0x41)

/**\name    Accel Interrupt pin 1 configuration register */
#define SMI230_ACCEL_INT1_IO_CONF_REG       UINT8_C(0x53)

/**\name    Accel Interrupt pin 2 configuration register */
#define SMI230_ACCEL_INT2_IO_CONF_REG       UINT8_C(0x54)

/**\name    Accel Interrupt latch configuration register */
#define SMI230_ACCEL_INT_LATCH_CONF_REG     UINT8_C(0x55)

/**\name    Accel Interrupt pin1 mapping register */
#define SMI230_ACCEL_INT1_MAP_REG           UINT8_C(0x56)

/**\name    Accel Interrupt pin2 mapping register */
#define SMI230_ACCEL_INT2_MAP_REG           UINT8_C(0x57)

/**\name    Accel Interrupt map register */
#define SMI230_ACCEL_INT1_INT2_MAP_DATA_REG UINT8_C(0x58)

/**\name    Accel Init control register */
#define SMI230_ACCEL_INIT_CTRL_REG          UINT8_C(0x59)

/**\name    Accel Self test register */
#define SMI230_ACCEL_SELF_TEST_REG          UINT8_C(0x6D)

/**\name    Accel Power mode configuration register */
#define SMI230_ACCEL_PWR_CONF_REG           UINT8_C(0x7C)

/**\name    Accel Power control (switch on or off ) register */
#define SMI230_ACCEL_PWR_CTRL_REG           UINT8_C(0x7D)

/**\name    Accel Soft reset register */
#define SMI230_ACCEL_SOFTRESET_REG          UINT8_C(0x7E)

/**\name    Accel unique chip identifier */
#define SMI230_ACCEL_CHIP_ID                UINT8_C(0x1F)

/**\name    Accel I2C slave address */
#define SMI230_ACCEL_I2C_ADDR_PRIMARY       UINT8_C(0x18)
#define SMI230_ACCEL_I2C_ADDR_SECONDARY     UINT8_C(0x19)

/**\name    Feature Config related Registers */
#define SMI230_ACCEL_RESERVED_5B_REG        UINT8_C(0x5B)
#define SMI230_ACCEL_RESERVED_5C_REG        UINT8_C(0x5C)
#define SMI230_ACCEL_FEATURE_CFG_REG        UINT8_C(0x5E)

/**\name    Interrupt masks */
#define SMI230_ACCEL_DATA_READY_INT         UINT8_C(0x80)

/**\name    Accel Bandwidth */
#define SMI230_ACCEL_BW_OSR4                UINT8_C(0x00)
#define SMI230_ACCEL_BW_OSR2                UINT8_C(0x01)
#define SMI230_ACCEL_BW_NORMAL              UINT8_C(0x02)

/**\name    smi230 Accel Range */
#define SMI230_ACCEL_RANGE_2G               UINT8_C(0x00)
#define SMI230_ACCEL_RANGE_4G               UINT8_C(0x01)
#define SMI230_ACCEL_RANGE_8G               UINT8_C(0x02)
#define SMI230_ACCEL_RANGE_16G              UINT8_C(0x03)

/**\name    Accel Output data rate */
#define SMI230_ACCEL_ODR_12_5_HZ            UINT8_C(0x05)
#define SMI230_ACCEL_ODR_25_HZ              UINT8_C(0x06)
#define SMI230_ACCEL_ODR_50_HZ              UINT8_C(0x07)
#define SMI230_ACCEL_ODR_100_HZ             UINT8_C(0x08)
#define SMI230_ACCEL_ODR_200_HZ             UINT8_C(0x09)
#define SMI230_ACCEL_ODR_400_HZ             UINT8_C(0x0A)
#define SMI230_ACCEL_ODR_800_HZ             UINT8_C(0x0B)
#define SMI230_ACCEL_ODR_1600_HZ            UINT8_C(0x0C)

/**\name    Accel Self test */
#define SMI230_ACCEL_SWITCH_OFF_SELF_TEST   UINT8_C(0x00)
#define SMI230_ACCEL_POSITIVE_SELF_TEST     UINT8_C(0x0D)
#define SMI230_ACCEL_NEGATIVE_SELF_TEST     UINT8_C(0x09)

/**\name    Accel Power mode */
#define SMI230_ACCEL_PM_ACTIVE              UINT8_C(0x00)
#define SMI230_ACCEL_PM_SUSPEND             UINT8_C(0x03)

/**\name    Accel Power control settings */
#define SMI230_ACCEL_POWER_DISABLE          UINT8_C(0x00)
#define SMI230_ACCEL_POWER_ENABLE           UINT8_C(0x04)

/**\name    Accel internal interrupt pin mapping */
#define SMI230_ACCEL_INTA_DISABLE           UINT8_C(0x00)
#define SMI230_ACCEL_INTA_ENABLE            UINT8_C(0x01)
#define SMI230_ACCEL_INTB_DISABLE           UINT8_C(0x00)
#define SMI230_ACCEL_INTB_ENABLE            UINT8_C(0x02)

/**\name    Accel Soft reset delay */
#define SMI230_ACCEL_SOFTRESET_DELAY_MS     UINT8_C(1)

/**\name    Mask definitions for ACCEL_ERR_REG register */
#define SMI230_FATAL_ERR_MASK               UINT8_C(0x01)
#define SMI230_ERR_CODE_MASK                UINT8_C(0x1C)

/**\name    Position definitions for ACCEL_ERR_REG register */
#define SMI230_CMD_ERR_POS                  UINT8_C(1)
#define SMI230_ERR_CODE_POS                 UINT8_C(2)

/**\name    Mask definition for ACCEL_STATUS_REG register */
#define SMI230_ACCEL_STATUS_MASK            UINT8_C(0x80)

/**\name    Position definitions for ACCEL_STATUS_REG  */
#define SMI230_ACCEL_STATUS_POS             UINT8_C(7)

/**\name    Mask definitions for odr, bandwidth and range */
#define SMI230_ACCEL_ODR_MASK               UINT8_C(0x0F)
#define SMI230_ACCEL_BW_MASK                UINT8_C(0x70)
#define SMI230_ACCEL_RANGE_MASK             UINT8_C(0x03)

/**\name    Position definitions for odr, bandwidth and range */
#define SMI230_ACCEL_BW_POS                 UINT8_C(4)

/**\name    Mask definitions for INT1_IO_CONF register */
#define SMI230_ACCEL_INT_EDGE_MASK          UINT8_C(0x01)
#define SMI230_ACCEL_INT_LVL_MASK           UINT8_C(0x02)
#define SMI230_ACCEL_INT_OD_MASK            UINT8_C(0x04)
#define SMI230_ACCEL_INT_IO_MASK            UINT8_C(0x08)
#define SMI230_ACCEL_INT_IN_MASK            UINT8_C(0x10)

/**\name    Position definitions for INT1_IO_CONF register */
#define SMI230_ACCEL_INT_EDGE_POS           UINT8_C(0)
#define SMI230_ACCEL_INT_LVL_POS            UINT8_C(1)
#define SMI230_ACCEL_INT_OD_POS             UINT8_C(2)
#define SMI230_ACCEL_INT_IO_POS             UINT8_C(3)
#define SMI230_ACCEL_INT_IN_POS             UINT8_C(4)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define SMI230_ACCEL_MAP_INTA_MASK          UINT8_C(0x01)

/**\name    Mask definitions for INT1/INT2 mapping register */
#define SMI230_ACCEL_MAP_INTA_POS           UINT8_C(0x00)

/**\name    Mask definitions for INT1_INT2_MAP_DATA register */
#define SMI230_ACCEL_INT1_DRDY_MASK         UINT8_C(0x04)
#define SMI230_ACCEL_INT2_DRDY_MASK         UINT8_C(0x40)

/**\name    Position definitions for INT1_INT2_MAP_DATA register */
#define SMI230_ACCEL_INT1_DRDY_POS          UINT8_C(2)
#define SMI230_ACCEL_INT2_DRDY_POS          UINT8_C(6)

/**\name    Asic Initialization value */
#define SMI230_ASIC_INITIALIZED             UINT8_C(0x01)

/*************************** SMI230 Gyroscope Macros *****************************/
/** Register map */
/* Gyro registers */

/**\name    Gyro Chip Id register */
#define SMI230_GYRO_CHIP_ID_REG                UINT8_C(0x00)

/**\name    Gyro X LSB data register */
#define SMI230_GYRO_X_LSB_REG                  UINT8_C(0x02)

/**\name    Gyro X MSB data register */
#define SMI230_GYRO_X_MSB_REG                  UINT8_C(0x03)

/**\name    Gyro Y LSB data register */
#define SMI230_GYRO_Y_LSB_REG                  UINT8_C(0x04)

/**\name    Gyro Y MSB data register */
#define SMI230_GYRO_Y_MSB_REG                  UINT8_C(0x05)

/**\name    Gyro Z LSB data register */
#define SMI230_GYRO_Z_LSB_REG                  UINT8_C(0x06)

/**\name    Gyro Z MSB data register */
#define SMI230_GYRO_Z_MSB_REG                  UINT8_C(0x07)

/**\name    Gyro Interrupt status register */
#define SMI230_GYRO_INT_STAT_1_REG             UINT8_C(0x0A)

/**\name    Gyro Range register */
#define SMI230_GYRO_RANGE_REG                  UINT8_C(0x0F)

/**\name    Gyro Bandwidth register */
#define SMI230_GYRO_BANDWIDTH_REG              UINT8_C(0x10)

/**\name    Gyro Power register */
#define SMI230_GYRO_LPM1_REG                   UINT8_C(0x11)

/**\name    Gyro Soft reset register */
#define SMI230_GYRO_SOFTRESET_REG              UINT8_C(0x14)

/**\name    Gyro Interrupt control register */
#define SMI230_GYRO_INT_CTRL_REG               UINT8_C(0x15)

/**\name    Gyro Interrupt Pin configuration register */
#define SMI230_GYRO_INT3_INT4_IO_CONF_REG      UINT8_C(0x16)

/**\name    Gyro Interrupt Map register */
#define SMI230_GYRO_INT3_INT4_IO_MAP_REG       UINT8_C(0x18)

/**\name    Gyro Self test register */
#define SMI230_GYRO_SELF_TEST_REG              UINT8_C(0x3C)

/**\name    Gyro unique chip identifier */
#define SMI230_GYRO_CHIP_ID                    UINT8_C(0x0F)

/**\name    Gyro I2C slave address */
#define SMI230_GYRO_I2C_ADDR_PRIMARY           UINT8_C(0x68)
#define SMI230_GYRO_I2C_ADDR_SECONDARY         UINT8_C(0x69)

/**\name    Gyro Range */
#define SMI230_GYRO_RANGE_2000_DPS             UINT8_C(0x00)
#define SMI230_GYRO_RANGE_1000_DPS             UINT8_C(0x01)
#define SMI230_GYRO_RANGE_500_DPS              UINT8_C(0x02)
#define SMI230_GYRO_RANGE_250_DPS              UINT8_C(0x03)
#define SMI230_GYRO_RANGE_125_DPS              UINT8_C(0x04)

/**\name    Gyro Output data rate and bandwidth */
#define SMI230_GYRO_BW_532_ODR_2000_HZ         UINT8_C(0x00)
#define SMI230_GYRO_BW_230_ODR_2000_HZ         UINT8_C(0x01)
#define SMI230_GYRO_BW_116_ODR_1000_HZ         UINT8_C(0x02)
#define SMI230_GYRO_BW_47_ODR_400_HZ           UINT8_C(0x03)
#define SMI230_GYRO_BW_23_ODR_200_HZ           UINT8_C(0x04)
#define SMI230_GYRO_BW_12_ODR_100_HZ           UINT8_C(0x05)
#define SMI230_GYRO_BW_64_ODR_200_HZ           UINT8_C(0x06)
#define SMI230_GYRO_BW_32_ODR_100_HZ           UINT8_C(0x07)
#define SMI230_GYRO_ODR_RESET_VAL              UINT8_C(0x80)

/**\name    Gyro Power mode */
#define SMI230_GYRO_PM_NORMAL                  UINT8_C(0x00)
#define SMI230_GYRO_PM_DEEP_SUSPEND            UINT8_C(0x20)
#define SMI230_GYRO_PM_SUSPEND                 UINT8_C(0x80)

/**\name    Gyro data ready interrupt enable value */
#define SMI230_GYRO_DRDY_INT_DISABLE_VAL       UINT8_C(0x00)
#define SMI230_GYRO_DRDY_INT_ENABLE_VAL        UINT8_C(0x80)

/**\name    Gyro data ready map values */
#define SMI230_GYRO_MAP_DRDY_TO_INT3           UINT8_C(0x01)
#define SMI230_GYRO_MAP_DRDY_TO_INT4           UINT8_C(0x80)
#define SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4 UINT8_C(0x81)

/**\name    Gyro Soft reset delay */
#define SMI230_GYRO_SOFTRESET_DELAY            UINT8_C(30)

/**\name    Gyro power mode config delay */
#define SMI230_GYRO_POWER_MODE_CONFIG_DELAY    UINT8_C(30)

/** Mask definitions for range, bandwidth and power */
#define SMI230_GYRO_RANGE_MASK                 UINT8_C(0x07)
#define SMI230_GYRO_BW_MASK                    UINT8_C(0x0F)
#define SMI230_GYRO_POWER_MASK                 UINT8_C(0xA0)

/** Position definitions for range, bandwidth and power */
#define SMI230_GYRO_POWER_POS                  UINT8_C(5)

/**\name    Mask definitions for SMI230_GYRO_INT_CTRL_REG register */
#define SMI230_GYRO_DATA_EN_MASK               UINT8_C(0x80)

/**\name    Position definitions for SMI230_GYRO_INT_CTRL_REG register */
#define SMI230_GYRO_DATA_EN_POS                UINT8_C(7)

/**\name    Mask definitions for SMI230_GYRO_INT3_INT4_IO_CONF_REG register */
#define SMI230_GYRO_INT3_LVL_MASK              UINT8_C(0x01)
#define SMI230_GYRO_INT3_OD_MASK               UINT8_C(0x02)
#define SMI230_GYRO_INT4_LVL_MASK              UINT8_C(0x04)
#define SMI230_GYRO_INT4_OD_MASK               UINT8_C(0x08)

/**\name    Position definitions for SMI230_GYRO_INT3_INT4_IO_CONF_REG register */
#define SMI230_GYRO_INT3_OD_POS                UINT8_C(1)
#define SMI230_GYRO_INT4_LVL_POS               UINT8_C(2)
#define SMI230_GYRO_INT4_OD_POS                UINT8_C(3)

/**\name    Mask definitions for SMI230_GYRO_INT_EN_REG register */
#define SMI230_GYRO_INT_EN_MASK                UINT8_C(0x80)

/**\name    Position definitions for SMI230_GYRO_INT_EN_REG register */
#define SMI230_GYRO_INT_EN_POS                 UINT8_C(7)

/**\name    Mask definitions for SMI230_GYRO_INT_MAP_REG register */
#define SMI230_GYRO_INT3_MAP_MASK              UINT8_C(0x01)
#define SMI230_GYRO_INT4_MAP_MASK              UINT8_C(0x80)

/**\name    Position definitions for SMI230_GYRO_INT_MAP_REG register */
#define SMI230_GYRO_INT3_MAP_POS               UINT8_C(0)
#define SMI230_GYRO_INT4_MAP_POS               UINT8_C(7)

/**\name    Mask definitions for GYRO_SELF_TEST register */
#define SMI230_GYRO_SELF_TEST_EN_MASK          UINT8_C(0x01)
#define SMI230_GYRO_SELF_TEST_RDY_MASK         UINT8_C(0x02)
#define SMI230_GYRO_SELF_TEST_RESULT_MASK      UINT8_C(0x04)
#define SMI230_GYRO_SELF_TEST_FUNCTION_MASK    UINT8_C(0x08)

/**\name    Position definitions for GYRO_SELF_TEST register */
#define SMI230_GYRO_SELF_TEST_RDY_POS          UINT8_C(1)
#define SMI230_GYRO_SELF_TEST_RESULT_POS       UINT8_C(2)
#define SMI230_GYRO_SELF_TEST_FUNCTION_POS     UINT8_C(3)

/*************************** Common Macros for both Accel and Gyro *****************************/
/**\name    SPI read/write mask to configure address */
#define SMI230_SPI_RD_MASK                     UINT8_C(0x80)
#define SMI230_SPI_WR_MASK                     UINT8_C(0x7F)

/**\name API success code */
#define SMI230_OK                              INT8_C(0)

/**\name API error codes */
#define SMI230_E_NULL_PTR                      INT8_C(-1)
#define SMI230_E_COM_FAIL                      INT8_C(-2)
#define SMI230_E_DEV_NOT_FOUND                 INT8_C(-3)
#define SMI230_E_OUT_OF_RANGE                  INT8_C(-4)
#define SMI230_E_INVALID_INPUT                 INT8_C(-5)
#define SMI230_E_CONFIG_STREAM_ERROR           INT8_C(-6)
#define SMI230_E_RD_WR_LENGTH_INVALID          INT8_C(-7)
#define SMI230_E_INVALID_CONFIG                INT8_C(-8)
#define SMI230_E_FEATURE_NOT_SUPPORTED         INT8_C(-9)

/**\name API warning codes */
#define SMI230_W_SELF_TEST_FAIL                INT8_C(1)

/***\name    Soft reset Value */
#define SMI230_SOFT_RESET_CMD                  UINT8_C(0xB6)

/**\name    Enable/disable macros */
#define SMI230_DISABLE                         UINT8_C(0)
#define SMI230_ENABLE                          UINT8_C(1)

/*! @name To define warnings for FIFO activity */
#define SMI230_W_FIFO_EMPTY                    INT8_C(1)
#define SMI230_W_PARTIAL_READ                  INT8_C(2)

/**\name    Constant values macros */
#define SMI230_SENSOR_DATA_SYNC_TIME_MS        UINT8_C(1)
#define SMI230_DELAY_BETWEEN_WRITES_MS         UINT8_C(1)
#define SMI230_SELF_TEST_DELAY_MS              UINT8_C(3)
#define SMI230_POWER_CONFIG_DELAY              UINT8_C(5)
#define SMI230_SENSOR_SETTLE_TIME_MS           UINT8_C(30)
#define SMI230_SELF_TEST_DATA_READ_MS          UINT8_C(50)
#define SMI230_ASIC_INIT_TIME_MS               UINT8_C(150)

#define SMI230_CONFIG_STREAM_SIZE              UINT16_C(6144)

/**\name    Sensor time array parameter definitions */
#define SMI230_SENSOR_TIME_MSB_BYTE            UINT8_C(2)
#define SMI230_SENSOR_TIME_XLSB_BYTE           UINT8_C(1)
#define SMI230_SENSOR_TIME_LSB_BYTE            UINT8_C(0)

/**\name   int pin active state */
#define SMI230_INT_ACTIVE_LOW                  UINT8_C(0)
#define SMI230_INT_ACTIVE_HIGH                 UINT8_C(1)

/**\name   interrupt pin output definition  */
#define SMI230_INT_MODE_PUSH_PULL              UINT8_C(0)
#define SMI230_INT_MODE_OPEN_DRAIN             UINT8_C(1)

/**\name    Sensor bit resolution */
#define SMI230_16_BIT_RESOLUTION               UINT8_C(16)

/*********************************SMI230 FIFO Macros**********************************/
/** Register map */
/*! @name FIFO Header Mask definitions */
#define SMI230_FIFO_HEADER_ACC_FRM             UINT8_C(0x84)
#define SMI230_FIFO_HEADER_ALL_FRM             UINT8_C(0x9C)
#define SMI230_FIFO_HEADER_SENS_TIME_FRM       UINT8_C(0x44)
#define SMI230_FIFO_HEADER_SKIP_FRM            UINT8_C(0x40)
#define SMI230_FIFO_HEADER_INPUT_CFG_FRM       UINT8_C(0x48)
#define SMI230_FIFO_HEAD_OVER_READ_MSB         UINT8_C(0x80)
#define SMI230_FIFO_SAMPLE_DROP_FRM            UINT8_C(0x50)

/* Accel registers */
#define SMI230_FIFO_LENGTH_0_ADDR              UINT8_C(0x24)
#define SMI230_FIFO_LENGTH_1_ADDR              UINT8_C(0x25)
#define SMI230_FIFO_DATA_ADDR                  UINT8_C(0x26)
#define SMI230_FIFO_DOWNS_ADDR                 UINT8_C(0x45)
#define SMI230_FIFO_WTM_0_ADDR                 UINT8_C(0x46)
#define SMI230_FIFO_WTM_1_ADDR                 UINT8_C(0x47)
#define SMI230_FIFO_CONFIG_0_ADDR              UINT8_C(0x48)
#define SMI230_FIFO_CONFIG_1_ADDR              UINT8_C(0x49)

/*! @name FIFO sensor data lengths */
#define SMI230_FIFO_ACCEL_LENGTH               UINT8_C(6)
#define SMI230_FIFO_WTM_LENGTH                 UINT8_C(2)
#define SMI230_FIFO_LENGTH_MSB_BYTE            UINT8_C(1)
#define SMI230_FIFO_DATA_LENGTH                UINT8_C(2)
#define SMI230_FIFO_CONFIG_LENGTH              UINT8_C(2)
#define SMI230_SENSOR_TIME_LENGTH              UINT8_C(3)
#define SMI230_FIFO_SKIP_FRM_LENGTH            UINT8_C(1)
#define SMI230_FIFO_INPUT_CFG_LENGTH           UINT8_C(1)

/*! @name FIFO byte counter mask definition */
#define SMI230_FIFO_BYTE_COUNTER_MSB_MASK      UINT8_C(0x3F)

/*! @name FIFO frame masks */
#define SMI230_FIFO_LSB_CONFIG_CHECK           UINT8_C(0x00)
#define SMI230_FIFO_MSB_CONFIG_CHECK           UINT8_C(0x80)
#define SMI230_FIFO_INTR_MASK                  UINT8_C(0x5C)

/*name FIFO config modes */
#define SMI230_ACC_STREAM_MODE                 UINT8_C(0x00)
#define SMI230_ACC_FIFO_MODE                   UINT8_C(0x01)

/*name Mask definitions for FIFO configuration modes */
#define SMI230_ACC_FIFO_MODE_CONFIG_MASK       UINT8_C(0x01)

/*! @name Mask definitions for FIFO_CONFIG_1 register */
#define SMI230_ACCEL_EN_MASK                   UINT8_C(0x40)
#define SMI230_ACCEL_INT1_EN_MASK              UINT8_C(0x08)
#define SMI230_ACCEL_INT2_EN_MASK              UINT8_C(0x04)

/*name Position definitions for FIFO_CONFIG_1 register */
#define SMI230_ACCEL_EN_POS                    UINT8_C(6)
#define SMI230_ACCEL_INT1_EN_POS               UINT8_C(3)
#define SMI230_ACCEL_INT2_EN_POS               UINT8_C(2)

/*! @name Position definitions for FIFO_DOWNS register */
#define SMI230_ACC_FIFO_DOWNS_MASK             UINT8_C(0xF0)

/*! @name FIFO down sampling bit positions */
#define SMI230_ACC_FIFO_DOWNS_POS              UINT8_C(0x04)

/*! @name FIFO down sampling user macros */
#define SMI230_ACC_FIFO_DOWN_SAMPLE_0          UINT8_C(0)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_1          UINT8_C(1)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_2          UINT8_C(2)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_3          UINT8_C(3)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_4          UINT8_C(4)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_5          UINT8_C(5)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_6          UINT8_C(6)
#define SMI230_ACC_FIFO_DOWN_SAMPLE_7          UINT8_C(7)

/*! @name Mask definitions for INT1_INT2_MAP_DATA register */
#define SMI230_ACCEL_INT2_FWM_MASK             UINT8_C(0x20)
#define SMI230_ACCEL_INT2_FFULL_MASK           UINT8_C(0x10)
#define SMI230_ACCEL_INT1_FWM_MASK             UINT8_C(0x02)
#define SMI230_ACCEL_INT1_FFULL_MASK           UINT8_C(0x01)

/*! @name Positions definitions for INT1_INT2_MAP_DATA register */
#define SMI230_ACCEL_INT1_FWM_POS              UINT8_C(1)
#define SMI230_ACCEL_INT2_FFULL_POS            UINT8_C(4)
#define SMI230_ACCEL_INT2_FWM_POS              UINT8_C(5)

/**\name    Absolute value */
#ifndef SMI230_ABS
#define SMI230_ABS(a) ((a) > 0 ? (a) : -(a))
#endif

/**\name    Utility Macros  */
#define SMI230_SET_LOW_BYTE   UINT16_C(0x00FF)
#define SMI230_SET_HIGH_BYTE  UINT16_C(0xFF00)
#define SMI230_SET_LOW_NIBBLE UINT8_C(0x0F)

/**\name Macro to SET and GET BITS of a register */
#define SMI230_SET_BITS(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     ((val << bitname##_POS) & bitname##_MASK))

#define SMI230_GET_BITS(reg_var, bitname)       ((reg_var & (bitname##_MASK)) >> \
                                                 (bitname##_POS))

#define SMI230_SET_BITS_POS_0(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     (val & bitname##_MASK))

#define SMI230_GET_BITS_POS_0(reg_var, bitname) (reg_var & (bitname##_MASK))

#define SMI230_SET_BIT_VAL_0(reg_var, bitname)  (reg_var & ~(bitname##_MASK))

/**\name     Macro definition for difference between 2 values */
#define SMI230_GET_DIFF(x, y)                   ((x) - (y))

/**\name     Macro definition to get LSB of 16 bit variable */
#define SMI230_GET_LSB(var)                     (uint8_t)(var & SMI230_SET_LOW_BYTE)

/**\name     Macro definition to get MSB of 16 bit variable */
#define SMI230_GET_MSB(var)                     (uint8_t)((var & SMI230_SET_HIGH_BYTE) >> 8)

/*************************************************************************/

/*!
 * @brief Interface selection enums
 */
enum smi230_intf {
    /*! I2C interface */
    SMI230_I2C_INTF,

    /*! SPI interface */
    SMI230_SPI_INTF
};

/*************************** Data structures *****************************/

/**\name    Typedef definitions */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 *
 *  @note : dev_addr is used for I2C read/write operations only.
 *          For SPI read/write operations this is dummy variable.
 */
typedef int8_t (*smi230_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**\name    delay function pointer */
typedef void (*smi230_delay_fptr_t)(uint32_t period);

/**\name    Structure Definitions */

/*!
 *  @brief Sensor XYZ data structure
 */
struct smi230_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;
};

/*!
 *  @brief Sensor XYZ data structure in float representation
 */
struct smi230_sensor_data_f
{
    /*! X-axis sensor data */
    float x;

    /*! Y-axis sensor data */
    float y;

    /*! Z-axis sensor data */
    float z;
};

/*!
 *  @brief Sensor configuration structure
 */
struct smi230_cfg
{
    /*! power mode */
    uint8_t power;

    /*! range */
    uint8_t range;

    /*! bandwidth */
    uint8_t bw;

    /*! output data rate */
    uint8_t odr;
};

/*!
 *  @brief Error Status structure
 */
struct smi230_err_reg
{
    /*! Indicates fatal error */
    uint8_t fatal_err;

    /*! Indicates error code */
    uint8_t err_code;
};

#define SMI230_ACCEL_ANYMOTION_ADR                0x00
#define SMI230_ACCEL_ANYMOTION_LEN                2
#define SMI230_ACCEL_ANYMOTION_THRESHOLD_MASK     0x07FF
#define SMI230_ACCEL_ANYMOTION_THRESHOLD_SHIFT    0
#define SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_MASK  0x0800
#define SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT 11
#define SMI230_ACCEL_ANYMOTION_DURATION_MASK      0x1FFF
#define SMI230_ACCEL_ANYMOTION_DURATION_SHIFT     0
#define SMI230_ACCEL_ANYMOTION_X_EN_MASK          0x2000
#define SMI230_ACCEL_ANYMOTION_X_EN_SHIFT         13
#define SMI230_ACCEL_ANYMOTION_Y_EN_MASK          0x4000
#define SMI230_ACCEL_ANYMOTION_Y_EN_SHIFT         14
#define SMI230_ACCEL_ANYMOTION_Z_EN_MASK          0x8000
#define SMI230_ACCEL_ANYMOTION_Z_EN_SHIFT         15

/*!
 *  @brief Anymotion config structure
 */
struct smi230_anymotion_cfg
{
    /* 11 bit threshold of anymotion detection (threshold = X mg * 2,048 (5.11 format)) */
    uint16_t threshold;

    /* 1 bit select between any- and nomotion behavior */
    uint16_t nomotion_sel;

    /* 13 bit set the duration for any- and nomotion (time = duration * 20ms (@50Hz)) */
    uint16_t duration;

    /* enable anymotion detection for x axis */
    uint16_t x_en;

    /* enable anymotion detection for y axis */
    uint16_t y_en;

    /* enable anymotion detection for z axis */
    uint16_t z_en;
};

#define SMI230_ACCEL_DATA_SYNC_ADR         0x02
#define SMI230_ACCEL_DATA_SYNC_LEN         1
#define SMI230_ACCEL_DATA_SYNC_MODE_MASK   0x0003
#define SMI230_ACCEL_DATA_SYNC_MODE_SHIFT  0

#define SMI230_ACCEL_DATA_SYNC_MODE_OFF    0x00
#define SMI230_ACCEL_DATA_SYNC_MODE_400HZ  0x01
#define SMI230_ACCEL_DATA_SYNC_MODE_1000HZ 0x02
#define SMI230_ACCEL_DATA_SYNC_MODE_2000HZ 0x03

/*!
 *  @brief Data Sync config structure
 */
struct smi230_data_sync_cfg
{
    /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
    uint8_t mode;
};

/*!
 *  @brief Enum to select accelerometer Interrupt pins
 */
enum smi230_accel_int_channel {
    /* interrupt Channel 1 for accel sensor */
    SMI230_INT_CHANNEL_1,

    /* interrupt Channel 2 for accel sensor */
    SMI230_INT_CHANNEL_2
};

/*!
 *  @brief Enum to select gyroscope Interrupt pins
 */
enum smi230_gyro_int_channel {
    /* interrupt Channel 3 for gyro sensor */
    SMI230_INT_CHANNEL_3,

    /* interrupt Channel 4 for gyro sensor */
    SMI230_INT_CHANNEL_4
};

/*!
 *  @brief Enum to select accelerometer interrupts
 */
enum smi230_accel_int_types {
    SMI230_ACCEL_DATA_RDY_INT,

    /* Accel data ready interrupt */
    SMI230_ACCEL_SYNC_DATA_RDY_INT,

    /* Accel synchronized data ready interrupt */
    SMI230_ACCEL_SYNC_INPUT,

    /* Accel anymotion interrupt for SMI230*/
    SMI230_ACCEL_ANYMOTION_INT,

    /* Accel FIFO watermark interrupt */
    SMI230_FIFO_WM_INT,

    /* Accel FIFO full interrupt */
    SMI230_FIFO_FULL_INT
};

/*!
 *  @brief Enum to select gyroscope interrupts
 */
enum smi230_gyro_int_types {
    /* Gyro data ready interrupt */
    SMI230_GYRO_DATA_RDY_INT
};

/*!
 *  @brief Interrupt pin configuration structure
 */
struct smi230_int_pin_cfg
{

    /*! interrupt pin level configuration
     * Assignable macros :
     * - SMI230_INT_ACTIVE_LOW
     * - SMI230_INT_ACTIVE_HIGH
     */
    uint8_t lvl : 1;

    /*! interrupt pin mode configuration
     * Assignable macros :
     * - SMI230_INT_MODE_PUSH_PULL
     * - SMI230_INT_MODE_OPEN_DRAIN
     */
    uint8_t output_mode : 1;

    /*! Enable interrupt pin
     * Assignable Macros :
     * - SMI230_ENABLE
     * - SMI230_DISABLE
     */
    uint8_t enable_int_pin : 1;
};

/*!
 *  @brief Interrupt channel structure for accel
 */
struct smi230_accel_int_channel_cfg
{
    /*! Accel Interrupt channel */
    enum smi230_accel_int_channel int_channel;

    /*! Select Accel Interrupt type */
    enum smi230_accel_int_types int_type;

    /*! Structure to configure accel interrupt pins */
    struct smi230_int_pin_cfg int_pin_cfg;
};

/*!
 *  @brief Interrupt channel structure for gyro
 */
struct smi230_gyro_int_channel_cfg
{
    /*! Gyro Interrupt channel */
    enum smi230_gyro_int_channel int_channel;

    /*! Select Gyro Interrupt type */
    enum smi230_gyro_int_types int_type;

    /*! Structure to configure gyro interrupt pins */
    struct smi230_int_pin_cfg int_pin_cfg;
};

/*!
 *  @brief Interrupt Configuration structure
 */
struct smi230_int_cfg
{
    /*! Configuration of first accel interrupt channel */
    struct smi230_accel_int_channel_cfg accel_int_config_1;

    /*! Configuration of second accel interrupt channel */
    struct smi230_accel_int_channel_cfg accel_int_config_2;

    /*! Configuration of first gyro interrupt channel */
    struct smi230_gyro_int_channel_cfg gyro_int_config_1;

    /*! Configuration of second gyro interrupt channel */
    struct smi230_gyro_int_channel_cfg gyro_int_config_2;
};

/*!
 *  @brief accel fifo configurations
 */
struct accel_fifo_config
{
    /*! Configure the fifo mode (0 = Stream mode, 1 = FIFO mode) */
    uint8_t mode;

    /*! To enable the accel */
    uint8_t accel_en;

    /*! To enable the interrupt_1 */
    uint8_t int1_en;

    /*! To enable the interrupt_2 */
    uint8_t int2_en;
};

/*! @name Structure to define FIFO frame configuration */
struct smi230_fifo_frame
{
    /*! Pointer to FIFO data */
    uint8_t *data;

    /*! Number of user defined bytes of FIFO to be read */
    uint16_t length;

    /*! Enables type of data to be streamed - accelerometer */
    uint16_t data_enable;

    /*! To index accelerometer bytes */
    uint16_t acc_byte_start_idx;

    /*! FIFO sensor time */
    uint32_t sensor_time;

    /*! Skipped frame count */
    uint8_t skipped_frame_count;

    /*! Type of data interrupt to be mapped */
    uint8_t data_int_map;

    /*! Water-mark level for water-mark interrupt */
    uint16_t wm_lvl;

    /*! Accelerometer frame length */
    uint8_t acc_frm_len;

    /*! Accelerometer frame length */
    uint8_t all_frm_len;

    /*! FIFO accelerometer configurations */
    struct accel_fifo_config fifo_conf;
};

/*!
 *  @brief
 *  This structure holds all relevant information about SMI230
 */
struct smi230_dev
{

    /*! Accel chip Id */
    uint8_t accel_chip_id;

    /*! Gyro chip Id */
    uint8_t gyro_chip_id;

    /*! Accel device Id */
    uint8_t accel_id;

    /*! Gyro device Id */
    uint8_t gyro_id;

    /*! 0 - I2C , 1 - SPI Interface */
    enum smi230_intf intf;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Structure to configure accel sensor  */
    struct smi230_cfg accel_cfg;

    /*! Structure to configure gyro sensor  */
    struct smi230_cfg gyro_cfg;

    /*! Config stream data buffer address will be assigned */
    const uint8_t *config_file_ptr;

    /*! Max read/write length (maximum supported length is 32).
     * To be set by the user */
    uint8_t read_write_len;

    /*! Read function pointer */
    smi230_com_fptr_t read;

    /*! Write function pointer */
    smi230_com_fptr_t write;

    /*! Delay function pointer */
    smi230_delay_fptr_t delay_ms;
};

#endif /* _SMI230_DEFS_H */

/** @}*/
