// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
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

/*! \file smi230_acc.c
 * \brief Sensor Driver for SMI230 sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include <linux/bug.h>
#include "smi230.h"

/****************************************************************************/

/** \name       Macros
 ****************************************************************************/
/**\name    Value of LSB_PER_G = (power(2, SMI230_16_BIT_RESOLUTION) / (2 * range)) */
#define LSB_PER_G UINT32_C(2048) /* for the 16-bit resolution and 16g range */

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/*!
 * @brief Accel self test diff xyz data structure
 */
struct selftest_delta_limit
{
    /*! Accel X  data */
    uint16_t x;

    /*! Accel Y  data */
    uint16_t y;

    /*! Accel Z  data */
    uint16_t z;
};

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct smi230_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_int_pin_config(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_data_ready_int(const struct smi230_accel_int_channel_cfg *int_config,
                                       const struct smi230_dev *dev);

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_sync_data_ready_int(const struct smi230_accel_int_channel_cfg *int_config,
                                            const struct smi230_dev *dev);

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_sync_input(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev);

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_accel_anymotion_int(const struct smi230_accel_int_channel_cfg *int_config,
                                      const struct smi230_dev *dev);

/*!
 * @brief This API writes the config stream data in memory using burst mode
 *
 * @param[in] stream_data : Pointer to store data of 32 bytes
 * @param[in] index       : Represents value in multiple of 32 bytes
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, const struct smi230_dev *dev);

/*!
 * @brief This API performs the pre-requisites needed to perform the self test
 *
 * @param[in] dev : structure instance of smi230_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t enable_self_test(struct smi230_dev *dev);

/*!
 * @brief This API reads the accel data with the positive excitation
 *
 * @param[out] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] dev   : structure instance of smi230_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t positive_excited_accel(struct smi230_sensor_data *accel_pos, const struct smi230_dev *dev);

/*!
 * @brief This API reads the accel data with the negative excitation
 *
 * @param[out] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 * @param[in] dev   : structure instance of smi230_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t negative_excited_accel(struct smi230_sensor_data *accel_neg, const struct smi230_dev *dev);

/*!
 * @brief This API validates the self test results
 *
 * @param[in] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Self test fail
 */
static int8_t validate_accel_self_test(const struct smi230_sensor_data *accel_pos,
                                       const struct smi230_sensor_data *accel_neg);

/*!
 * @brief This API converts lsb value of axes to mg for self-test
 *
 * @param[in] accel_data_diff     : Pointer variable used to pass accel difference
 * values in g
 *
 * @param[out] accel_data_diff_mg : Pointer variable used to store accel
 * difference values in mg
 *
 * @return None
 */
static void convert_lsb_g(const struct selftest_delta_limit *accel_data_diff,
                          struct selftest_delta_limit *accel_data_diff_mg);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data.
 *
 * @param[out] acc              : Structure instance of smi230_sensor_data
 *                                where the parsed data bytes are stored.
 * @param[in]  data_start_index : Index value of the accelerometer data bytes
 *                                which is to be parsed from the FIFO data.
 * @param[in]  fifo             : Structure instance of smi230_fifo_frame.
 *
 * @return None
 * @retval None
 */
static void unpack_accel_data(struct smi230_sensor_data *acc,
                              uint16_t data_start_index,
                              const struct smi230_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in both header and header-less mode. It updates the current data
 * byte to be parsed.
 *
 * @param[in,out] acc       : Structure instance of smi230_sensor_data where
 *                            where the parsed data bytes are stored.
 * @param[in,out] idx       : Index value of number of bytes parsed.
 * @param[in,out] acc_idx   : Index value of accelerometer data (x,y,z axes)
 *                            frame to be parsed.
 * @param[in]     frame     : Either data is enabled by user in header-less
 *                            mode or header frame value in header mode.
 * @param[in]     fifo      : Structure instance of smi230_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_W_FIFO_EMPTY - Warning : FIFO is empty
 */
static int8_t unpack_accel_frame(struct smi230_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct smi230_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 *
 * @param[in,out] data_index : Index of the FIFO data which contains skipped
 *                             frame count.
 * @param[in] fifo           : Structure instance of smi230_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval SMI230_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t unpack_skipped_frame(uint16_t *data_index, struct smi230_fifo_frame *fifo);

/*!
 * @brief This internal API is used to move the data index ahead of the
 * current frame length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 *
 * @param[in,out] data_index           : Index of the FIFO data which is to be
 *                                       moved ahead of the current frame length
 * @param[in]     current_frame_length : Number of bytes in the current frame.
 * @param[in]     fifo                 : Structure instance of smi230_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval SMI230_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct smi230_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 *
 * @param[in,out] data_index : Index of the FIFO data which has the sensor time.
 * @param[in]     fifo       : Structure instance of smi230_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval SMI230_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t unpack_sensortime_frame(uint16_t *data_index, struct smi230_fifo_frame *fifo);

/*!
 * @brief This internal API is used to reset the FIFO related configurations
 * in the FIFO frame structure for the next FIFO read.
 *
 * @param[in, out] fifo     : Structure instance of smi230_fifo_frame.
 * @param[in]      dev      : Structure instance of smi230_dev.
 *
 * @return None
 * @retval None
 */
static void reset_fifo_frame_structure(struct smi230_fifo_frame *fifo);

/*!
 * @brief This internal API is used to parse accelerometer data from the FIFO
 * data in header mode.
 *
 * @param[out] acc          : Structure instance of smi230_sens_data where
 *                            the parsed accelerometer data bytes are stored.
 * @param[in] accel_length  : Number of accelerometer frames (x,y,z data).
 * @param[in] fifo          : Structure instance of smi230_fifo_frame.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval SMI230_W_PARTIAL_READ - Warning : There are more frames to be read
 */
static int8_t extract_acc_header_mode(struct smi230_sensor_data *acc,
                                      uint16_t *accel_length,
                                      struct smi230_fifo_frame *fifo);

/*!
 * @brief This API sets the FIFO watermark interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_fifo_wm_int(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev);

/*!
 * @brief This API sets the FIFO full interrupt for accel sensor
 *
 * @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_fifo_full_int(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 */
int8_t smi230_acc_init(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        if (dev->intf == SMI230_SPI_INTF)
        {
            /* Set dummy byte in case of SPI interface */
            dev->dummy_byte = SMI230_ENABLE;

            /* Dummy read of Chip-ID in SPI mode */
            rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);
        }
        else
        {
            /* Make dummy byte 0 in case of I2C interface */
            dev->dummy_byte = SMI230_DISABLE;
        }
        if (rslt == SMI230_OK)
        {
            rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);

            if (rslt == SMI230_OK)
            {
                /* Check for chip id validity */
                if (chip_id == SMI230_ACCEL_CHIP_ID)
                {
                    /* Store the chip ID in dev structure */
                    dev->accel_chip_id = chip_id;
                }
                else
                {
                    rslt = SMI230_E_DEV_NOT_FOUND;
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to write the binary configuration in the sensor.
 */
int8_t smi230_acc_write_config_file(const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Config loading disable*/
    uint8_t config_load = SMI230_DISABLE;
    uint8_t current_acc_pwr_ctrl = 0;
    uint16_t index = 0;
    uint8_t reg_data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check if config file pointer is not null */
    if ((rslt == SMI230_OK) && (dev->config_file_ptr != NULL))
    {

        /* Check whether the read/write length is valid */
        if (dev->read_write_len > 0)
        {
            /* deactivate accel, otherwise post processing can not be enabled safely */
            rslt = get_regs(SMI230_ACCEL_PWR_CTRL_REG, &current_acc_pwr_ctrl, 1, dev);
            if (rslt != SMI230_OK)
            {
                return rslt;
            }

            rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG, &config_load, 1, dev);
            if (rslt == SMI230_OK)
            {
                /*delay required to switch power modes*/
                dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
            }
            else
            {
                return rslt;
            }

            /* Disable config loading*/
            rslt = set_regs(SMI230_ACCEL_INIT_CTRL_REG, &config_load, 1, dev);

            if (rslt == SMI230_OK)
            {
                for (index = 0; index < SMI230_CONFIG_STREAM_SIZE;
                     index += dev->read_write_len)
                {
                    /* Write the config stream */
                    rslt = stream_transfer_write((dev->config_file_ptr + index), index, dev);
                }
                if (rslt == SMI230_OK)
                {
                    /* Enable config loading and FIFO mode */
                    config_load = SMI230_ENABLE;

                    rslt = set_regs(SMI230_ACCEL_INIT_CTRL_REG, &config_load, 1, dev);

                    if (rslt == SMI230_OK)
                    {
                        /* Wait till ASIC is initialized. Refer the data-sheet for more information */
                        dev->delay_ms(SMI230_ASIC_INIT_TIME_MS);

                        /* Check for config initialization status (1 = OK)*/
                        rslt = get_regs(SMI230_ACCEL_INTERNAL_STAT_REG, &reg_data, 1, dev);
                    }
                    if (rslt == SMI230_OK && reg_data != 1)
                    {
                        rslt = SMI230_E_CONFIG_STREAM_ERROR;
                    }
                    else
                    {
                        /* reactivate accel */
                        rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG, &current_acc_pwr_ctrl, 1, dev);
                        if (rslt == SMI230_OK)
                        {
                            /*delay required to switch power modes*/
                            dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
                        }
                    }
                }
            }
        }
        else
        {
            rslt = SMI230_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API writes the feature configuration to the accel sensor.
 */
int8_t smi230_acc_write_feature_config(uint8_t reg_addr, const uint16_t *reg_data, uint8_t len,
                                   const struct smi230_dev *dev)
{

    int8_t rslt;
    uint16_t read_length = (reg_addr * 2) + (len * 2);
    int i;
    uint8_t feature_data[CONFIG_SMI230_MAX_BUFFER_LEN];

    if (WARN(read_length > CONFIG_SMI230_MAX_BUFFER_LEN, "SMI230 buffer overflow\n")) {
        return SMI230_E_COM_FAIL;
    }

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {
        /* Read feature space up to the given feature position */
        rslt = smi230_acc_get_regs(SMI230_ACCEL_FEATURE_CFG_REG, &feature_data[0], read_length, dev);

        if (rslt == SMI230_OK)
        {
            /* Apply the given feature config. */
            for (i = 0; i < len; ++i)
            {
                /* Be careful: the feature config space is 16bit aligned! */
                feature_data[(reg_addr * 2) + (i * 2)] = reg_data[i] & 0xFF;
                feature_data[(reg_addr * 2) + (i * 2) + 1] = reg_data[i] >> 8;
            }

            /* Write back updated feature space */
            rslt = smi230_acc_set_regs(SMI230_ACCEL_FEATURE_CFG_REG, &feature_data[0], read_length, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the data from the given register address of accel sensor.
 */
int8_t smi230_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Reading from the register */
            rslt = get_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = SMI230_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API writes the given data to the register address
 *  of accel sensor.
 */
int8_t smi230_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Writing to the register */
            rslt = set_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = SMI230_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the error status from the accel sensor.
 */
int8_t smi230_acc_get_error_status(struct smi230_err_reg *err_reg, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        if (err_reg != NULL)
        {
            /* Read the error codes */
            rslt = get_regs(SMI230_ACCEL_ERR_REG, &data, 1, dev);

            if (rslt == SMI230_OK)
            {
                /* Fatal error */
                err_reg->fatal_err = SMI230_GET_BITS_POS_0(data, SMI230_FATAL_ERR);

                /* User error */
                err_reg->err_code = SMI230_GET_BITS(data, SMI230_ERR_CODE);
            }
        }
        else
        {
            rslt = SMI230_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the status of the accel sensor.
 */
int8_t smi230_acc_get_status(uint8_t *status, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (status != NULL))
    {
        /* Read the status */
        rslt = get_regs(SMI230_ACCEL_STATUS_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Updating the status */
            *status = SMI230_GET_BITS(data, SMI230_ACCEL_STATUS);
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API resets the accel sensor.
 */
int8_t smi230_acc_soft_reset(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        data = SMI230_SOFT_RESET_CMD;

        /* Reset accel device */
        rslt = set_regs(SMI230_ACCEL_SOFTRESET_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Delay 1 ms after reset value is written to its register */
            dev->delay_ms(SMI230_ACCEL_SOFTRESET_DELAY_MS);

            /* After soft reset SPI mode in the initialization phase, need to  perform a dummy SPI read
             * operation, The soft-reset performs a fundamental reset to the device,
             * which is largely equivalent to a power cycle. */
            if (dev->intf == SMI230_SPI_INTF)
            {
                /* Dummy SPI read operation of Chip-ID */
                rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel config value i.e. odr, band width and range from the sensor,
 * store it in the smi230_dev structure instance passed by the user.
 *
 */
int8_t smi230_acc_get_meas_conf(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);

        if (rslt == SMI230_OK)
        {
            dev->accel_cfg.odr = data[0] & SMI230_ACCEL_ODR_MASK;
            dev->accel_cfg.bw = (data[0] & SMI230_ACCEL_BW_MASK) >> 4;
            dev->accel_cfg.range = data[1] & SMI230_ACCEL_RANGE_MASK;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of accel sensor.
 */
int8_t smi230_acc_set_meas_conf(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint8_t bw, range, odr;
    uint8_t is_odr_invalid = FALSE, is_bw_invalid = FALSE, is_range_invalid = FALSE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        odr = dev->accel_cfg.odr;
        bw = dev->accel_cfg.bw;
        range = dev->accel_cfg.range;

        /* Check for valid ODR */
        if ((odr < SMI230_ACCEL_ODR_12_5_HZ) || (odr > SMI230_ACCEL_ODR_1600_HZ))
        {
            /* Updating the status */
            is_odr_invalid = TRUE;
        }

        /* Check for valid bandwidth */
        if (bw > SMI230_ACCEL_BW_NORMAL)
        {
            /* Updating the status */
            is_bw_invalid = TRUE;
        }

        /* Check for valid Range */
        if (range > SMI230_ACCEL_RANGE_16G)
        {
            /* Updating the status */
            is_range_invalid = TRUE;
        }

        /* If ODR, BW and Range are valid, write it to accel config. registers */
        if ((!is_odr_invalid) && (!is_bw_invalid) && (!is_range_invalid))
        {
            /* Read accel config. register */
            rslt = get_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);
            if (rslt == SMI230_OK)
            {
                /* Update data with new odr and bw values */
                data[0] = SMI230_SET_BITS_POS_0(data[0], SMI230_ACCEL_ODR, odr);
                data[0] = SMI230_SET_BITS(data[0], SMI230_ACCEL_BW, bw);

                /* Update data with current range values */
                data[1] = SMI230_SET_BITS_POS_0(data[1], SMI230_ACCEL_RANGE, range);

                /* write to range register */
                rslt = set_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);
            }
        }
        else
        {
            /* Invalid configuration present in ODR, BW, Range */
            rslt = SMI230_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel power mode from the sensor, store it in the smi230_dev structure
 * instance passed by the user.
 */
int8_t smi230_acc_get_power_mode(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_ACCEL_PWR_CONF_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Updating the current power mode */
            dev->accel_cfg.power = data;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the accel sensor.
 */
int8_t smi230_acc_set_power_mode(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        power_mode = dev->accel_cfg.power;

        /* Configure data array to write to accel power configuration register */
        if (power_mode == SMI230_ACCEL_PM_ACTIVE)
        {
            data[0] = SMI230_ACCEL_PM_ACTIVE;
            data[1] = SMI230_ACCEL_POWER_ENABLE;
        }
        else if (power_mode == SMI230_ACCEL_PM_SUSPEND)
        {
            data[0] = SMI230_ACCEL_PM_SUSPEND;
            data[1] = SMI230_ACCEL_POWER_DISABLE;
        }
        else
        {
            /* Invalid power input */
            rslt = SMI230_E_INVALID_INPUT;
        }

        if (rslt == SMI230_OK)
        {
            /*enable accel sensor*/
            rslt = set_regs(SMI230_ACCEL_PWR_CONF_REG, &data[0], 1, dev);

            if (rslt == SMI230_OK)
            {
                /*delay between power ctrl and power config*/
                dev->delay_ms(SMI230_POWER_CONFIG_DELAY);

                /* write to accel power configuration register */
                rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG, &data[1], 1, dev);

                if (rslt == SMI230_OK)
                {
                    /*delay required to switch power modes*/
                    dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
                }
            }

        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data from the sensor,
 * store it in the smi230_sensor_data structure instance
 * passed by the user.
 */
int8_t smi230_acc_get_data(struct smi230_sensor_data *accel, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (accel != NULL))
    {
        /* Read accel sensor data */
        rslt = get_regs(SMI230_ACCEL_X_LSB_REG, data, 6, dev);

        if (rslt == SMI230_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            accel->x = ((int16_t) msblsb); /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            accel->y = ((int16_t) msblsb); /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            accel->z = ((int16_t) msblsb); /* Data in Z axis */
        }

    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary accel interrupt
 * based on the user settings in the smi230_int_cfg
 * structure instance.
 */
int8_t smi230_acc_set_int_config(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (int_config != NULL))
    {
        switch (int_config->int_type)
        {
            case SMI230_ACCEL_DATA_RDY_INT:

                /* Data ready interrupt */
                rslt = set_accel_data_ready_int(int_config, dev);
                break;
            case SMI230_ACCEL_SYNC_DATA_RDY_INT:

                /* synchronized data ready interrupt */
                rslt = set_accel_sync_data_ready_int(int_config, dev);
                break;
            case SMI230_ACCEL_SYNC_INPUT:

                /* input for synchronization on accel */
                rslt = set_accel_sync_input(int_config, dev);
                break;
            case SMI230_ACCEL_ANYMOTION_INT:

                /* Anymotion interrupt */
                rslt = set_accel_anymotion_int(int_config, dev);
                break;
            case SMI230_FIFO_WM_INT:

                /* FIFO watermark interrupt */
                rslt = set_fifo_wm_int(int_config, dev);
                break;
            case SMI230_FIFO_FULL_INT:

                /* FIFO full interrupt */
                rslt = set_fifo_full_int(int_config, dev);
                break;
            default:
                rslt = SMI230_E_INVALID_CONFIG;
                break;
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the temperature of the sensor in degree Celcius.
 */
int8_t smi230_acc_get_sensor_temperature(const struct smi230_dev *dev, int32_t *sensor_temp)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint16_t msb, lsb;
    uint16_t msblsb;
    int16_t temp;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (sensor_temp != NULL))
    {
        /* Read sensor temperature */
        rslt = get_regs(SMI230_TEMP_MSB_REG, data, 2, dev);

        if (rslt == SMI230_OK)
        {
            msb = (data[0] << 3); /* MSB data */
            lsb = (data[1] >> 5); /* LSB data */
            msblsb = (uint16_t) (msb + lsb);

            if (msblsb > 1023)
            {
                /* Updating the msblsb */
                temp = (int16_t) (msblsb - 2048);
            }
            else
            {
                temp = (int16_t) msblsb;
            }

            /* sensor temperature */
            *sensor_temp = (temp * 125) + 23000;
        }

    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief This API reads the sensor time of the accel sensor.
 */
int8_t smi230_acc_get_sensor_time(const struct smi230_dev *dev, uint32_t *sensor_time)
{
    int8_t rslt;
    uint8_t data[3] = { 0 };
    uint32_t byte2, byte1, byte0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (sensor_time != NULL))
    {
        /* Read 3-byte sensor time */
        rslt = get_regs(SMI230_ACCEL_SENSORTIME_0_REG, data, 3, dev);

        if (rslt == SMI230_OK)
        {
            byte0 = data[0]; /* Lower byte */
            byte1 = (data[1] << 8); /* Middle byte */
            byte2 = (data[2] << 16); /* Higher byte */

            /* Sensor time */
            *sensor_time = (byte2 | byte1 | byte0);
        }

    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self test functionality of the sensor
 *  is working or not.
 */
int8_t smi230_acc_perform_selftest(struct smi230_dev *dev)
{
    int8_t rslt;
    int8_t self_test_rslt = 0;
    struct smi230_sensor_data accel_pos, accel_neg;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        /* pre-requisites for self test */
        rslt = enable_self_test(dev);

        if (rslt == SMI230_OK)
        {
            rslt = positive_excited_accel(&accel_pos, dev);

            if (rslt == SMI230_OK)
            {
                rslt = negative_excited_accel(&accel_neg, dev);

                if (rslt == SMI230_OK)
                {
                    /* Validate the self test result */
                    rslt = validate_accel_self_test(&accel_pos, &accel_neg);

                    /* Store the status of self test result */
                    self_test_rslt = rslt;

                    /* Perform soft reset */
                    rslt = smi230_acc_soft_reset(dev);

                    /* Check to ensure bus operations are success */
                    if (rslt == SMI230_OK)
                    {
                        /* Restore self_test_rslt as return value */
                        rslt = self_test_rslt;
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 */
int8_t smi230_acc_set_fifo_config(const struct accel_fifo_config *config, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store the default value of FIFO configuration
     * reserved registers
     */
    uint8_t data_array[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == SMI230_OK)
    {
        /* Get the FIFO configurations from the FIFO configure_1 and configure_2 register */
        rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_0_ADDR, data_array, 2, dev);
        if (rslt == SMI230_OK)
        {
            /* To set the stream mode or FIFO mode */
            data_array[0] = SMI230_SET_BITS_POS_0(data_array[0], SMI230_ACC_FIFO_MODE_CONFIG, config->mode);

            /* To enable the Accel in FIFO configuration */
            data_array[1] = SMI230_SET_BITS(data_array[1], SMI230_ACCEL_EN, config->accel_en);

            /* To enable the interrupt_1 in FIFO configuration */
            data_array[1] = SMI230_SET_BITS(data_array[1], SMI230_ACCEL_INT1_EN, config->int1_en);

            /* To enable the interrupt_2 in FIFO configuration */
            data_array[1] = SMI230_SET_BITS(data_array[1], SMI230_ACCEL_INT2_EN, config->int2_en);

            rslt = smi230_acc_set_regs(SMI230_FIFO_CONFIG_0_ADDR, data_array, 2, dev);
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO configuration from the sensor.
 */
int8_t smi230_acc_get_fifo_config(struct accel_fifo_config *config, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (config != NULL))
    {
        /* Get the FIFO configuration value */
        rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_0_ADDR, data, SMI230_FIFO_CONFIG_LENGTH, dev);
        if (rslt == SMI230_OK)
        {
            /* Get mode selection */
            config->mode = SMI230_GET_BITS_POS_0(data[0], SMI230_ACC_FIFO_MODE_CONFIG);

            /* Get the accel enable */
            config->accel_en = SMI230_GET_BITS(data[1], SMI230_ACCEL_EN);

            /* Get the interrupt_1 enable/disable */
            config->int1_en = SMI230_GET_BITS(data[1], SMI230_ACCEL_INT1_EN);

            /* Get the interrupt_2 enable/disable */
            config->int2_en = SMI230_GET_BITS(data[1], SMI230_ACCEL_INT2_EN);
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the FIFO data.
 */
int8_t smi230_acc_read_fifo_data(struct smi230_fifo_frame *fifo, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store FIFO configuration data */
    uint8_t config_data = 0;

    /* Variable to define FIFO address */
    uint8_t addr = SMI230_FIFO_DATA_ADDR;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (fifo != NULL))
    {
        /* Clear the FIFO data structure */
        reset_fifo_frame_structure(fifo);

        /* Read FIFO data */
        rslt = smi230_acc_get_regs(addr, fifo->data, fifo->length, dev);
        if (rslt == SMI230_OK)
        {
            /* Get the set FIFO frame configurations */
            rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_1_ADDR, &config_data, 1, dev);
            if (rslt == SMI230_OK)
            {
                /* Get sensor enable status, of which the data
                 * is to be read
                 */
                fifo->data_enable = (uint16_t)((uint16_t)config_data & SMI230_ACCEL_EN_MASK);
            }
        }
        else
        {
            rslt = SMI230_E_COM_FAIL;
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 */
int8_t smi230_acc_get_fifo_length(uint16_t *fifo_length, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store FIFO data length */
    uint8_t data[SMI230_FIFO_DATA_LENGTH] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (fifo_length != NULL))
    {
        /* read fifo length */
        rslt = smi230_acc_get_regs(SMI230_FIFO_LENGTH_0_ADDR, data, SMI230_FIFO_DATA_LENGTH, dev);
        if (rslt == SMI230_OK)
        {
            /* Get the MSB byte of FIFO length */
            data[1] = SMI230_GET_BITS_POS_0(data[1], SMI230_FIFO_BYTE_COUNTER_MSB);

            /* Get total FIFO length */
            (*fifo_length) = (uint16_t)((uint16_t)(data[1] << 8) | data[0]);
        }
        else
        {
            rslt = SMI230_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t smi230_acc_get_fifo_wm(uint16_t *wm, const struct smi230_dev *dev)
{
    int8_t rslt;

    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {
        rslt = smi230_acc_get_regs(SMI230_FIFO_WTM_0_ADDR, data, SMI230_FIFO_WTM_LENGTH, dev);
        if ((rslt == SMI230_OK) && (wm != NULL))
        {
            *wm = (data[1] << 8) | (data[0]);
        }
        else
        {
            rslt = SMI230_E_NULL_PTR;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t smi230_acc_set_fifo_wm(uint16_t wm, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store data */
    uint8_t data[2] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {
        /* Get LSB value of FIFO water-mark */
        data[0] = SMI230_GET_LSB(wm);

        /* Get MSB value of FIFO water-mark */
        data[1] = SMI230_GET_MSB(wm);

        /* Set the FIFO water-mark level */
        rslt = smi230_acc_set_regs(SMI230_FIFO_WTM_0_ADDR, data, SMI230_FIFO_WTM_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief This API parses and extracts the accelerometer frames from FIFO data
 * read by the "smi230_read_fifo_data" API and stores it in the "accel_data"
 * structure instance.
 */
int8_t smi230_acc_extract_accel(struct smi230_sensor_data *accel_data,
                            uint16_t *accel_length,
                            struct smi230_fifo_frame *fifo,
                            const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (accel_data != NULL) && (accel_length != NULL) && (fifo != NULL))
    {
        /* Parsing the FIFO data in header mode */
        rslt = extract_acc_header_mode(accel_data, accel_length, fifo);
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the down sampling rates which is configured for
 * accelerometer FIFO data.
 */
int8_t smi230_acc_get_fifo_down_sample(uint8_t *fifo_downs, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (fifo_downs != NULL))
    {
        /* Read the accelerometer FIFO down data sampling rate */
        rslt = smi230_acc_get_regs(SMI230_FIFO_DOWNS_ADDR, &data, 1, dev);
        if (rslt == SMI230_OK)
        {
            (*fifo_downs) = SMI230_GET_BITS(data, SMI230_ACC_FIFO_DOWNS);
        }
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the down-sampling rates for accelerometer
 * FIFO data.
 *
 * @note Reduction of sample rate by a factor 2**fifo_downs
 */
int8_t smi230_acc_set_fifo_down_sample(uint8_t fifo_downs, const struct smi230_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store sampling rate */
    uint8_t data = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {
        /* Set the accelerometer FIFO down sampling rate */
        rslt = smi230_acc_get_regs(SMI230_FIFO_DOWNS_ADDR, &data, 1, dev);
        if (rslt == SMI230_OK)
        {
            data = SMI230_SET_BITS(data, SMI230_ACC_FIFO_DOWNS, fifo_downs);
            rslt = smi230_acc_set_regs(SMI230_FIFO_DOWNS_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct smi230_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = SMI230_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = SMI230_OK;
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint16_t index;
    uint16_t temp_len = len + dev->dummy_byte;
    uint8_t temp_buff[CONFIG_SMI230_MAX_BUFFER_LEN];

    if (WARN(temp_len > CONFIG_SMI230_MAX_BUFFER_LEN, "SMI230 buffer overflow\n")) {
        return SMI230_E_COM_FAIL;
    }

    if (dev->intf == SMI230_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = reg_addr | SMI230_SPI_RD_MASK;
    }

    /* Read the data from the register */
    rslt = dev->read(dev->accel_id, reg_addr, temp_buff, temp_len);

    if (rslt == SMI230_OK)
    {
        for (index = 0; index < len; index++)
        {
            /* Updating the data buffer */
            reg_data[index] = temp_buff[index + dev->dummy_byte];
        }
    }
    else
    {
        /* Failure case */
        rslt = SMI230_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API writes the data to the given register address.
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    if (dev->intf == SMI230_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr & SMI230_SPI_WR_MASK);
    }

    /* write to an accel register */
    rslt = dev->write(dev->accel_id, reg_addr, reg_data, len);

    if (rslt != SMI230_OK)
    {
        /* Updating the error status */
        rslt = SMI230_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = 0, data, is_channel_invalid = FALSE;

    switch (int_config->int_channel)
    {
        case SMI230_INT_CHANNEL_1:

            /* update reg_addr based on channel inputs */
            reg_addr = SMI230_ACCEL_INT1_IO_CONF_REG;
            break;

        case SMI230_INT_CHANNEL_2:

            /* update reg_addr based on channel inputs */
            reg_addr = SMI230_ACCEL_INT2_IO_CONF_REG;
            break;

        default:
            is_channel_invalid = TRUE;
            break;
    }

    if (!is_channel_invalid)
    {
        /* Read interrupt pin configuration register */
        rslt = get_regs(reg_addr, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Update data with user configured smi230_int_cfg structure */
            data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_LVL, int_config->int_pin_cfg.lvl);
            data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_OD, int_config->int_pin_cfg.output_mode);

            if (int_config->int_type == SMI230_ACCEL_SYNC_INPUT)
            {
                data = SMI230_SET_BITS_POS_0(data, SMI230_ACCEL_INT_EDGE, SMI230_ENABLE);
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_IN, int_config->int_pin_cfg.enable_int_pin);
                data = SMI230_SET_BIT_VAL_0(data, SMI230_ACCEL_INT_IO);
            }
            else
            {
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_IO, int_config->int_pin_cfg.enable_int_pin);
                data = SMI230_SET_BIT_VAL_0(data, SMI230_ACCEL_INT_IN);
            }

            /* Write to interrupt pin configuration register */
            rslt = set_regs(reg_addr, &data, 1, dev);
        }
    }
    else
    {
        rslt = SMI230_E_INVALID_INPUT;
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for accel sensor.
 */
static int8_t set_accel_data_ready_int(const struct smi230_accel_int_channel_cfg *int_config,
                                       const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == SMI230_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_1:

                /* Updating the data */
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_DRDY, conf);
                break;

            case SMI230_INT_CHANNEL_2:

                /* Updating the data */
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_DRDY, conf);
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == SMI230_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the synchronized data ready interrupt for accel sensor
 */
static int8_t set_accel_sync_data_ready_int(const struct smi230_accel_int_channel_cfg *int_config,
                                            const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {

        data = SMI230_ACCEL_INTA_DISABLE;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_1:
                reg_addr = SMI230_ACCEL_INT1_MAP_REG;
                break;

            case SMI230_INT_CHANNEL_2:
                reg_addr = SMI230_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
            {
                /*interrupt A mapped to INT1/INT2 */
                data = SMI230_ACCEL_INTA_ENABLE;
            }

            /* Write to interrupt map register */
            rslt = set_regs(reg_addr, &data, 1, dev);

            if (rslt == SMI230_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the given interrupt channel as input for accel sensor
 */
static int8_t set_accel_sync_input(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {
        /*set input interrupt configuration*/
        rslt = set_int_pin_config(int_config, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the anymotion interrupt for accel sensor
 */
static int8_t set_accel_anymotion_int(const struct smi230_accel_int_channel_cfg *int_config,
                                      const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data, reg_addr = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {

        data = SMI230_ACCEL_INTB_DISABLE;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_1:
                reg_addr = SMI230_ACCEL_INT1_MAP_REG;
                break;

            case SMI230_INT_CHANNEL_2:
                reg_addr = SMI230_ACCEL_INT2_MAP_REG;
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
            {
                /*interrupt B mapped to INT1/INT2 */
                data = SMI230_ACCEL_INTB_ENABLE;
            }

            /* Write to interrupt map register */
            rslt = set_regs(reg_addr, &data, 1, dev);

            if (rslt == SMI230_OK)
            {
                /*set input interrupt configuration*/
                rslt = set_int_pin_config(int_config, dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API writes the config stream data in memory using burst mode.
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
    uint8_t asic_lsb = ((index / 2) & 0x0F);

    /* Write to feature config register */
    rslt = set_regs(SMI230_ACCEL_RESERVED_5B_REG, &asic_lsb, 1, dev);
    if (rslt == SMI230_OK)
    {
        /* Write to feature config register */
        rslt = set_regs(SMI230_ACCEL_RESERVED_5C_REG, &asic_msb, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Write to feature config registers */
            rslt = set_regs(SMI230_ACCEL_FEATURE_CFG_REG, (uint8_t *)stream_data, dev->read_write_len, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API performs the pre-requisites needed to perform the self test
 */
static int8_t enable_self_test(struct smi230_dev *dev)
{
    int8_t rslt;

    /* Configuring sensors to perform accel self test */
    dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
    dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;

    /*check the chip id of the accel variant and assign the range */
    dev->accel_cfg.range = SMI230_ACCEL_RANGE_16G;

    dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;

    /* Enable Accel sensor */
    rslt = smi230_acc_set_power_mode(dev);
    if (rslt == SMI230_OK)
    {
        /* Configure sensors with above configured settings */
        rslt = smi230_acc_set_meas_conf(dev);

        if (rslt == SMI230_OK)
        {
            /* Self test delay */
            dev->delay_ms(SMI230_SELF_TEST_DELAY_MS);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data with the positive excitation
 */
static int8_t positive_excited_accel(struct smi230_sensor_data *accel_pos, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = SMI230_ACCEL_POSITIVE_SELF_TEST;

    /* Enable positive excitation for all 3 axes */
    rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
    if (rslt == SMI230_OK)
    {
        /* Read accel data after 50ms delay */
        dev->delay_ms(SMI230_SELF_TEST_DATA_READ_MS);
        rslt = smi230_acc_get_data(accel_pos, dev);
    }

    return rslt;
}

/*!
 * @brief This API reads the accel data with the negative excitation
 */
static int8_t negative_excited_accel(struct smi230_sensor_data *accel_neg, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = SMI230_ACCEL_NEGATIVE_SELF_TEST;

    /* Enable negative excitation for all 3 axes */
    rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
    if (rslt == SMI230_OK)
    {
        /* Read accel data after 50ms delay */
        dev->delay_ms(SMI230_SELF_TEST_DATA_READ_MS);
        rslt = smi230_acc_get_data(accel_neg, dev);

        if (rslt == SMI230_OK)
        {
            /* Disable self test */
            reg_data = SMI230_ACCEL_SWITCH_OFF_SELF_TEST;
            rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &reg_data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API validates the self test results
 */
static int8_t validate_accel_self_test(const struct smi230_sensor_data *accel_pos,
                                       const struct smi230_sensor_data *accel_neg)
{
    int8_t rslt;

    /*! Structure for difference of accel values in g */
    struct selftest_delta_limit accel_data_diff = { 0 };

    /*! Structure for difference of accel values in mg */
    struct selftest_delta_limit accel_data_diff_mg = { 0 };

    accel_data_diff.x = (uint16_t) (SMI230_ABS(accel_pos->x) + SMI230_ABS(accel_neg->x));
    accel_data_diff.y = (uint16_t) (SMI230_ABS(accel_pos->y) + SMI230_ABS(accel_neg->y));
    accel_data_diff.z = (uint16_t) (SMI230_ABS(accel_pos->z) + SMI230_ABS(accel_neg->z));

    /*! Converting LSB of the differences of
     * accel values to mg */
    convert_lsb_g(&accel_data_diff, &accel_data_diff_mg);

    /* Validating accel data by comparing with minimum value of the axes in mg */
    /* x axis limit 1000mg, y axis limit 1000mg and z axis limit 500mg */
    if (accel_data_diff_mg.x >= 1000 && accel_data_diff_mg.y >= 1000 && accel_data_diff_mg.z >= 500)
    {
        /* Updating Okay status */
        rslt = SMI230_OK;
    }
    else
    {
        /* Updating Error status */
        rslt = SMI230_W_SELF_TEST_FAIL;
    }

    return rslt;
}

/*!
 *  @brief This API converts lsb value of axes to mg for self-test.
 */
static void convert_lsb_g(const struct selftest_delta_limit *accel_data_diff,
                          struct selftest_delta_limit *accel_data_diff_mg)
{
    /* accel x value in mg */
    accel_data_diff_mg->x = (uint16_t) ((accel_data_diff->x / (int32_t)LSB_PER_G) * 1000);

    /* accel y value in mg */
    accel_data_diff_mg->y = (uint16_t) ((accel_data_diff->y / (int32_t)LSB_PER_G) * 1000);

    /* accel z value in mg */
    accel_data_diff_mg->z = (uint16_t) ((accel_data_diff->z / (int32_t)LSB_PER_G) * 1000);
}

/*!
 * @brief This internal API is used to parse and store the skipped frame count
 * from the FIFO data.
 */
static int8_t unpack_skipped_frame(uint16_t *data_index, struct smi230_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = SMI230_OK;

    /* Validate data index */
    if ((*data_index) >= fifo->length)
    {
        /* Update the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = SMI230_W_FIFO_EMPTY;
    }
    else
    {
        /* Update skipped frame count in the FIFO structure */
        fifo->skipped_frame_count = fifo->data[(*data_index)];

        /* Move the data index by 1 byte */
        (*data_index) = (*data_index) + 1;

        /* More frames could be read */
        rslt = SMI230_W_PARTIAL_READ;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to reset the FIFO related configurations in
 * the FIFO frame structure for the next FIFO read.
 */
static void reset_fifo_frame_structure(struct smi230_fifo_frame *fifo)
{
    /* Reset FIFO data structure */
    fifo->acc_byte_start_idx = 0;
    fifo->sensor_time = 0;
    fifo->skipped_frame_count = 0;
}

/*!
 * @brief This internal API is used to parse accelerometer data from the
 * FIFO data.
 */
static void unpack_accel_data(struct smi230_sensor_data *acc,
                              uint16_t data_start_index,
                              const struct smi230_fifo_frame *fifo)
{
    /* Variables to store LSB value */
    uint16_t data_lsb;

    /* Variables to store MSB value */
    uint16_t data_msb;

    /* Accelerometer raw x data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw y data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accelerometer raw z data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->z = (int16_t)((data_msb << 8) | data_lsb);

}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO data in header mode. It updates the current data
 * byte to be parsed.
 */
static int8_t unpack_accel_frame(struct smi230_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct smi230_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = SMI230_OK;

    switch (frame)
    {
        /* If frame contains only accelerometer data */
        case SMI230_FIFO_HEADER_ACC_FRM:

            /* Partially read, then skip the data */
            if (((*idx) + fifo->acc_frm_len) > fifo->length)
            {
                /* Update the data index as complete*/
                (*idx) = fifo->length;

                /* FIFO is empty */
                rslt = SMI230_W_FIFO_EMPTY;
                break;
            }

            /* Get the accelerometer data */
            unpack_accel_data(&acc[(*acc_idx)], *idx, fifo);

            /* Update data index */
            (*idx) = (*idx) + SMI230_FIFO_ACCEL_LENGTH;

            /* Update accelerometer frame index */
            (*acc_idx)++;

            break;
        default:

            /* Move the data index to the last byte in case of invalid values */
            (*idx) = fifo->length;

            /* FIFO is empty */
            rslt = SMI230_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to move the data index ahead of the
 * current_frame_length parameter when unnecessary FIFO data appears while
 * extracting the user specified data.
 */
static int8_t move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct smi230_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = SMI230_OK;

    /* Validate data index */
    if (((*data_index) + current_frame_length) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = SMI230_W_FIFO_EMPTY;
    }
    else
    {
        /* Move the data index to next frame */
        (*data_index) = (*data_index) + current_frame_length;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse and store the sensor time from the
 * FIFO data.
 */
static int8_t unpack_sensortime_frame(uint16_t *data_index, struct smi230_fifo_frame *fifo)
{
    /* Variables to define error */
    int8_t rslt = SMI230_OK;

    /* Variables to define 3 bytes of sensor time */
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /* Validate data index */
    if (((*data_index) + SMI230_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /* Move the data index to the last byte */
        (*data_index) = fifo->length;

        /* FIFO is empty */
        rslt = SMI230_W_FIFO_EMPTY;
    }
    else
    {
        /* Get sensor time from the FIFO data */
        sensor_time_byte3 = fifo->data[(*data_index) + SMI230_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + SMI230_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        /* Update sensor time in the FIFO structure */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);

        /* Move the data index by 3 bytes */
        (*data_index) = (*data_index) + SMI230_SENSOR_TIME_LENGTH;

    }

    return rslt;
}

/*!
 * @brief This internal API is used to parse the accelerometer data from the
 * FIFO in header mode.
 */
static int8_t extract_acc_header_mode(struct smi230_sensor_data *acc,
                                      uint16_t *accel_length,
                                      struct smi230_fifo_frame *fifo)
{
    /* Variable to define error */
    int8_t rslt = SMI230_OK;

    /* Variable to define header frame */
    uint8_t frame_header = 0;

    /* Variable to index the data bytes */
    uint16_t data_index;

    /* Variable to index accelerometer frames */
    uint16_t accel_index = 0;

    /* Variable to indicate accelerometer frames read */
    uint16_t frame_to_read = *accel_length;

    for (data_index = fifo->acc_byte_start_idx; data_index < fifo->length;)
    {
        /* Get frame header byte */
        frame_header = fifo->data[data_index];

        /* Index shifted to next byte where data starts */
        data_index++;
        switch (frame_header)
        {
            /* If header defines accelerometer frame */
            case SMI230_FIFO_HEADER_ACC_FRM:
            case SMI230_FIFO_HEADER_ALL_FRM:

                /* Unpack from normal frames */
                rslt = unpack_accel_frame(acc, &data_index, &accel_index, frame_header, fifo);
                break;

            /* If header defines sensor time frame */
            case SMI230_FIFO_HEADER_SENS_TIME_FRM:
                rslt = unpack_sensortime_frame(&data_index, fifo);
                break;

            /* If header defines skip frame */
            case SMI230_FIFO_HEADER_SKIP_FRM:
                rslt = unpack_skipped_frame(&data_index, fifo);
                break;

            /* If header defines Input configuration frame */
            case SMI230_FIFO_HEADER_INPUT_CFG_FRM:
                rslt = move_next_frame(&data_index, SMI230_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines sample drop frame */
            case SMI230_FIFO_SAMPLE_DROP_FRM:
                rslt = move_next_frame(&data_index, SMI230_FIFO_INPUT_CFG_LENGTH, fifo);
                break;

            /* If header defines invalid frame or end of valid data */
            case SMI230_FIFO_HEAD_OVER_READ_MSB:

                /* Move the data index to the last byte to mark completion */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = SMI230_W_FIFO_EMPTY;
                break;
            default:

                /* Move the data index to the last byte in case of invalid values */
                data_index = fifo->length;

                /* FIFO is empty */
                rslt = SMI230_W_FIFO_EMPTY;
                break;
        }

        /* Break if Number of frames to be read is complete or FIFO is mpty */
        if ((frame_to_read == accel_index) || (rslt == SMI230_W_FIFO_EMPTY))
        {
            break;
        }
    }

    /* Update the accelerometer frame index */
    (*accel_length) = accel_index;

    /* Update the accelerometer byte index */
    fifo->acc_byte_start_idx = data_index;

    return rslt;
}

/*!
 * @brief This API sets the FIFO water mark interrupt for accel sensor.
 */
static int8_t set_fifo_wm_int(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == SMI230_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_1:

                /* Updating the data */
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_FWM, conf);
                break;

            case SMI230_INT_CHANNEL_2:

                /* Updating the data */
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_FWM, conf);
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == SMI230_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the FIFO full interrupt for accel sensor.
 */
static int8_t set_fifo_full_int(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, conf;

    /* Read interrupt map register */
    rslt = get_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == SMI230_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_1:

                /* Updating the data */
                data = SMI230_SET_BITS_POS_0(data, SMI230_ACCEL_INT1_FFULL, conf);
                break;

            case SMI230_INT_CHANNEL_2:

                /* Updating the data */
                data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_FFULL, conf);
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            /* Configure interrupt pins */
            rslt = set_int_pin_config(int_config, dev);

            if (rslt == SMI230_OK)
            {
                /* Write to interrupt map register */
                rslt = set_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);
            }
        }
    }

    return rslt;
}

/** @}*/
