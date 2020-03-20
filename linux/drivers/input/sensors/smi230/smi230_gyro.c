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

/*! \file smi230_gyro.c
 * \brief Sensor Driver for SMI230 sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "smi230.h"

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

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
 *  @brief This API reads the data from the given register address of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out]reg_data  : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct smi230_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of gyro sensor.
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
 * @brief This API sets the data ready interrupt for gyro sensor.
 *
 * @param[in] int_config  : Structure instance of smi230_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_gyro_data_ready_int(const struct smi230_gyro_int_channel_cfg *int_config,
                                      const struct smi230_dev *dev);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of smi230_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_int_pin_config(const struct smi230_gyro_int_channel_cfg *int_config, const struct smi230_dev *dev);

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 *
 *  @param[in] selftest : Variable used to enable or disable
 *  the Gyro self test feature
 *  Value   |  Description
 *  --------|---------------
 *  0x00    | SMI230_DISABLE
 *  0x01    | SMI230_ENABLE
 *
 *  @param[in] dev : Structure instance of smi230_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 *
 */
static int8_t set_gyro_selftest(uint8_t selftest, const struct smi230_dev *dev);

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
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 */
int8_t smi230_gyro_init(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        /* Read gyro chip id */
        rslt = get_regs(SMI230_GYRO_CHIP_ID_REG, &chip_id, 1, dev);

        if (rslt == SMI230_OK)
        {
            if (chip_id == SMI230_GYRO_CHIP_ID)
            {
                /* Store the chip ID in dev structure */
                dev->gyro_chip_id = chip_id;
            }
            else
            {
                rslt = SMI230_E_DEV_NOT_FOUND;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of gyro sensor.
 */
int8_t smi230_gyro_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
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
 * @brief This API writes the given data to the register address
 * of gyro sensor.
 */
int8_t smi230_gyro_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
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
 * @brief This API resets the gyro sensor.
 */
int8_t smi230_gyro_soft_reset(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        /* Reset gyro device */
        data = SMI230_SOFT_RESET_CMD;
        rslt = set_regs(SMI230_GYRO_SOFTRESET_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* delay 30 ms after writing reset value to its register */
            dev->delay_ms(SMI230_GYRO_SOFTRESET_DELAY);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro odr and range from the sensor, store it in the smi230_dev
 * structure instance passed by the user.
 */
int8_t smi230_gyro_get_meas_conf(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_GYRO_RANGE_REG, data, 2, dev);

        if (rslt == SMI230_OK)
        {
            dev->gyro_cfg.range = data[0];
            dev->gyro_cfg.odr = (data[1] & SMI230_GYRO_BW_MASK);
            dev->gyro_cfg.bw = dev->gyro_cfg.odr;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of gyro sensor.
 */
int8_t smi230_gyro_set_meas_conf(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;
    uint8_t odr, range;
    uint8_t is_range_invalid = FALSE, is_odr_invalid = FALSE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        odr = dev->gyro_cfg.odr;
        range = dev->gyro_cfg.range;

        if (odr > SMI230_GYRO_BW_32_ODR_100_HZ)
        {
            /* Updating the status */
            is_odr_invalid = TRUE;
        }

        if (range > SMI230_GYRO_RANGE_125_DPS)
        {
            /* Updating the status */
            is_range_invalid = TRUE;
        }

        /* If ODR and Range is valid, write it to gyro config. registers */
        if ((!is_odr_invalid) && (!is_range_invalid))
        {
            /* Read range value from the range register */
            rslt = get_regs(SMI230_GYRO_BANDWIDTH_REG, &data, 1, dev);

            if (rslt == SMI230_OK)
            {
                data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_BW, odr);

                /* Write odr value to odr register */
                rslt = set_regs(SMI230_GYRO_BANDWIDTH_REG, &data, 1, dev);

                if (rslt == SMI230_OK)
                {
                    /* Read range value from the range register */
                    rslt = get_regs(SMI230_GYRO_RANGE_REG, &data, 1, dev);

                    if (rslt == SMI230_OK)
                    {
                        data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_RANGE, range);

                        /* Write range value to range register */
                        rslt = set_regs(SMI230_GYRO_RANGE_REG, &data, 1, dev);
                    }
                }
            }

        }
        else
        {
            /* Invalid configuration present in ODR, Range */
            rslt = SMI230_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro power mode from the sensor,
 * store it in the smi230_dev structure instance
 * passed by the user.
 *
 */
int8_t smi230_gyro_get_power_mode(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_GYRO_LPM1_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Updating the power mode in the dev structure */
            dev->gyro_cfg.power = data;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the gyro sensor.
 */
int8_t smi230_gyro_set_power_mode(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode, data;
    uint8_t is_power_switching_mode_valid = TRUE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        /*read the previous power state*/
        rslt = get_regs(SMI230_GYRO_LPM1_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            power_mode = dev->gyro_cfg.power;

            /*switching between normal mode and the suspend modes is allowed, it is not possible to switch
             * between suspend and deep suspend and vice versa. Check for invalid power switching (i.e)
             * deep suspend to suspend */
            if ((power_mode == SMI230_GYRO_PM_SUSPEND) && (data == SMI230_GYRO_PM_DEEP_SUSPEND))
            {
                /* Updating the status */
                is_power_switching_mode_valid = FALSE;
            }

            /* Check for invalid power switching (i.e) from suspend to deep suspend */
            if ((power_mode == SMI230_GYRO_PM_DEEP_SUSPEND) && (data == SMI230_GYRO_PM_SUSPEND))
            {
                /* Updating the status */
                is_power_switching_mode_valid = FALSE;
            }

            /* Check if power switching mode is valid*/
            if (is_power_switching_mode_valid)
            {
                /* Write power to power register */
                rslt = set_regs(SMI230_GYRO_LPM1_REG, &power_mode, 1, dev);

                if (rslt == SMI230_OK)
                {
                    /* Time required to switch the power mode */
                    dev->delay_ms(SMI230_GYRO_POWER_MODE_CONFIG_DELAY);
                }

            }
            else
            {
                /* Updating the error */
                rslt = SMI230_E_INVALID_INPUT;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro data from the sensor,
 * store it in the smi230_sensor_data structure instance
 * passed by the user.
 */
int8_t smi230_gyro_get_data(struct smi230_sensor_data *gyro, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (gyro != NULL))
    {
        /* read gyro sensor data */
        rslt = get_regs(SMI230_GYRO_X_LSB_REG, data, 6, dev);

        if (rslt == SMI230_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            gyro->x = (int16_t)msblsb; /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            gyro->y = (int16_t)msblsb; /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            gyro->z = (int16_t)msblsb; /* Data in Z axis */
        }

    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary gyro interrupt
 * based on the user settings in the smi230_int_cfg
 * structure instance.
 */
int8_t smi230_gyro_set_int_config(const struct smi230_gyro_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (int_config != NULL))
    {

        switch (int_config->int_type)
        {
            case SMI230_GYRO_DATA_RDY_INT:
            {
                /* Data ready interrupt */
                rslt = set_gyro_data_ready_int(int_config, dev);
            }
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
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not.
 */
int8_t smi230_gyro_perform_selftest(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, loop_break = 1;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        /* Enable the gyro self-test */
        rslt = set_gyro_selftest(SMI230_ENABLE, dev);

        if (rslt == SMI230_OK)
        {
            /* Loop till self-test ready bit is set */
            while (loop_break)
            {
                /* Read self-test register to check if self-test ready bit is set */
                rslt = get_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, dev);

                if (rslt == SMI230_OK)
                {
                    data = SMI230_GET_BITS(data, SMI230_GYRO_SELF_TEST_RDY);

                    if (data)
                    {
                        /* If self-test ready bit is set, exit the loop */
                        loop_break = 0;
                    }

                }
                else
                {
                    /* Exit the loop in case of communication failure */
                    loop_break = 0;
                }
            }

            if (rslt == SMI230_OK)
            {
                /* Read self-test register to check for self-test Ok bit */
                rslt = get_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, dev);

                if (rslt == SMI230_OK)
                {
                    data = SMI230_GET_BITS(data, SMI230_GYRO_SELF_TEST_RESULT);

                    rslt = smi230_gyro_soft_reset(dev);

                    if (rslt == SMI230_OK)
                    {
                        /* Updating the self test result */
                        rslt = (int8_t) data;
                    }
                }
            }
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
 * @brief This API reads the data from the given register address of gyro sensor.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    if (dev->intf == SMI230_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr | SMI230_SPI_RD_MASK);
    }

    /* read a gyro register */
    rslt = dev->read(dev->gyro_id, reg_addr, reg_data, len);

    if (rslt != SMI230_OK)
    {
        /* Updating the error */
        rslt = SMI230_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address of gyro sensor.
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    if (dev->intf == SMI230_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr & SMI230_SPI_WR_MASK);
    }

    /* write to a gyro register */
    rslt = dev->write(dev->gyro_id, reg_addr, reg_data, len);

    if (rslt != SMI230_OK)
    {
        /* Updating the error */
        rslt = SMI230_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 */
static int8_t set_gyro_data_ready_int(const struct smi230_gyro_int_channel_cfg *int_config,
                                      const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data[2] = { 0 };

    /* read interrupt map register */
    rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);

    if (rslt == SMI230_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_3:

                /* Data to enable new data ready interrupt */
                data[0] = SMI230_SET_BITS_POS_0(data[0], SMI230_GYRO_INT3_MAP, conf);
                break;

            case SMI230_INT_CHANNEL_4:

                /* Data to enable new data ready interrupt */
                data[0] = SMI230_SET_BITS(data[0], SMI230_GYRO_INT4_MAP, conf);
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            /*condition to check disabling the interrupt in single channel when both
             * interrupts channels are enabled*/
            if (data[0] & SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4)
            {
                /* Updating the data */
                /* Data to enable new data ready interrupt */
                data[1] = SMI230_GYRO_DRDY_INT_ENABLE_VAL;
            }
            else
            {
                data[1] = SMI230_GYRO_DRDY_INT_DISABLE_VAL;
            }

            /* write data to interrupt map register */
            rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);

            if (rslt == SMI230_OK)
            {
                /* Configure interrupt pin */
                rslt = set_int_pin_config(int_config, dev);

                if (rslt == SMI230_OK)
                {
                    /* write data to interrupt control register */
                    rslt = set_regs(SMI230_GYRO_INT_CTRL_REG, &data[1], 1, dev);
                }

            }
        }

    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct smi230_gyro_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Read interrupt configuration register */
    rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1, dev);

    if (rslt == SMI230_OK)
    {
        switch (int_config->int_channel)
        {
            /* Interrupt pin or channel 3 */
            case SMI230_INT_CHANNEL_3:

                /* Update data with user configured smi230_int_cfg structure */
                data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_INT3_LVL, int_config->int_pin_cfg.lvl);
                data = SMI230_SET_BITS(data, SMI230_GYRO_INT3_OD, int_config->int_pin_cfg.output_mode);
                break;

            case SMI230_INT_CHANNEL_4:

                /* Update data with user configured smi230_int_cfg structure */
                data = SMI230_SET_BITS(data, SMI230_GYRO_INT4_LVL, int_config->int_pin_cfg.lvl);
                data = SMI230_SET_BITS(data, SMI230_GYRO_INT4_OD, int_config->int_pin_cfg.output_mode);
                break;

            default:
                break;
        }

        /* write to interrupt configuration register */
        rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1, dev);
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 */
static int8_t set_gyro_selftest(uint8_t selftest, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for valid selftest input */
    if ((selftest == SMI230_ENABLE) || (selftest == SMI230_DISABLE))
    {
        /* Read self test register */
        rslt = get_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
        {
            /* Enable self-test */
            data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_SELF_TEST_EN, selftest);

            /* write self test input value to self-test register */
            rslt = set_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, dev);
        }

    }
    else
    {
        rslt = SMI230_E_INVALID_INPUT;
    }

    return rslt;
}

/** @}*/
