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

/*! \file smi230.h
 * \brief Sensor Driver for SMI230 sensors */
#ifndef _SMI230_H
#define _SMI230_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* header files */
#include "smi230_defs.h"

/*********************************************************************/
/* (extern) variable declarations */
/*********************************************************************/
/* function prototype declarations */
/*********************** SMI230 Accelerometer function prototypes ************************/

/*!
 *  @brief This API is the entry point for accel sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel sensor.
 *
 *  @param[in,out] dev  : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success /-ve value -> Error
 */
int8_t smi230_acc_init(struct smi230_dev *dev);

/*!
 *  @brief This API is used to write the binary configuration in the sensor
 *
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_write_config_file(const struct smi230_dev *dev);

/*!
 *  @brief This API writes the feature configuration to the accel sensor.
 *
 *  @param[in] reg_addr : Address offset of the feature setting inside the feature conf region.
 *  @param[in] reg_data : Feature settings.
 *  @param[in] len : Number of 16 bit words to be written.
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_write_feature_config(uint8_t reg_addr, const uint16_t *reg_data, uint8_t len,
                                   const struct smi230_dev *dev);

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
int8_t smi230_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev);

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
int8_t smi230_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev);

/*!
 *  @brief This API reads the error status from the accel sensor.
 *
 *  Below table mention the types of error which can occur in the sensor
 *@verbatim
 *************************************************************************
 *        Error           |       Description
 *************************|***********************************************
 *                        |       Fatal Error, chip is not in operational
 *        fatal           |       state (Boot-, power-system).
 *                        |       This flag will be reset only by
 *                        |       power-on-reset or soft reset.
 *************************|***********************************************
 *                        |       Value        Name       Description
 *        error_code      |       000        no_error     no error
 *                        |       001        accel_err      error in
 *                        |                               ACCEL_CONF
 *************************************************************************
 *@endverbatim
 *  @param[out] err_reg : Pointer to structure variable which stores the
 *  error status read from the sensor.
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_get_error_status(struct smi230_err_reg *err_reg, const struct smi230_dev *dev);

/*!
 *  @brief This API reads the status of the accel sensor.
 *
 *  Below table lists the sensor status flags
 *@verbatim
 *************************************************************************
 *        Status                    |       Description
 ***********************************|*************************************
 *        drdy_accel                | Data ready for Accel.
 *************************************************************************
 *@endverbatim
 *
 *  @param[out] status : Variable used to store the sensor status flags
 *  which is read from the sensor.
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t smi230_acc_get_status(uint8_t *status, const struct smi230_dev *dev);

/*!
 *  @brief This API resets the accel sensor.
 *
 *  @param[in] dev  : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_soft_reset(const struct smi230_dev *dev);

/*!
 * @brief This API reads the accel config values ie odr, band width and range from the sensor,
 * store it in the smi230_dev structure instance
 * passed by the user.
 *  @param[in,out]  dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t smi230_acc_get_meas_conf(struct smi230_dev *dev);

/*!
 *  @brief This API sets the Output data rate, range and bandwidth
 *  of accel sensor.
 *  @param[in] dev  : Structure instance of smi230_dev.
 *
 *  @note : The user must select one among the following macros to
 *  select range value for SMI230 accel
 *      config                         |   value
 *      -------------------------------|---------------------------
 *      SMI230_ACCEL_RANGE_2G          |   0x00
 *      SMI230_ACCEL_RANGE_4G          |   0x01
 *      SMI230_ACCEL_RANGE_8G          |   0x02
 *      SMI230_ACCEL_RANGE_16G         |   0x03
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_set_meas_conf(const struct smi230_dev *dev);

/*!
 * @brief This API reads the accel power mode from the sensor,
 * store it in the smi230_dev structure instance
 * passed by the user.
 *  @param[in,out]  dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t smi230_acc_get_power_mode(struct smi230_dev *dev);

/*!
 *  @brief This API sets the power mode of the accel sensor.
 *
 *  @param[in] dev  : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_set_power_mode(const struct smi230_dev *dev);

/*!
 *  @brief This API reads the accel data from the sensor,
 *  store it in the smi230_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[in]  dev    : Structure instance of smi230_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval zero -> Success /-ve value -> Error
 */
int8_t smi230_acc_get_data(struct smi230_sensor_data *accel, const struct smi230_dev *dev);

/*!
 *  @brief This API configures the necessary accel interrupt
 *  based on the user settings in the smi230_accel_int_channel_cfg
 *  structure instance.
 *
 *  @param[in] int_config  : Structure instance of smi230_accel_int_channel_cfg.
 *  @param[in] dev         : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_set_int_config(const struct smi230_accel_int_channel_cfg *int_config, const struct smi230_dev *dev);

/*!
 *  @brief This API reads the temperature of the sensor in degree Celcius.
 *
 *  @param[in]  dev             : Structure instance of smi230_dev.
 *  @param[out] sensor_temp     : Pointer to store sensor temperature in degree Celcius
 *
 *  @note Temperature data output must be divided by a factor of 1000
 *
 *  Consider sensor_temp = 19520 , Then the actual temperature is 19.520 degree Celsius
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_get_sensor_temperature(const struct smi230_dev *dev, int32_t *sensor_temp);

/*!
 *  @brief This API reads the sensor time of the accel sensor.
 *
 *  @param[in]  dev             : Structure instance of smi230_dev.
 *  @param[out] sensor_time     : Pointer to store sensor time
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_acc_get_sensor_time(const struct smi230_dev *dev, uint32_t *sensor_time);

/*!
 *  @brief This API checks whether the self test functionality of the sensor
 *  is working or not
 *
 *  @param[in] dev    : Structure instance of smi230_dev
 *
 *  @return results of self test
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 *
 *   Return value                  |   Result of self test
 * --------------------------------|---------------------------------
 *  SMI230_OK                      |  Self test success
 *  SMI230_W_SELF_TEST_FAIL        |  self test fail
 */
int8_t smi230_acc_perform_selftest(struct smi230_dev *dev);

/*********************** SMI230 Gyroscope function prototypes ****************************/

/*!
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 *
 *  @param[in,out] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_gyro_init(struct smi230_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out] reg_data : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_gyro_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev);

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
int8_t smi230_gyro_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev);

/*!
 *  @brief This API resets the gyro sensor.
 *
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 */
int8_t smi230_gyro_soft_reset(const struct smi230_dev *dev);

/*!
 *  @brief This API reads the gyro odr and range from the sensor,
 *  store it in the smi230_dev structure instance
 *  passed by the user.
 *
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 *  @note : band width also updated, which is same as odr
 */
int8_t smi230_gyro_get_meas_conf(struct smi230_dev *dev);

/*!
 *  @brief This API sets the output data rate, range and bandwidth
 *  of gyro sensor.
 *
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 *
 *  @note : No need to give the band width parameter,
 *          odr will update the band width.
 */
int8_t smi230_gyro_set_meas_conf(const struct smi230_dev *dev);

/*!
 *  @brief This API gets the power mode of the gyro sensor and store it
 *  inside the instance of smi230_dev passed by the user.
 *
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_gyro_get_power_mode(struct smi230_dev *dev);

/*!
 *  @brief This API sets the power mode of the gyro sensor.
 *
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_gyro_set_power_mode(const struct smi230_dev *dev);

/*!
 *  @brief This API reads the gyro data from the sensor,
 *  store it in the smi230_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] gyro   : Structure pointer to store gyro data
 *  @param[in] dev     : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_gyro_get_data(struct smi230_sensor_data *gyro, const struct smi230_dev *dev);

/*!
 *  @brief This API configures the necessary gyro interrupt
 *  based on the user settings in the smi230_gyro_int_channel_cfg
 *  structure instance.
 *
 *  @param[in] int_config  : Structure instance of smi230_gyro_int_channel_cfg.
 *  @param[in] dev         : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_gyro_set_int_config(const struct smi230_gyro_int_channel_cfg *int_config, const struct smi230_dev *dev);

/*!
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not
 *
 *  @param[in]  dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 *
 *   Return value                  |   Result of self test
 * --------------------------------|---------------------------------
 *  SMI230_OK                      |  Self test success
 *  SMI230_W_SELF_TEST_FAIL        |  self test fail
 */
int8_t smi230_gyro_perform_selftest(const struct smi230_dev *dev);

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 *
 * @param[in] config        : Structure instance of FIFO configurations.
 * @param[in] dev           : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */
int8_t smi230_acc_set_fifo_config(const struct accel_fifo_config *config, const struct smi230_dev *dev);

/*!
 * @brief This API gets the FIFO configuration from the sensor.
 *
 * @param[out] config   : Structure instance to get FIFO configuration value.
 * @param[in]  dev      : Structure instance of smi230_dev.
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */
int8_t smi230_acc_get_fifo_config(struct accel_fifo_config *config, const struct smi230_dev *dev);

/*!
 * @brief This API reads FIFO data.
 *
 * @param[in, out] fifo     : Structure instance of smi230_fifo_frame.
 * @param[in]      dev      : Structure instance of smi230_dev.
 *
 * @note APS has to be disabled before calling this function.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */
int8_t smi230_acc_read_fifo_data(struct smi230_fifo_frame *fifo, const struct smi230_dev *dev);

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 *
 * @param[out] fifo_length  : Pointer variable to store the value of FIFO byte
 *                            counter.
 * @param[in]  dev          : Structure instance of smi230_dev.
 *
 * @note The byte counter is updated each time a complete frame is read or
 * written.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */
int8_t smi230_acc_get_fifo_length(uint16_t *fifo_length, const struct smi230_dev *dev);

/*!
 * @brief This API gets the FIFO water mark level which is set in the sensor.
 *
 * @param[out] wm        : Pointer variable to store FIFO water-mark level.
 * @param[in]  dev            : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */
int8_t smi230_acc_get_fifo_wm(uint16_t *wm, const struct smi230_dev *dev);

/*!
 * @brief This API sets the FIFO water mark level which is set in the sensor.
 *
 * @param[out] wm        : Pointer variable to store FIFO water-mark level.
 * @param[in]  dev            : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */
int8_t smi230_acc_set_fifo_wm(uint16_t wm, const struct smi230_dev *dev);

/*!
 * This API parses and extracts the accelerometer frames from FIFO data read by
 * the "smi230_read_fifo_data" API and stores it in the "accel_data" structure
 * instance.
 *
 * @param[out]    accel_data   : Structure instance of smi230_sensor_data
 *                               where the parsed data bytes are stored.
 * @param[in,out] accel_length : Number of accelerometer frames.
 * @param[in,out] fifo         : Structure instance of smi230_fifo_frame.
 * @param[in]     dev          : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_W_FIFO_EMPTY - Warning : FIFO is empty
 * @retval SMI230_W_PARTIAL_READ - Warning : There are more frames to be read
 */
int8_t smi230_acc_extract_accel(struct smi230_sensor_data *accel_data,
                            uint16_t *accel_length,
                            struct smi230_fifo_frame *fifo,
                            const struct smi230_dev *dev);

/*!
 * @brief This API gets the down sampling rate, configured for FIFO
 * accelerometer.
 *
 * @param[out] fifo_downs : Pointer variable to store the down sampling rate
 * @param[in]  dev            : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 */

int8_t smi230_acc_get_fifo_down_sample(uint8_t *fifo_downs, const struct smi230_dev *dev);

/*!
 * @brief This API sets the down sampling rate for FIFO accelerometer FIFO data.
 *
 * @param[in] fifo_downs : Variable to set the down sampling rate.
 * @param[in] dev            : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 *
 * @retval SMI230_OK - Success.
 * @retval SMI230_E_NULL_PTR - Error: Null pointer error
 * @retval SMI230_E_COM_FAIL - Error: Communication fail
 * @retval SMI230_E_OUT_OF_RANGE - Error: Out of range
 */
int8_t smi230_acc_set_fifo_down_sample(uint8_t fifo_downs, const struct smi230_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* _SMI230_H */

/** @}*/
