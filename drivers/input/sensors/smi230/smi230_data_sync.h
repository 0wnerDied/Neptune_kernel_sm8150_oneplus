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

/*! \file smi230_data_sync.h
 * \brief Sensor Driver for SMI230 sensors */

#ifndef _SMI230_DATA_SYNC_H
#define _SMI230_DATA_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
/* header files */
#include "smi230_defs.h"

/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/

/**********************************************************************************/
/* function prototype declarations */

/*!
 *  @brief This API is the entry point for smi230 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 *
 *  @param[in,out] dev  : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_init(struct smi230_dev *dev);

/*!
 *  @brief This API uploads the smi230 config file onto the device.
 *
 *  @param[in,out] dev  : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_apply_config_file(struct smi230_dev *dev);

/*!
 *  @brief This API is used to enable/disable the data synchronization
 *  feature.
 *
 *  @param[in] sync_cfg : configure sync feature
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_configure_data_synchronization(struct smi230_data_sync_cfg sync_cfg, struct smi230_dev *dev);

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 *
 *  @param[in] anymotion_cfg : configure anymotion feature
 *  @param[in] dev : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_configure_anymotion(struct smi230_anymotion_cfg anymotion_cfg, const struct smi230_dev *dev);

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the smi230_sensor_data structure instance
 *  passed by the user.
 *
 *  @param[out] accel  : Structure pointer to store accel data
 *  @param[out] gyro   : Structure pointer to store gyro  data
 *  @param[in]  dev    : Structure instance of smi230_dev.
 *
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_get_synchronized_data(struct smi230_sensor_data *accel,
                                    struct smi230_sensor_data *gyro,
                                    const struct smi230_dev *dev);

/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the smi230_int_cfg
 *  structure instance.
 *
 *  @param[in] int_config : Structure instance of accel smi230_int_cfg.
 *  @param[in] dev         : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
int8_t smi230_set_data_sync_int_config(const struct smi230_int_cfg *int_config, const struct smi230_dev *dev);

#ifdef __cplusplus
}
#endif

#endif

/** @}*/
