// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
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
 
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#include "smi230_driver.h"
#define MODULE_TAG MODULE_NAME
#include "smi230_log.h"
#include "smi230.h"

/* #define SMI230_DEBUG 1 */
#define SMI230_MAX_BUFFER_SIZE      32

static struct spi_device *smi230_acc_device;
static struct spi_device *smi230_gyro_device;

static struct smi230_dev smi230_spi_dev;

static int8_t smi230_spi_write(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	struct spi_message msg;
	uint8_t buffer[SMI230_MAX_BUFFER_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf = buffer,
		.len = len + 1,
	};

	if (len > SMI230_MAX_BUFFER_SIZE)
		return -EINVAL;

	buffer[0] = reg_addr;
	memcpy(&buffer[1], data, len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	if (dev_addr == SMI230_ACCEL_CHIP_ID)
		return spi_sync(smi230_acc_device, &msg);
	else
		return spi_sync(smi230_gyro_device, &msg);
}

static int8_t smi230_spi_read(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
#ifdef SMI230_DEBUG
	int ret, index;
#endif
	struct spi_message msg;
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg_addr,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		}
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);
#ifdef SMI230_DEBUG
        for (index = 0; index < len; index++)
		PINFO("before spi_sync :%x\n", data[index]);
	if (dev_addr == SMI230_ACCEL_CHIP_ID)
		return spi_sync(smi230_acc_device, &msg);
	else
		return spi_sync(smi230_gyro_device, &msg);
        for (index = 0; index < len; index++)
		PINFO("spi_sync :%x\n", data[index]);
	return ret;
#else
	if (dev_addr == SMI230_ACCEL_CHIP_ID)
		return spi_sync(smi230_acc_device, &msg);
	else
		return spi_sync(smi230_gyro_device, &msg);
#endif
}

/* ACC driver */
static int smi230_acc_probe(struct spi_device *device)
{
	int err;

	device->bits_per_word = 8;
	err = spi_setup(device);
	if (err < 0) {
		PERR("spi_setup err!\n");
		return err;
	}


	/* chip_id is used to differentiate acc/gyro on spi read/write */
	smi230_spi_dev.accel_id = SMI230_ACCEL_CHIP_ID;

	smi230_acc_device = device;

	err = smi230_acc_init(&smi230_spi_dev);
        if (err == SMI230_OK)
		PINFO("Bosch Sensor Device %s initialized", SENSOR_ACC_NAME);
	else {
		smi230_acc_device = NULL;
		PERR("Bosch Sensor Device %s initialization failed, error %d",
				SENSOR_ACC_NAME, err);
	}

	if (smi230_gyro_device == NULL) {
		PERR("%s spi_device is supposed to be initialized", SENSOR_GYRO_NAME);
		return 0;
	}
	else
		return smi230_probe(&device->dev, &smi230_spi_dev);
}

static int smi230_acc_remove(struct spi_device *device)
{
	return smi230_remove(&device->dev);
}

static const struct spi_device_id smi230_acc_id[] = {
	{ SENSOR_ACC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, smi230_acc_id);

static const struct of_device_id smi230_acc_of_match[] = {
	{ .compatible = SENSOR_ACC_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, smi230_acc_of_match);

static struct spi_driver smi230_acc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_ACC_NAME,
		.of_match_table = smi230_acc_of_match,
	},
	.id_table = smi230_acc_id,
	.probe    = smi230_acc_probe,
	.remove	= smi230_acc_remove,
};

/* GYRO driver */
static int smi230_gyro_probe(struct spi_device *device)
{
	int err;

	device->bits_per_word = 8;
	err = spi_setup(device);
	if (err < 0) {
		PERR("spi_setup err!\n");
		return err;
	}

	/* chip_id is used to differentiate acc/gyro on spi read/write */
	smi230_spi_dev.gyro_id = SMI230_GYRO_CHIP_ID;

	smi230_gyro_device = device;

	err = smi230_gyro_init(&smi230_spi_dev);
        if (err == SMI230_OK)
		PINFO("Bosch Sensor Device %s initialized", SENSOR_GYRO_NAME);
	else {
		smi230_gyro_device = NULL;
		PERR("Bosch Sensor Device %s initialization failed, error %d",
			       SENSOR_GYRO_NAME, err);
	}
	return err;
}

static const struct spi_device_id smi230_gyro_id[] = {
	{ SENSOR_GYRO_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, smi230_gyro_id);

static const struct of_device_id smi230_gyro_of_match[] = {
	{ .compatible = SENSOR_GYRO_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, smi230_gyro_of_match);

static struct spi_driver smi230_gyro_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_GYRO_NAME,
		.of_match_table = smi230_gyro_of_match,
	},
	.id_table = smi230_gyro_id,
	.probe    = smi230_gyro_probe,
};

static int __init smi230_module_init(void)
{
	int err;

	smi230_spi_dev.delay_ms = smi230_delay;
	smi230_spi_dev.read_write_len = 32;
	smi230_spi_dev.intf = SMI230_SPI_INTF;
	smi230_spi_dev.read = smi230_spi_read;
	smi230_spi_dev.write = smi230_spi_write;

	/* make sure gyro driver registered first,
	 * while acc driver uses gyro driver */
	err = spi_register_driver(&smi230_gyro_driver);
	if (err != 0)
		return err;

	return spi_register_driver(&smi230_acc_driver);
}

static void __exit smi230_module_exit(void)
{
	spi_unregister_driver(&smi230_acc_driver);
	spi_unregister_driver(&smi230_gyro_driver);
}

module_init(smi230_module_init);
module_exit(smi230_module_exit);

MODULE_DESCRIPTION("SMI230 SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
