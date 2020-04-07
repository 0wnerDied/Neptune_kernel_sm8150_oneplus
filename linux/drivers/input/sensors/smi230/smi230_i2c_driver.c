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
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "smi230_driver.h"

#define MODULE_TAG MODULE_NAME
#include "smi230_log.h"
#include "smi230.h"

#define SMI230_MAX_RETRY_I2C_XFER 10
#define SMI230_I2C_WRITE_DELAY_TIME 10

struct i2c_adapter *smi230_i2c_adapter;

static struct smi230_dev smi230_i2c_dev;

static int8_t smi230_i2c_read(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int32_t retry;

	struct i2c_msg msg[] = {
		{
		.addr = dev_addr,
		.flags = 0,
		.len = 1,
		.buf = &reg_addr,
		},

		{
		.addr = dev_addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data,
		},
	};
	for (retry = 0; retry < SMI230_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(smi230_i2c_adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			usleep_range(SMI230_I2C_WRITE_DELAY_TIME * 1000,
				SMI230_I2C_WRITE_DELAY_TIME * 1000);
	}

	/*PDEBUG("client->addr 0x%x, reg addr 0x%x, len %d, *data %x", client->addr, reg_addr, len, *data);*/
	if (SMI230_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

static int8_t smi230_i2c_write(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int32_t retry;

	/*PDEBUG("client->addr 0x%x, reg addr 0x%x, len %d, *data %x", client->addr, reg_addr, len, *data);*/

	struct i2c_msg msg = {
		.addr = dev_addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};
	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		PERR("Allocate mem failed\n");
		return -ENOMEM;
	}
	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);
	for (retry = 0; retry < SMI230_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(smi230_i2c_adapter, &msg, 1) > 0)
			break;
		else
			usleep_range(SMI230_I2C_WRITE_DELAY_TIME * 1000,
				SMI230_I2C_WRITE_DELAY_TIME * 1000);
	}
	kfree(msg.buf);
	if (SMI230_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

/* ACC driver */
static int smi230_acc_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		return err;
	}

	if ((smi230_i2c_adapter != NULL) &&
			(smi230_i2c_adapter == client->adapter)) {
		PINFO("%s i2c adapter is at %x", SENSOR_ACC_NAME, (unsigned int)client->adapter);
	}
	else {
		PERR("%s i2c driver is not initialized yet before ACC driver!", SENSOR_GYRO_NAME);
		err = -EIO;
		return err;
	}

	smi230_i2c_dev.accel_id = client->addr;

	err = smi230_acc_init(&smi230_i2c_dev);
        if (err == SMI230_OK)
		PINFO("Bosch Sensor Device %s initialized", SENSOR_ACC_NAME);
	else {
		PERR("Bosch Sensor Device %s initialization failed, error %d",
				SENSOR_ACC_NAME, err);
	}

	return smi230_probe(&client->dev, &smi230_i2c_dev);
}

static int smi230_acc_remove(struct i2c_client *client)
{
	return smi230_remove(&client->dev);
}

static const struct i2c_device_id smi230_acc_id[] = {
	{ SENSOR_ACC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smi230_acc_id);

static const struct of_device_id smi230_acc_of_match[] = {
	{ .compatible = SENSOR_ACC_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, smi230_acc_of_match);

struct i2c_driver smi230_acc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_ACC_NAME,
		.of_match_table = smi230_acc_of_match,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = smi230_acc_id,
	.probe = smi230_acc_probe,
	.remove = smi230_acc_remove,
};


/* GYRO driver */
static int smi230_gyro_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		return err;
	}

	if (smi230_i2c_adapter == NULL) {
		smi230_i2c_adapter = client->adapter;
		PINFO("%s i2c adapter is at %x", SENSOR_GYRO_NAME, (unsigned int)client->adapter);
	}
	else {
		PERR("%s i2c driver should be initialized first!", SENSOR_GYRO_NAME);
		err = -EIO;
		return err;
	}

	smi230_i2c_dev.gyro_id = client->addr;

	err = smi230_gyro_init(&smi230_i2c_dev);
        if (err == SMI230_OK)
		PINFO("Bosch Sensor Device %s initialized", SENSOR_GYRO_NAME);
	else {
		PERR("Bosch Sensor Device %s initialization failed, error %d",
				SENSOR_GYRO_NAME, err);
	}

	return err;
}

static const struct i2c_device_id smi230_gyro_id[] = {
	{ SENSOR_GYRO_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smi230_gyro_id);

static const struct of_device_id smi230_gyro_of_match[] = {
	{ .compatible = SENSOR_GYRO_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, smi230_gyro_of_match);

static struct i2c_driver smi230_gyro_driver = {
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

	smi230_i2c_dev.delay_ms = smi230_delay;
	smi230_i2c_dev.read_write_len = 32;
	smi230_i2c_dev.intf = SMI230_I2C_INTF;
	smi230_i2c_dev.read = smi230_i2c_read;
	smi230_i2c_dev.write = smi230_i2c_write;

	/* make sure gyro driver registered first,
	 * while acc driver uses gyro driver */
	err = i2c_add_driver(&smi230_gyro_driver);
	if (err != 0)
		return err;

	return i2c_add_driver(&smi230_acc_driver);
}

static void __exit smi230_module_exit(void)
{
	i2c_del_driver(&smi230_acc_driver);
	i2c_del_driver(&smi230_gyro_driver);
}

module_init(smi230_module_init);
module_exit(smi230_module_exit);

MODULE_DESCRIPTION("SMI230 SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
