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
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/jiffies.h>

#include "smi230_data_sync.h"
#include "smi230_log.h"
#include "smi230.h"

#define DRIVER_VERSION "0.1"

#define SMI230_DEBUG 1
#define SMI230_MAX_RETRY_I2C_XFER 10
#define SMI230_I2C_WRITE_DELAY_TIME 10
#define SMI230_ACC_ENABLE_INT2 1
#define SENSOR_NAME "SMI230"
#define SMI230_MIN_VALUE      -32768
#define SMI230_MAX_VALUE      32767

struct smi230_client_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input;
	int IRQ;
	uint8_t gpio_pin;
	struct work_struct irq_work;
};


struct smi230_dev smi230_device;
struct i2c_adapter *smi230_i2c_adapter;

static ssize_t smi230_acc_reg_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t data = 0;
	int err = 0;
	int i;

	for (i = 0; i <= 0x7e; i++) {
		err = smi230_acc_get_regs(i, &data, 1, &smi230_device);
		if (err) {
			PERR("falied");
			return err;
		}
		printk("0x%x = 0x%x", i, data);
		if ( i % 15 == 0 )
			printk("\n");
	}

	return 0;
}

static ssize_t smi230_acc_show_chip_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t chip_id[2] = {0};
	int err = 0;

	err = smi230_acc_get_regs(SMI230_ACCEL_CHIP_ID_REG, chip_id, 2, &smi230_device);
	if (err) {
		PERR("falied");
		return err;
	}
	return snprintf(buf, 96, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[0], chip_id[1]);
}

static ssize_t smi230_acc_show_acc_pw_cfg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	err = smi230_acc_get_power_mode(&smi230_device);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, 96, "%x (0:active 3:suspend)\n", smi230_device.accel_cfg.power);
}

static ssize_t smi230_acc_store_acc_pw_cfg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long pw_cfg;

	err = kstrtoul(buf, 10, &pw_cfg);
	if (err)
		return err;
	if (pw_cfg == 3) {
		smi230_device.accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
		err = smi230_acc_set_power_mode(&smi230_device);
	}
	else if (pw_cfg == 0) {
		smi230_device.accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
		err = smi230_acc_set_power_mode(&smi230_device);
	}

	PDEBUG("set power cfg to %ld, err %d", pw_cfg, err);

	if (err) {
		PERR("failed");
		return err;
	}
	return count;
}

static ssize_t smi230_acc_show_acc_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct smi230_sensor_data data = {0};

        err = smi230_acc_get_data(&data, &smi230_device);
	if (err < 0)
		return err;
	return snprintf(buf, 48, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t smi230_acc_show_gyro_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct smi230_sensor_data data = {0};

        err = smi230_gyro_get_data(&data, &smi230_device);
	if (err < 0)
		return err;
	return snprintf(buf, 48, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t smi230_acc_show_driver_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 128,
		"Driver version: %s\n", DRIVER_VERSION);
}

static ssize_t smi230_acc_show_sync_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	struct smi230_sensor_data accel_data;
	struct smi230_sensor_data gyro_data;

	err = smi230_get_synchronized_data(&accel_data, &gyro_data, &smi230_device);
        if (err != SMI230_OK)
		return err;

	return snprintf(buf, 48, "acc: %hd %hd %hd gyro: %hd %hd %hd\n",
			accel_data.x, accel_data.y, accel_data.z,
			gyro_data.x, gyro_data.y, gyro_data.z
			);
}

static DEVICE_ATTR(data_sync, S_IRUGO,
	smi230_acc_show_sync_data, NULL);
static DEVICE_ATTR(regs_dump, S_IRUGO,
	smi230_acc_reg_dump, NULL);
static DEVICE_ATTR(chip_id, S_IRUGO,
	smi230_acc_show_chip_id, NULL);
static DEVICE_ATTR(acc_pw_cfg, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_acc_pw_cfg, smi230_acc_store_acc_pw_cfg);
static DEVICE_ATTR(acc_value, S_IRUGO,
	smi230_acc_show_acc_value, NULL);
static DEVICE_ATTR(gyro_value, S_IRUGO,
	smi230_acc_show_gyro_value, NULL);
static DEVICE_ATTR(driver_version, S_IRUGO,
	smi230_acc_show_driver_version, NULL);

static struct attribute *smi230_attributes[] = {
	&dev_attr_data_sync.attr,
	&dev_attr_regs_dump.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_acc_pw_cfg.attr,
	&dev_attr_acc_value.attr,
	&dev_attr_gyro_value.attr,
	&dev_attr_driver_version.attr,
	NULL
};

static struct attribute_group smi230_attribute_group = {
	.attrs = smi230_attributes
};

static int smi230_input_init(struct smi230_client_data *client_data)
{
	int err = 0;
	struct input_dev *dev = input_allocate_device();

	if (dev == NULL)
		return -ENOMEM;

	dev->id.bustype = BUS_I2C;
	dev->name = SENSOR_NAME;
	input_set_drvdata(dev, client_data);
	client_data->input = dev;

	input_set_capability(dev, EV_MSC, MSC_GESTURE);
	input_set_abs_params(dev, ABS_X, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Y, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Z, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);

	err = input_register_device(dev);
	if (err)
		input_free_device(dev);
	return err;
}

#if defined(SMI230_ACC_ENABLE_INT1) || defined(SMI230_ACC_ENABLE_INT2)
static void smi230_data_ready_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_sensor_data accel_data;
	struct smi230_sensor_data gyro_data;
	struct smi230_sensor_data accel_sync;
	int err = 0;

        err = smi230_acc_get_data(&accel_data, &smi230_device);
        if (err != SMI230_OK)
		return;
	err = smi230_get_synchronized_data(&accel_sync, &gyro_data, &smi230_device);
        if (err != SMI230_OK)
		return;

	PINFO("synced data acc: %hd %hd %hd gyro: %hd %hd %hd\n",
			accel_data.x, accel_data.y, accel_data.z,
			gyro_data.x, gyro_data.y, gyro_data.z
			);

	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_sync.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_sync.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_sync.z);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.z);

	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.z);
	
	input_sync(client_data->input);
}

static void smi230_irq_work_func(struct work_struct *work)
{
	struct smi230_client_data *client_data =
		container_of(work, struct smi230_client_data, irq_work);

	smi230_data_ready_handle(client_data);
}

static irqreturn_t smi230_irq_handle(int irq, void *handle)
{
	struct smi230_client_data *client_data = handle;
	int err = 0;

	PDEBUG("smi230 int timestamp %llu", ktime_get_ns());

	err = schedule_work(&client_data->irq_work);
	if (err < 0)
		PERR("schedule_work failed\n");

	return IRQ_HANDLED;
}

static void smi230_free_irq(struct smi230_client_data *client_data)
{
	free_irq(client_data->IRQ, client_data);
	gpio_free(client_data->gpio_pin);
}

static int smi230_request_irq(struct smi230_client_data *client_data)
{
	int err = 0;
	client_data->gpio_pin = of_get_named_gpio_flags(
		client_data->dev->of_node,
		"gpio_irq", 0, NULL);
	PINFO("SMI230_ACC gpio number:%d\n", client_data->gpio_pin);
	err = gpio_request_one(client_data->gpio_pin,
				GPIOF_IN, "smi230_acc_interrupt");
	if (err < 0) {
		PDEBUG("gpio_request_one\n");
		return err;
	}
	err = gpio_direction_input(client_data->gpio_pin);
	if (err < 0) {
		PDEBUG("gpio_direction_input\n");
		return err;
	}
	client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
	err = request_irq(client_data->IRQ, smi230_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME, client_data);
	if (err < 0) {
		PDEBUG("request_irq\n");
		return err;
	}
	INIT_WORK(&client_data->irq_work, smi230_irq_work_func);
	return err;
}
#endif

static void smi230_input_destroy(struct smi230_client_data *client_data)
{
	struct input_dev *dev = client_data->input;
	input_unregister_device(dev);
	/* to avoid underflow of refcount, do a checck before call free device*/
	if (dev->devres_managed)
		input_free_device(dev);
}

int smi230_remove(struct device *dev)
{
	int err = 0;
	struct smi230_client_data *client_data = dev_get_drvdata(dev);

	if (NULL != client_data) {
		smi230_free_irq(client_data);
		sysfs_remove_group(&client_data->input->dev.kobj,
				&smi230_attribute_group);
		smi230_input_destroy(client_data);
		kfree(client_data);
	}
	return err;
}

static int smi230_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;
	struct smi230_client_data *client_data = NULL;
	struct smi230_data_sync_cfg sync_cfg;
	struct smi230_int_cfg int_config;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		goto exit_directly;
	}

	if (smi230_i2c_adapter == NULL) {
		smi230_i2c_adapter = client->adapter;
		PINFO("acc i2c adapter is at %x", (unsigned int)client->adapter);
	}

	smi230_device.accel_id = client->addr;
	/*In theory, the gyro_id should be passed in by kernel in its own driver
	 * However, in smi230 driver, this 2 sensors are in one chip, connected
	 * on the same I2C/SPI bus and more importatly they work together in
	 * sync mode. Therefore it makes more sense to put them in one driver,
	 * and probed once with accel_id passed in and gyro_id set as a offset
	 * to it.*/
	smi230_device.gyro_id = client->addr + 0x50;

	client_data = kzalloc(sizeof(struct smi230_client_data),
						GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_directly;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;
	client_data->dev = &client->dev;

	/* check chip id */
	err = smi230_init(&smi230_device);
        if (err == SMI230_OK)
		PINFO("Bosch Sensor Device %s initialized", SENSOR_NAME);
	else {
		PERR("Bosch Sensor Device initialization failed");
		err = SMI230_E_DEV_NOT_FOUND;
		goto exit_free_client_data;
	}

	/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
	err |= smi230_acc_soft_reset(&smi230_device);

	/*! Max read/write length (maximum supported length is 32).
	 To be set by the user */
	/*set accel power mode */
	smi230_device.accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	err |= smi230_acc_set_power_mode(&smi230_device);

	/* gyro driver should be initialized before acc */
	smi230_device.gyro_cfg.power = SMI230_GYRO_PM_NORMAL;
	err |= smi230_gyro_set_power_mode(&smi230_device);

	/* API uploads the smi230 config file onto the device and wait for 150ms 
	   to enable the data synchronization - delay taken care inside the function */
	err |= smi230_apply_config_file(&smi230_device);
        if (err == SMI230_OK)
		PINFO("Firmware load succeed!");
	else {
		PERR("Firmware load error!");
		goto exit_free_client_data;
	}

	/*assign accel range setting*/
	smi230_device.accel_cfg.range = SMI230_ACCEL_RANGE_4G;
	/*assign gyro range setting*/
	smi230_device.gyro_cfg.range = SMI230_GYRO_RANGE_2000_DPS;
	/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
	sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_2000HZ;
	err |= smi230_configure_data_synchronization(sync_cfg, &smi230_device);


	/*set accel interrupt pin configuration*/
	/*configure host data ready interrupt */
	int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = SMI230_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	/*configure Accel syncronization input interrupt pin */
	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = SMI230_ACCEL_SYNC_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	/*set gyro interrupt pin configuration*/
	int_config.gyro_int_config_1.int_channel = SMI230_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = SMI230_ENABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	/* Enable synchronization interrupt pin */
	err |= smi230_set_data_sync_int_config(&int_config, &smi230_device);
	if (err != SMI230_OK) {
		PERR("HW init failed");
		goto exit_free_client_data;
	}

#ifndef SMI230_DEBUG 
	/*if not for the convenience of debuging, sensors should be disabled at startup*/
	smi230_device.accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	err |= smi230_acc_set_power_mode(&smi230_device);
	smi230_device.gyro_cfg.power = SMI230_GYRO_PM_SUSPEND;
	err |= smi230_gyro_set_power_mode(&smi230_device);
	if (err != SMI230_OK) {
		PERR("set power mode failed");
		goto exit_free_client_data;
	}
#endif

	/*acc input device init */
	err = smi230_input_init(client_data);
	if (err < 0) {
		PERR("input init failed");
		goto exit_free_client_data;
	}

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&smi230_attribute_group);
	if (err < 0) {
		PERR("sysfs create failed");
		goto exit_cleanup_input;
	}

	/*request irq and config*/
	#if defined(SMI230_ACC_ENABLE_INT1) || defined(SMI230_ACC_ENABLE_INT2)
	err = smi230_request_irq(client_data);
	if (err < 0) {
		PERR("Request irq failed");
		goto exit_cleanup_sysfs;
	}
	#endif

	PINFO("Sensor %s was probed successfully", SENSOR_NAME);

	return 0;
exit_cleanup_sysfs:
	sysfs_remove_group(&client_data->input->dev.kobj,
		&smi230_attribute_group);
exit_cleanup_input:
	smi230_input_destroy(client_data);
exit_free_client_data:
	if (client_data != NULL)
		kfree(client_data);
exit_directly:
	return err;
}

static int smi230_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	err = smi230_remove(&client->dev);
	return err;
}

static const struct i2c_device_id smi230_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smi230_id);

static const struct of_device_id smi230_of_match[] = {
	{ .compatible = "SMI230", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smi230_of_match);

struct i2c_driver smi230_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = smi230_of_match,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = smi230_id,
	.probe = smi230_i2c_probe,
	.remove = smi230_i2c_remove,
};

static void smi230_i2c_delay(uint32_t msec)
{
	unsigned long mseond = msec;
	unsigned long min = mseond * (1000);
	/* if the time less than 20ms */
	if (msec <= 20)
		usleep_range(min, (min + 1000));
	else
		msleep(msec);
}

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

static int __init SMI230_init(void)
{
	smi230_device.intf = SMI230_I2C_INTF;
	smi230_device.read = smi230_i2c_read;
	smi230_device.write = smi230_i2c_write;
	smi230_device.delay_ms = smi230_i2c_delay;
	smi230_device.read_write_len = 32;

	return i2c_add_driver(&smi230_driver);
}

static void __exit SMI230_exit(void)
{
	i2c_del_driver(&smi230_driver);
}

module_init(SMI230_init);
module_exit(SMI230_exit);

MODULE_DESCRIPTION("SMI230 SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
