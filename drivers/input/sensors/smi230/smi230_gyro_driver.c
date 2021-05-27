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
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "smi230_driver.h"
#include "smi230_data_sync.h"

#define MODULE_TAG MODULE_NAME
#include "smi230_log.h"
#include "smi230.h"

#define SMI230_MIN_VALUE      -32768
#define SMI230_MAX_VALUE      32767

#ifdef CONFIG_SMI230_GYRO_FIFO
#define SMI230_MAX_GYRO_FIFO_FRAME 100
#define SMI230_MAX_GYRO_FIFO_BYTES (SMI230_MAX_GYRO_FIFO_FRAME * SMI230_FIFO_GYRO_FRAME_LENGTH)

static uint8_t fifo_buf[SMI230_MAX_GYRO_FIFO_BYTES];
#endif

struct smi230_client_data {
	struct device *dev;
	struct input_dev *input;
	int IRQ;
	uint8_t gpio_pin;
	struct work_struct irq_work;
};

static struct smi230_dev *p_smi230_dev;

static ssize_t smi230_gyro_show_chip_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t chip_id[2] = {0};
	int err = 0;

	err = smi230_gyro_get_regs(SMI230_GYRO_CHIP_ID_REG, chip_id, 2, p_smi230_dev);
	if (err) {
		PERR("falied");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[0], chip_id[1]);
}

static ssize_t smi230_gyro_reg_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t data = 0;
	int err = 0;
	int i;

	for (i = 0; i <= 0x3F; i++) {
		err = smi230_gyro_get_regs(i, &data, 1, p_smi230_dev);
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

static ssize_t smi230_gyro_show_fifo_wm(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	uint8_t fifo_wm;

	err = smi230_gyro_get_fifo_wm(&fifo_wm, p_smi230_dev);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "fifo water mark is %d\n", fifo_wm);
}

static ssize_t smi230_gyro_store_fifo_wm(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	uint8_t fifo_wm;

	err = kstrtou8(buf, 10, &fifo_wm);
	err |= smi230_gyro_set_fifo_wm(fifo_wm, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("set fifo wm faild");
		return err;
	}

	PDEBUG("set fifo wm to %d", fifo_wm);

	return count;
}

static ssize_t smi230_gyro_store_pw_cfg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long pw_cfg;

	err = kstrtoul(buf, 10, &pw_cfg);
	if (err)
		return err;
	if (pw_cfg != 0) {
		p_smi230_dev->gyro_cfg.power = SMI230_GYRO_PM_SUSPEND;
		err = smi230_gyro_set_power_mode(p_smi230_dev);
	}
	else {
		p_smi230_dev->gyro_cfg.power = SMI230_GYRO_PM_NORMAL;
		err = smi230_gyro_set_power_mode(p_smi230_dev);
	}

	PDEBUG("set power cfg to %ld, err %d", pw_cfg, err);

	if (err) {
		PERR("failed");
		return err;
	}
	return count;
}

static ssize_t smi230_gyro_show_pw_cfg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	err = smi230_gyro_get_power_mode(p_smi230_dev);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "%x (0:active non-zero:suspend)\n", p_smi230_dev->gyro_cfg.power);
}

static ssize_t smi230_gyro_show_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct smi230_sensor_data data = {0};

	err = smi230_gyro_get_data(&data, p_smi230_dev);
	if (err < 0)
		return err;
	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t smi230_gyro_show_driver_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"Driver version: %s\n", DRIVER_VERSION);
}

static DEVICE_ATTR(chip_id, S_IRUGO,
	smi230_gyro_show_chip_id, NULL);
static DEVICE_ATTR(gyro_regs_dump, S_IRUGO,
	smi230_gyro_reg_dump, NULL);
static DEVICE_ATTR(gyro_fifo_wm, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_gyro_show_fifo_wm, smi230_gyro_store_fifo_wm);
static DEVICE_ATTR(gyro_pw_cfg, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_gyro_show_pw_cfg, smi230_gyro_store_pw_cfg);
static DEVICE_ATTR(gyro_value, S_IRUGO,
	smi230_gyro_show_value, NULL);
static DEVICE_ATTR(driver_version, S_IRUGO,
	smi230_gyro_show_driver_version, NULL);

static struct attribute *smi230_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_gyro_regs_dump.attr,
	&dev_attr_gyro_fifo_wm.attr,
	&dev_attr_gyro_pw_cfg.attr,
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
	dev->name = SENSOR_GYRO_NAME;
	dev_set_name(&dev->dev, SENSOR_GYRO_NAME);
	input_set_drvdata(dev, client_data);
	client_data->input = dev;

	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_capability(dev, EV_MSC, MSC_GESTURE);
	input_set_abs_params(dev, ABS_X, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Y, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Z, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);

	err = input_register_device(dev);
	if (err)
		input_free_device(dev);
	return err;
}

#ifdef CONFIG_SMI230_RAW_DATA
static void smi230_gyro_raw_data_ready_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_sensor_data gyro_data;
	int err = 0;

	err = smi230_gyro_get_data(&gyro_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;

	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.z);

	input_sync(client_data->input);
}
#endif

#ifdef CONFIG_SMI230_GYRO_FIFO
static struct smi230_sensor_data fifo_gyro_data[SMI230_MAX_GYRO_FIFO_FRAME];

static void smi230_gyro_fifo_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_fifo_frame fifo;
	int err = 0, i;
	uint8_t fifo_length;

	err = smi230_gyro_get_fifo_length(&fifo_length, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("FIFO get length error!");
		return;
	}

	PINFO("GYRO FIFO length %d", fifo_length);
	fifo.data = fifo_buf;
	fifo.length = fifo_length * SMI230_FIFO_GYRO_FRAME_LENGTH;
	err = smi230_gyro_read_fifo_data(&fifo, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("FIFO read data error!");
		return;
	}

	/* this event here is to indicate IRQ timing if needed */
	input_event(client_data->input, EV_MSC, MSC_RAW, (int)fifo.length);
	input_sync(client_data->input);

	err = smi230_gyro_extract_fifo(fifo_gyro_data,
                            &fifo_length,
                            &fifo,
                            p_smi230_dev);

	for (i = 0; i < fifo_length; i++) {
		input_event(client_data->input, EV_ABS, ABS_X, (int)fifo_gyro_data[i].x);
		input_event(client_data->input, EV_ABS, ABS_Y, (int)fifo_gyro_data[i].y);
		input_event(client_data->input, EV_ABS, ABS_Z, (int)fifo_gyro_data[i].z);
	}
}
#endif
static void smi230_irq_work_func(struct work_struct *work)
{
	struct smi230_client_data *client_data =
		container_of(work, struct smi230_client_data, irq_work);

	/* int status reg is not available to tell the interupt source */
#ifdef CONFIG_SMI230_RAW_DATA
	smi230_gyro_raw_data_ready_handle(client_data);
#endif
#ifdef CONFIG_SMI230_GYRO_FIFO
	smi230_gyro_fifo_handle(client_data);
#endif
}

static irqreturn_t smi230_irq_handle(int irq, void *handle)
{
	struct smi230_client_data *client_data = handle;
	int err = 0;

	err = schedule_work(&client_data->irq_work);
	if (err < 0)
		PERR("schedule_work failed\n");

	return IRQ_HANDLED;
}

static void smi230_free_irq(struct smi230_client_data *client_data)
{
	cancel_work_sync(&client_data->irq_work);
	free_irq(client_data->IRQ, client_data);
	gpio_free(client_data->gpio_pin);
}

static int smi230_request_irq(struct smi230_client_data *client_data)
{
	int err = 0;

	INIT_WORK(&client_data->irq_work, smi230_irq_work_func);

	client_data->gpio_pin = of_get_named_gpio_flags(
		client_data->dev->of_node,
		"gpio_irq", 0, NULL);
	PINFO("SMI230_GYRO gpio number:%d\n", client_data->gpio_pin);
	err = gpio_request_one(client_data->gpio_pin,
				GPIOF_IN, "smi230_gyro_interrupt");
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
			SENSOR_GYRO_NAME, client_data);
	if (err < 0) {
		PDEBUG("request_irq\n");
		return err;
	}
	return err;
}

static void smi230_input_destroy(struct smi230_client_data *client_data)
{
	struct input_dev *dev = client_data->input;
	input_unregister_device(dev);
	/* to avoid underflow of refcount, do a checck before call free device*/
	if (dev->devres_managed)
		input_free_device(dev);
}

int smi230_gyro_remove(struct device *dev)
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

int smi230_gyro_probe(struct device *dev, struct smi230_dev *smi230_dev)
{
	int err = 0;
	struct smi230_client_data *client_data = NULL;
#ifdef CONFIG_SMI230_GYRO_FIFO
	struct gyro_fifo_config fifo_config;
#endif
	struct smi230_int_cfg int_config;

	if (dev == NULL || smi230_dev == NULL)
		return -EINVAL;

	p_smi230_dev = smi230_dev;

	client_data = kzalloc(sizeof(struct smi230_client_data),
						GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_directly;
	}

	client_data->dev = dev;

	/* gyro driver should be initialized before acc */
	p_smi230_dev->gyro_cfg.power = SMI230_GYRO_PM_NORMAL;
	err |= smi230_gyro_set_power_mode(p_smi230_dev);

#ifdef CONFIG_SMI230_RAW_DATA
	PINFO("GYRO raw data is enabled");
	p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_32_ODR_100_HZ;
	p_smi230_dev->gyro_cfg.range = SMI230_GYRO_RANGE_125_DPS;
        err |= smi230_gyro_set_meas_conf(p_smi230_dev);

	smi230_delay(100);

	/*disable gyro int on channel 3 */
	int_config.gyro_int_config_1.int_channel = SMI230_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	/*enable gyro fifo int on channel 4 */
	int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	PINFO("GYRO FIFO set int3 config");
	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_1, p_smi230_dev);
	PINFO("GYRO FIFO set int4 config");
	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_2, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("FIFO HW init failed");
		goto exit_free_client_data;
	}

	smi230_delay(100);
#endif

#ifdef CONFIG_SMI230_GYRO_FIFO
	PINFO("GYRO FIFO is enabled");

	p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_32_ODR_100_HZ;
	p_smi230_dev->gyro_cfg.bw = SMI230_GYRO_BW_32_ODR_100_HZ;
	PINFO("GYRO FIFO set meas");
        err |= smi230_gyro_set_meas_conf(p_smi230_dev);

	smi230_delay(100);

	/*disable gyro int on channel 3 */
	int_config.gyro_int_config_1.int_channel = SMI230_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	/*enable gyro fifo int on channel 4 */
	int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = SMI230_GYRO_FIFO_INT;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	PINFO("GYRO FIFO set int3 config");
	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_1, p_smi230_dev);
	PINFO("GYRO FIFO set int4 config");
	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_2, p_smi230_dev);

	PINFO("GYRO FIFO set water mark");
	err |= smi230_gyro_set_fifo_wm(100, p_smi230_dev); 
	fifo_config.mode = SMI230_GYRO_FIFO_MODE;
	/* 0x88 to enable wm int, 0x80 to disable */
	fifo_config.wm_en = 0x88;
	/* disable external event sync on both int3 and int 4 */
	fifo_config.int3_en = 0;
	fifo_config.int4_en = 0;
	PINFO("GYRO FIFO set fifo config");
	err |= smi230_gyro_set_fifo_config(&fifo_config, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("FIFO HW init failed");
		goto exit_free_client_data;
	}

	smi230_delay(100);
#endif

#ifndef CONFIG_SMI230_DEBUG 
	/*if not for the convenience of debuging, sensors should be disabled at startup*/
	p_smi230_dev->gyro_cfg.power = SMI230_GYRO_PM_SUSPEND;
	err |= smi230_gyro_set_power_mode(p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("set power mode failed");
		goto exit_free_client_data;
	}
#endif

	/*input device init */
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
	err = smi230_request_irq(client_data);
	if (err < 0) {
		PERR("Request irq failed");
		goto exit_cleanup_sysfs;
	}

	PINFO("Sensor %s was probed successfully", SENSOR_GYRO_NAME);

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
