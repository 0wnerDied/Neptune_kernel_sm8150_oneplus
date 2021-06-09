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

#define SMI230_ACC_ENABLE_INT2 1
#define SMI230_MIN_VALUE      -32768
#define SMI230_MAX_VALUE      32767

#ifdef CONFIG_SMI230_ACC_FIFO
#define SMI230_MAX_ACC_FIFO_BYTES 1024
/*1024 bytes fifo host max 147 data frames*/
#define SMI230_MAX_ACC_FIFO_FRAME 147

static uint8_t fifo_buf[SMI230_MAX_ACC_FIFO_BYTES];
#endif

#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
#define SMI_ACC_MAXSAMPLE        5000
#define G_MAX                    23920640
struct smi_acc_sample {
	int xyz[3];
	unsigned int tsec;
	unsigned long long tnsec;
};
#endif

struct smi230_client_data {
	struct device *dev;
	struct input_dev *input;
	int IRQ;
	uint8_t gpio_pin;
	struct work_struct irq_work;
	uint64_t timestamp;
#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
	bool read_acc_boot_sample;
	int acc_bufsample_cnt;
	bool acc_buffer_smi230_samples;
	bool acc_enable;
	struct kmem_cache *smi_acc_cachepool;
	struct smi_acc_sample *smi230_acc_samplist[SMI_ACC_MAXSAMPLE];
	int max_buffer_time;
	struct input_dev *accbuf_dev;
	int report_evt_cnt;
	struct mutex acc_sensor_buff;
#endif
};

static struct smi230_dev *p_smi230_dev;
#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
static inline int smi230_check_acc_early_buff_enable_flag(
		struct smi230_client_data *client_data)
{
	if (client_data->acc_buffer_smi230_samples == true)
		return 1;
	else
		return 0;
}
static void smi230_check_acc_enable_flag(struct smi230_client_data *client_data,
		unsigned long data)
{
	if (data == 0)
		client_data->acc_enable = true;
	else
		client_data->acc_enable = false;
}
#else
static inline int smi230_check_acc_early_buff_enable_flag(
		struct smi230_client_data *client_data)
{
	return 0;
}
static void smi230_check_acc_enable_flag(struct smi230_client_data *client_data,
		unsigned long data)
{

}
#endif
static ssize_t smi230_acc_reg_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t data = 0;
	int err = 0;
	int i;

	for (i = 0; i <= 0x7e; i++) {
		err = smi230_acc_get_regs(i, &data, 1, p_smi230_dev);
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

	err = smi230_acc_get_regs(SMI230_ACCEL_CHIP_ID_REG, chip_id, 2, p_smi230_dev);
	if (err) {
		PERR("falied");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[0], chip_id[1]);
}

static ssize_t smi230_acc_show_fifo_wm(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	uint16_t fifo_wm;

	err = smi230_acc_get_fifo_wm(&fifo_wm, p_smi230_dev);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "fifo water mark is %ud\n", fifo_wm);
}

static ssize_t smi230_acc_store_fifo_wm(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	uint16_t fifo_wm;

	err = kstrtou16(buf, 10, &fifo_wm);
	err |= smi230_acc_set_fifo_wm(fifo_wm, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("set fifo wm faild");
		return err;
	}

	PDEBUG("set fifo wm to %d", fifo_wm);

	return count;
}

static ssize_t smi230_acc_show_acc_pw_cfg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	err = smi230_acc_get_power_mode(p_smi230_dev);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "%x (0:active 3:suspend)\n", p_smi230_dev->accel_cfg.power);
}

static ssize_t smi230_acc_store_acc_pw_cfg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long pw_cfg;

	struct smi230_client_data *client_data = dev_get_drvdata(dev);

	err = kstrtoul(buf, 10, &pw_cfg);
	if (err)
		return err;

	smi230_check_acc_enable_flag(client_data, pw_cfg);

	err = smi230_check_acc_early_buff_enable_flag(client_data);
	if (err)
		return count;

	if (pw_cfg == 3) {
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
		err = smi230_acc_set_power_mode(p_smi230_dev);
	}
	else if (pw_cfg == 0) {
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
		err = smi230_acc_set_power_mode(p_smi230_dev);
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

	err = smi230_acc_get_data(&data, p_smi230_dev);
	if (err < 0)
		return err;
	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t smi230_acc_show_driver_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"Driver version: %s\n", DRIVER_VERSION);
}

static ssize_t smi230_acc_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t sensor_temp = 0;
	int err;

	err = smi230_acc_get_sensor_temperature(p_smi230_dev, &sensor_temp);
	if (err != SMI230_OK)
		return err;

	return snprintf(buf, PAGE_SIZE, "temperature: %d\n", sensor_temp);

}

static ssize_t smi230_acc_show_sync_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	struct smi230_sensor_data accel_data;
	struct smi230_sensor_data gyro_data;

	err = smi230_get_synchronized_data(&accel_data, &gyro_data, p_smi230_dev);
	if (err != SMI230_OK)
		return err;

	return snprintf(buf, PAGE_SIZE, "acc: %hd %hd %hd gyro: %hd %hd %hd\n",
			accel_data.x, accel_data.y, accel_data.z,
			gyro_data.x, gyro_data.y, gyro_data.z
			);
}
#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
static int smi_acc_read_bootsampl(struct smi230_client_data *client_data,
		unsigned long enable_read)
{
	int i = 0;

	client_data->acc_buffer_smi230_samples = false;

	if (enable_read) {
		for (i = 0; i < client_data->acc_bufsample_cnt; i++) {
			PDEBUG("acc=%d,x=%d,y=%d,z=%d,sec=%d,ns=%lld\n",
				i, client_data->smi230_acc_samplist[i]->xyz[0],
				client_data->smi230_acc_samplist[i]->xyz[1],
				client_data->smi230_acc_samplist[i]->xyz[2],
				client_data->smi230_acc_samplist[i]->tsec,
				client_data->smi230_acc_samplist[i]->tnsec);
			input_report_abs(client_data->accbuf_dev, ABS_X,
				client_data->smi230_acc_samplist[i]->xyz[0]);
			input_report_abs(client_data->accbuf_dev, ABS_Y,
				client_data->smi230_acc_samplist[i]->xyz[1]);
			input_report_abs(client_data->accbuf_dev, ABS_Z,
				client_data->smi230_acc_samplist[i]->xyz[2]);
			input_report_abs(client_data->accbuf_dev, ABS_RX,
				client_data->smi230_acc_samplist[i]->tsec);
			input_report_abs(client_data->accbuf_dev, ABS_RY,
				client_data->smi230_acc_samplist[i]->tnsec);
			input_sync(client_data->accbuf_dev);
		}
	} else {
		/* clean up */
		if (client_data->acc_bufsample_cnt != 0) {
			for (i = 0; i < SMI_ACC_MAXSAMPLE; i++)
				kmem_cache_free(client_data->smi_acc_cachepool,
					client_data->smi230_acc_samplist[i]);
			kmem_cache_destroy(client_data->smi_acc_cachepool);
			client_data->acc_bufsample_cnt = 0;
		}

	}
	/*SYN_CONFIG indicates end of data*/
	input_event(client_data->accbuf_dev, EV_SYN, SYN_CONFIG, 0xFFFFFFFF);
	input_sync(client_data->accbuf_dev);
	PDEBUG("End of acc samples bufsample_cnt=%d\n",
			client_data->acc_bufsample_cnt);
	return 0;
}
static ssize_t read_acc_boot_sample_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct smi230_client_data *client_data = dev_get_drvdata(dev);

	return snprintf(buf, 16, "%u\n",
			client_data->read_acc_boot_sample);
}
static ssize_t read_acc_boot_sample_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;

	struct smi230_client_data *client_data = dev_get_drvdata(dev);

	unsigned long enable = 0;

	err = kstrtoul(buf, 10, &enable);
	if (err)
		return err;
	if (enable > 1) {
		PERR("Invalid value of input, input=%ld\n", enable);
		return -EINVAL;
	}
	mutex_lock(&client_data->acc_sensor_buff);
	err = smi_acc_read_bootsampl(client_data, enable);
	mutex_unlock(&client_data->acc_sensor_buff);
	if (err)
		return err;

	client_data->read_acc_boot_sample = enable;
	return count;
}
#endif

static DEVICE_ATTR(data_sync, S_IRUGO,
	smi230_acc_show_sync_data, NULL);
static DEVICE_ATTR(acc_regs_dump, S_IRUGO,
	smi230_acc_reg_dump, NULL);
static DEVICE_ATTR(chip_id, S_IRUGO,
	smi230_acc_show_chip_id, NULL);
static DEVICE_ATTR(acc_fifo_wm, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_fifo_wm, smi230_acc_store_fifo_wm);
static DEVICE_ATTR(acc_pw_cfg, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_acc_pw_cfg, smi230_acc_store_acc_pw_cfg);
static DEVICE_ATTR(acc_value, S_IRUGO,
	smi230_acc_show_acc_value, NULL);
static DEVICE_ATTR(driver_version, S_IRUGO,
	smi230_acc_show_driver_version, NULL);
static DEVICE_ATTR(temperature, 0644,
		smi230_acc_temperature_show, NULL);
#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
static DEVICE_ATTR(read_acc_boot_sample, 0644,
		read_acc_boot_sample_show, read_acc_boot_sample_store);
#endif

static struct attribute *smi230_attributes[] = {
	&dev_attr_data_sync.attr,
	&dev_attr_acc_regs_dump.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_acc_fifo_wm.attr,
	&dev_attr_acc_pw_cfg.attr,
	&dev_attr_acc_value.attr,
	&dev_attr_driver_version.attr,
	&dev_attr_temperature.attr,
#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
	&dev_attr_read_acc_boot_sample.attr,
#endif
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
	dev->name = SENSOR_ACC_NAME;
	//dev_set_name(&dev->dev, SENSOR_ACC_NAME);
	input_set_drvdata(dev, client_data);
	client_data->input = dev;

	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_capability(dev, EV_MSC, MSC_GESTURE);
	input_set_capability(dev, EV_MSC, MSC_TIMESTAMP);
	input_set_abs_params(dev, ABS_X, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Y, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Z, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);

	err = input_register_device(dev);
	if (err)
		input_free_device(dev);
	return err;
}

#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
static void store_acc_boot_sample(struct smi230_client_data *client_data,
		int x, int y, int z, struct timespec ts)
{
	if (false == client_data->acc_buffer_smi230_samples)
		return;
	mutex_lock(&client_data->acc_sensor_buff);
	if (ts.tv_sec <  client_data->max_buffer_time) {
		if (client_data->acc_bufsample_cnt < SMI_ACC_MAXSAMPLE) {
			client_data->smi230_acc_samplist[client_data
				->acc_bufsample_cnt]->xyz[0] = x;
			client_data->smi230_acc_samplist[client_data
				->acc_bufsample_cnt]->xyz[1] = y;
			client_data->smi230_acc_samplist[client_data
				->acc_bufsample_cnt]->xyz[2] = z;
			client_data->smi230_acc_samplist[client_data
				->acc_bufsample_cnt]->tsec = ts.tv_sec;
			client_data->smi230_acc_samplist[client_data
				->acc_bufsample_cnt]->tnsec = ts.tv_nsec;
			client_data->acc_bufsample_cnt++;
		}
	} else {
		PINFO("End of ACC buffering %d\n",
				client_data->acc_bufsample_cnt);
		client_data->acc_buffer_smi230_samples = false;
		if (client_data->acc_enable == false) {
			/*set accel power mode */
			p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
			smi230_acc_set_power_mode(p_smi230_dev);
		}
	}
	mutex_unlock(&client_data->acc_sensor_buff);
}
#else
static void store_acc_boot_sample(struct smi230_client_data *client_data,
		int x, int y, int z, struct timespec ts)
{
}
#endif
#ifdef CONFIG_ENABLE_SMI230_ACC_GYRO_BUFFERING
static int smi230_acc_early_buff_init(struct smi230_client_data *client_data)
{
	int i = 0, err = 0;

	client_data->acc_bufsample_cnt = 0;
	client_data->report_evt_cnt = 5;
	client_data->max_buffer_time = 40;

	client_data->smi_acc_cachepool = kmem_cache_create("acc_sensor_sample",
			sizeof(struct smi_acc_sample),
			0,
			SLAB_HWCACHE_ALIGN, NULL);
	if (!client_data->smi_acc_cachepool) {
		PERR("smi_acc_cachepool cache create failed\n");
		err = -ENOMEM;
		return 0;
	}
	for (i = 0; i < SMI_ACC_MAXSAMPLE; i++) {
		client_data->smi230_acc_samplist[i] =
			kmem_cache_alloc(client_data->smi_acc_cachepool,
					GFP_KERNEL);
		if (!client_data->smi230_acc_samplist[i]) {
			err = -ENOMEM;
			goto clean_exit1;
		}
	}

	client_data->accbuf_dev = input_allocate_device();
	if (!client_data->accbuf_dev) {
		err = -ENOMEM;
		PERR("input device allocation failed\n");
		goto clean_exit1;
	}
	client_data->accbuf_dev->name = "smi230_accbuf";
	client_data->accbuf_dev->id.bustype = BUS_I2C;
	input_set_events_per_packet(client_data->accbuf_dev,
			client_data->report_evt_cnt * SMI_ACC_MAXSAMPLE);
	set_bit(EV_ABS, client_data->accbuf_dev->evbit);
	input_set_abs_params(client_data->accbuf_dev, ABS_X,
			-G_MAX, G_MAX, 0, 0);
	input_set_abs_params(client_data->accbuf_dev, ABS_Y,
			-G_MAX, G_MAX, 0, 0);
	input_set_abs_params(client_data->accbuf_dev, ABS_Z,
			-G_MAX, G_MAX, 0, 0);
	input_set_abs_params(client_data->accbuf_dev, ABS_RX,
			-G_MAX, G_MAX, 0, 0);
	input_set_abs_params(client_data->accbuf_dev, ABS_RY,
			-G_MAX, G_MAX, 0, 0);
	err = input_register_device(client_data->accbuf_dev);
	if (err) {
		PERR("unable to register input device %s\n",
				client_data->accbuf_dev->name);
		goto clean_exit2;
	}

	client_data->acc_buffer_smi230_samples = true;
	client_data->acc_enable = false;

	mutex_init(&client_data->acc_sensor_buff);

	/*set accel power mode */
	p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	smi230_acc_set_power_mode(p_smi230_dev);

	return 1;

clean_exit2:
	input_free_device(client_data->accbuf_dev);
clean_exit1:
	for (i = 0; i < SMI_ACC_MAXSAMPLE; i++)
		kmem_cache_free(client_data->smi_acc_cachepool,
				client_data->smi230_acc_samplist[i]);
	kmem_cache_destroy(client_data->smi_acc_cachepool);
	return 0;
}
static void smi230_acc_input_cleanup(struct smi230_client_data *client_data)
{
	int i = 0;

	input_unregister_device(client_data->accbuf_dev);
	input_free_device(client_data->accbuf_dev);
	for (i = 0; i < SMI_ACC_MAXSAMPLE; i++)
		kmem_cache_free(client_data->smi_acc_cachepool,
				client_data->smi230_acc_samplist[i]);
	kmem_cache_destroy(client_data->smi_acc_cachepool);
}
#else
static int smi230_acc_early_buff_init(struct smi230_client_data *client_data)
{
	return 1;
}
static void smi230_acc_input_cleanup(struct smi230_client_data *client_data)
{
}
#endif

#ifdef CONFIG_SMI230_RAW_DATA
static void smi230_raw_data_ready_handle(
		struct smi230_client_data *client_data)
{
	struct smi230_sensor_data accel_data;
	int err = 0;
	struct timespec ts;

	ts = ns_to_timespec(client_data->timestamp);

	err = smi230_acc_get_data(&accel_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;
	input_event(client_data->input, EV_MSC, MSC_TIMESTAMP, ts.tv_sec);
	input_event(client_data->input, EV_MSC, MSC_TIMESTAMP, ts.tv_nsec);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.z);

	input_sync(client_data->input);

	store_acc_boot_sample(client_data, accel_data.x,
			accel_data.y, accel_data.z, ts);
}
#endif

#ifdef CONFIG_SMI230_DATA_SYNC
static void smi230_data_sync_ready_handle(
		struct smi230_client_data *client_data)
{
	struct smi230_sensor_data accel_data;
	struct smi230_sensor_data gyro_data;
	struct smi230_sensor_data accel_sync;
	int err = 0;

	err = smi230_acc_get_data(&accel_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;
	err = smi230_get_synchronized_data(&accel_sync, &gyro_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;

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
#endif

#ifdef CONFIG_SMI230_ACC_FIFO
static struct smi230_sensor_data fifo_accel_data[SMI230_MAX_ACC_FIFO_FRAME];

static void smi230_acc_fifo_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_fifo_frame fifo;
	int err = 0, i;
	uint16_t fifo_length;

	PINFO("ACC FIFO dump!");

	err = smi230_acc_get_fifo_length(&fifo.length, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("FIFO get length error!");
		return;
	}

	fifo.data = fifo_buf;
	err = smi230_acc_read_fifo_data(&fifo, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("FIFO read data error!");
		return;
	}

	/* this event here is to indicate IRQ timing*/
	input_event(client_data->input, EV_MSC, MSC_RAW, (int)fifo.length);
	input_sync(client_data->input);

	/* make sure all frames are read out,
	 * the actual frame numbers will be returned
	 * through fifo_length itself*/
	fifo_length = SMI230_MAX_ACC_FIFO_FRAME;
	err = smi230_acc_extract_accel(fifo_accel_data,
                            &fifo_length,
                            &fifo,
                            p_smi230_dev);

	for (i = 0; i < fifo_length; i++) {
		input_event(client_data->input, EV_ABS, ABS_X, (int)fifo_accel_data[i].x);
		input_event(client_data->input, EV_ABS, ABS_Y, (int)fifo_accel_data[i].y);
		input_event(client_data->input, EV_ABS, ABS_Z, (int)fifo_accel_data[i].z);

		input_sync(client_data->input);
	}
}
#endif

uint64_t smi230_acc_get_alarm_timestamp(void)
{
	uint64_t ts_ap;
	struct timespec tmp_time;

	get_monotonic_boottime(&tmp_time);
	ts_ap = (uint64_t)tmp_time.tv_sec * 1000000000 + tmp_time.tv_nsec;
	return ts_ap;
}

static void smi230_irq_work_func(struct work_struct *work)
{
	struct smi230_client_data *client_data =
		container_of(work, struct smi230_client_data, irq_work);

	/* int status reg is not available to tell the interupt source */
#ifdef CONFIG_SMI230_RAW_DATA
	smi230_raw_data_ready_handle(client_data);
#endif
#ifdef CONFIG_SMI230_ACC_FIFO
	smi230_acc_fifo_handle(client_data);
#endif
#ifdef CONFIG_SMI230_DATA_SYNC
	smi230_data_sync_ready_handle(client_data);
#endif
}

static irqreturn_t smi230_irq_handle(int irq, void *handle)
{
	struct smi230_client_data *client_data = handle;
	int err = 0;

	client_data->timestamp = smi230_acc_get_alarm_timestamp();

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
			SENSOR_ACC_NAME, client_data);
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

int smi230_acc_remove(struct device *dev)
{
	int err = 0;
	struct smi230_client_data *client_data = dev_get_drvdata(dev);

	if (NULL != client_data) {
		smi230_acc_input_cleanup(client_data);
		smi230_free_irq(client_data);
		sysfs_remove_group(&client_data->input->dev.kobj,
				&smi230_attribute_group);
		smi230_input_destroy(client_data);
		kfree(client_data);
	}
	return err;
}

int smi230_acc_probe(struct device *dev, struct smi230_dev *smi230_dev)
{
	int err = 0;
	struct smi230_client_data *client_data = NULL;
#ifdef CONFIG_SMI230_DATA_SYNC
	struct smi230_data_sync_cfg sync_cfg;
#endif
#ifdef CONFIG_SMI230_ACC_FIFO
	struct accel_fifo_config fifo_config;
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

	/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
	err |= smi230_acc_soft_reset(p_smi230_dev);

	/*! Max read/write length (maximum supported length is 32).
	 To be set by the user */
	/*set accel power mode */
	p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	err |= smi230_acc_set_power_mode(p_smi230_dev);

#ifdef CONFIG_SMI230_RAW_DATA
	PINFO("ACC raw data is enabled");

	p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
	p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_2G;

        err |= smi230_acc_set_meas_conf(p_smi230_dev);

	smi230_delay(100);

	/*disable acc int1*/
	int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = SMI230_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;

	/*configure int2 as accel raw data ready interrupt pin */
	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = SMI230_ACCEL_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	err |= smi230_acc_set_int_config(&int_config.accel_int_config_1, p_smi230_dev);
	err |= smi230_acc_set_int_config(&int_config.accel_int_config_2, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("ACC raw data init failed");
		goto exit_free_client_data;
	}

	smi230_delay(100);
#endif

#ifdef CONFIG_SMI230_ACC_FIFO
	PINFO("ACC FIFO is enabled");

	p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
	p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
        err |= smi230_acc_set_meas_conf(p_smi230_dev);

	smi230_delay(100);

	/*disable acc int1*/
	int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = SMI230_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;

	/*configure int2 as accel FIFO interrupt pin */
	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = SMI230_FIFO_WM_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	err |= smi230_acc_set_int_config(&int_config.accel_int_config_1, p_smi230_dev);
	err |= smi230_acc_set_int_config(&int_config.accel_int_config_2, p_smi230_dev);

	err |= smi230_acc_set_fifo_wm(700, p_smi230_dev);

	fifo_config.mode = SMI230_ACC_FIFO_MODE;
	fifo_config.accel_en = 1;
	err |= smi230_acc_set_fifo_config(&fifo_config, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("FIFO HW init failed");
		goto exit_free_client_data;
	}

	smi230_delay(100);
#endif

#ifdef CONFIG_SMI230_DATA_SYNC
	PINFO("DATA sync is enabled");
	/* API uploads the smi230 config file onto the device and wait for 150ms 
	   to enable the data synchronization - delay taken care inside the function */
	err |= smi230_apply_config_file(p_smi230_dev);
        if (err == SMI230_OK)
		PINFO("Configuration file transfer succeed!");

	else {
		PERR("Configuration file transfer error!");
		goto exit_free_client_data;
	}

	/*assign accel range setting*/
	p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;
	/*assign gyro range setting*/
	p_smi230_dev->gyro_cfg.range = SMI230_GYRO_RANGE_2000_DPS;
	/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
	sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_2000HZ;
	err |= smi230_configure_data_synchronization(sync_cfg, p_smi230_dev);


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
	err |= smi230_set_data_sync_int_config(&int_config, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("Data sync HW init failed");
		goto exit_free_client_data;
	}
#endif

#ifndef CONFIG_SMI230_DEBUG 
	/*if not for the convenience of debuging, sensors should be disabled at startup*/
	p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	err |= smi230_acc_set_power_mode(p_smi230_dev);
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
	err = smi230_request_irq(client_data);
	if (err < 0) {
		PERR("Request irq failed");
		goto exit_cleanup_sysfs;
	}

	err = smi230_acc_early_buff_init(client_data);
	if (!err)
		return err;

	PINFO("Sensor %s was probed successfully", SENSOR_ACC_NAME);

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
