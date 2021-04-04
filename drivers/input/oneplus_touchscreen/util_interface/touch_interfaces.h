/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : touch_interfaces.h
*
* Function	  : touchpanel public interface
* 
* Version	  : V1.0
*
***********************************************************/

#ifndef TOUCH_INTERFACES_H
#define TOUCH_INTERFACES_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>

#define MAX_I2C_RETRY_TIME 2

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)
#define DUMMY_BYTES (1)
#define SPI_TANSFER_LEN		512

typedef enum {
	SPIWRITE = 1,
	SPIREAD = 2
} SPI_RW;

struct touch_dma_buf {
	unsigned char read_buf[1];
	unsigned char read_byte_buf[2];
	unsigned char read_word_buf[2];
	unsigned char write_buf[32];
};

#define TPD_I2C_INFO(a, arg...)  pr_err("[TP]touch_interface: " a, ##arg)

// Initialized from touchpanel_common_driver.c
extern struct touch_dma_buf *i2c_dma_buffer;

/**
 * touch_i2c_read_block - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @length: data size we want to send
 * @data: data we want to send
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer length(transfer success) or most likely negative errno(transfer error)
 */
static inline int touch_i2c_read_block(struct i2c_client *client, u16 addr,
			 unsigned short length, unsigned char *data)
{
	int retval;
	unsigned char retry;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = i2c_dma_buffer->read_buf;
	msg[0].buf[0] = addr & 0xff;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = data;

	for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
		if (likely(i2c_transfer(client->adapter, msg, 2) == 2))
			return length;

		msleep(20);
	}
	if (retry == MAX_I2C_RETRY_TIME) {
		dev_err(&client->dev, "%s: I2C read over retry limit\n",
			__func__);
		retval = -EIO;
	}
	return retval;
}

/**
 * touch_i2c_write_block - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @length: data size we want to send
 * @data: data we want to send
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer length(transfer success) or most likely negative errno(transfer error)
 */
static inline int touch_i2c_write_block(struct i2c_client *client, u16 addr,
			  unsigned short length, unsigned char const *data)
{
	int retval;
	unsigned char retry;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = i2c_dma_buffer->write_buf;
	msg[0].len = length + 1;
	msg[0].buf[0] = addr & 0xff;

	memcpy(i2c_dma_buffer->write_buf + 1, data, length);

	for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
		if (likely(i2c_transfer(client->adapter, msg, 1) == 1))
			return length;

		msleep(20);
	}
	if (retry == MAX_I2C_RETRY_TIME) {
		dev_err(&client->dev, "%s: I2C write over retry limit\n",
			__func__);
		retval = -EIO;
	}

	return retval;
}

/**
 * touch_i2c_read_byte - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to read
 *
 * Actully, This function call touch_i2c_read_block for IIC transfer,
 * Returning zero(transfer success) or most likely negative errno(transfer error)
 */
static inline int touch_i2c_read_byte(struct i2c_client *client, unsigned short addr)
{
	int retval = 0;
	unsigned char *buf = i2c_dma_buffer->read_byte_buf;

	if (unlikely(!client)) {
		dump_stack();
		return -1;
	}

	retval = touch_i2c_read_block(client, addr, 1, buf);
	if (likely(retval == 1))
		retval = buf[0] & 0xff;

	return retval;
}

/**
 * touch_i2c_write_byte - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @data: data we want to send
 *
 * Actully, This function call touch_i2c_write_block for IIC transfer,
 * Returning zero(transfer success) or most likely negative errno(transfer error)
 */
static inline int touch_i2c_write_byte(struct i2c_client *client, unsigned short addr,
			 unsigned char data)
{
	int retval;
	unsigned char data_send = data;

	if (unlikely(!client)) {
		dump_stack();
		return -EINVAL;
	}

	retval = touch_i2c_write_block(client, addr, 1, &data_send);
	if (likely(retval == 1))
		retval = 0;

	return retval;
}

/**
 * touch_i2c_read_word - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @data: data we want to read
 *
 * Actully, This func call touch_i2c_Read_block for IIC transfer,
 * Returning negative errno else a 16-bit unsigned "word" received from the device.
 */
static inline int touch_i2c_read_word(struct i2c_client *client, unsigned short addr)
{
	int retval;
	unsigned char *buf = i2c_dma_buffer->read_word_buf;

	if (unlikely(!client)) {
		dump_stack();
		return -EINVAL;
	}

	retval = touch_i2c_read_block(client, addr, 2, buf);
	if (likely(retval >= 0))
		retval = buf[1] << 8 | buf[0];

	return retval;
}

/**
 * touch_i2c_write_word - Using for "read word" through IIC
 * @client: Handle to slave device
 * @addr: addr to write
 * @data: data we want to send
 *
 * Actully, This function call touch_i2c_write_block for IIC transfer,
 * Returning zero(transfer success) or most likely negative errno(transfer error)
 */
static inline int touch_i2c_write_word(struct i2c_client *client, unsigned short addr,
			 unsigned short data)
{
	int retval;
	unsigned char buf[2];

	if (unlikely(!client)) {
		dump_stack();
		return -EINVAL;
	}

	buf[0] = data & 0xff;
	buf[1] = (data >> 8) & 0xff;

	retval = touch_i2c_write_block(client, addr, 2, buf);
	if (likely(retval == 2))
		retval = 0;

	return retval;
}

/**
 * touch_i2c_read - Using for "read data from ic after writing or not" through IIC
 * @client: Handle to slave device
 * @writebuf: buf to write
 * @writelen: data size we want to send
 * @readbuf:  buf we want save data
 * @readlen:  data size we want to receive
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer msg length(transfer success) or most likely negative errno(transfer EIO error)
 */
static inline int touch_i2c_read(struct i2c_client *client, char *writebuf, int writelen,
		   char *readbuf, int readlen)
{
	int retval = 0;
	int retry = 0;

	if (unlikely(client == NULL)) {
		TPD_I2C_INFO("%s: i2c_client == NULL!\n", __func__);
		return -1;
	}

	if (likely(readlen > 0)) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
				 },
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};

			for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
				if (likely(i2c_transfer(client->adapter, msgs, 2) == 2))
					return 2;

				msleep(20);
			}
		} else {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};

			for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
				if (likely(i2c_transfer(client->adapter, msgs, 1) == 1))
					return 1;

				msleep(20);
			}
		}

		if (retry == MAX_I2C_RETRY_TIME) {
			TPD_I2C_INFO("%s: i2c_transfer(read) over retry limit\n",
				 __func__);
			retval = -EIO;
		}
	}

	return retval;
}

/**
 * touch_i2c_write - Using for "write data to ic" through IIC
 * @client: Handle to slave device
 * @writebuf: buf data wo want to send
 * @writelen: data size we want to send
 *
 * Actully, This function call i2c_transfer for IIC transfer,
 * Returning transfer msg length(transfer success) or most likely negative errno(transfer EIO error)
 */
static inline int touch_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int retval = 0;
	int retry = 0;

	if (unlikely(client == NULL)) {
		TPD_I2C_INFO("%s: i2c_client == NULL!", __func__);
		return -1;
	}

	if (likely(writelen > 0)) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
		};

		for (retry = 0; retry < MAX_I2C_RETRY_TIME; retry++) {
			if (likely(i2c_transfer(client->adapter, msgs, 1) == 1))
				return 1;

			msleep(20);
		}
		if (retry == MAX_I2C_RETRY_TIME) {
			TPD_I2C_INFO("%s: i2c_transfer(write) over retry limit\n",
				 __func__);
			retval = -EIO;
		}
	}

	return retval;
}
#endif
