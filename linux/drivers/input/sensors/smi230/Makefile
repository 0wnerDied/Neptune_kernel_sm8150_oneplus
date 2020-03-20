#
# Makefile for Bosch sensor driver.
#

ifneq ($(CONFIG_BOSCH_DRIVER_LOG_FUNC),n)
	EXTRA_CFLAGS += -DBOSCH_DRIVER_LOG_FUNC
endif

EXTRA_CFLAGS += -DSMI230_GYRO_USE_BASIC_I2C_FUNC

#so far, only i2c supported, spi is not implemetned yet
ifeq ($(CONFIG_SMI230_I2C),y)

obj-$(CONFIG_SMI230_DRIVER)    += smi230.o
smi230-objs := smi230_driver.o smi230_log.o smi230_data_sync.o smi230_acc.o smi230_gyro.o

endif
