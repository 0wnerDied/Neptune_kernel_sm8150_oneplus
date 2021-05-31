#
# Makefile for Bosch sensor driver.
#

ifneq ($(CONFIG_SMI230_BOSCH_DRIVER_LOG_FUNC),n)
	EXTRA_CFLAGS += -DBOSCH_DRIVER_LOG_FUNC
endif


obj-$(CONFIG_INPUT_SMI230)    += smi230.o
smi230-objs := smi230_log.o smi230_data_sync.o smi230_acc.o smi230_gyro.o

ifeq ($(CONFIG_SMI230_ACC_DRIVER),y)
	smi230-objs += smi230_acc_driver.o
endif

ifeq ($(CONFIG_SMI230_GYRO_DRIVER),y)
	smi230-objs += smi230_gyro_driver.o
endif

ifeq ($(CONFIG_SMI230_I2C),y)
	smi230-objs += smi230_i2c_driver.o
else
	smi230-objs += smi230_spi_driver.o
endif
