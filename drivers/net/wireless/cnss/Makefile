# Makefile for CNSS platform driver

ifeq ($(CONFIG_CNSS),y)
obj-$(CONFIG_CNSS_PCI)	+= cnss_pci.o
obj-$(CONFIG_CNSS_SDIO)	+= cnss_sdio.o
obj-$(CONFIG_CNSS)	+= cnss_common.o
endif

# Add sdio cnss dlkm support, just only support cnss_sdio,
# and need to extend if adding pci cnss dlmk support.
ifeq ($(CONFIG_CNSS),m)
ifeq ($(CONFIG_CNSS_SDIO),y)
obj-$(CONFIG_CNSS) += cnss.o
cnss-y := cnss_sdio.o
cnss-y += cnss_common.o
ccflags-y += -DCONFIG_CNSS -DCNSS_SDIO_DLKM
endif
endif

obj-$(CONFIG_CNSS_LOGGER)	+= logger/
