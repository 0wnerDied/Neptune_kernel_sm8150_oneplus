# SPDX-License-Identifier: GPL-2.0

EROFS_VERSION = "1.1 (190701)"

ccflags-y += -Wall -DEROFS_VERSION=\"$(EROFS_VERSION)\"

obj-$(CONFIG_EROFS_FS) += erofs.o
# staging requirement: to be self-contained in its own directory
ccflags-y += -I$(src)/include
erofs-objs := super.o inode.o data.o namei.o dir.o utils.o
erofs-$(CONFIG_EROFS_FS_XATTR) += xattr.o
erofs-$(CONFIG_EROFS_FS_ZIP) += lz4.o
erofs-$(CONFIG_EROFS_FS_ZIP) += unzip_vle.o decompressor.o zmap.o
erofs-$(CONFIG_ARM64) += $(addprefix lz4armv8/, \
                          lz4accel.o           \
                          lz4armv8.o           \
                          )
