#!/bin/bash

export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=/home/zhulitao/aarch64-linux-elf/bin/aarch64-linux-elf-
export CROSS_COMPILE_ARM32=/home/zhulitao/arm-linux-eabi/bin/arm-linux-eabi-

echo
echo "Setting defconfig"
echo

make weeb_defconfig

echo
echo "Compiling kernel"
echo 

make "$@" || exit 1