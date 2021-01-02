#!/bin/bash

# HOME path
export HOME=/home/zhulitao

# Compiler environment
export ARCH=arm64
export SUBARCH=arm64
export CLANG_PATH=$HOME/clang/bin
export PATH="$CLANG_PATH:$PATH"
export CROSS_COMPILE=aarch64-linux-gnu-
export CROSS_COMPILE_ARM32=arm-linux-gnueabi-
export LD_LIBRARY_PATH=lib:$LD_LIBRARY_PATH
export KBUILD_BUILD_USER=Vwool0xE9
export KBUILD_BUILD_HOST=Atndko

echo
echo "Setting defconfig"
echo

./generator ramdisk/init.qcom.post_boot.sh init/execprog.h

make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip neptune_defconfig

echo
echo "Compiling kernel"
echo

make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip -j$(nproc --all) || exit 1
