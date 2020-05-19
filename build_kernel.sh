#!/bin/bash

export ARCH=arm64
export SUBARCH=arm64
export CLANG_PATH=/home/zhulitao/clang/bin
export PATH=${CLANG_PATH}:${PATH}
export CLANG_TRIPLE=aarch64-linux-gnu-
export CROSS_COMPILE=/home/zhulitao/clang/bin/aarch64-linux-gnu-
export CROSS_COMPILE_ARM32=/home/zhulitao/clang/bin/arm-linux-gnueabi-
export LD_LIBRARY_PATH=/home/zhulitao/clang/lib:$LD_LIBRARY_PATH
export KBUILD_BUILD_HOST=Neptune

echo
echo "Setting defconfig"
echo
# cp defconfig .config
make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip weeb_defconfig

echo
echo "Compiling kernel"
echo 

make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip -j$(nproc --all) || exit 1