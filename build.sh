#!/bin/bash

# HOME path
export HOME=/home/Neptune

# Compiler environment
export CLANG_PATH=$HOME/llvm-18.1.0-rc1-x86_64/bin
export PATH="$CLANG_PATH:$PATH"
export CROSS_COMPILE=$HOME/gcc64/bin/aarch64-linux-android-
export CROSS_COMPILE_ARM32=$HOME/gcc32/bin/arm-linux-androideabi-
export CLANG_TRIPLE=aarch64-linux-gnu-
export KBUILD_BUILD_USER=0wnerDied
export KBUILD_BUILD_HOST=Neptune

echo
echo "Setting defconfig"
echo

make CC=clang LD=ld.lld AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip neptune_defconfig

echo
echo "Compiling kernel"
echo

make CC=clang LD=ld.lld AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip -j$(nproc --all) || exit 1
