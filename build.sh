#!/bin/bash

# HOME path
export HOME=/home/atndko

# Compiler environment
export CLANG_PATH=$HOME/linux-x86/clang-r437112/bin
export PATH="$CLANG_PATH:$PATH"
export CROSS_COMPILE=$HOME/gcc64/bin/aarch64-linux-android-
export CROSS_COMPILE_ARM32=$HOME/gcc32/bin/arm-linux-androideabi-
export CLANG_TRIPLE=aarch64-linux-gnu-
export KBUILD_BUILD_USER=Vwool0xE9
export KBUILD_BUILD_HOST=Atndko

echo
echo "Setting defconfig"
echo

make CC=clang LD=ld.lld AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip neptune_defconfig

echo
echo "Compiling kernel"
echo

make CC=clang LD=ld.lld AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip -j$(nproc --all) || exit 1
