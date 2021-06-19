#!/bin/bash

# HOME path
export HOME=/home/atndko

# Compiler environment
export CROSS_COMPILE=$HOME/gcc-arm64/bin/aarch64-elf-
export CROSS_COMPILE_ARM32=$HOME/gcc-arm/bin/arm-eabi-
export KBUILD_BUILD_USER=Vwool0xE9
export KBUILD_BUILD_HOST=Atndko

echo
echo "Setting defconfig"
echo

make neptune_defconfig

echo
echo "Compiling kernel"
echo

make -j$(nproc --all) || exit 1
