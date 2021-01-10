#!/bin/bash

git clean -fdx
git reset --hard

cp ./arch/arm64/configs/neptune_defconfig .config
