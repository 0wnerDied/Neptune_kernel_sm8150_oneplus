#!/bin/bash

git clean -fdx
git reset --hard

cp defconfig_gcc .config