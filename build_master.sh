#!/bin/bash

KNAME="NeptuneKernel"
MIN_HEAD=$(git rev-parse HEAD)
VERSION="$(cat version)-$(date +%Y%m%d)-$(echo ${MIN_HEAD:0:8})"
ZIPNAME="${KNAME}-$(cat version)-$(echo ${MIN_HEAD:0:8})"

export LOCALVERSION="-${KNAME}-$(echo "${VERSION}")"

if [[ "${1}" != "skip" ]] ; then
	./build_clean.sh
fi

./build_kernel.sh || exit 1

if [ -e arch/arm64/boot/Image.gz ] ; then
	echo
	echo "Building Kernel Package"
	echo
	rm $ZIPNAME.zip 2>/dev/null
	rm -rf kernelzip 2>/dev/null
	mkdir kernelzip
	echo "kernel.string=Neptune Kernel $(cat version) by Ciel_U.T
do.devicecheck=1
do.modules=0
do.systemless=1
do.cleanup=1
do.cleanuponabort=0
device.name1=OnePlus7
device.name2=guacamoleb
device.name3=OnePlus7Pro
device.name4=guacamole
device.name5=OnePlus7ProTMO
device.name6=guacamolet
device.name7=OnePlus7T
device.name8=hotdogb
device.name9=OnePlus7TPro
device.name10=hotdog
device.name11=OnePlus7TProNR
device.name12=hotdogg
supported.versions=10
block=/dev/block/bootdevice/by-name/boot;
is_slot_device=1;
ramdisk_compression=auto;" > kernelzip/props
	cp -rp flasher/* kernelzip/
	find arch/arm64/boot/dts -name '*.dtb' -exec cat {} + > kernelzip/dtb
	cd kernelzip/
	7z a -mx9 $ZIPNAME-tmp.zip *
	7z a -mx0 $ZIPNAME-tmp.zip ../arch/arm64/boot/Image.gz
	zipalign -v 4 $ZIPNAME-tmp.zip ../$ZIPNAME.zip
	rm $ZIPNAME-tmp.zip
	cd ..
	ls -al $ZIPNAME.zip
fi