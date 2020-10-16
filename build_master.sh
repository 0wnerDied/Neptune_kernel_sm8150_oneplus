#!/bin/bash

MIN_HEAD=$(git rev-parse HEAD)
VERSION="$(cat version)-$(date +%Y%m%d)-$(echo ${MIN_HEAD:0:8})"

export LOCALVERSION="-NeptuneKernel-$(echo "${VERSION}")"

if [[ "${1}" != "skip" ]] ; then
	./build_clean.sh
fi

./build_kernel.sh || exit 1

if [ -e arch/arm64/boot/Image.gz ] ; then
	echo
	echo "Building Kernel Package"
	echo
	rm NeptuneKernel-$VERSION.zip 2>/dev/null
	rm -rf kernelzip 2>/dev/null
	mkdir kernelzip
	cp -rp flasher/* kernelzip/
	find arch/arm64/boot/dts -name '*.dtb' -exec cat {} + > kernelzip/dtb
	cd kernelzip/
	7z a -mx9 NeptuneKernel-$VERSION-tmp.zip *
	7z a -mx0 NeptuneKernel-$VERSION-tmp.zip ../arch/arm64/boot/Image.gz
	zipalign -v 4 NeptuneKernel-$VERSION-tmp.zip ../NeptuneKernel-$VERSION.zip
	rm NeptuneKernel-$VERSION-tmp.zip
	cd ..
	ls -al NeptuneKernel-$VERSION.zip
fi