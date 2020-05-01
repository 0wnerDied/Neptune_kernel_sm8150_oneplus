#!/bin/bash

VERSION="$(cat version)"

if [[ "${1}" != "skip" ]] ; then
	./build_clean.sh
fi

./build_kernel.sh || exit 1

if [ -e arch/arm64/boot/Image.gz ] ; then
	echo
	echo "Building Kernel Package"
	echo
	rm HentaiKernel-$VERSION.zip 2>/dev/null
	rm -rf kernelzip 2>/dev/null
	mkdir kernelzip
	cp -rp /home/zhulitao/anykernel/* kernelzip/
	find arch/arm64/boot/dts -name '*.dtb' -exec cat {} + > kernelzip/dtb
	cd kernelzip/
	7z a -mx9 HentaiKernel-$VERSION-tmp.zip *
	7z a -mx0 HentaiKernel-$VERSION-tmp.zip ../arch/arm64/boot/Image.gz
	zipalign -v 4 HentaiKernel-$VERSION-tmp.zip ../HentaiKernel-$VERSION.zip
	rm HentaiKernel-$VERSION-tmp.zip
	cd ..
	ls -al HentaiKernel-$VERSION.zip
fi