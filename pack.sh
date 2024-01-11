#!/bin/bash

# Kernel version configuration
KNAME="NoahKernel"
MIN_HEAD=$(git rev-parse HEAD)
VERSION="$(cat version)-$(date +%m.%d.%y)-$(echo ${MIN_HEAD:0:8})"
ZIPNAME="${KNAME}-$(cat version)-$(echo ${MIN_HEAD:0:8})"

export LOCALVERSION="-${KNAME}-$(echo "${VERSION}")"

# Never dirty compile
if [[ "${1}" != "skip" ]] ; then
	./clean.sh
fi

# Let's build
START=$(date +"%s")
./build.sh || exit 1

# Kernel Output
if [ -e arch/arm64/boot/Image.gz ] ; then
	echo
	echo "Building Kernel Package"
	echo
	rm $ZIPNAME.zip 2>/dev/null
	rm -rf kernelzip 2>/dev/null
	# Import Anykernel3 folder
	mkdir kernelzip
	cp -rp scripts/AnyKernel3/* kernelzip/
	find arch/arm64/boot/dts -name '*.dtb' -exec cat {} + > kernelzip/dtb
	cd kernelzip/
	7z a -mx9 $ZIPNAME-tmp.zip *
	7z a -mx0 $ZIPNAME-tmp.zip ../arch/arm64/boot/Image.gz
	zipalign -v 4 $ZIPNAME-tmp.zip ../$ZIPNAME.zip
	rm $ZIPNAME-tmp.zip
	cd ..
	ls -al $ZIPNAME.zip
fi

# Show compilation time
END=$(date +"%s")
DIFF=$((END - START))
echo -e "Kernel compiled successfully in $((DIFF / 60)) minute(s) and $((DIFF % 60)) second(s)"

# Copy the kernel to Windows file
cp $ZIPNAME.zip /mnt/d/kernel/
