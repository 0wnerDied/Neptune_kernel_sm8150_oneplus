# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
properties() { '
kernel.string=NeptuneKernel by Vwool0xE9
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
supported.versions=11 - 12
'; } # end properties

# shell variables
block=/dev/block/by-name/boot;
is_slot_device=1;
ramdisk_compression=auto;

## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. tools/ak3-core.sh;

android_version="$(file_getprop /system/build.prop "ro.build.version.release")";

# Detect device and system
if [ -e /system/etc/buildinfo/oem_build.prop ]; then
  os="stock";
  os_string="OxygenOS/HydrogenOS";
else
  os="custom";
  os_string="a custom ROM";
fi
ui_print " " "You are on $os_string!";

if [ $os == "custom" ]; then
  if [ -f $home/Image.gz ]; then
    mv $home/Image.gz $home/Image.gz-dtb;
  fi;
  if [ -f $home/dtb ]; then
    cat $home/dtb >> $home/Image.gz-dtb;
  fi;
fi;

## AnyKernel install
dump_boot;

# Unified with custom ROMs
if [ $os == "custom" ]; then
  patch_cmdline "msm_drm.is_stock" "msm_drm.is_stock=0"
  if [ $android_version == "11" ]; then
    patch_cmdline "msm_drm.is_a12" "msm_drm.is_a12=0"
  else
    patch_cmdline "msm_drm.is_a12" "msm_drm.is_a12=1"
  fi
else
  patch_cmdline "msm_drm.is_stock" "msm_drm.is_stock=1"
fi

# Override DTB
if [ $os == "stock" ]; then
  mv $home/dtb $home/split_img/;
fi

# Move resetprop_static
cp -rfp $home/resetprop_static /data/local/tmp/resetprop_static;
chmod 755 /data/local/tmp/resetprop_static;

if mountpoint -q /data; then
  # Optimize F2FS extension list (@arter97)
  for list_path in $(find /sys/fs/f2fs* -name extension_list); do
    hash="$(md5sum $list_path | sed 's/extenstion/extension/g' | cut -d' ' -f1)"

    # Skip update if our list is already active
    if [[ $hash == "43df40d20dcb96aa7e8af0e3d557d086" ]]; then
      echo "Extension list up-to-date: $list_path"
      continue
    fi

    ui_print " " "Optimizing F2FS extension list..."
    echo "Updating extension list: $list_path"

    echo "Clearing extension list"

    hot_count="$(grep -n 'hot file extens' $list_path | cut -d':' -f1)"
    list_len="$(cat $list_path | wc -l)"
    cold_count="$((list_len - hot_count))"

    cold_list="$(head -n$((hot_count - 1)) $list_path | grep -v ':')"
    hot_list="$(tail -n$cold_count $list_path)"

    for ext in $cold_list; do
      [ ! -z $ext ] && echo "[c]!$ext" > $list_path
    done

    for ext in $hot_list; do
      [ ! -z $ext ] && echo "[h]!$ext" > $list_path
    done

    echo "Writing new extension list"

    for ext in $(cat $home/f2fs-cold.list | grep -v '#'); do
      [ ! -z $ext ] && echo "[c]$ext" > $list_path
    done

    for ext in $(cat $home/f2fs-hot.list); do
      [ ! -z $ext ] && echo "[h]$ext" > $list_path
    done
  done
fi

# Install the boot image
write_boot;

## end install
