# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers

# set up working directory variables
test "$home" || home=$PWD;
bootimg=$home/boot.img;
bin=$home/tools;
patch=$home/patch;
ramdisk=$home/ramdisk;
split_img=$home/split_img;

## AnyKernel setup
eval $(cat $home/props | grep -v '\.')

## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. tools/ak3-core.sh;

# Detect device and system
hotdog="$(grep -wom 1 hotdog*.* /system/build.prop | sed 's/.....$//')";
guacamole="$(grep -wom 1 guacamole*.* /system/build.prop | sed 's/.....$//')";
userflavor="$(file_getprop /system/build.prop "ro.build.user"):$(file_getprop /system/build.prop "ro.build.flavor")";
userflavor2="$(file_getprop2 /system/build.prop "ro.build.user"):$(file_getprop2 /system/build.prop "ro.build.flavor")";
if [ "$userflavor" == "jenkins:$hotdog-user" ] || [ "$userflavor2" == "jenkins:$guacamole-user" ]; then
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

# Override DTB
if [ $os == "stock" ]; then
  mv $home/dtb $home/split_img/;
fi

# Clean up existing ramdisk overlays
rm -rf $ramdisk/overlay;
rm -rf $ramdisk/overlay.d;

# Choose whether or not remove limit to learn battery capacity
case "$ZIPFILE" in
  *BATTERY*)
    ui_print " " "Removing limit to learn battery capacity..."
    patch_cmdline "battery_capacity.remove_op_capacity" "battery_capacity.remove_op_capacity=1"
    ;;
  *)
    ui_print " " "Keeping the limit for learning battery capacity..."
    patch_cmdline "battery_capacity.remove_op_capacity" "battery_capacity.remove_op_capacity=0"
    ;;
esac

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

write_boot;
## end install
