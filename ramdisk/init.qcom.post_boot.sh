#! /vendor/bin/sh

exec > /dev/kmsg 2>&1

if ! grep -v '#' /vendor/etc/fstab.qcom | grep -q f2fs; then
  # ECD18g== is the f2fs magic code under little-endian
  if [[ $(dd if=/dev/block/platform/soc/1d84000.ufshc/by-name/userdata bs=4 skip=256 count=1 2>/dev/null | base64) == "ECD18g==" ]]; then
    # fstab is missing entry for f2fs, add one
    sed -e "s@/dev/block/bootdevice/by-name/userdata.*@$(cat /vendor/etc/fstab.qcom | grep ext4 | grep /data | grep -v '#' | while read a b c d e; do echo $a $b f2fs noatime,nosuid,nodev,discard,fsync_mode=nobarrier latemount,wait,check,encryptable=ice,wrappedkey,keydirectory=/metadata/vold/metadata_encryption,quota,formattable,reservedsize=128M; done)@g" /vendor/etc/fstab.qcom | uniq > /dev/fstab.qcom
    chmod 644 /dev/fstab.qcom
    mount --bind /dev/fstab.qcom /vendor/etc/fstab.qcom
    chcon u:object_r:vendor_configs_file:s0 /vendor/etc/fstab.qcom
    cat /dev/fstab.qcom | while read a; do echo $a; done
    echo "Patched /vendor/etc/fstab.qcom for f2fs"
  fi
fi

if ! mount | grep -q /vendor/bin/init.qcom.post_boot.sh && [ ! -f /dev/ep/execprog ]; then
  # Run under a new tmpfs to avoid /dev selabel
  mkdir /dev/ep
  mount -t tmpfs nodev /dev/ep
  cp -p "$0" /dev/ep/execprog
  rm "$0"
  chown root:shell /dev/ep/execprog
  exec /dev/ep/execprog
fi

if ! mount | grep -q /vendor/bin/init.qcom.post_boot.sh && [ ! -f /sbin/recovery ] && [ ! -f /dev/ep/.post_boot ]; then
  # Run once
  touch /dev/ep/.post_boot

  # Disable Houston and cc_ctl
  mount --bind /dev/ep/.post_boot /system/priv-app/Houston/Houston.apk
  mount --bind /dev/ep/.post_boot /system/priv-app/OPAppCategoryProvider/OPAppCategoryProvider.apk

  chmod 755 "$0"

  # Setup swap
  while [ ! -e /dev/block/vbswap0 ]; do
    sleep 1
  done
  if ! grep -q vbswap /proc/swaps; then
    # 4GB
    echo 4294967296 > /sys/devices/virtual/block/vbswap0/disksize

    # Set swappiness reflecting the device's RAM size
    RamStr=$(cat /proc/meminfo | grep MemTotal)
    RamMB=$((${RamStr:16:8} / 1024))
    if [ $RamMB -le 6144 ]; then
        echo 200 > /proc/sys/vm/rswappiness
    elif [ $RamMB -le 8192 ]; then
        echo 160 > /proc/sys/vm/rswappiness
    else
        echo 130 > /proc/sys/vm/rswappiness
    fi

    mkswap /dev/block/vbswap0
    swapon /dev/block/vbswap0
  fi

  # Hook up to existing init.qcom.post_boot.sh
  # Replace msm_irqbalance.conf
  echo "PRIO=1,1,1,1,0,0,0,0
# arch_timer,arch_mem_timer,arm-pmu,msm_drm,kgsl-3d0,glink_lpass
IGNORED_IRQ=19,38,21,115,332,188" > /dev/ep/msm_irqbalance.conf
  chmod 644 /dev/ep/msm_irqbalance.conf
  mount --bind /dev/ep/msm_irqbalance.conf /vendor/etc/msm_irqbalance.conf
  chcon "u:object_r:vendor_configs_file:s0" /vendor/etc/msm_irqbalance.conf
  killall msm_irqbalance

  mount --bind "$0" /vendor/bin/init.qcom.post_boot.sh
  chcon "u:object_r:qti_init_shell_exec:s0" /vendor/bin/init.qcom.post_boot.sh

  # lazy unmount /dev/ep for invisibility
  umount -l /dev/ep

  exit
fi

# Check ROM
file_getprop() { grep "^$2=" "$1" | cut -d= -f2-; }
file_getprop2() { grep "^$2=" "$1" | cut -d= -f2- | sed -n 2p; }
hotdog="$(grep -wom 1 hotdog*.* /system/build.prop | sed 's/.....$//')";
guacamole="$(grep -wom 1 guacamole*.* /system/build.prop | sed 's/.....$//')";
userflavor="$(file_getprop /system/build.prop "ro.build.user"):$(file_getprop /system/build.prop "ro.build.flavor")";
userflavor2="$(file_getprop2 /system/build.prop "ro.build.user"):$(file_getprop2 /system/build.prop "ro.build.flavor")";
if [ "$userflavor" == "jenkins:$hotdog-user" ] || [ "$userflavor2" == "jenkins:$guacamole-user" ]; then
  os="stock";
fi

# Disable wsf, beacause we are using efk.
# wsf Range : 1..1000 So set to bare minimum value 1.
echo 1 > /proc/sys/vm/watermark_scale_factor

# Enable bus-dcvs
for device in /sys/devices/platform/soc
do
    for cpubw in $device/*cpu-cpu-llcc-bw/devfreq/*cpu-cpu-llcc-bw
    do
	echo "bw_hwmon" > $cpubw/governor
	echo 40 > $cpubw/polling_interval
	echo "2288 4577 7110 9155 12298 14236 15258" > $cpubw/bw_hwmon/mbps_zones
	echo 4 > $cpubw/bw_hwmon/sample_ms
	echo 50 > $cpubw/bw_hwmon/io_percent
	echo 20 > $cpubw/bw_hwmon/hist_memory
	echo 10 > $cpubw/bw_hwmon/hyst_length
	echo 30 > $cpubw/bw_hwmon/down_thres
	echo 0 > $cpubw/bw_hwmon/guard_band_mbps
	echo 250 > $cpubw/bw_hwmon/up_scale
	echo 1600 > $cpubw/bw_hwmon/idle_mbps
	echo 14236 > $cpubw/max_freq
    done

    for llccbw in $device/*cpu-llcc-ddr-bw/devfreq/*cpu-llcc-ddr-bw
    do
	echo "bw_hwmon" > $llccbw/governor
	echo 40 > $llccbw/polling_interval
	echo "1720 2929 3879 5931 6881 7980" > $llccbw/bw_hwmon/mbps_zones
	echo 4 > $llccbw/bw_hwmon/sample_ms
	echo 80 > $llccbw/bw_hwmon/io_percent
	echo 20 > $llccbw/bw_hwmon/hist_memory
	echo 10 > $llccbw/bw_hwmon/hyst_length
	echo 30 > $llccbw/bw_hwmon/down_thres
	echo 0 > $llccbw/bw_hwmon/guard_band_mbps
	echo 250 > $llccbw/bw_hwmon/up_scale
	echo 1600 > $llccbw/bw_hwmon/idle_mbps
	echo 6881 > $llccbw/max_freq
    done

    for npubw in $device/*npu-npu-ddr-bw/devfreq/*npu-npu-ddr-bw
    do
	echo 1 > /sys/devices/virtual/npu/msm_npu/pwr
	echo "bw_hwmon" > $npubw/governor
	echo 40 > $npubw/polling_interval
	echo "1720 2929 3879 5931 6881 7980" > $npubw/bw_hwmon/mbps_zones
	echo 4 > $npubw/bw_hwmon/sample_ms
	echo 80 > $npubw/bw_hwmon/io_percent
	echo 20 > $npubw/bw_hwmon/hist_memory
	echo 6  > $npubw/bw_hwmon/hyst_length
	echo 30 > $npubw/bw_hwmon/down_thres
	echo 0 > $npubw/bw_hwmon/guard_band_mbps
	echo 250 > $npubw/bw_hwmon/up_scale
	echo 0 > $npubw/bw_hwmon/idle_mbps
	echo 0 > /sys/devices/virtual/npu/msm_npu/pwr
    done

    #Enable mem_latency governor for L3, LLCC, and DDR scaling
    for memlat in $device/*cpu*-lat/devfreq/*cpu*-lat
    do
	echo "mem_latency" > $memlat/governor
	echo 10 > $memlat/polling_interval
	echo 400 > $memlat/mem_latency/ratio_ceil
    done

    #Enable userspace governor for L3 cdsp nodes
    for l3cdsp in $device/*cdsp-cdsp-l3-lat/devfreq/*cdsp-cdsp-l3-lat
    do
	echo "cdspl3" > $l3cdsp/governor
    done

    #Enable compute governor for gold latfloor
    for latfloor in $device/*cpu-ddr-latfloor*/devfreq/*cpu-ddr-latfloor*
    do
	echo "compute" > $latfloor/governor
	echo 10 > $latfloor/polling_interval
    done

    #Gold L3 ratio ceil
    for l3gold in $device/*cpu4-cpu-l3-lat/devfreq/*cpu4-cpu-l3-lat
    do
	echo 4000 > $l3gold/mem_latency/ratio_ceil
    done

    #Prime L3 ratio ceil
    for l3prime in $device/*cpu7-cpu-l3-lat/devfreq/*cpu7-cpu-l3-lat
    do
	echo 20000 > $l3prime/mem_latency/ratio_ceil
    done
done

# Post-setup services
setprop vendor.post_boot.parsed 1

# Let kernel know our image version/variant/crm_version
if [ -f /sys/devices/soc0/select_image ]; then
    image_version="10:"
    image_version+=`getprop ro.build.id`
    image_version+=":"
    image_version+=`getprop ro.build.version.incremental`
    image_variant=`getprop ro.product.name`
    image_variant+="-"
    image_variant+=`getprop ro.build.type`
    oem_version=`getprop ro.build.version.codename`
    echo 10 > /sys/devices/soc0/select_image
    echo $image_version > /sys/devices/soc0/image_version
    echo $image_variant > /sys/devices/soc0/image_variant
    echo $oem_version > /sys/devices/soc0/image_crm_version
fi

# Parse misc partition path and set property
misc_link=$(ls -l /dev/block/bootdevice/by-name/misc)
real_path=${misc_link##*>}
setprop persist.vendor.mmi.misc_dev_path $real_path

# Blkio
echo 2000 > /dev/blkio/blkio.group_idle
echo 0 > /dev/blkio/background/blkio.group_idle
echo 1000 > /dev/blkio/blkio.weight
echo 200 > /dev/blkio/background/blkio.weight

# VM
echo 10 > /proc/sys/vm/dirty_background_ratio
echo 3000 > /proc/sys/vm/dirty_expire_centisecs
echo 0 > /proc/sys/vm/page-cluster
echo 32768 > /proc/sys/vm/min_free_kbytes

# Remove unused swapfile
rm -f /data/vendor/swap/swapfile 2>/dev/null
sync

# Disable sleep_disabled
echo N > /sys/module/lpm_levels/parameters/sleep_disabled

# Set governor for big and prime clusters
echo "schedutil" > /sys/devices/system/cpu/cpufreq/policy4/scaling_governor
echo 2323200 > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq
echo "schedutil" > /sys/devices/system/cpu/cpufreq/policy7/scaling_governor
echo 2649600 > /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq

# Setting b.L scheduler parameters
echo 95 95 > /proc/sys/kernel/sched_upmigrate
echo 85 85 > /proc/sys/kernel/sched_downmigrate

# Set readahead
sleep 20
find /sys/devices -name read_ahead_kb | while read node; do echo 128 > $node; done

# Only disable fake enforcing for OxygenOS/HydrogenOS users
if [ $os == "stock" ]; then
  echo 1 > /sys/module/selinux/parameters/fake_enforce_disabled
fi
chmod 0000 -R /sys/module/selinux/

exit 0

# Binary will be appended afterwards
