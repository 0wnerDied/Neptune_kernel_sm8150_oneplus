#! /vendor/bin/sh

exec > /dev/kmsg 2>&1

if [ ! -f /sbin/recovery ] && [ ! -f /dev/.post_boot ]; then
  # Run once
  touch /dev/.post_boot

  # Setup binaries
  MKSWAPSIZE=6081
  tail -c $MKSWAPSIZE "$0" > /dev/mkswap
  echo SIZE: $(($(stat -c%s "$0") - $MKSWAPSIZE))
  head -c $(($(stat -c%s "$0") - $MKSWAPSIZE)) "$0" >> "$0".tmp
  mv "$0".tmp "$0"
  chmod 755 "$0"
  chmod 755 /dev/mkswap

  # Setup swap
  while [ ! -e /dev/block/vbswap0 ]; do
    sleep 1
  done
  if ! grep -q vbswap /proc/swaps; then
    # 4GB
    echo 4294967296 > /sys/devices/virtual/block/vbswap0/disksize
    echo 140 > /proc/sys/vm/swappiness
    # System mkswap behaves incorrectly with vbswap
    /dev/mkswap /dev/block/vbswap0
    swapon /dev/block/vbswap0
    rm /dev/mkswap
  fi

  # Stune speed up
  echo 1 > /dev/stune/schedtune.prefer_idle
  echo 100 > /dev/stune/schedtune.boost

  chmod 755 "$0"

  # Disable Houston and cc_ctl
  mount --bind /dev/.post_boot /system/priv-app/Houston/Houston.apk
  mount --bind /dev/.post_boot /system/priv-app/OPAppCategoryProvider/OPAppCategoryProvider.apk
  rm -f /data/dalvik-cache/arm64/system@priv-app@Houston*
  rm -f /data/dalvik-cache/arm64/system@priv-app@OPAppCategoryProvider*

  # Workaround vdc slowing down boot
  ( for i in $(seq 1 20); do
      PID=$(pgrep -f "vdc checkpoint restoreCheckpoint")
      if [ ! -z $PID ]; then
        echo "Killing checkpoint vdc process $PID"
        kill -9 $PID
        exit
      fi
      sleep 1
    done
    echo "Timed out while looking for checkpoint vdc process"
  ) &

  # Hide app dirs as well
  mkdir /dev/.empty_dir
  while [ ! -e /data/data/ ]; do sleep 0.01; done
  mount --bind /dev/.empty_dir /data/data/com.oneplus.houston
  mount --bind /dev/.empty_dir /data/data/net.oneplus.provider.appcategoryprovider

  # Hook up to existing init.qcom.post_boot.sh
  while [ ! -f /vendor/bin/init.qcom.post_boot.sh ]; do
    sleep 1
  done
  if ! mount | grep -q /vendor/bin/init.qcom.post_boot.sh; then
    # Replace msm_irqbalance.conf
    echo "PRIO=1,1,1,1,0,0,0,0
#arch_timer, arm-pmu, arch_mem_timer, msm_drm, glink_lpass, kgsl
IGNORED_IRQ=19,21,38,115,188,332" > /dev/msm_irqbalance.conf
    chmod 644 /dev/msm_irqbalance.conf
    mount --bind /dev/msm_irqbalance.conf /vendor/etc/msm_irqbalance.conf
    chcon "u:object_r:vendor_configs_file:s0" /vendor/etc/msm_irqbalance.conf
    killall msm_irqbalance

    mount --bind "$0" /vendor/bin/init.qcom.post_boot.sh
    chcon "u:object_r:qti_init_shell_exec:s0" /vendor/bin/init.qcom.post_boot.sh
    exit
  fi
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

# blkio
echo 2000 > /dev/blkio/blkio.group_idle
echo 0 > /dev/blkio/background/blkio.group_idle
echo 1000 > /dev/blkio/blkio.weight
echo 200 > /dev/blkio/background/blkio.weight

# VM
echo 10 > /proc/sys/vm/dirty_background_ratio
echo 3000 > /proc/sys/vm/dirty_expire_centisecs
echo 0 > /proc/sys/vm/page-cluster
echo 32768 > /proc/sys/vm/min_free_kbytes

# stune
echo 0 > /dev/stune/schedtune.prefer_idle
echo 0 > /dev/stune/schedtune.boost
echo 1 > /dev/stune/foreground/schedtune.prefer_idle
echo 1 > /dev/stune/top-app/schedtune.prefer_idle
echo 2 > /dev/stune/top-app/schedtune.boost

# Remove unused swapfile
rm -f /data/vendor/swap/swapfile 2>/dev/null
sync

# s2idle
echo "s2idle" > /sys/power/mem_sleep

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

exit 0

# Binary will be appended afterwards
