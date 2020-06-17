## Build Information
```
Kernel: Weeb Kernel
Type: BETA
Device: OnePlus 7/T/Pro
Compiler: Proton Clang
Branch: staging
HEAD: 27f7a163b0ca
Build Number: r69
```

## JSONs for OTA
**Stable Channel:**
https://raw.githubusercontent.com/RaphielGang/android_kernel_oneplus_sm8150/master/updater/update.json

**Beta Channel:**
https://raw.githubusercontent.com/idkwhoiam322/weeb_kernel_oneplus_sm8150/staging/updater/update.json

## Changelog
```
Changelog since v2.0-Ember:
[ 2.1 Beta 1 ]
- Fix phone kicking into fastboot when compiled inline with a ROM
- Load WCNSS config from userspace [ Fixes Hotspot for some custom ROMS ]
- Fix booting into recovery
- Unified OxygenOS and Custom ROM builds
[ This breaks userspace toggling of HBM on OOS, such as by a kernel manager. I don't care so don't @ me. OOS doesn't use sysfs for changing display modes or auto HBM ( via auto brightness ) and it does not provide a userspace toggle like custom ROMs do so no normal functionality is affected. ]
- Latest CAF tag [ LA.UM.8.1.r1-15100-sm8150.0 ] merged in for kernel, wlan and techpack drivers
- Merge LTS tag 4.14.184
- Some optimizations by arter97
- Delays in mutex and [hr]timer removed [ thanks to kerneltoast and kdrag0n ( for queued spinlocks ) ]
- Scheduler improvements
- Raised max boost frequency [ app launches, memory pressure events ] and reduced duration to 1000ms,
- wireguard: Update to version 1.0.20200611
- Sync SLMK June updates
- Rip out remaining Houston and Control Center changes and replace them with no-op drivers
- drm optimizations from Google
- Locking optimizations backported by kdrag0n
- ion fixes by Google
- Several treewide optimizations by Sultan
- Disable some additional unnecessary/redundant logging
- Enable USB_ACM [ User request from XDA ]
- Some qcacld reverts that supposedly improve performance
- Sync several changes from kernel common
- Several improvements to the scheduler
- ion updates from msm-4.19
- Move drm and ufs driver to async probe
- Merged slmk May updates
- gcc 10 fixes
- f2fs updates
- Some miscellaneous changes
- Silence some more logs
- Disable proximity sensor debugging
- Add rapid gc by arter97 for f2fs
- Some rice from CAF upstream from other branches
- Mark UFS as performance critical
- Force QoS of adsprpc to silver cores
- Disable FORTIFY_SOURCE
[ https://lore.kernel.org/lkml/20200505215503.691205-1-Jason@zx2c4.com/ ]
- Don't toggle EAS on app launch
- Add back some dropped CAF changes [ no-op but for cleaner CAF upstream merges in the future ]
- Boost cpufreq/devfreq to max on fp scan
- Add optimize charging
- CAF's down migration changes
```
