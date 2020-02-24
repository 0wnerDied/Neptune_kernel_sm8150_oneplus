## Build Information
```
Kernel: Weeb Kernel
Type: BETA
Device: OnePlus 7/T/Pro
Compiler: Clang + LTO
Branch: staging
HEAD: a6ab4d88ea5f1
Build Number: r62
```

## JSONs for OTA
**Stable Channel:**
https://raw.githubusercontent.com/RaphielGang/android_kernel_oneplus_sm8150/master/updater/update.json

**Beta Channel:**
https://raw.githubusercontent.com/idkwhoiam322/weeb_kernel_oneplus_sm8150/staging/updater/update.json

## Changelog
```
Changelog since 1.0-C2:
[ r44 ]
- Revert perf_critical patchset in favour of significantly better idle drain at the cost of jitter but not really any significant loss in performance
- Several CAF updates treewide
- Several Sultan rice treewide that I didn't want to add till first release
- Disable DEBUG_FS
- Reverse mac address provided by firmware [ Seems to help with some folks not having WiFi with qcacld inline ]
- Add haptic level adjustment [ Hot garbage btw, adding it cuz user request ]
- Some mm patches from mainline
-- Some additional fixups to the above backported by @ celtare21 aka Kuran Kaname
- missed an mm revert so it's there now ^^'
- Move some drivers' init to async to slightly improve boot times 
[ r62 ]
- Move to in-tree wireguard
- Update wireguard to 0.0.20200215
- Use latest AOSP clang now
- Compile with LTO and LLD
- BBR improvements
- Merge CAF tag "LA.UM.8.1.r1-14300-sm8150.0" for kernel, qcacld-3.0, fw-api, qca-wifi-host-cmn, and data-kernel
- Merge 4.14.171 from kernel.org
- Move to simple_lmk [ Disable LMKD and PSI ]
- Re-do device tree completely, fixes some stuff not applying properly and custom ROM support
- Some scheduler rice from Pixel 4 Android R source drop: Important for !SCHED_WALT, ie. pure PELT
- Properly remove sched_boost's influence on task placement
- Disable EAS on App launch
- atomic and locking backports by kdrag0n
- Binder, Clang, and UFS improvements from Pixel 4 Android R source drop
```