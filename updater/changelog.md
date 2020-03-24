## Build Information
```
Kernel: Weeb Kernel
Type: BETA
Device: OnePlus 7/T/Pro
Compiler: Proton Clang + Polly
Branch: staging
HEAD: e32a00e33e62c
Build Number: r64
```

## JSONs for OTA
**Stable Channel:**
https://raw.githubusercontent.com/RaphielGang/android_kernel_oneplus_sm8150/master/updater/update.json

**Beta Channel:**
https://raw.githubusercontent.com/idkwhoiam322/weeb_kernel_oneplus_sm8150/staging/updater/update.json

## Changelog
```
Changelog since 1.0-C2:
[ 1.1 Beta 1 ]
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
[ 1.1 Beta 2 ]
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
[ 1.1 Beta 3 ]
- Remove atomic and locking backports by kdrag0n
- Update wireguard to 0.0.20200215
- Scheduler optimizations
- Update SLMK to latest
- WiFi optimizations by arter97
- Ensure EAS is enabled while screen is off
- Update some of Sultan's commits to latest their latest revision
- Update binder to 4.19
- Disable LTO in favour of faster builds
- Enable Clang's Polly optimizations
- Enable UASP support
- Revert broken ext4 commits by Sultan
- Revert size optimization for qcacld and techpack in case of potential latency regressions
- Merge CAF tag LA.UM.8.1.r1-14500-sm8150.0 for kernel, qcacld and techpack/data
- Silence some tracing and spammy logging
- boost DEVFREQ_MSM_CPU_LLCCBW device on mm pressure events
- Sultan's fix to OnePlus's techpack code that causes somewhat rare panics
- display optimization from Google's R tag
- Merge Linux Stable tag 4.14.174
- some network improvements
- Refined a scheduler commit
- ion from 4.19
- add back sched_boost's influence on scheduler ( seems better both for perf and battery )
```