## Build Information
```
Kernel: Weeb Kernel
Type: STABLE
Device: OnePlus 7/T/Pro/5G
Compiler: GCC
Branch: master
HEAD: 65ce97e1d647e
Build Number: v1.0-C2
```

## JSONs for OTA
**Stable Channel:**
https://raw.githubusercontent.com/RaphielGang/android_kernel_oneplus_sm8150/master/updater/update.json

**Beta Channel:**
https://raw.githubusercontent.com/idkwhoiam322/weeb_kernel_oneplus_sm8150/staging/updater/update.json

## Changelog
```
Initial Release!
- Cleanly based over latest CAF tag with minimal OnePlus changes
-- RAM Boost MUST be disabled
- Merged latest LTS subversion tag
- BBR as the default TCP network congestion control
- vDSO 32 patches to improve 32-bit performance
- vmalloc patches backported from mainline
- UFS optimizations
- Latest CFQ I/O scheduler
- Removed VLAs treewide
- Removed RTB logging
- Block userspace from messing with cpufreq completely
- cpu_input_boost driver by kerneltoast to handle cpu boosting
- devfreq_boost driver by kerneltoast to handle devfreq boosting
- Use userspace LMKD alongside PSI
- Use full PELT with sched_boost added ( but all its influences removed )
  to satisfy userspace
- kcal support
- wireguard support
- clean up and optimize some OnePlus changes
- Upstream KGSL and qseecom to latest CAF
- Import several optimizations from Pixel 4
- Don't boost cpu/task utilization by default with schedtune.boost
-- This behaviour is only when schedtune.boost is set to 2
-- Tasks are still biased to big cluster by default for top-app
- force some kernel threads to big cluster
```