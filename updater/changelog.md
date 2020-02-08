## Build Information
```
Kernel: Weeb Kernel
Type: BETA
Device: OnePlus 7/T/Pro/5G
Compiler: GCC
Branch: staging
HEAD: fa9b29f6fe6c8
Build Number: r44
```

## JSONs for OTA
**Stable Channel:**
https://raw.githubusercontent.com/RaphielGang/android_kernel_oneplus_sm8150/master/updater/update.json

**Beta Channel:**
https://raw.githubusercontent.com/idkwhoiam322/weeb_kernel_oneplus_sm8150/staging/updater/update.json

## Changelog
```
Changelog for r44:
[ since 1.0-C2 ]
- Revert perf_critical patchset in favour of significantly better idle drain at the cost of jitter but not really any significant loss in performance
- Several CAF updates treewide
- Several Sultan rice treewide that I didn't want to add till first release
- Disable DEBUG_FS
- Reverse mac address provided by firmware [ Seems to help with some folks not having WiFi with qcacld inline ]
- Add haptic level adjustment [ Hot garbage btw, adding it cuz user request ]
- Some mm patches from mainline
-- Some additional fixups to the above backported by @ celtare21 aka Kuran Kaname
- missed an mm revert so it's there now ^^'
```