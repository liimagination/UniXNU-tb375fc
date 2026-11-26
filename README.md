# UniXNU-tb375fc

UniXNU device port for **Lenovo Xiaoxin Pad Pro 12.7 (2025) / TB375FC** (MediaTek Dimensity 8300 / MT6897, 8 GB LPDDR5X, 2944×1840 120 Hz Tianma NT36532, **ARM Mali-G615 MC5**) running **Full macOS 27.3 (`26C5035r`) Desktop** with **in-process hardware GPU acceleration**.

> **DMCA & Legal Compliance (UniXNU Guidelines):**
> - **Zero** proprietary Apple binaries, IPSW/DMG images, kernelcaches, or full Apple XNU source files are committed.
> - Upstream XNU modifications are provided strictly as minimal unified `.patch` diffs under `patches/`.
> - Vendor firmware (`novatek_ts_fw_tm.bin`, `novatek_ts_fw_boe.bin`, `mali_csffw.bin`) must be extracted from the device's own stock Android `/vendor/firmware/` and kept locally outside Git.

## Current Hardware Status (Boot 152 Verified)

- **Hardware GPU (Mali-G615 MC5, `gpu_id=0xb8a31030`, `PAN_ARCH=11`) & 8528 Mbps LPDDR5X**:
  - Real in-process `arm64e` Metal plugin (`NativeAdrenoMetal.bundle`, PAC-bridged) running `metal2vulkan` (`87a869f`) + Mesa PanVK v11 (`libvulkan_panfrost.dylib`) directly inside `WindowServer`, `Dock`, `Finder`, `System Settings`, `MenuBarAgent`, etc. (no RPC daemon).
  - **GPU OPP0 `1400 MHz`** + **`cpufreq-hw` (`2.0 / 2.8 / 3.0 GHz`)** + **LPDDR5X `8528 Mbps` (`40.8 GB/s`, 2.76× bandwidth)** unlocked via official MT6897 ATF SMC `MTK_SIP_VCOREFS_CONTROL (0xC2000506)` (`MTK_SIP_DVFSRC_INIT` + `SW_REQ=0x9050`).
  - **Hardware AFBC** enabled on private `OPTIMAL` render targets (`0` GPU/AS faults).
  - Multi-process CSF MCU scheduling: `Slot 0` (`AS1+CSG0`, `grpPrio=2`) pinned to `WindowServer`, `Slots 1..6` shared across background Metal processes via real `CSG_STATE_SUSPEND` / `CSG_STATE_RESUME` LRU multiplexing.
  - Zero-copy `IOSurface` import via `VK_EXT_external_memory_host` (`MALI_SEL_BO_IMPORT_HOST` with read-only `kIODirectionOut` fallback).
  - Full step-by-step technical bring-up guide: [docs/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md](docs/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md).
- **Display & Blit Pipeline (`1.25–2.16 ms`)**:
  - 1472×920 logical (2× HiDPI → 2944×1840 physical 120 Hz), `CA_FORCE_TRIPLE_BUFFERED_LATENCY` + `freeWakeup`.
  - Exact-sequence GPU-to-display sync (`tb375fc_mali_wait_ws_seq`, `0.52–1.92 ms` steady wait) + **3-thread unbound work-stealing 16-row blit** with 64 KiB branchless LUT and flat-quad skip (`1.25–2.16 ms` per blit).
  - Dynamic Island (`MenuBarAgent` CenterHUD) vertically aligned with menu bar icons (`island-y=17.5`).
  - Stage Manager (`WindowManager` `GloballyEnabled=true`, `AutoHide=false`) verified persistent with `0` crashes.
- **Native Power, Charging & Battery Display (`AppleSmartBattery`)**:
  - [drivers/power/Tb375fcPower.cpp](drivers/power/Tb375fcPower.cpp): real MM8013A fuel gauge (`i2c@11d02000`, `0x55`) + MT6375 charger (`i2c5@11f01000`, `0x34`, `aicr=2500mA`) + `disp_pwm0@1100e000` backlight dimming.
  - Full `AppleSmartBattery` (`IOPMPowerSource`) service verified live in `powerd`, `pmset -g batt` (`-InternalBattery-0`), `pmset -g adapter` (`13W`), `system_profiler SPPowerDataType`, Menu Bar percentage, and System Settings → Battery.
- **Long-Run Kernel Stability**:
  - `vm_compression_limit=524288` + `kmem_io = 512 MiB` expands `kmem_ptr_range_0`, `kmem_data_private_range`, and `kmem_data_shared_range` to ~2.1 GiB each (`kmem_alloc` failures = `0`, 0 panics).
- **Touchscreen (MT6897 SPI2 + Novatek NT36532 + EINT9 Hardware Latch)**:
  - Hardware rising-edge latch on `MT6897` `eint-n @ 0x11FE0000` (`EINT9` = local bit 5, `MTK_EINT_STA` only): zero idle SPI polling, responsive single taps and drags, `csum_err=0`.
- **Storage Persistence & Offline Peripheral Prep**:
  - Bounded UFS 4.0 LUN2 `PersistOverlay` (`data-persist=2`) preserving user settings and shader caches across reboots.
  - Offline peripheral prep (compiled offline, default inert `=0`, NOT linked into Boot 152):
    - Native onboard MT6897 ConnSys Wi-Fi (`soc7_0`) & Bluetooth (`BGF/BEIF`): [drivers/wireless/Tb375fcConnSys.cpp](drivers/wireless/Tb375fcConnSys.cpp) + [drivers/wireless/NATIVE-CONNSYS-WIFI-BT-PLAN.md](drivers/wireless/NATIVE-CONNSYS-WIFI-BT-PLAN.md)
    - USB-C Host (`mtu3`/`xHCI`): [drivers/usb/Tb375fcUsbHost.cpp](drivers/usb/Tb375fcUsbHost.cpp)

## Recommended Boot-Args (Boot 152)

```text
mali=4 vm_compressor=2 vm_compression_limit=524288 aicr=2500 data-persist=2 disp-hz=120 vsync=120 touch=4
```

## Branch Layout

| Branch | Scope | Status |
| :--- | :--- | :--- |
| `master` | UniXNU Format 2 skeleton (`board.toml`, `porting.toml`) + minimal XNU `.patch` diffs + P0 8-core SMP golden desktop baseline + `validation_disables=0x10` | Verified (Boot 48 / 69) |
| `feat/touch-nt36532` | P1 Internal SPI2 Tianma NT36532 touchscreen + `Tb375fcTouchHid` | Verified (Boot 58) |
| `feat/ufs-persist-jit` | P2 Bounded UFS 4.0 LUN2 `PersistOverlay` + persistence-safe `BootCacheControl` + SwiftShader disk JIT cache | Verified (Boot 66 / 70) |
| `feat/display-120hz` | P4 MT6897 dual-DSI read-only probe + `NativeLegacyDisplay` `disp-hz=120` | Verified (Boot 67 / 70) |
| `feat/mali-g615-csf` | P3 Part 1 GPUEB power-on + Mali-G615 LPAE `AS0` + CSF MCU v2.5 `GLB_PING` | Verified (Boot 62) |
| `feat/mali-panvk-m2v2` | **Latest Active Branch (Boot 152)**: In-process `arm64e` Mali-G615 PanVK v11 + `metal2vulkan` + 8528 Mbps LPDDR5X + 1400 MHz GPU + AFBC + 1.3 ms 3-thread blit + `AppleSmartBattery` + Stage Manager + offline native ConnSys Wi-Fi/BT prep | Verified (Boot 152) |
