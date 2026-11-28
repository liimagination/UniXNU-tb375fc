# UniXNU-tb375fc

UniXNU device port for **Lenovo Xiaoxin Pad Pro 12.7 (2025) / TB375FC** (MediaTek Dimensity 8300 / MT6897, 8 GB LPDDR5X, 2944×1840 120 Hz Tianma NT36532, **ARM Mali-G615 MC5**, **MT6897 ConnSys `soc7_0` + MT6637 Wi-Fi 6**) running **Full macOS 27.3 (`26C5035r`) Desktop** with **in-process hardware GPU acceleration** and **native onboard Wi-Fi 6**.

> **DMCA & Legal Compliance (UniXNU Guidelines):**
> - **Zero** proprietary Apple binaries, IPSW/DMG images, kernelcaches, or full Apple XNU source files are committed.
> - Upstream XNU modifications are provided strictly as minimal unified `.patch` diffs under `patches/`.
> - Vendor firmware (`novatek_ts_fw_tm.bin`, `novatek_ts_fw_boe.bin`, `mali_csffw.bin`, `WIFI_RAM_CODE_soc7_0_1_1.bin`, `soc7_0_ram_wmmcu_1_1_hdr.bin`) must be extracted from the device's own stock Android `/vendor/firmware/` and kept locally outside Git.

## Current Hardware Status (Boot 161 Verified)

- **Native Onboard MT6897 Wi-Fi 6 (`soc7_0` + MT6637 -> `en0`)**:
  - Clean-room kernel driver under [drivers/wireless/](drivers/wireless/) (`Tb375fcConnSys.cpp`, `Tb375fcConnSysABI.h`, `Tb375fcWifiEngine.h`, `Tb375fcWifiCrypto.h`) + userspace controller [userspace/wireless/tb375fc-connsys-ctl.c](userspace/wireless/tb375fc-connsys-ctl.c) + `en0` bridge in [drivers/net/Tb375fcDummyEth.cpp](drivers/net/Tb375fcDummyEth.cpp).
  - Full AXI WFDMA firmware download, MCU bring-up, 802.11 scan/auth/assoc, WPA2-PSK 4-way handshake, AES-CCMP encryption/decryption, and live IPv4 networking over `en0` (verified 5 GHz WPA2-PSK, 0 RX/TX drops, HTTPS 200 OK).
- **Hardware GPU (Mali-G615 MC5, `gpu_id=0xb8a31030`, `PAN_ARCH=11`) & 8528 Mbps LPDDR5X**:
  - Real in-process `arm64e` Metal plugin (`NativeAdrenoMetal.bundle`, PAC-bridged) running `metal2vulkan` (`87a869f`) + Mesa PanVK v11 (`libvulkan_panfrost.dylib`) directly inside `WindowServer`, `Dock`, `Finder`, `System Settings`, `MenuBarAgent`, etc. (no RPC daemon).
  - **GPU OPP0 `1400 MHz`** + **`cpufreq-hw` (`2.0 / 2.8 / 3.0 GHz`)** + **LPDDR5X `8528 Mbps` (`40.8 GB/s`, 2.76× bandwidth)** unlocked via official MT6897 ATF SMC `MTK_SIP_VCOREFS_CONTROL (0xC2000506)` (`MTK_SIP_DVFSRC_INIT` + `SW_REQ=0x9050`).
  - **Hardware AFBC** with superblock header zero-init in `panvk_BindImageMemory2` + streamlined CSF submission (**6,453+ commits, 223,456 render passes, 0 MMU faults**).
  - **8-bit `BGRA8` Display Framebuffer** (`bytes=5416960`, 50% bandwidth vs `RGhA`): **77–88 FPS** sustained on regular desktop animations (`bgra blit` **0.53–1.58 ms**, `wsWait` **2.3–4.4 ms**).
  - Multi-process CSF MCU scheduling: `Slot 0` (`AS1+CSG0`, `grpPrio=2`) pinned to `WindowServer`, `Slots 1..6` shared across background Metal processes via real `CSG_STATE_SUSPEND` / `CSG_STATE_RESUME` LRU multiplexing.
  - Zero-copy `IOSurface` import via `VK_EXT_external_memory_host` (`MALI_SEL_BO_IMPORT_HOST` with read-only `kIODirectionOut` fallback).
  - Full step-by-step technical bring-up guide: [docs/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md](docs/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md).
- **128 GiB Persistent UFS 4.0 LUN2 Storage**:
  - Full UFS 4.0 LUN2 persistent read/write storage under [drivers/storage/](drivers/storage/) (`Tb375fcPersistStore.cpp`, `NativeRamBlock.cpp`, `Tb375fcUfsTransport.c`) providing a persistent 128 GiB APFS Data volume (`data-persist=2`) preserving user settings, Wi-Fi firmware, and shared shader caches across reboots.
- **Display & UI Pipeline**:
  - 1472×920 logical (2× HiDPI → 2944×1840 physical 120 Hz), `CA_FORCE_TRIPLE_BUFFERED_LATENCY` + `freeWakeup`, exact-sequence GPU-to-display sync (`tb375fc_mali_wait_ws_seq`), 3-thread unbound work-stealing 16-row blit.
  - Dynamic Island (`MenuBarAgent` CenterHUD) vertically aligned with menu bar icons (`island-y=17.5`) and Stage Manager (`WindowManager`) enabled.
- **Native Power, Charging & Battery Display (`AppleSmartBattery`)**:
  - [drivers/power/Tb375fcPower.cpp](drivers/power/Tb375fcPower.cpp): real MM8013A fuel gauge (`i2c@11d02000`, `0x55`) + MT6375 charger (`i2c5@11f01000`, `0x34`, `aicr=2500mA`) + `disp_pwm0@1100e000` backlight dimming.
  - Full `AppleSmartBattery` (`IOPMPowerSource`) service verified live in `powerd`, `pmset`, Menu Bar percentage, and System Settings → Battery.
- **Long-Run Kernel Stability & Touchscreen**:
  - `vm_compression_limit=524288` + `kmem_io = 512 MiB` expands kmem ranges to ~2.1 GiB each (`kmem_alloc` failures = `0`, 0 panics).
  - MT6897 SPI2 + Novatek NT36532 + `EINT9` hardware rising-edge latch (`eint-n @ 0x11FE0000`, `csum_err=0`, zero idle SPI polling).
- **Offline Peripheral Prep**:
  - USB-C Host (`mtu3`/`xHCI`): [drivers/usb/Tb375fcUsbHost.cpp](drivers/usb/Tb375fcUsbHost.cpp) (compiled offline, default inert `=0`).

## Recommended Boot-Args (Boot 161)

```text
mali=4 connsys=1 vm_compressor=2 vm_compression_limit=524288 aicr=2500 data-persist=2 disp-hz=120 vsync=120 touch=4
```

## Branch Layout

| Branch | Scope | Status |
| :--- | :--- | :--- |
| `master` | UniXNU Format 2 skeleton (`board.toml`, `porting.toml`) + minimal XNU `.patch` diffs + P0 8-core SMP golden desktop baseline + `validation_disables=0x10` | Verified (Boot 48 / 69) |
| `feat/touch-nt36532` | P1 Internal SPI2 Tianma NT36532 touchscreen + `Tb375fcTouchHid` | Verified (Boot 58) |
| `feat/ufs-persist-jit` | P2 Bounded UFS 4.0 LUN2 `PersistOverlay` + persistence-safe `BootCacheControl` + SwiftShader disk JIT cache | Verified (Boot 66 / 70) |
| `feat/display-120hz` | P4 MT6897 dual-DSI read-only probe + `NativeLegacyDisplay` `disp-hz=120` | Verified (Boot 67 / 70) |
| `feat/mali-g615-csf` | P3 Part 1 GPUEB power-on + Mali-G615 LPAE `AS0` + CSF MCU v2.5 `GLB_PING` | Verified (Boot 62) |
| `feat/mali-panvk-m2v2` | **Latest Active Branch (Boot 161)**: Native MT6897 Wi-Fi 6 (`soc7_0` + MT6637 -> `en0`), 128 GiB persistent UFS LUN2 APFS Data, 8-bit `BGRA8` FB (77–88 FPS), Mali-G615 PanVK v11 + AFBC ON (0 faults), 8528 Mbps LPDDR5X, 1400 MHz GPU, `AppleSmartBattery`, Stage Manager | Verified (Boot 161) |
