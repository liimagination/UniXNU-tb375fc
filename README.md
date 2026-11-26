# UniXNU-tb375fc

UniXNU device port for **Lenovo Xiaoxin Pad Pro 12.7 (2025) / TB375FC** (MediaTek Dimensity 8300 / MT6897, 8 GB LPDDR5X, 2944×1840 120 Hz Tianma NT36532, **ARM Mali-G615 MC5**) running **Full macOS 27.3 (`26C5035r`) Desktop** with **in-process hardware GPU acceleration**.

> **DMCA & Legal Compliance (UniXNU Guidelines):**
> - **Zero** proprietary Apple binaries, IPSW/DMG images, kernelcaches, or full Apple XNU source files are committed.
> - Upstream XNU modifications are provided strictly as minimal unified `.patch` diffs under `patches/`.
> - Vendor firmware (`novatek_ts_fw_tm.bin`, `novatek_ts_fw_boe.bin`, `mali_csffw.bin`) must be extracted from the device's own stock Android `/vendor/firmware/` and kept locally outside Git.

## Current Hardware Status (Boot 148 Verified)

- **Hardware GPU (Mali-G615 MC5, `gpu_id=0xb8a31030`, `PAN_ARCH=11`)**:
  - Real in-process `arm64e` Metal plugin (`NativeAdrenoMetal.bundle`, PAC-bridged) running `metal2vulkan` (`87a869f`) + Mesa PanVK v11 (`libvulkan_panfrost.dylib`) directly inside `WindowServer`, `Dock`, `Finder`, `System Settings`, `MenuBarAgent`, etc. (no RPC daemon).
  - MT6897 GPUEB power-on + OPP boost (`1206 MHz`, max OPP0 `1400 MHz`) + `cpufreq-hw` (`2.0 / 2.8 / 3.0 GHz`).
  - Multi-process CSF MCU scheduling: `Slot 0` (`AS1+CSG0`) pinned to `WindowServer`, `Slots 1..6` shared across background Metal processes via real `CSG_STATE_SUSPEND` / `CSG_STATE_RESUME` LRU multiplexing.
  - Zero-copy `IOSurface` import via `VK_EXT_external_memory_host` (`MALI_SEL_BO_IMPORT_HOST` with read-only `kIODirectionOut` fallback).
  - Full step-by-step technical bring-up guide: [docs/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md](docs/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md).
- **Long-Run Kernel Stability**:
  - `vm_compression_limit=524288` + `kmem_io = 512 MiB` expands `kmem_ptr_range_0`, `kmem_data_private_range`, and `kmem_data_shared_range` to ~2.1 GiB each (`kmem_alloc` failures = `0` across 3.6+ hours / 38,000+ continuous frames, 0 panics).
- **Touchscreen (MT6897 SPI2 + Novatek NT36532 + EINT9 Hardware Latch)**:
  - Hardware rising-edge latch on `MT6897` `eint-n @ 0x11FE0000` (`EINT9` = local bit 5, `MTK_EINT_STA` only): zero idle SPI polling, responsive single taps and drags, `csum_err=0`.
- **Display & Desktop Polish**:
  - 1472×920 logical (2× HiDPI → 2944×1840 physical 120 Hz), `CA_FORCE_TRIPLE_BUFFERED_LATENCY` + `freeWakeup`, GPU-to-display `tb375fc_mali_wait_ws_idle()` sync + 2-P-core lockstep even/odd interleaved blit.
  - Dynamic Island (`MenuBarAgent` CenterHUD) vertically aligned with menu bar icons (`island-y=17.5`).
- **Storage Persistence**:
  - Bounded UFS 4.0 LUN2 `PersistOverlay` (`data-persist=2`) preserving user settings and shader caches across reboots.

## Recommended Boot-Args (Boot 148)

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
| `feat/mali-panvk-m2v2` | **Latest Active Branch (Boot 148)**: In-process `arm64e` Mali-G615 PanVK v11 + `metal2vulkan` + multi-process CSG Suspend/Resume + EINT9 touch + ~2.1 GiB `kmem` VA pools + GPU-display sync | Verified (Boot 148) |
