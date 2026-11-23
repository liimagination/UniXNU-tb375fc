# UniXNU-tb375fc

UniXNU device port for **Lenovo Xiaoxin Pad Pro 12.7 (2025) / TB375FC** (MediaTek Dimensity 8300 / MT6897, 8 GB RAM, 2944×1840 120 Hz Tianma NT36532) running **Full macOS 27.3 (`26C5035r`) Desktop**.

> **DMCA & Legal Compliance (UniXNU Guidelines):**
> - **Zero** proprietary Apple binaries, IPSW/DMG images, kernelcaches, or full Apple XNU source files are committed.
> - Upstream XNU modifications are provided strictly as minimal unified `.patch` diffs under `patches/`.
> - Vendor firmware (`novatek_ts_fw_tm.bin`, `mali_csffw.bin`) must be extracted from the device's own stock Android `/vendor/firmware/` and kept locally outside Git.

## Branch Layout

| Branch | Scope | Status |
| :--- | :--- | :--- |
| `master` | UniXNU Format 2 skeleton (`board.toml`, `porting.toml`) + minimal XNU `.patch` diffs + P0 8-core SMP golden desktop baseline + `validation_disables=0x10` | Verified (Boot 48 / 69) |
| `feat/touch-nt36532` | P1 Internal SPI2 Tianma NT36532 touchscreen + `Tb375fcTouchHid` | Verified (Boot 58) |
| `feat/ufs-persist-jit` | P2 Bounded UFS 4.0 LUN2 `PersistOverlay` + persistence-safe `BootCacheControl` + SwiftShader disk JIT cache | Verified (Boot 66 / 70) |
| `feat/display-120hz` | P4 MT6897 dual-DSI read-only probe + `NativeLegacyDisplay` `disp-hz=120` | Verified (Boot 67 / 70) |
| `feat/mali-g615-csf` | P3 Part 1 GPUEB power-on + Mali-G615 LPAE `AS0` + CSF MCU v2.5 `GLB_PING` | Verified (Boot 62) |
