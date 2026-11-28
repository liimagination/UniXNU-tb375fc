# TB375FC（MT6897）macOS 27.3 → iPadOS 27 移植借鉴与实战参考手册

> **沉淀来源**：联想小新 Pad Pro 12.7（2025）/ **TB375FC**（MediaTek Dimensity 8300 / MT6897）从零点亮 **macOS 27.3（`26C5035r`）** 到 **Boot 161**（8 核 SMP、128 GiB 持久化 UFS 4.0、Mali-G615 MC5 硬件 GPU + AFBC + 8-bit `BGRA8` 77–88 FPS、120 Hz 显示、NT36532 EINT9 触控、MM8013A/MT6375 电池充电、原生 MT6897 `soc7_0` Wi-Fi 6 上网）的 161 次实机迭代全部硬核经验。
>
> **目标读者**：在同一块 **TB375FC（MT6897）** 硬件上同步推进 **iPadOS 27** bring-up 的开发者 / AI Agent。所有地址、SMC、IRQ、补丁与坑位均已在实机上反复验证。

---

## 1. TB375FC（MT6897）实机已验证硬件地图

### 1.1 设备与平台基准

| 项目 | 实机验证值 | 备注 |
| :--- | :--- | :--- |
| 机型 / 代号 | Lenovo Xiaoxin Pad Pro 12.7 (2025) / **TB375FC** | `fastboot getvar product` 必须等于 `P98300DA2` |
| Fastboot 槽位 / 序列号 | `current-slot=a` / `HA227LJ3` | 仅刷写 `vendor_boot_a`（64 MiB） |
| SoC / CPU 拓扑 | **MT6897**（4× A510 @ 2.2 GHz + 3× A715 @ 3.2 GHz + 1× A715 Prime @ 3.35 GHz） | `MPIDR_EL1` 核心号在 `Aff1:Aff0`（`0x000..0x700`），**不是** `Aff0`！ |
| 内存（RAM） | 8 GB LPDDR5X（Darwin 可用 **7.03 GiB**） | 上电默认低频，需 ATF SMC 解锁 **8528 Mbps（40.8 GB/s，2.76×）** |
| 存储（UFS 4.0） | 256 GB UFS 4.0，用户区位于 **LUN2**（4 KiB 逻辑块） | LK 退出前执行了 `ufs_aio_exit()`（`HCE=0`），内核必须冷启动 UFSHCI |
| 屏幕 | 2944×1840 @ 120 Hz 双 DSI（Tianma NT36532） | 物理显存 `0xFC16F000`（`0x014AA000` B），2× HiDPI 逻辑 `1472×920` |
| GPU | **ARM Mali-G615 MC5**（`GPU_ID=0xB8A31030`，`PAN_ARCH=11`，5 核，CSF MCU v2.5） | 需先经 GPUEB（`0xBABADADA`）上电并切 OPP0 `1400 MHz` |
| 触控芯片 | Tianma **Novatek NT36532**（SPI2 + GPIO9 `EINT9` + GPIO60 RESET） | SRAM 型触控，每次冷启动必须下载固件 `novatek_ts_fw_tm.bin` |
| 电池与充电 | **MM8013A** 电量计（10,200 mAh）+ **MT6375** 充电芯片 + `disp_pwm0` 背光 | 已原生对接 `AppleSmartBattery`（`IOPMPowerSource`） |
| 无线芯片 | **MT6897 ConnSys `soc7_0`**（Connac 2.x AXI WFDMA）+ **MT6637** A-Die | 已跑通原生 Wi-Fi 6 5 GHz WPA2-PSK/AES-CCMP（`en0`） |
| USB 调试口 | **MT6897 MTU3** USB Device（CDC ACM，宿主 `/dev/cu.usbmodem*`） | 排障生命线；前期务必先打通 USB CDC Shell |

---

### 1.2 已验证物理 MMIO / 保留内存 / ATF SMC 总表

#### A. 关键 MMIO 与保留物理内存

| 子系统 | 物理基址 | 大小 | 关键寄存器 / 说明 |
| :--- | :--- | :--- | :--- |
| **Display FB Layer0** | `0xFC16F000` | `0x014AA000` | 2944×1840×4 线性帧缓冲，UEFI/LK 已点亮双 DSI |
| **Backlight PWM0** | `0x1100E000` | `0x1000` | `disp_pwm0` 屏幕背光调光 |
| **Mali-G615 GPU** | `0x13000000` | `0x00200000` | Control + MMU + MCU + Doorbells 0..23（2 MiB） |
| **GPUEB GPR / MBOX0** | `0x13C2F000` / `0x13C62000` | `0x1000` | GPUEB 邮箱控制（`IRQ_SET=0x04`, `IRQ_CLR=0x74`） |
| **MFG RPC** | `0x13F90000` | `0x1000` | Mali 电源域 RPC |
| **GPUEB Shared DRAM** | `0x7FE70000` | `0x4000` | 魔数 `0xBABADADA`，`OPP_NUM=0x000C`（切 OPP0 `1400 MHz`） |
| **UFS 4.0 Host (UFSHCI)** | `0x112B0000` | `0x3000` | `REG_UFS_XOUFS_CTRL=0x140`, `REG_UFS_REFCLK_CTRL=0x144` |
| **UFS PHY / AO / PDN** | `0x112A0000` / `0x112B8000` / `0x112BB000` | `0x10000` / `0x1000` | UFS PHY 与时钟门控 |
| **SPMI PMIF Master** | `0x1CC04000` | `0x1000` | PMIC 电源轨控制（UFS / ConnSys 共用） |
| **GPIO / IOCFG_RT** | `0x10005000` / `0x11B20000` | `0x1000` | Pinmux 与上拉下拉配置 |
| **EINT-N（触控中断）** | `0x11FE0000` | `0x1000` | GPIO9 = `eint-n` local **bit 5**（`1<<5`）；`STA=0x000`, `ACK=0x040`, `SENS=0x140`, `POL=0x300` |
| **I2C5（MT6375 充电）** | `0x11F01000` | `0x1000` | MT6897 I2C v2（`SLAVE_ADDR1=0x94`），7-bit 从地址 **`0x34`** |
| **I2C（MM8013A 电量计）** | `0x11D02000` | `0x1000` | 7-bit 从地址 **`0x55`** |
| **SPM / SRCLKEN_RC / IFRBUS_AO** | `0x1C001000` / `0x1C00D000` / `0x1002C000` | `0x1000` | ConnSys 上电钥匙 `SPM_POWERON_CFG_EN=0x0B160001` |
| **ConnSys 统一窗口** | `0x18000000` | `0x00900000` | 覆盖 ConnInfra(`+0x11000`=`0x02050400`), WFDMA0(`+0x24000`), WFSYS(`+0x4B0010`=`0x02040200`), BGFSYS |
| **ConnSys EMI 保留内存** | `0x8CC00000` | `0x00D00000` | 13 MiB（Wi-Fi RO `+0x380000`, WMMCU date `+0x4954E0`, BEIF BT `+0xBA0000`） |
| **Wi-Fi DMA 保留内存池** | `0x8D900000` | `0x01400000` | 20 MiB 连续物理 DMA 池（给 WFDMA0 TX/RX 环使用） |

#### B. 已验证 MT6897 BL31 ATF SiP SMC 列表

| SMC Function ID | 名称 | 实机用途与关键参数 |
| :--- | :--- | :--- |
| `0x84000000` / `0x8400000A` / `0xC4000003` | ARM PSCI (`VERSION` / `FEATURES` / `CPU_ON`) | 唤醒 7 个从核（注意按 `Aff1:Aff0` 传递 MPIDR） |
| `0xC2000506` | `MTK_SIP_VCOREFS_CONTROL` | `MTK_SIP_DVFSRC_INIT` + `SW_REQ=0x9050` → 解锁 **8528 Mbps LPDDR5X** |
| `0xC2000276` | `MTK_SIP_UFS_CONTROL` | UFS 4.0 冷启动、PHY 复位与链路初始化（见 `Tb375fcUfsTransport.c:302`） |
| `0xC2000534` | `MTK_SIP_KERNEL_CONNSYS_CONTROL` | ConnInfra ATF 初始化、芯片 ID 轮询、D-Die/A-Die 配置 |
| `0xC2000535` | `MTK_SIP_KERNEL_WLAN_CONTROL` | WFSYS `soc7_0` 电源与复位控制 |
| `0xC2000536` | `MTK_SIP_KERNEL_BT_CONTROL` | BGFSYS 蓝牙电源控制（预留） |

---

## 2. macOS 27 vs iPadOS 27：复用矩阵与关键差异

| 模块 | 复用度 | macOS 27 现成实现 | 迁移到 iPadOS 27 的关键差异与注意点 |
| :--- | :---: | :--- | :--- |
| **UEFI (`DarwinBoot` + `XnuLz`)** | 95% | `uefi/DarwinBootPkg/` + `xnulz-m7/` | 完全复用内存窗口逻辑、35 MiB FAT16+LZMA 解压框架；只需调整 iPadOS ADT/boot-args 与 kernelcache 大小布局（`Layout.h`） |
| **刷机与安全护栏** | 100% | `fastboot-noclear` + `verify_lzmadec.c` | **完全一致**：禁止系统 `fastboot`，64 MiB 传输超时 ≥ 300s，刷前必检 `product=P98300DA2` & `current-slot=a` |
| **XNU 底层补丁** | 90% | `start.s`、`vm_kern.c`、`IOUserClient.cpp`、软件 AES | `MPIDR Aff1:Aff0` 补丁、`kmem_io=512M` + `vm_compression_limit=524288`、软件 AES 直接照搬；需额外关注 iPadOS **AMFI / TrustCache / MobileKeyBag / 沙盒策略**（比 macOS 更严格） |
| **8 核 SMP & GICv3** | 100% | `drivers/cpu/ArmCpu.cpp` | 硬件无关 OS 类型，100% 直接复用 |
| **USB CDC 调试串口** | 100% | `tb375fc-usb-cdc` + `tb375fc-cdc-shell` | **强烈建议在 M1 阶段最先接入！** iPadOS 早期默认无终端 UI，全靠 CDC 拿 `kprintf`/shell/panic 日志 |
| **UFS 4.0 LUN2 存储** | 95% | `drivers/storage/`（`Tb375fcUfsTransport.c` + `Tb375fcPersistStore.cpp`） | 传输层 100% 复用，**但 LBA 安全写区间必须严格隔离**（见第 3.2 节，切勿踩坏 macOS 27 的 128 GiB Data 分区与 Android 分区！） |
| **显示与帧缓冲** | 80% | `drivers/display/NativeLegacyDisplay.cpp` | 底层 3 线程无锁 work-stealing 16-row blit 与 120 Hz VSync 直接复用；**注意**：iPadOS 27 合成器（`backboardd`/IOMobileFramebuffer）在启用 Metal 后会请求 **64-bit `RGhA`（half-float）** 或 `IOMobileFramebuffer` 接口，需重点核对 `IOMobileFramebuffer` vs `IOFramebuffer`/`IOAV` 适配层 |
| **NT36532 触控屏** | 90% | `touch-fix/`（SPI2 + `Nvt36532Protocol.c` + `EINT9`） | SPI2 驱动、固件下载、`EINT9` 硬件上升沿锁存 100% 直接复用！唯一升级点：iPadOS **原生吃多点触控 Digitizer HID 报告**，无需像 macOS 那样把单指映射成鼠标指针，体验会更好 |
| **Mali-G615 GPU 栈** | 90% | `Tb375fcMaliDriver.cpp` + `panthor_darwin_shim.c` + PanVK v11 + `metal2vulkan` + `NativeAdrenoMetal.bundle` | 内核驱动、GPUEB/DVFSRC、AFBC header 清零、零拷贝 `IOSurface` 全部复用；差异点：① `Slot 0` 固定给 iPadOS 主渲染进程（`backboardd` / `RenderServer` 而非 `WindowServer`）；② iPadOS **极其高频使用 `MTLResourceStorageModeMemoryless` 附件**，必须保留 `Memoryless → Private` 补丁；③ Metal 插件路径/签名按 iPadOS dyld/dyld_shared_cache 规则部署 |
| **电池/快充/背光** | 100% | `drivers/power/Tb375fcPower.cpp` | `AppleSmartBattery`（`IOPMPowerSource`）与 `disp_pwm0` 对 iPadOS `powerd`/SpringBoard 状态栏电量同样原生生效 |
| **原生 Wi-Fi 6** | 85% | `drivers/wireless/` + `tb375fc-connsys-ctl.c` + `Tb375fcDummyEth.cpp` | 硬件驱动（`soc7_0` WFDMA + MT6637 + WPA2-PSK/AES-CCMP）100% 直接复用；差异点：iPadOS 上层 `wifid` / `CommCenter` / `configd` 对 `en0` 类型的识别可能要求模拟 `IO80211Interface` 或配置有线/Wi-Fi fallback 路由 |

---

## 3. 安全与防砖铁律（同步做双系统必看）

### 3.1 刷机铁律

1. **永远不要用系统自带 `fastboot`**，必须使用 [fastboot-noclear](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/postmmu-diag/m5-gui/fastboot-noclear)（避免清空关键分区头/参数）。
2. **刷机前必检**：
   - `fastboot-noclear -s HA227LJ3 getvar product` → 必须是 `P98300DA2`
   - `fastboot-noclear -s HA227LJ3 getvar current-slot` → 必须是 `a`
3. **USB 传输超时**：64 MiB `vendor_boot_a` 在该数据线/主控上稳定耗时 **~205 秒**，脚本超时必须设 **`>= 300s`**，否则会被误判超时而中断传输。
4. **LZMA 离线强校验**：每次生成 `fatpart.lzma` 和 `kernel.lzma` 后，**必须**用与 UEFI 源码一致的 `verify_lzmadec`（编译自 `verify_lzmadec.c`）跑一遍解压校验，防止 LZMA 字典/参数不兼容导致 UEFI 阶段静默死机。
5. **CDC 串口重枚举等待**：`fastboot reboot` 后，UEFI 会极早短暂枚举一次 CDC，随后 XNU 接管时重新枚举；自动化脚本需在重启后等待 ~20s，并在 `ENXIO`/`EIO` 时自动重开 `/dev/cu.usbmodem*`。

### 3.2 UFS 4.0 LUN2 物理 LBA 边界图（⚠️ 防踩坏 macOS 27 与 Android！）

TB375FC 的 UFS LUN2 `userdata` 总范围约 `[2,803,712 .. 62,410,743]`（每块 4 KiB）。当前已被占用的区间：

```text
LUN2 LBA (4 KiB/block，userdata 总范围 [2,803,712 .. 62,410,743] ≈ 227.38 GiB):
  0 .. 2,803,711            : Android GPT + super / vendor_a / boot / persist 等系统分区（严禁写入！）
  2,803,712 .. 16,349,999   : Android userdata 保留区（约 51.67 GiB）
  16,350,000 .. 49,906,175  : ⚠️ macOS 27.3 128 GiB 持久化 APFS Data 卷（活数据！严禁 iPadOS 写入！）
  49,914,112 .. 54,211,840  : ⚠️ macOS 27.3 只读系统根镜像（RO roots + manifest + hashes，16.39 GiB，强制保护区！严禁写入！）
  54,220,000 .. 62,410,743  : 约 31.25 GiB 高位连续空闲安全写区（原早期 PersistOverlay 仅占开头 ~1 GiB）
```

> **强烈建议**：iPadOS 27 的 `Tb375fcUfsTransport.c` 在初始化早期保持 **strictly read-only**（只允许 SCSI `READ(10)`），等进入持久化阶段前先只读扫描高位 `[54,500,000 .. 62,410,743]` 确认全零后再把 iPadOS 的写窗口硬编码锁死在独立 LBA 区间内，实现 **Android / macOS 27 / iPadOS 27 三系统存储物理互不干扰**。

---

## 4. XNU 内核与早期 Bring-Up 必踩 7 大硬坑（直接避雷）

### 坑 1：MT6897 8 核 `MPIDR_EL1` 编号不在 `Aff0`
- **症状**：只起来 1 个核，或唤醒从核瞬间跑飞/panic。
- **根因**：MT6897 8 个核的 `MPIDR_EL1` 为 `0x000, 0x100, ..., 0x700`（核心号在 `Aff1`，`Aff0=0`）。默认 XNU `osfmk/arm64/start.s` 在非 `HAS_CLUSTER` 下用 `and x0, x15, #0xFF` 截断导致所有核都被算成 CPU 0！
- **解法**：直接应用 [xnu-start-s-ram8g.patch](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/patches/xnu-start-s-ram8g.patch)（改为 `and x0, x15, #0xFFFF`），搭配 [ArmCpu.cpp](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/drivers/cpu/ArmCpu.cpp) 通过 `ICC_SGI1R_EL1` 发硬件 IPI。

### 坑 2：7.03 GiB RAM 下跑 5 分钟必爆 `kmem_alloc` / `KERN_NO_SPACE` Panic
- **症状**：刚进桌面/SpringBoard 正常，开几个 App 或做几次动画后突然 `panic: zalloc / kmem_alloc failed`。
- **根因**：XNU 默认划掉 `1 GiB` 给 `kmem_io` 且 VM 压缩池占掉大量虚拟空间，导致 `kmem_ptr_range_0`、`kmem_data_private_range`、`kmem_data_shared_range` 极小。
- **解法**（已验证 0 panic 长稳运行）：
  1. 应用 [xnu-vm-kern-kmem-io-512m.patch](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/patches/xnu-vm-kern-kmem-io-512m.patch)（将 `kmem_io` 从 `1 GiB` 压至 `512 MiB`）。
  2. boot-args 必加：`vm_compressor=2 vm_compression_limit=524288`（把三大 kmem 池各扩至 **~2.1 GiB**）。

### 坑 3：无 Apple 硬件 AES 引擎导致 APFS / Keybag 卡死或崩溃
- **症状**：挂载 APFS 加密/系统卷或启动 `launchd` 时在 AES 指令/协处理器路径挂死。
- **解法**：链接 `libSoftwareAES.bound.a`（纯软件 AES 实现），在 prelink 阶段接管内核 AES 符号。

### 坑 4：IOKit 驱动被 Sandbox 拦死（用户态打不开 `IOUserClient`）
- **症状**：GPU 驱动（`Tb375fcMaliDriver`）或 Wi-Fi 驱动（`Tb375fcConnSys`）在 `root` 下能打开，但在沙盒进程（`WindowServer`/`backboardd`/App）中 `IOServiceOpen` 返回 `kIOReturnNotPermitted`。
- **解法**：应用 [xnu-iouserclient-sandbox-grant-all.patch](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/patches/xnu-iouserclient-sandbox-grant-all.patch)，在需要的 `IOService` 上设置 `setProperty("NativeSandboxGrantAll", kOSBooleanTrue)`。

### 坑 5：arm64e PAC 指针签名 & vtable slot 320 / `OSMetaClass` 静态初始化
- **症状**：新增 C++ IOKit 驱动链接进 prelinked kernelcache 后，启动即发生 PAC 校验异常或虚表跳转崩溃。
- **解法**：复用 `build_m7_kernel.py` + `apply_late_blocks_and_sdt_fixed.py` 的 late-block 重定位与 PAC 签名修正流程；每新增一个 `OSMetaClass` 静态构造对象，注意核对 `__mod_init_func` 大小增量（如 M7 记录的 `+0x8`）。

### 坑 6：LK 退出后关闭了 UFS 控制器（`HCE=0`）
- **症状**：XNU 启动后读 `0x112B0000` 看到 `HCE=0`，直接发命令超时。
- **解法**：直接复用 [Tb375fcUfsTransport.c](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/drivers/storage/Tb375fcUfsTransport.c) 的 Stage B 冷启动序列：`REG_UFS_XOUFS_CTRL |= 0x80` → `REG_UFS_REFCLK_CTRL = 1` → `MTK_SIP_UFS_CONTROL (0xC2000276)` → 复位 PHY → 拉高 `HCE=1` → UniPro `DME_LINKSTARTUP`。

### 坑 7：Data 卷空间误配引发 `ENOSPC` 与 Load 飙升至 200+
- **症状**：开机几分钟后系统卡死、平均负载飙到 200+、界面图标损坏（我们在 Boot 160 踩过：忘了传 `--data-seed` 导致误挂 244 MiB 临时内存盘，被系统缓存瞬间写满）。
- **解法**：持久化模式构建脚本**必须强校验/自动绑定大容量 Data 种子**（如 `macos-data-seed-128g.dbr`），绝不允许隐式回退到 256 MiB 易失种子。

---

## 5. 各核心子系统移植实战要点（推荐路线 M0 → M7）

### 5.1 显示与帧缓冲（iPadOS 重点注意 `RGhA` 与 120 Hz）

1. **早期阶段**：先沿用 LK 留下的 2944×1840 `BGRA8` 物理帧缓冲（`0xFC16F000`），用纯 CPU blit 验证图形上屏。
2. **⚠️ iPadOS 27 特有格式提醒**（已在 `NativeLegacyDisplay.cpp:133` 留下实测注释）：
   - 一旦注册了 Metal GPU 设备，**iPadOS 27 的渲染服务默认会优先选择 64-bit `RGhA`（RGBA16F half-float）作为主屏幕帧缓冲格式**；若显示驱动只接受 `BGRA`（`0x42475241`），所有 Metal 帧都会被拒绝导致黑屏！
   - 解决路径分两步走：
     - **先点亮**：支持 `RGhA`（`0x52476841`），直接复用 `NativeLegacyDisplay.cpp` 里的 **3 线程无锁 work-stealing 16-row + 4 KiB L1-resident LUT 转换器**（实测 1.25–2.16 ms/帧）。
     - **再提速**：把显示像素格式切回 **8-bit `BGRA8`**（显存带宽减半至 5.4 MB/帧），实测常规桌面动画直接提至 **77–88 FPS**，`bgra blit` 降至 **0.53–1.58 ms**！
3. **精准帧同步**：复用 `tb375fc_mali_wait_ws_seq`（精确等待最新提交序号完成再 blit，避免撕裂与多余 `waitAll` 卡顿）。

### 5.2 NT36532 触控屏（对 iPadOS 极度友好）

- 完整复用 [touch-fix/](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/kernel/touch-fix) 的实机验证参数：
  - **SPI2**：`clk26m=1`、`manual_cs=1`（GPIO12）、`bus_hz=1000000`（1 MHz）、`tick_dly=0`。
  - **硬件复位**：GPIO60 拉低 10 ms → 拉高 30 ms，再读 6-byte Trim ID 校验通信。
  - **SRAM 固件加载**：从本地读取 `novatek_ts_fw_tm.bin` 写入 NT36532 ILM/DLM SRAM（校验 `0x001FB533` 标志与校验和）。
  - **EINT9 硬件上升沿锁存**：映射 `0x11FE0000`（`eint-n`），监听 local **bit 5**（`MTK_EINT_STA` 只读判触发 + `MTK_EINT_ACK` 清锁存）——**空闲时 0 SPI 流量、0 CPU 占用、`csum_err=0`**。
- **iPadOS 专属改造**：iPadOS `backboardd` 原生消费标准多点触控 Digitizer（Usage Page `0x0D`）HID 报告；把 `Tb375fcTouchProbe.cpp` 里输出给 macOS 的相对/绝对鼠标报告换成多指 Digitizer 报告即可获得原生手势（Home 条上滑、控制中心下拉、双指缩放）。

### 5.3 Mali-G615 MC5 硬件 GPU + PanVK v11 + Metal2Vulkan

完整架构与实机踩坑细节见 [MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/MALI_G615_MACOS27_GPU_BRINGUP_GUIDE.md)，iPadOS 27 可直接抄作业的核心决策：

1. **跳过离进程 GPU Daemon，直接做进程内（In-Process）`arm64e` Metal 插件**：
   - 早期 M7 做过离进程 RPC Daemon，IPC 开销大且易死锁；Boot 148+ 改为进程内 `NativeAdrenoMetal.bundle`（内嵌 `metal2vulkan` 静态库 + `libvulkan_panfrost.dylib`）后延迟骤降且零崩溃。
2. **必须解锁 LPDDR5X 8528 Mbps + GPU 1400 MHz**：
   - 默认频率下内存带宽仅 ~14.8 GB/s，高分屏合成会严重瓶颈；开机通过 ATF SMC `0xC2000506`（`SW_REQ=0x9050`）解锁 **8528 Mbps（40.8 GB/s）** + GPUEB 切 **OPP0 1400 MHz** 是流畅度的前提。
3. **7 个硬件 AS/CSG 槽位分配（适配 iPadOS 主进程名）**：
   - `Slot 0`（`AS1 + CSG0`, `grpPrio=2`）永久固定给主合成器（在 macOS 上匹配 `WindowServer`，**在 iPadOS 上务必改成匹配 `backboardd` / 主 RenderServer 进程**）。
   - `Slots 1..6`（`AS2..7 + CSG1..6`）供 `SpringBoard` 和前台 App 通过真实 `CSG_STATE_SUSPEND` / `CSG_STATE_RESUME` LRU 复用。
4. **硬件 AFBC 零 Fault 开启条件**：
   - 必须在 `panvk_image.c` 的 `panvk_BindImageMemory2` 中对 private `OPTIMAL` 纹理的 **AFBC superblock header 区域清零**（否则 Mali 硬件解码器会读到脏头直接触发 `MMU FAULT`）。
   - 在 `Tb375fcMaliDriver.cpp` 中移除每次提交多余的 `WAIT(waitAll)` 与帧中 L2 invalidate（`0x233`）。实测 Boot 161 跑过 **6,453+ 次提交、223,456 个 render pass、659,399 次 draw call，0 MMU Fault**。
5. **iPadOS 特别关键的 `metal2vulkan` / Frontend 补丁**：
   - 必须保留 [native-bridge-metal-frontend-tb375fc.patch](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/patches/native-bridge-metal-frontend-tb375fc.patch) 中将 `MTLResourceStorageModeMemoryless` 转为 `MTLResourceStorageModePrivate` 的改动——iPadOS 深度依赖 Apple TBDR 的 memoryless 附件，不转会直接创建纹理失败。
   - 必须保留 [m2v-87a869f-panvk-v11.patch](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/patches/m2v-87a869f-panvk-v11.patch)（放宽 PanVK v11 暂未暴露的 `RobustImageAccess2` / `TessellationShader` / `ShaderClipDistance` floor 检查）。
6. **提前规避已知的 55+ pass 重动画瓶颈（macOS Boot 162 待优化项）**：
   - 我们在 macOS 舞台调度（55.5 pass/帧）中已定位到根因：`metal2vulkan` 每个 render pass 前后都发射 `ALL_COMMANDS → ALL_COMMANDS` 全局屏障（55–110 次/帧），导致 Mesa `panvk_vX_cmd_buffer.c` 把空闲的 `COMPUTE` 队列（`q=2`）也拖进 3 路 `SYNC64` 等待，并且每次屏障执行 3× `cs_flush_caches(OTHER=INVAL)` + 对空 `q=2` 做 `finish_cs` L2+LSC flush。
   - **iPadOS 的 SpringBoard 液态玻璃 / 多任务卡片同样是高频多 pass 渲染**，若同步把 `panvk` / `m2v` 这处 barrier 精简掉，macOS 27 与 iPadOS 27 将同时受益（目标把 55 pass 帧耗时从 ~36 ms 压到 <16 ms）。

### 5.4 原生电池、充电与背光（开箱即用）

- 直接复用 [Tb375fcPower.cpp](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/drivers/power/Tb375fcPower.cpp)：
  - `i2c@11d02000` 从地址 `0x55` 读 MM8013A 电量、电压、电流、温度。
  - `i2c5@11f01000` 从地址 `0x34` 配置 MT6375 充电输入限流 `aicr=2500mA`（默认不设时容易仅 500 mA 边用边掉电！）。
  - `disp_pwm0@1100e000` 调背光；发布标准 `AppleSmartBattery`（`IOPMPowerSource`）供系统电量图标与 `powerd` 读取。

### 5.5 原生 MT6897 Wi-Fi 6（`soc7_0` + MT6637）

- 完整复用 [drivers/wireless/](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/drivers/wireless) + [tb375fc-connsys-ctl.c](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/userspace/wireless/tb375fc-connsys-ctl.c) + [Tb375fcDummyEth.cpp](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/UniXNU-tb375fc/drivers/net/Tb375fcDummyEth.cpp)：
  1. 启动参数带 `connsys=1`（安全惰性挂接，不影响开机；进系统后由用户态触发上电，避免早期硬件异常拖慢开机）。
  2. 只读从平板 Android `vendor_a` 提取两份原厂固件 `WIFI_RAM_CODE_soc7_0_1_1.bin` 和 `soc7_0_ram_wmmcu_1_1_hdr.bin` 缓存到持久化卷。
  3. 上电链路：SPM/SRCLKEN/SPMI + ATF SMC `0xC2000534`（ConnInfra `0x02050400`）→ `0xC2000535`（WFSYS `0x02040200`）→ WFDMA0（`0x18024000`）+ EMI（`0x8CC00000`）+ 20 MiB DMA 池（`0x8D900000`）完成 FWDL → 轮询 ROM Ready `0x1D1E` → 802.11 扫描/关联 + WPA2-PSK 4 次握手 + AES-CCMP → 桥接到 `en0`。
  4. **注意**：iPadOS 校园网/Portal 认证在无浏览器弹窗阶段不方便输入学号密码，初期联调强烈推荐用 **手机 5 GHz WPA2-PSK 热点** 验证（非 ASCII 中文 SSID 在串口 shell 中务必用 `$(printf '...')` 转义）。

---

## 6. 高频故障速查表（161 次实机总结）

| 现象 | 最常见根因 | 快速排查 / 解法 |
| :--- | :--- | :--- |
| 刷机后连 Apple Logo 都不出 / 秒重启 | `kernel.lzma` 或 `fatpart.lzma` 损坏；或 `Layout.h` 偏移越界 | 离线跑 `verify_lzmadec`；核对 `desktop-tier3-manifest.json` 长度 |
| 卡 Apple Logo 不动 | ① Data 卷满（`ENOSPC`）；② APFS/Preboot UUID 不匹配；③ 新驱动 panic | 连 USB CDC 看最后 50 行 `kprintf`；检查 `df -h` 与 `policies/*/verification.json` |
| 开机几分钟后突然 Panic | `kmem` 池耗尽（`KERN_NO_SPACE`） | 确认已打 `xnu-vm-kern-kmem-io-512m.patch` 且 boot-args 有 `vm_compression_limit=524288` |
| 进系统后黑屏但 CDC 有响应 | 合成器切换到了 `RGhA` 但显示驱动拒收；或 `IOSurface` 映射失败 | 查 `NativeLegacyDisplay` 统计里的 `refused` 格式码（`0x52476841` = `RGhA`） |
| 进系统后 GPU 报 `MMU FAULT` | AFBC 纹理头未清零；或 `mali` PTE/cache 属性不对 | 检查 `panvk_BindImageMemory2` 是否清零 AFBC header |
| 动画卡顿但 GPU 占用不高 | ① 忘了开 DVFSRC 8528 Mbps；② 仍在用 `RGhA` 16-bit FB；③ 55+ pass `ALL_COMMANDS` barrier | 核对启动日志 `DVFSRC SW_REQ=0x9050`、`GPU OPP0 1400MHz`、`fmt=BGRA` |
| 边插电边掉电 | MT6375 默认限流 500 mA | boot-args 加 `aicr=2500` 并确认 `Tb375fcPower` I2C5 `0x34` 写入成功 |
| 串口 `pgrep` / `killall` 报错或命令截断 | 精简用户态无 `sysmond`；USB CDC 单行缓冲区有限 | 用 `ps ax` + `kill -TERM <PID>`；CDC 命令保持 `< 200` 字符 |

---

## 7. 推荐的 iPadOS 27 里程碑推进顺序

1. **M0（安全基线）**：独立工作目录，不碰 macOS 27 任何目录与 LUN2 `[16350000..49906176)`；确认 `fastboot-noclear` + `Boot 161` 随时可 3 分钟救砖。
2. **M1（Post-MMU + 8 核 SMP + USB CDC Shell）**：复用 UEFI + `start.s` + `ArmCpu.cpp` + `libSoftwareAES.bound.a` + USB CDC，先拿到稳定可交互的 iPadOS 27 内核串口。
3. **M2（只读 UFS + PID 1 `launchd` + TrustCache/AMFI）**：复用只读 `Tb375fcUfsTransport.c` 挂载 iPadOS 27 OS/Preboot 卷，过 `launchd` 与签名校验（注意 iPadOS 沙盒/MobileKeyBag）。
4. **M3（CPU Blit 点亮 `SpringBoard`）**：先不开 GPU，确认 `backboardd` / `SpringBoard` 能以 CPU 渲染方式出画面，同时接入 `Tb375fcPower.cpp`（电池/快充/背光）。
5. **M4（NT36532 原生多点触控）**：复用 `touch-fix/` + `EINT9`，改成标准 Digitizer 多点触控 HID 报告，实机可点按滑动 `SpringBoard`。
6. **M5（Mali-G615 进程内硬件 GPU + 8528 Mbps + BGRA8/AFBC）**：将 `Slot 0` 绑定到 `backboardd`/主渲染进程，接入 `NativeAdrenoMetal.bundle`（含 `Memoryless → Private` 与 AFBC header 清零），同步应用 Boot 162 barrier 精简优化。
7. **M6（持久化独立 LBA 区 + 原生 Wi-Fi 6 `soc7_0`）**：在 LUN2 高位空闲 LBA 划出独立 iPadOS Data 区，复用 `Tb375fcConnSys` + `tb375fc-connsys-ctl` 打通原生 Wi-Fi 6。

---

## 8. 仓库与关键复用文件索引

- **公开干净源码仓（DMCA-Clean）**：`https://github.com/liimagination/UniXNU-tb375fc`（分支 `feat/mali-panvk-m2v2`）
- **私人完整备份仓（含固件、Boot 161/70 镜像、预编译产物、全量源码与关键脚本）**：`https://github.com/liimagination/UniXNU-tb375fc-private-backup`（分支 `main` / `backup/boot161`）
- **本地参考参考 Linux 硬件源码**：`ram-expansion/refs/android_kernel_modules_tb375fc`（后续做蓝牙/音频/USB-C Host 查寄存器专用）
