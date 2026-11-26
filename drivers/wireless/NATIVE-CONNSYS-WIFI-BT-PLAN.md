# TB375FC（MT6897 / 天玑 8300）原生板载 ConnSys Wi-Fi & 蓝牙 Bring-Up 方案

> **当前状态（2026-11-26 离线准备完成）**
> - **线上平板（Boot 152）未被触碰 / 未重启 / 未刷机**：可直接上手验收 GPU（1400 MHz + 8528 Mbps LPDDR5X + AFBC + 1.3 ms blit）、流畅度、电量显示与台前调度。
> - **不走 USB 网卡 / 不走 USB-C Host**：本方案完全面向 **MT6897 原生板载 ConnSys（Wi-Fi 6 `soc7_0` / Connac 2.x + 蓝牙 5.x BGF/BEIF + 外置 MT6637 A-Die）**。
> - **离线内核基础驱动已编写并编译通过**：[`Tb375fcConnSys.cpp`](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/kernel/connsys/Tb375fcConnSys.cpp)（默认 `connsys=0` 完全静默，未链接进 Boot 152）。

---

## 1. 为什么原生板载方案比预期更可行（三大关键硬件突破）

通过核对 TB375FC 真机设备树（`live-fdt.dtb`）与 MT6897 硬件架构，确认以下三大关键优势大幅降低了原生 Wi-Fi / 蓝牙适配门槛：

1. **EL3 BL31 ATF 已内置绝大部分底层寄存器时序（可直接 SMC 调用）**
   - `MTK_SIP_KERNEL_CONNSYS_CONTROL` (`0xC2000534`)：封装了 D-Die 初始化、EMI Remap、RFSPI Master 配置、MT6637 A-Die 探测与配置、AFE 校准、子系统 PLL 与低功耗配置。
   - `MTK_SIP_KERNEL_BT_CONTROL` (`0xC2000536`)：封装了蓝牙 BGF MCU 三段式上电（`PWR_ON_{TOP,MID,END}`）、`CONN_INFRA_FORCE_ON_OFF`、FW Own 控制与 **BEIF 门铃中断（`SMC_BT_SET_BEIF_REG = 22`）**。
   - `MTK_SIP_KERNEL_WLAN_CONTROL` (`0xC2000535`)：封装了 Wi-Fi `PCCIF_ON/OFF` 与 `ENABLE_FWDL_MODE`。
2. **Preloader / LK 已经完成共享 EMI 保留与 MPU 权限配置**
   - `consys_emi_reserved`：`0x8cc00000`（`13 MiB`，`0x00d00000`），且 MT6897 平台明确注明 **EMI MPU 已由 LK 配置完毕**，内核无需重写安全域 MPU。
   - `wifi-reserve-memory_dma`：`0x8d900000`（`20 MiB`，`0x01400000`），天然连续物理内存池，直接供 Wi-Fi WFDMA 使用。
3. **当前 XNU 内核已完整内置 Apple 无线框架（零额外 kext 依赖）**
   - **蓝牙**：`xnu-cdc-board.macho` 已内置完整 `IOBluetoothFamily`（含 `IOBluetoothHCIController` 虚表、`SendACLData`、`ProcessACLData`、`IOBluetoothL2CAPChannel`、`IOBluetoothHIDChannel`）。
   - **Wi-Fi**：已内置 `IO80211Controller` / `IO80211SkywalkInterface`；且 UniXNU 上游已验证 `WifiNet.cpp` + `nw_abi.h` 架构（只需将硬件后端替换为 MT6897 AXI WFDMA，OS 层与系统设置 / `wifid` 接入路径可直接复用）。

---

## 2. TB375FC 原生硬件资源清单（`live-fdt.dtb` 实测核对）

| 子系统 | 物理地址 / 参数 | 大小 / 说明 |
| --- | --- | --- |
| **SPM 电源域** | `0x1c001000` (`+0xE04`) | `MT6897_POWER_DOMAIN_CONN` (`CONN_PWR_CON`) |
| **ifrbus_ao 总线保护** | `0x1002c000` | `prot0` (`+0x000..0x00c`, bit 25/26) & `connsys0` (`+0x1c0..0x1cc`, bit 1/0) |
| **SPMI-M PMIF** | `0x1cc04000` (`SWINF_2 = +0x880`) | 无 `spmi-dev-mask`，全 PMIC 走 SPMI-M |
| **MT6363 主 PMIC** | SPMI `slvid = 0x4` | `VRFIO18` (`EN=0x1bcd`, `VOSEL=0x1f2c` -> `1.7V`)、`VCN13` (`EN=0x1d07`, `VOSEL=0x1b3f` -> `1.35V`) |
| **MT6368 副 PMIC** | SPMI `slvid = 0x5` | `VCN33_1` (`EN=0x1cc0` -> `3.3V`)、`VCN33_2` (`EN=0x1cce` -> `3.3V`)、`VANT18` (`EN=0x1c23` -> `1.8V`) |
| **MT6685 时钟缓冲** | SPMI `slvid = 0x9` | DCXO / `XO-CONCK1`, `XO-CONCK2` |
| **ConnInfra D-Die** | `0x18000000` / `0x18011000` | `CONN_HW_VER = 0x02050400`；A-Die = `MT6637` (`0x66378A0x`) |
| **BT BGFSYS** | `0x18800000` / `0x18812000` | `BGF_IP_VER = 0x02080000`、`SW_DBG_CTL (+0x16C) = 0x1D1E` (ROM Ready) |
| **BT 中断 (GIC SPI)** | `0x37c` (924), `0x266` (646), `0x37b` (923) | BGF2AP BTIF / SW / Wakeup IRQ |
| **BT BEIF HCI 环** | `0x8D7A0000` (`0x8cc00000 + 0xBA0000`) | `32 KiB` 无锁双工共享内存环（`key=0x55665566`, `guard=0xDEADBEEF`），直接传输标准蓝牙 **H4 HCI** 包 |
| **Wi-Fi `soc7_0` AXI** | `0x18000000` (`size 0x700000`) | Connac 2.x (`chip_id=0x7961`)，WFDMA Host DMA0 @ `0x18024000` |
| **Wi-Fi 中断 (GIC SPI)** | `0x379` (921), `0x265` (645) | WFDMA IRQ & CONN2AP SW IRQ |
| **Wi-Fi DMA 池** | `0x8d900000` (`20 MiB`) | LK 预留连续物理 DMA 池 |

---

## 3. 必需的板载固件文件（仅本地使用，严禁上传公开仓库）

回宿舍后，可从原厂 Android 分区 `/vendor/firmware/`（或 `vendor_a` 镜像）提取以下固件到平板 macOS 侧 `/usr/local/firmware/mt6897/`（或打包到私有 payload）：

- **蓝牙（Stage 2 必需）**：
  - `soc7_0_ram_mcu_1_1_hdr.bin`（BGF MCU RAM 补丁）
  - `soc7_0_ram_bt_1_1_hdr.bin`（BT 协议栈 RAM 固件）
- **Wi-Fi（Stage 3 必需）**：
  - `WIFI_RAM_CODE_soc7_0_1_1.bin`（Wi-Fi WM/WA RAM 固件）
  - `soc7_0_ram_wmmcu_1_1_hdr.bin`（WMMCU 补丁）
  - `soc7_0_patch_mcu_1_1_hdr.bin`（若 `/vendor/firmware/` 存在）
  - `wifi.cfg` / `txpwr.dat`（可选，板级射频校准/功率表）

> **注意**：上述 `.bin` 属于厂商专有二进制固件，**只能放本地或私有备份仓**，严禁提交到公开仓 `UniXNU-tb375fc`。

---

## 4. 分阶段 Bring-Up 路线（回宿舍验收完 Boot 152 后按序推进）

### Phase 0：安全只读探针（`connsys=1`，零写操作、零风险）
1. 将 `Tb375fcConnSys.o` 链接进下一个实验内核（保留 Boot 152 镜像不动），传入 boot-arg `connsys=1`。
2. 验证输出：
   - `SPM_CONN_PWR_CON (0x1c001e04)` 与 `ifrbus_ao (0x1002c000)` 初始状态；
   - `SPMI-M SWINF_2` 成功读出 `MT6363 (slv=4)`、`MT6368 (slv=5)`、`MT6685 (slv=9)` 的 `HWCID` 与 5 路 LDO 当前状态；
   - `BEIF EMI (0x8D7A0000)` 映射正常且无总线异常。

### Phase 1：`conninfra` 上电与 D-Die / A-Die 握手（`connsys=2`）
- 使能 `MT6363 VRFIO18 (1.7V)` + `VCN13 (1.35V)` -> 打开 `SPM CONN` 电源域 -> 释放 `ifrbus_ao` 总线保护 -> 顺序调用 BL31 `MTK_SIP_KERNEL_CONNSYS_CONTROL`：
  `INIT_ATF_DATA (0)` → `POLLING_CHIPID (10)` → `EMI_SET_REMAPPING (70)` → `D_DIE_CFG (11)` → `SPI_MASTER_CFG (12)` → `A_DIE_CFG (13)` → `AFE_WBG_CAL (15)` → `SUBSYS_PLL_INITIAL (16)` → `LOW_POWER_SETTING (17)`。
- **验收标准**：`CONN_HW_VER == 0x02050400` 且 `A_DIE_HW_VER == 0x66378A0x`（MT6637），无总线 hang。此阶段同时是蓝牙和 Wi-Fi 的共同底座。

### Phase 2：优先攻克原生蓝牙（`connsys=3` → 固件加载 → `IOBluetoothHCIController`）
> **为什么先做蓝牙？** 蓝牙协议面极薄：只要 BGF MCU 上电 + 加载两个固件 + 打通 `0x8D7A0000` 的 BEIF 共享内存环，就能直接收发标准 **Bluetooth H4 HCI** 指令包，苹果自带 `IOBluetoothFamily` 立刻接管全套配对、HID 键鼠/触控笔与音频！

1. **Stage 2a（已编写在 `Tb375fcConnSys.cpp`）**：`connsys=3` 调用 `MTK_SIP_KERNEL_BT_CONTROL` `PWR_ON_{TOP,MID,END}`，验证 `BGF_IP_VERSION == 0x02080000`、`BGF_MCU_CFG_SW_DBG_CTL == 0x1D1E`，并初始化 `0x8D7A0000` 32 KiB BEIF 环。
2. **Stage 2b（固件加载 + HCI Reset 闭环）**：
   - 将 `soc7_0_ram_mcu_1_1_hdr.bin` 与 `soc7_0_ram_bt_1_1_hdr.bin` 经 EMI / STP 分片发送至 BGF ROM（或写入 RO BT EMI 窗口 `0x8cc00000` 偏移区 + 触发固件校验）。
   - 注册 GIC SPI `0x266` / `0x37c` 中断回调调用 `tb375fc_bt_beif_recv_hci()`。
   - 发送标准 H4 `HCI_Reset`（`01 03 0C 00`），收到 `Command Complete`（`04 0E 04 01 03 0C 00`）即宣告原生蓝牙硬件链路打通！
3. **Stage 2c（接入 macOS `IOBluetoothFamily`）**：
   - 实现轻量子类 `Tb375fcBluetoothHCIController : public IOBluetoothHCIController`，重写 `SendHCIRequest` / `SendACLData` 映射到 `tb375fc_bt_beif_send_hci()`，在 BEIF RX 回调中按 H4 包类型（`0x04 Event` / `0x02 ACL`）分发给 `ProcessEventData` / `ProcessACLData`。

### Phase 3：原生 Wi-Fi 6 `soc7_0`（Connac 2.x AXI WFDMA + UniXNU `WifiNet`）
1. **Stage 3a（已编写探针在 `Tb375fcConnSys.cpp`）**：`connsys=4` 打开 `MT6368 VCN33_1/VCN33_2/VANT18`，调用 `MTK_SIP_KERNEL_WLAN_CONTROL (0xC2000535)` `SMC_WLAN_PCCIF_ON (3)`，验证 `WF_WFDMA_HOST_DMA0 (0x18024000)` 可读及 `0x8d900000`（20 MiB）DMA 池映射。
2. **Stage 3b（WFDMA 环 + 固件加载）**：
   - 在 `0x8d900000` DMA 池上分配 Connac 2.x TX/RX 描述符环：TX16（FWDL）、TX15（MCU CMD）、TX0..2（AP DATA）、RX0/1（AP DATA）、RX2（MCU EVENT）、RX3（TX DONE）。
   - 调用 `SMC_WLAN_ENABLE_FWDL_MODE (5)`，通过 TX16 下载 `soc7_0_ram_wmmcu_1_1_hdr.bin` 与 `WIFI_RAM_CODE_soc7_0_1_1.bin`，等待 `CONNSYS_ROM_DONE_CHECK (0x1D1E)` 与 FW Ready Event。
3. **Stage 3c（复用 UniXNU `nw_abi.h` + `WifiNet.cpp`）**：
   - 参考 `UniXNU/docs/zh/porting/network.md` 与 `UniXNU/Ath12kWifi` 的 `nw_abi.h`（`nw_wifi_scan` / `nw_wifi_connect` / `nw_wifi_tx` / `nw_wifi_events`），封装一层精简的 `mt6897_soc7_0` STA 状态机（Scan / Auth / Assoc / 4-way handshake / 802.3 TX/RX），直接挂接到 UniXNU 已验证的 XNU `IO80211Controller` / `Apple80211` OS 层，无缝接入 macOS 系统设置 Wi-Fi。
