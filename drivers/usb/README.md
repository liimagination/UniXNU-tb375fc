# TB375FC P1 USB-C Host Mode Bring-Up (Offline Prep)

> **安全警示**：TB375FC 仅有一个 USB-C 接口。启用 `usb-host=1` 会将控制器从 USB CDC Device 模式切换到 xHCI Host 模式并开启 MT6375 5V OTG 升压供电，**将立即断开与 Mac 的 USB CDC 串口连接**。请务必**仅在人在设备旁、可按音量下+电源键进 Fastboot 恢复时**测试！

## 硬件拓扑（已从 `live-fdt.dtb` 与 MT6897 Linux 6.1 BSP 实证）

| 模块 | 物理基址 | 大小 | 中断 / 总线 | 用途 |
|------|----------|------|-------------|------|
| `xhci0@11200000` | `0x11200000` | `0x1000` | GIC SPI `0xa1` (IRQ 193) | 标准 xHCI 1.1 主机控制器 MMIO |
| `mtu3@11201000` (MAC) | `0x11201000` | `0x2e00` | GIC SPI `0xa0` (IRQ 192) | MediaTek USB3 DRD 设备端 MAC |
| `mtu3` IPPC | `0x11203e00` | `0x100` | MMIO | IP 电源/时钟/U2&U3 端口 Host/Device 选择 |
| `usb2-phy0@11e40000` | `0x11e40000` | `0x400` | `mediatek,mt6897-xsphy` | USB2 PHY（`XSP_U2PHYDTM1` 强制 `IDDIG=0` Host 模式） |
| `usb3-phy0@11e43000` | `0x11e43000` | `0x200` + `0x500` | `mediatek,mt6897-xsphy` | USB3.1 Gen2 PHY |
| `mt6375@34` (OTG Boost) | `i2c5@11f01000` | slave `0x34` | I2C5 | 5.0V / 1.3A OTG VBUS 升压（供 USB-C Hub/网卡/蓝牙适配器供电） |

## 启用方式（后续回到设备旁测试）

1. 将已离线编译验证通过的 `Tb375fcUsbHost.o` 加入 `link.filelist`。
2. **第一阶段（安全只读探测）**：添加 boot-arg `usb-host=2`（不切换设备模式、不开启升压，仍保持 CDC 串口，仅读取 IPPC & xHCI 能力寄存器）。
3. **第二阶段（真机 Host 实测）**：添加 boot-arg `usb-host=1` 并插入 USB-C 扩展坞 + USB 键鼠 / AX88179 USB 网卡 / USB 蓝牙适配器；确认 `PORTSC` 检测到外设后，再对接 XNU 内置的 `AppleUSBXHCI`（内核中已包含完整 `AppleUSBXHCI` / `AppleUSBXHCIPort` / `IOUSBHostDevice` / `IOBluetoothHostControllerUSBTransport` 类符号）。
