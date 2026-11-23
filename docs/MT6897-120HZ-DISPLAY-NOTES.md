# MT6897 (TB375FC) 原生显示与 120 Hz 寄存器速查表（P3 离线整理）

来源：[live-fdt.dts](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/touch-prep/live-fdt.dts)、[panel-nt36532-dsi-vdo-tianma.ko](file:///Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/touch-prep/stock-ramdisk/panel-nt36532-dsi-vdo-tianma.ko) 与 Boot 46 实测 `GICD_TYPER=0x79051e`（支持 SPI INTID 最大 `991`）。

## 1. 双链路显示管线 MMIO 基址与 GIC 中断号

| 模块 | 物理基址 | GIC SPI（DTS） | GIC INTID（`SPI + 32`） | 备注 |
| :--- | :--- | :--- | :--- | :--- |
| `dispsys-config` (MMSYS0/1/OVLSYS0/1) | `0x14000000`, `0x14200000`, `0x14400000`, `0x14600000` | — | — | 每块 `0x1000`，主路由与时钟门控 |
| `disp-mutex0..3` | `0x14001000`, `0x14201000`, `0x14401000`, `0x14601000` | `0x162`, `0x182` | `386`, `418` | **P3 首选硬件帧完成 / SOF / EOF 事件源** |
| `dsi0` / `dsi1`（双 DSI） | `0x1400d000`, `0x1420d000` | `0x170`, `0x19c` | `400`, `444` | 双链路事件需按同一次面板刷新合并去重 |
| `disp-dsc-wrap0` / `wrap1` | `0x1400c000`, `0x1420c000` | `0x16e`, `0x16f` | `398`, `399` | 双 DSC 压缩封装 |
| `disp-ovl0..3` (Pipe 0) | `0x14402000`..`0x14405000` | `0x183`..`0x186` | `419`..`422` | 帧缓冲扫描输出（`fb_base=0xfc16f000`, `vram=0x3e90000`） |
| `disp1-ovl0..3` (Pipe 1) | `0x14602000`..`0x14605000` | `0x1af`..`0x1b2` | `463`..`466` | 右半屏双管线 OVL |
| `disp_pwm0` | `0x1100e000` | — | — | 已在 `Tb375fcPower.c` 实机验证背光调光 |
| 双路背光 I2C | `i2c_lcd_backlight@36`, `i2c_lcd_second_backlight@36` | — | — | 主/副背光驱动 |

> 所有显示 IRQ INTID（`386`–`466`）均远小于 MT6897 GICv3 `max_spi = 992`，可直接通过 `ArmCpu` 的 `armcpu_spi_attach()` 注册真实硬件 vblank 中断！

## 2. 本机 Tianma NT36532 面板模式符号（`panel-nt36532-dsi-vdo-tianma.ko`）

- 面板标识：`nt36532_dsi_vdo_144hz_tianma_lcm_drv`（`atag,videolfb`）
- 模块内建模式：`default_mode`（144 Hz）、`performance_mode_120hz`（`ext_params_120hz`）、`performance_mode_60hz`（`ext_params_60hz`）、`performance_mode_30hz`（`ext_params_30hz`）
- 切换入口：`mode_switch()` → `mode_switch_to_120()`（`0x226c`）、`mode_switch_to_144`
- **120 Hz 解码时序**：`2944×1840`，`HTotal=3132`，`VTotal=2050`，**`PCLK = 770.472 MHz`**（`770,472,000 / (3132 × 2050) = 120.000 Hz`，帧周期 `8.333 ms`）。
- **macOS EDID 限制**：传统 EDID 1.4 Detailed Timing Descriptor 的 16-bit 像素时钟上限为 `655.35 MHz`，无法表达 `770.472 MHz`；P3 D3 阶段在 `NativeLegacyDisplay.cpp` 暴露 120 Hz 时必须走 **DisplayID 1.3/2.0 Type I/VII Timing** 或直接配置 **IOAV Timing** 描述。

## 3. P3 首批只读探测建议（零写寄存器）

1. 在保持现有 GOP 帧缓冲不变的前提下，只读映射 `disp-mutex0@14001000` 和 `dsi0@1400d000` / `dsi1@1420d000`，轮询/统计帧完成中断状态位与 `cntvct_el0` 间隔，先量出当前 UEFI/LK 留下的物理刷新率（60 Hz / 120 Hz / 144 Hz）。
2. 确认双 DSI 事件合并策略后，再通过 `armcpu_spi_attach(386, ...)` 接管真实硬件 vblank 替代 `vsync=` 软定时器。
