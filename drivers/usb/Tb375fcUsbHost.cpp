/*
 * Tb375fcUsbHost.cpp — Offline-Prepared MT6897 / TB375FC USB-C Host Mode Bring-Up Driver
 *
 * Strictly isolated in local-artifacts/macos27-firstboot/ram-expansion/kernel/usb-host/.
 *
 * SAFETY WARNING:
 *   TB375FC has ONLY ONE USB-C port. Enabling usb-host=1 switches the single USB-C
 *   controller (mtu3@11201000 / xhci0@11200000) out of USB CDC Device mode into
 *   xHCI Host mode and turns on MT6375 5V OTG VBUS boost. This WILL disconnect the
 *   Mac CDC serial console!
 *
 *   NEVER enable usb-host=1 remotely/unattended! Only test when physically beside
 *   the tablet (able to hold Vol-Down + Power to return to Fastboot if needed).
 *
 * Boot-arg control (default OFF = 0):
 *   usb-host=0 (default): Completely inert, does not touch USB or MT6375.
 *   usb-host=2          : Safe read-only diagnostic probe of mtu3 IPPC + xHCI caps
 *                         (leaves USB CDC Device mode intact).
 *   usb-host=1          : Full USB Host bring-up:
 *                         1. MT6375 5V / 1.3A OTG VBUS boost (i2c5@11f01000, slave 0x34)
 *                         2. MT6897 xsphy USB2 PHY Host IDDIG override (0x11e40000)
 *                         3. MT6897 mtu3 IPPC switch Device -> xHCI Host (0x11203e00)
 *                         4. xHCI 1.1 MMIO probe & root-hub PORTSC scan (0x11200000, SPI 0xa1)
 *
 * All register addresses and bit masks verified from:
 *   - TB375FC live-fdt.dtb
 *   - MT6897 Linux 6.1 BSP (mtu3_hw_regs.h, mtu3_host.c, phy-mtk-xsphy.c, mt6375-charger.c)
 */

#include <stdint.h>
#include <stdbool.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>

extern "C" {
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void delay(int usec);
extern void kprintf(const char *fmt, ...);
}

/* ---- MMIO helpers ---- */
static inline uint32_t u_r32(vm_offset_t base, uint32_t off) {
    uint32_t v = *(volatile uint32_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}
static inline void u_w32(vm_offset_t base, uint32_t off, uint32_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint32_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}
static inline uint16_t u_r16(vm_offset_t base, uint32_t off) {
    uint16_t v = *(volatile uint16_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}
static inline void u_w16(vm_offset_t base, uint32_t off, uint16_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint16_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}
static inline uint8_t u_r8(vm_offset_t base, uint32_t off) {
    uint8_t v = *(volatile uint8_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}
static inline void u_w8(vm_offset_t base, uint32_t off, uint8_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint8_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}

/* ---- Hardware Physical Addresses (TB375FC live-fdt.dtb) ---- */
#define PHYS_XHCI0_BASE         0x11200000ULL  /* size 0x1000, GIC SPI 0xa1 (IRQ 193) */
#define PHYS_MTU3_MAC_BASE      0x11201000ULL  /* size 0x2e00, GIC SPI 0xa0 (IRQ 192) */
#define PHYS_MTU3_IPPC_BASE     0x11203e00ULL  /* size 0x100  (page 0x11203000 + 0xe00) */
#define PHYS_XSPHY_U2_BASE      0x11e40000ULL  /* size 0x400  (usb2-phy0) */
#define PHYS_XSPHY_U3_GLB_BASE  0x11e43000ULL  /* size 0x200  (xsphy global) */
#define PHYS_I2C5_MT6375        0x11f01000ULL  /* i2c5 -> MT6375 PMU @ 0x34 */

#define TB375FC_XHCI_SPI        0xa1U

/* ---- MT6897 mtu3 IPPC registers (relative to 0x11203e00) ---- */
#define U3D_SSUSB_IP_PW_CTRL0   0x0000U
#define U3D_SSUSB_IP_PW_CTRL1   0x0004U
#define U3D_SSUSB_IP_PW_CTRL2   0x0008U
#define U3D_SSUSB_IP_PW_STS1    0x0010U
#define U3D_SSUSB_IP_PW_STS2    0x0014U
#define U3D_SSUSB_OTG_STS       0x0018U
#define U3D_SSUSB_IP_XHCI_CAP   0x0024U
#define U3D_SSUSB_IP_DEV_CAP    0x0028U
#define U3D_SSUSB_U3_CTRL_0P    0x0030U
#define U3D_SSUSB_U2_CTRL_0P    0x0050U
#define U3D_SSUSB_HW_ID         0x00A0U

#define SSUSB_IP_SW_RST         (1U << 0)
#define SSUSB_IP_HOST_PDN       (1U << 0)
#define SSUSB_IP_DEV_PDN        (1U << 0)

#define SSUSB_U3_MAC_RST_B_STS  (1U << 16)
#define SSUSB_XHCI_RST_B_STS    (1U << 11)
#define SSUSB_SYS125_RST_B_STS  (1U << 10)
#define SSUSB_REF_RST_B_STS     (1U << 8)
#define SSUSB_SYSPLL_STABLE     (1U << 0)

#define SSUSB_U3_PORT_HOST_SEL  (1U << 2)
#define SSUSB_U3_PORT_PDN       (1U << 1)
#define SSUSB_U3_PORT_DIS       (1U << 0)

#define SSUSB_U2_PORT_RG_IDDIG     (1U << 12)
#define SSUSB_U2_PORT_FORCE_IDDIG  (1U << 11)
#define SSUSB_U2_PORT_VBUSVALID    (1U << 9)
#define SSUSB_U2_PORT_OTG_SEL      (1U << 7)
#define SSUSB_U2_PORT_HOST         (1U << 2)
#define SSUSB_U2_PORT_PDN          (1U << 1)
#define SSUSB_U2_PORT_DIS          (1U << 0)
#define SSUSB_U2_PORT_HOST_SEL     (SSUSB_U2_PORT_VBUSVALID | SSUSB_U2_PORT_HOST)

/* ---- MT6897 xsphy USB2 PHY registers (relative to 0x11e40000) ---- */
#define XSP_USBPHYACR0          0x0300U
#define XSP_USBPHYACR6          0x0318U
#define XSP_USBPHYACR4          0x0320U
#define XSP_U2PHYDTM0           0x0368U
#define XSP_U2PHYDTM1           0x036CU

#define P2A0_RG_INTR_EN         (1U << 5)
#define P2A6_RG_BC11_SW_EN      (1U << 23)
#define P2A6_RG_OTG_VBUSCMP_EN  (1U << 20)
#define P2A4_U2_GPIO_CTR_MSK    ((1U << 9) | (1U << 8))

#define P2D_FORCE_UART_EN       (1U << 26)
#define P2D_FORCE_SUSPENDM      (1U << 18)
#define P2D_RG_SUSPENDM         (1U << 3)

#define P2D_RG_UART_EN          (1U << 16)
#define P2D_FORCE_IDDIG         (1U << 9)
#define P2D_RG_VBUSVALID        (1U << 5)
#define P2D_RG_SESSEND          (1U << 4)
#define P2D_RG_AVALID           (1U << 2)
#define P2D_RG_IDDIG            (1U << 1)

/* ---- MT6897 I2C5 polling registers (for MT6375 OTG boost) ---- */
#define I2C_DATA_PORT           0x00
#define I2C_INTR_MASK           0x08
#define I2C_INTR_STAT           0x0C
#define I2C_CONTROL             0x10
#define I2C_TRANSFER_LEN        0x14
#define I2C_TRANSAC_LEN         0x18
#define I2C_START               0x24
#define I2C_FIFO_ADDR_CLR       0x38
#define I2C_TRANSFER_LEN_AUX    0x44
#define I2C_SLAVE_ADDR1         0x94

#define I2C_INTR_COMP           (1U << 0)
#define I2C_INTR_ERR_MASK       ((1U << 1) | (1U << 2) | (1U << 3))
#define I2C_INTR_ALL            0x01FFU

#define I2C_CTRL_RS             (1U << 1)
#define I2C_CTRL_CLK_EXT_EN     (1U << 3)
#define I2C_CTRL_DIR_CHANGE     (1U << 4)
#define I2C_CTRL_ACKERR_DET     (1U << 5)
#define I2C_CTRL_LEN_CHANGE     (1U << 6)

/* ---- MT6375 PMU Bank 0x01 (7-bit I2C slave 0x34) OTG Boost Registers ---- */
#define MT6375_SLAVE_PMU        0x34
#define MT6375_REG_DEV_INFO     0x00
#define MT6375_REG_TM_PAS_CODE1 0x07  /* hidden-mode unlock: write 0x69 / lock: 0x00 */
#define MT6375_REG_CHG_TOP1     0x20  /* bit0=CHG_EN, bit1=BUCK_EN, bit2=OTG_EN */
#define MT6375_REG_CHG_TOP2     0x21  /* bits[7:6]=CLK_FREQ */
#define MT6375_REG_CHG_VSYS     0x29  /* bit7=BLEED_DIS_EN */
#define MT6375_REG_OTG_V        0x31  /* bits[5:0]=OTG_CV: 4.85V + (sel-20)*25mV -> 26=5.00V */
#define MT6375_REG_OTG_C        0x32  /* bits[2:0]=OTG_CC: 0=0.5A,1=0.7A,2=1.1A,3=1.3A,4=1.8A */
#define MT6375_REG_CHG_DUMY0    0x35  /* bits[1:0]=COMP_CLAMP */
#define MT6375_REG_CHG_HD_BUCK5 0x40  /* bits[7:6]=BUCK_RAMPOFT */
#define MT6375_MSK_OTG_EN       0x04
#define MT6375_VID_MASK         0xF0
#define MT6375_VID_EXPECTED     0x70

/* ---- xHCI 1.1 Capability & Operational Registers ---- */
#define XHCI_CAPLENGTH_VER      0x00U
#define XHCI_HCSPARAMS1         0x04U
#define XHCI_HCSPARAMS2         0x08U
#define XHCI_HCSPARAMS3         0x0CU
#define XHCI_HCCPARAMS1         0x10U
#define XHCI_DBOFF              0x14U
#define XHCI_RTSOFF             0x18U

#define XHCI_OP_USBCMD          0x00U
#define XHCI_OP_USBSTS          0x04U
#define XHCI_OP_PORTSC_BASE     0x400U

#define XHCI_USBCMD_RUN         (1U << 0)
#define XHCI_USBCMD_HCRST       (1U << 1)
#define XHCI_USBSTS_HCH         (1U << 0)
#define XHCI_USBSTS_CNR         (1U << 11)

static int i2c5_wait_done(vm_offset_t base) {
    for (int i = 0; i < 2000; i++) {
        uint16_t st = u_r16(base, I2C_INTR_STAT);
        if (st & (I2C_INTR_COMP | I2C_INTR_ERR_MASK)) {
            u_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
            return (st & I2C_INTR_ERR_MASK) ? -1 : 0;
        }
        delay(10);
    }
    u_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
    return -2;
}

static int i2c5_read8(vm_offset_t base, uint8_t slave7, uint8_t reg, uint8_t *out) {
    if (!base || !out) return -3;
    u_w16(base, I2C_INTR_MASK, 0);
    u_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
    u_w16(base, I2C_FIFO_ADDR_CLR, 0x0001);
    u_w16(base, I2C_CONTROL, I2C_CTRL_ACKERR_DET | I2C_CTRL_CLK_EXT_EN | I2C_CTRL_DIR_CHANGE | I2C_CTRL_RS);
    u_w16(base, I2C_SLAVE_ADDR1, (uint16_t)(slave7 << 1));
    u_w16(base, I2C_TRANSFER_LEN, 1);
    u_w16(base, I2C_TRANSFER_LEN_AUX, 1);
    u_w16(base, I2C_TRANSAC_LEN, 2);
    u_w8(base, I2C_DATA_PORT, reg);
    u_w16(base, I2C_START, 0x0001);
    int rc = i2c5_wait_done(base);
    if (rc != 0) return rc;
    *out = u_r8(base, I2C_DATA_PORT);
    return 0;
}

static int i2c5_write8(vm_offset_t base, uint8_t slave7, uint8_t reg, uint8_t val) {
    if (!base) return -3;
    u_w16(base, I2C_INTR_MASK, 0);
    u_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
    u_w16(base, I2C_FIFO_ADDR_CLR, 0x0001);
    u_w16(base, I2C_CONTROL, I2C_CTRL_ACKERR_DET | I2C_CTRL_CLK_EXT_EN);
    u_w16(base, I2C_SLAVE_ADDR1, (uint16_t)(slave7 << 1));
    u_w16(base, I2C_TRANSFER_LEN, 2);
    u_w16(base, I2C_TRANSAC_LEN, 1);
    u_w8(base, I2C_DATA_PORT, reg);
    u_w8(base, I2C_DATA_PORT, val);
    u_w16(base, I2C_START, 0x0001);
    return i2c5_wait_done(base);
}

static int i2c5_update_bits(vm_offset_t base, uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t cur = 0;
    int rc = i2c5_read8(base, MT6375_SLAVE_PMU, reg, &cur);
    if (rc != 0) return rc;
    uint8_t nxt = (uint8_t)((cur & ~mask) | (val & mask));
    return i2c5_write8(base, MT6375_SLAVE_PMU, reg, nxt);
}

/* ---- Step 1: MT6375 5.0V / 1.3A OTG VBUS Boost ---- */
static int tb375fc_mt6375_set_otg_vbus(bool enable) {
    vm_offset_t i2c5 = ml_io_map(PHYS_I2C5_MT6375, 0x1000);
    if (!i2c5) {
        kprintf("[tb375fc-usbhost] mt6375: ml_io_map(i2c5) failed\n");
        return -1;
    }
    uint8_t dev = 0;
    if (i2c5_read8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_DEV_INFO, &dev) != 0 ||
        (dev & MT6375_VID_MASK) != MT6375_VID_EXPECTED) {
        kprintf("[tb375fc-usbhost] mt6375: invalid VID dev=0x%02x\n", dev);
        return -2;
    }

    if (enable) {
        /* 1. Disable buck charging first so OTG boost does not fight CHG_EN */
        i2c5_update_bits(i2c5, MT6375_REG_CHG_TOP1, 0x01, 0x00);

        /* 2. MT6375 hidden-mode boost compensation sequence (mt6375_set_boost_param) */
        i2c5_write8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_TM_PAS_CODE1, 0x69);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_TOP2,     0xC0, 0x00);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_DUMY0,    0x03, 0x03);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_HD_BUCK5, 0xC0, 0xC0);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_VSYS,     0x80, 0x00);
        i2c5_write8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_TM_PAS_CODE1, 0x00);

        /* 3. Program 5.00V (sel=26 -> 4850 + 6*25 mV) and 1.3A current limit (sel=3) */
        i2c5_update_bits(i2c5, MT6375_REG_OTG_V, 0x3F, 26);
        i2c5_update_bits(i2c5, MT6375_REG_OTG_C, 0x07, 3);

        /* 4. Assert OTG_EN (bit 2 of CHG_TOP1) */
        i2c5_update_bits(i2c5, MT6375_REG_CHG_TOP1, MT6375_MSK_OTG_EN, MT6375_MSK_OTG_EN);
        delay(20000);
    } else {
        i2c5_update_bits(i2c5, MT6375_REG_CHG_TOP1, MT6375_MSK_OTG_EN, 0x00);
        i2c5_write8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_TM_PAS_CODE1, 0x69);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_TOP2,     0xC0, 0x40);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_DUMY0,    0x03, 0x00);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_HD_BUCK5, 0xC0, 0x40);
        i2c5_update_bits(i2c5, MT6375_REG_CHG_VSYS,     0x80, 0x80);
        i2c5_write8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_TM_PAS_CODE1, 0x00);
    }

    uint8_t top1 = 0, otgv = 0, otgc = 0;
    i2c5_read8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_CHG_TOP1, &top1);
    i2c5_read8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_OTG_V, &otgv);
    i2c5_read8(i2c5, MT6375_SLAVE_PMU, MT6375_REG_OTG_C, &otgc);
    kprintf("[tb375fc-usbhost] mt6375 OTG boost=%u top1=0x%02x otgv=0x%02x otgc=0x%02x\n",
            enable ? 1U : 0U, top1, otgv, otgc);
    return 0;
}

/* ---- Step 2: MT6897 xsphy USB2 PHY Host Mode ---- */
static void tb375fc_xsphy_set_host_mode(vm_offset_t u2_phy) {
    /* Power-on pulse matching u2_phy_instance_power_on() */
    u_w32(u2_phy, XSP_USBPHYACR6, u_r32(u2_phy, XSP_USBPHYACR6) & ~P2A6_RG_BC11_SW_EN);
    u_w32(u2_phy, XSP_USBPHYACR0, u_r32(u2_phy, XSP_USBPHYACR0) | P2A0_RG_INTR_EN);
    u_w32(u2_phy, XSP_USBPHYACR4, u_r32(u2_phy, XSP_USBPHYACR4) & ~P2A4_U2_GPIO_CTR_MSK);

    uint32_t dtm0 = u_r32(u2_phy, XSP_U2PHYDTM0);
    dtm0 &= ~(P2D_FORCE_UART_EN | P2D_FORCE_SUSPENDM | P2D_RG_SUSPENDM);
    u_w32(u2_phy, XSP_U2PHYDTM0, dtm0);

    /* Force IDDIG=0 (A-device / Host) + VBUSVALID=1 + AVALID=1 + SESSEND=0 */
    uint32_t dtm1 = u_r32(u2_phy, XSP_U2PHYDTM1);
    dtm1 &= ~(P2D_RG_UART_EN | P2D_RG_IDDIG | P2D_RG_SESSEND);
    dtm1 |= (P2D_FORCE_IDDIG | P2D_RG_VBUSVALID | P2D_RG_AVALID);
    u_w32(u2_phy, XSP_U2PHYDTM1, dtm1);
    delay(1000);
    kprintf("[tb375fc-usbhost] xsphy u2 host: acr6=0x%08x dtm0=0x%08x dtm1=0x%08x\n",
            u_r32(u2_phy, XSP_USBPHYACR6), u_r32(u2_phy, XSP_U2PHYDTM0), u_r32(u2_phy, XSP_U2PHYDTM1));
}

/* ---- Step 3: MT6897 mtu3 IPPC Switch to Host Mode ---- */
static int tb375fc_mtu3_enable_host(vm_offset_t ippc) {
    uint32_t hw_id    = u_r32(ippc, U3D_SSUSB_HW_ID);
    uint32_t xhci_cap = u_r32(ippc, U3D_SSUSB_IP_XHCI_CAP);
    uint32_t num_u2p  = (xhci_cap >> 8) & 0xFFU;
    uint32_t num_u3p  = xhci_cap & 0xFFU;
    kprintf("[tb375fc-usbhost] ippc hw_id=0x%08x xhci_cap=0x%08x (u2=%u u3=%u)\n",
            hw_id, xhci_cap, num_u2p, num_u3p);

    /* Power down device IP, power up host IP */
    u_w32(ippc, U3D_SSUSB_IP_PW_CTRL2, u_r32(ippc, U3D_SSUSB_IP_PW_CTRL2) | SSUSB_IP_DEV_PDN);
    u_w32(ippc, U3D_SSUSB_IP_PW_CTRL1, u_r32(ippc, U3D_SSUSB_IP_PW_CTRL1) & ~SSUSB_IP_HOST_PDN);

    for (uint32_t i = 0; i < num_u3p && i < 4U; i++) {
        uint32_t off = U3D_SSUSB_U3_CTRL_0P + (i * 4U);
        uint32_t v = u_r32(ippc, off);
        v &= ~(SSUSB_U3_PORT_PDN | SSUSB_U3_PORT_DIS);
        v |= SSUSB_U3_PORT_HOST_SEL;
        u_w32(ippc, off, v);
    }

    for (uint32_t i = 0; i < num_u2p && i < 4U; i++) {
        uint32_t off = U3D_SSUSB_U2_CTRL_0P + (i * 4U);
        uint32_t v = u_r32(ippc, off);
        v &= ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS | SSUSB_U2_PORT_RG_IDDIG);
        v |= (SSUSB_U2_PORT_HOST_SEL | SSUSB_U2_PORT_FORCE_IDDIG);
        u_w32(ippc, off, v);
    }

    uint32_t want = SSUSB_SYSPLL_STABLE | SSUSB_REF_RST_B_STS | SSUSB_SYS125_RST_B_STS | SSUSB_XHCI_RST_B_STS;
    uint32_t sts1 = 0;
    for (int i = 0; i < 200; i++) {
        sts1 = u_r32(ippc, U3D_SSUSB_IP_PW_STS1);
        if ((sts1 & want) == want) {
            kprintf("[tb375fc-usbhost] ippc clocks stable: sts1=0x%08x sts2=0x%08x\n",
                    sts1, u_r32(ippc, U3D_SSUSB_IP_PW_STS2));
            return 0;
        }
        delay(500);
    }
    kprintf("[tb375fc-usbhost] ippc clock wait timeout: sts1=0x%08x want=0x%08x\n", sts1, want);
    return -1;
}

/* ---- Step 4: xHCI MMIO Probe & Root-Hub Port Status ---- */
static void tb375fc_xhci_probe(vm_offset_t xhci, bool do_reset) {
    uint32_t capver = u_r32(xhci, XHCI_CAPLENGTH_VER);
    uint8_t caplen  = (uint8_t)(capver & 0xFFU);
    uint16_t hciver = (uint16_t)((capver >> 16) & 0xFFFFU);
    uint32_t hcs1   = u_r32(xhci, XHCI_HCSPARAMS1);
    uint32_t hcs2   = u_r32(xhci, XHCI_HCSPARAMS2);
    uint32_t hcc1   = u_r32(xhci, XHCI_HCCPARAMS1);
    uint32_t max_slots = hcs1 & 0xFFU;
    uint32_t max_ports = (hcs1 >> 24) & 0xFFU;

    kprintf("[tb375fc-usbhost] xhci0@0x%llx: caplen=0x%x ver=0x%04x slots=%u ports=%u hcs2=0x%08x hcc1=0x%08x\n",
            (unsigned long long)PHYS_XHCI0_BASE, caplen, hciver, max_slots, max_ports, hcs2, hcc1);

    if (caplen < 0x20U || caplen > 0x100U) {
        kprintf("[tb375fc-usbhost] xhci0: unexpected caplen=0x%x (controller powered down?)\n", caplen);
        return;
    }

    uint32_t usbcmd = u_r32(xhci, caplen + XHCI_OP_USBCMD);
    uint32_t usbsts = u_r32(xhci, caplen + XHCI_OP_USBSTS);
    kprintf("[tb375fc-usbhost] xhci0 op BEFORE: usbcmd=0x%08x usbsts=0x%08x\n", usbcmd, usbsts);

    if (do_reset) {
        /* Halt & HCRST */
        u_w32(xhci, caplen + XHCI_OP_USBCMD, usbcmd & ~XHCI_USBCMD_RUN);
        for (int i = 0; i < 100 && !(u_r32(xhci, caplen + XHCI_OP_USBSTS) & XHCI_USBSTS_HCH); i++) {
            delay(500);
        }
        u_w32(xhci, caplen + XHCI_OP_USBCMD, XHCI_USBCMD_HCRST);
        for (int i = 0; i < 200; i++) {
            if (!(u_r32(xhci, caplen + XHCI_OP_USBCMD) & XHCI_USBCMD_HCRST) &&
                !(u_r32(xhci, caplen + XHCI_OP_USBSTS) & XHCI_USBSTS_CNR)) {
                break;
            }
            delay(1000);
        }
        kprintf("[tb375fc-usbhost] xhci0 op AFTER HCRST: usbcmd=0x%08x usbsts=0x%08x\n",
                u_r32(xhci, caplen + XHCI_OP_USBCMD), u_r32(xhci, caplen + XHCI_OP_USBSTS));
    }

    for (uint32_t p = 0; p < max_ports && p < 8U; p++) {
        uint32_t portsc = u_r32(xhci, caplen + XHCI_OP_PORTSC_BASE + p * 0x10U);
        uint32_t speed  = (portsc >> 10) & 0xFU;
        uint32_t pls    = (portsc >> 5) & 0xFU;
        kprintf("[tb375fc-usbhost] xhci0 PORT[%u]: portsc=0x%08x ccs=%u ped=%u pp=%u speed=%u pls=%u\n",
                p + 1U, portsc,
                (portsc & 1U), (portsc >> 1) & 1U, (portsc >> 9) & 1U, speed, pls);
    }
}

static uint32_t g_usb_host_mode = 0;

static void __attribute__((noreturn)) tb375fc_usbhost_thread(void *param __unused, wait_result_t wr __unused) {
    /* Wait until early boot finishes so kernel logs and I2C5 are settled */
    delay_for_interval(6, NSEC_PER_SEC);

    vm_offset_t ippc_page = ml_io_map(PHYS_MTU3_IPPC_BASE & ~0xFFFULL, 0x1000);
    vm_offset_t xhci_base = ml_io_map(PHYS_XHCI0_BASE, 0x1000);
    if (!ippc_page || !xhci_base) {
        kprintf("[tb375fc-usbhost] ml_io_map failed (ippc=%p xhci=%p)\n",
                (void *)ippc_page, (void *)xhci_base);
        for (;;) delay_for_interval(3600, NSEC_PER_SEC);
    }
    vm_offset_t ippc = ippc_page + (PHYS_MTU3_IPPC_BASE & 0xFFFU);

    if (g_usb_host_mode == 2U) {
        /* SAFE READ-ONLY PROBE: does NOT touch VBUS, PHY, or USB Device CDC! */
        kprintf("[tb375fc-usbhost] mode=2 (SAFE READ-ONLY PROBE): ippc hw_id=0x%08x pw_ctrl1=0x%08x pw_ctrl2=0x%08x sts1=0x%08x xhci_cap=0x%08x\n",
                u_r32(ippc, U3D_SSUSB_HW_ID),
                u_r32(ippc, U3D_SSUSB_IP_PW_CTRL1),
                u_r32(ippc, U3D_SSUSB_IP_PW_CTRL2),
                u_r32(ippc, U3D_SSUSB_IP_PW_STS1),
                u_r32(ippc, U3D_SSUSB_IP_XHCI_CAP));
        tb375fc_xhci_probe(xhci_base, false);
        for (;;) delay_for_interval(3600, NSEC_PER_SEC);
    }

    if (g_usb_host_mode == 1U) {
        kprintf("[tb375fc-usbhost] mode=1: BRINGING UP USB-C HOST (5V OTG BOOST + XSPHY + MTU3 IPPC + XHCI0)\n");
        tb375fc_mt6375_set_otg_vbus(true);
        vm_offset_t u2_phy = ml_io_map(PHYS_XSPHY_U2_BASE, 0x1000);
        if (u2_phy) {
            tb375fc_xsphy_set_host_mode(u2_phy);
        }
        if (tb375fc_mtu3_enable_host(ippc) == 0) {
            delay_for_interval(500, NSEC_PER_MSEC);
            tb375fc_xhci_probe(xhci_base, true);
            /* Poll root-hub PORTSC periodically so hot-plugged USB-C Hub / mouse / BT dongle is visible in dmesg */
            for (;;) {
                delay_for_interval(5, NSEC_PER_SEC);
                tb375fc_xhci_probe(xhci_base, false);
            }
        }
    }

    for (;;) delay_for_interval(3600, NSEC_PER_SEC);
}

static void tb375fc_usbhost_init(void) {
    uint32_t mode = 0;
    if (!PE_parse_boot_argn("usb-host", &mode, sizeof(mode)) || mode == 0U) {
        /* Inert by default so USB CDC serial console is never disturbed */
        return;
    }
    g_usb_host_mode = mode;
    thread_t th = THREAD_NULL;
    if (kernel_thread_start(tb375fc_usbhost_thread, NULL, &th) == KERN_SUCCESS && th != THREAD_NULL) {
        thread_deallocate(th);
        kprintf("[tb375fc-usbhost] scheduled (usb-host=%u, SPI=0x%x)\n", mode, TB375FC_XHCI_SPI);
    }
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, tb375fc_usbhost_init);
