/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/*
 * Tb375fcConnSys.cpp — Native MT6897 (Dimensity 8300) Onboard ConnSys
 *                      Wi-Fi 6 (soc7_0 / Connac 2.x AXI WFDMA) & BT (BGF/BEIF)
 *                      XNU Kernel Driver + Root-Only Interactive IOUserClient
 *
 * Target: Lenovo Xiaoxin Pad Pro 12.7 (2025) / TB375FC (MT6897 + MT6637 A-Die)
 *
 * Clean-room XNU implementation verified against TB375FC live-fdt.dtb and
 * MT6897 hardware register specifications.
 *
 * Boot-arg control (default OFF = 0):
 *   connsys=0 : Completely inert (zero MMIO, SPMI, SMC, or IOService).
 *   connsys=1 : Safe read-only diagnostic probe + publish root-only IOService
 *               "Tb375fcConnSys" for interactive bring-up via tb375fc-connsys-ctl.
 *   connsys=2 : Mode 1 + automatic Stage 1 ConnInfra & MT6637 A-Die bring-up.
 *   connsys=4 : Mode 1 + automatic Stage 1 + Stage 3 Wi-Fi soc7_0 MCU power-on.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>

#include "Tb375fcConnSysABI.h"
#include "../ufs-rw/Tb375fcUfsTransport.h"

extern "C" {
struct ucred;
typedef struct ucred *kauth_cred_t;
extern kauth_cred_t kauth_cred_get(void);
extern uint32_t     kauth_cred_getuid(kauth_cred_t cred);
extern void         proc_selfname(char *buf, int size);
extern void         delay(int usec);
extern void         kprintf(const char *fmt, ...);

void tb375fc_connsys_init(void);
}

namespace {

/* =========================================================================
 * Low-level MMIO, Memory Mapping & ARM SMCCC64 Helpers
 * ========================================================================= */

struct MappedRegion {
    IOMemoryDescriptor *desc;
    IOMemoryMap        *map;
    uintptr_t           va;
    uint64_t            size;
    bool                is_wc;
};

static bool mapRegion(uint64_t phys, uint64_t size, bool write_combine, MappedRegion *out) {
    out->desc = IOMemoryDescriptor::withPhysicalAddress(phys, size, kIODirectionInOut);
    if (!out->desc) return false;
    IOOptionBits cacheOpt = write_combine ? kIOMapWriteCombineCache : kIOMapInhibitCache;
    out->map = out->desc->createMappingInTask(kernel_task, 0, kIOMapAnywhere | cacheOpt);
    if (!out->map && write_combine) {
        write_combine = false;
        out->map = out->desc->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapInhibitCache);
    }
    if (!out->map) {
        out->desc->release();
        out->desc = nullptr;
        return false;
    }
    out->va    = static_cast<uintptr_t>(out->map->getVirtualAddress());
    out->size  = size;
    out->is_wc = write_combine;
    return out->va != 0;
}

static inline uint32_t cs_r32(uintptr_t base, uint32_t off) {
    uint32_t v = *reinterpret_cast<volatile uint32_t *>(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}

static inline void cs_w32(uintptr_t base, uint32_t off, uint32_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *reinterpret_cast<volatile uint32_t *>(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}

static inline void cs_upd32(uintptr_t base, uint32_t off, uint32_t mask, uint32_t val) {
    uint32_t cur = cs_r32(base, off);
    cs_w32(base, off, (cur & ~mask) | (val & mask));
}

/* Safe byte copy to/from non-cacheable or device memory (aligned 32-bit fast path) */
static void io_copy_from(void *dst, uintptr_t src_va, uint32_t len) {
    auto *d = static_cast<uint8_t *>(dst);
    uint32_t i = 0;
    while (i < len && ((src_va + i) & 3u) != 0u) {
        uint32_t w = *reinterpret_cast<volatile uint32_t *>((src_va + i) & ~3ULL);
        d[i] = static_cast<uint8_t>((w >> (((src_va + i) & 3u) * 8u)) & 0xFFu);
        ++i;
    }
    while (i + 4u <= len) {
        uint32_t w = *reinterpret_cast<volatile uint32_t *>(src_va + i);
        memcpy(d + i, &w, 4u);
        i += 4u;
    }
    while (i < len) {
        uint32_t w = *reinterpret_cast<volatile uint32_t *>((src_va + i) & ~3ULL);
        d[i] = static_cast<uint8_t>((w >> (((src_va + i) & 3u) * 8u)) & 0xFFu);
        ++i;
    }
    __asm__ volatile("dsb ld" ::: "memory");
}

static void io_copy_to(uintptr_t dst_va, const void *src, uint32_t len, bool is_wc) {
    const auto *s = static_cast<const uint8_t *>(src);
    if (is_wc) {
        memcpy(reinterpret_cast<void *>(dst_va), s, len);
        __asm__ volatile("dsb sy" ::: "memory");
        return;
    }
    uint32_t i = 0;
    while (i < len && ((dst_va + i) & 3u) != 0u) {
        uintptr_t a = (dst_va + i) & ~3ULL;
        uint32_t shift = static_cast<uint32_t>(((dst_va + i) & 3u) * 8u);
        uint32_t w = *reinterpret_cast<volatile uint32_t *>(a);
        w = (w & ~(0xFFu << shift)) | (static_cast<uint32_t>(s[i]) << shift);
        *reinterpret_cast<volatile uint32_t *>(a) = w;
        ++i;
    }
    while (i + 4u <= len) {
        uint32_t w = 0;
        memcpy(&w, s + i, 4u);
        *reinterpret_cast<volatile uint32_t *>(dst_va + i) = w;
        i += 4u;
    }
    while (i < len) {
        uintptr_t a = (dst_va + i) & ~3ULL;
        uint32_t shift = static_cast<uint32_t>(((dst_va + i) & 3u) * 8u);
        uint32_t w = *reinterpret_cast<volatile uint32_t *>(a);
        w = (w & ~(0xFFu << shift)) | (static_cast<uint32_t>(s[i]) << shift);
        *reinterpret_cast<volatile uint32_t *>(a) = w;
        ++i;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

struct SmcRes {
    uint64_t a0;
    uint64_t a1;
    uint64_t a2;
    uint64_t a3;
};

static inline SmcRes cs_smc(uint64_t fid, uint64_t x1 = 0, uint64_t x2 = 0,
                            uint64_t x3 = 0, uint64_t x4 = 0, uint64_t x5 = 0) {
    register uint64_t r0 __asm__("x0") = fid;
    register uint64_t r1 __asm__("x1") = x1;
    register uint64_t r2 __asm__("x2") = x2;
    register uint64_t r3 __asm__("x3") = x3;
    register uint64_t r4 __asm__("x4") = x4;
    register uint64_t r5 __asm__("x5") = x5;
    register uint64_t r6 __asm__("x6") = 0;
    register uint64_t r7 __asm__("x7") = 0;
    __asm__ volatile(
        "smc #0\n"
        : "+r"(r0), "+r"(r1), "+r"(r2), "+r"(r3), "+r"(r4), "+r"(r5), "+r"(r6), "+r"(r7)
        :
        : "memory", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17"
    );
    return SmcRes{r0, r1, r2, r3};
}

/* =========================================================================
 * TB375FC / MT6897 Physical Memory Map (verified from live-fdt.dtb & specs)
 * ========================================================================= */

static constexpr uint64_t PHYS_GPIO_BASE        = 0x10005000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_IFRBUS_AO        = 0x1002C000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_IOCFG_RT         = 0x11B20000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_SPM_BASE         = 0x1C001000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_SRCLKEN_RC       = 0x1C00D000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_SPMI_PMIF_M      = 0x1CC04000ULL; /* size 0x1000 */

/* Unified ConnSys window covering ConnInfra, WFDMA0, WFSYS, and BGFSYS */
static constexpr uint64_t PHYS_CONNSYS_BASE     = 0x18000000ULL;
static constexpr uint32_t CONNSYS_WIN_SIZE      = 0x00900000U;   /* 0x18000000..0x188FFFFF */

/* Offsets relative to PHYS_CONNSYS_BASE (0x18000000) */
static constexpr uint32_t OFF_RGU_WF_PWR_CTL    = 0x000010U; /* 0x1800_0010 */
static constexpr uint32_t OFF_RGU_WF_CPU_RST_B  = 0x000120U; /* 0x1800_0120 */
static constexpr uint32_t OFF_CFG_ON_PWRCTRL1   = 0x001210U; /* 0x1800_1210 [16]=CONN_INFRA_RDY */
static constexpr uint32_t OFF_CFG_ON_WF_SLP_CTL = 0x001440U; /* 0x1800_1440 [0]=slp_prot_sw_en */
static constexpr uint32_t OFF_CFG_ON_WF_SLP_STA = 0x001444U; /* 0x1800_1444 [29,31]=slp_prot_rdy */
static constexpr uint32_t OFF_CONN_CFG_IP_VER   = 0x011000U; /* 0x1801_1000 expect 0x02050400 */
static constexpr uint32_t OFF_CLKGEN_COEX0_SET  = 0x012064U; /* 0x1801_2064 = 0x01010101 */
static constexpr uint32_t OFF_CLKGEN_COEX1_SET  = 0x012074U; /* 0x1801_2074 = 0x00000101 */
static constexpr uint32_t OFF_WFDMA0_BASE       = 0x024000U; /* 0x1802_4000 */
static constexpr uint32_t OFF_WFDMA0_INT_STA    = 0x024200U; /* 0x1802_4200 */
static constexpr uint32_t OFF_WFDMA0_INT_ENA    = 0x024204U; /* 0x1802_4204 */
static constexpr uint32_t OFF_WFDMA0_GLO_CFG    = 0x024208U; /* 0x1802_4208 */
static constexpr uint32_t OFF_SYSRAM_ADIE_ID    = 0x053800U; /* 0x1805_3800 */
static constexpr uint32_t OFF_HOST_CSR_LPCTL    = 0x060010U; /* 0x1806_0010 */
static constexpr uint32_t OFF_HOST_CSR_MISC     = 0x0600F0U; /* 0x1806_00F0 */
static constexpr uint32_t OFF_HOST_CSR_WAKE_WF  = 0x0601A4U; /* 0x1806_01A4 [0]=wakepu_wf */
static constexpr uint32_t OFF_HOST_CSR_PWR_STA  = 0x060A10U; /* 0x1806_0A10 [30]=wfsys_rgu_off */
static constexpr uint32_t OFF_HOST_CSR_MON_EN   = 0x060B00U; /* 0x1806_0B00 [0]=1 */
static constexpr uint32_t OFF_HOST_CSR_MON_SEL  = 0x060B04U; /* 0x1806_0B04 [4:0]=0xC */
static constexpr uint32_t OFF_HOST_CSR_MON_OUT  = 0x060B10U; /* 0x1806_0B10 expect 0x1D1E */
static constexpr uint32_t OFF_WF_AP2WF_REMAP1   = 0x400120U; /* 0x1840_0120 = 0x810F0000 */
static constexpr uint32_t OFF_WF_TOP_IP_VER     = 0x4B0010U; /* 0x184B_0010 expect 0x02040200 */
static constexpr uint32_t OFF_WF_SLPPROT_STA    = 0x4C300CU; /* 0x184C_300C [23]=src1,[21]=src2 */
static constexpr uint32_t OFF_WF_BUSHANGCR      = 0x4F0440U; /* 0x184F_0440 */
static constexpr uint32_t OFF_WF_REMAP0_CTRL0   = 0x500000U; /* 0x1850_0000 */
static constexpr uint32_t OFF_WF_REMAP0_CTRL3   = 0x50000CU; /* 0x1850_000C */

/* LK-reserved & MPU-configured shared DRAM pools (live-fdt.dtb) */
static constexpr uint64_t PHYS_CONSYS_EMI_BASE  = 0x8CC00000ULL;
static constexpr uint32_t CONSYS_EMI_SIZE       = 0x00D00000U; /* 13 MiB */
static constexpr uint32_t EMI_RO_WIFI_OFFSET    = 0x00380000U; /* bt(0x2e0000)+gps(0xa0000) */
static constexpr uint32_t EMI_WMMCU_DATE_OFFSET = 0x004954E0U; /* 0xF04954E0 & 0xFFFFFF */
static constexpr uint32_t BEIF_EMI_OFFSET       = 0x00BA0000U; /* 0x8D7A0000 */
static constexpr uint64_t PHYS_WIFI_DMA_POOL    = 0x8D900000ULL;
static constexpr uint32_t WIFI_DMA_POOL_SIZE    = 0x01400000U; /* 20 MiB */

/* Expected Hardware Versions */
static constexpr uint32_t EXPECTED_CONN_HW_VER  = 0x02050400U;
static constexpr uint32_t EXPECTED_WF_IP_VER    = 0x02040200U; /* MT6897 soc7_0 */
static constexpr uint32_t CONNSYS_ROM_DONE      = 0x00001D1EU;
static constexpr uint32_t WFSYS_ON_TOP_KEY      = 0x57460000U;

/* GPIO & IOCFG_RT Pinmux Offsets (D-Die <-> MT6637 A-Die interface) */
static constexpr uint32_t GPIO_MODE24_OFF       = 0x480U;
static constexpr uint32_t GPIO_MODE25_OFF       = 0x490U;
static constexpr uint32_t IOCFG_RT_DRV_CFG3_OFF = 0x030U;
static constexpr uint32_t IOCFG_RT_PUPD_CFG0    = 0x0D0U;

/* SPM & ifrbus_ao Offsets */
static constexpr uint32_t SPM_POWERON_CFG_EN    = 0x0000U; /* write key 0x0B160001 mask 0xFFFF0001 */
static constexpr uint32_t SPM_CONN_PWR_CON      = 0x0E04U;
static constexpr uint32_t PWR_RST_B_BIT         = (1U << 0);
static constexpr uint32_t PWR_ISO_BIT           = (1U << 1);
static constexpr uint32_t PWR_ON_BIT            = (1U << 2);
static constexpr uint32_t PWR_ON_2ND_BIT        = (1U << 3);
static constexpr uint32_t PWR_CLK_DIS_BIT       = (1U << 4);
static constexpr uint32_t PWR_ACK_BIT           = (1U << 30);
static constexpr uint32_t PWR_ACK_2ND_BIT       = (1U << 31);

static constexpr uint32_t IFR_PROT0_EN          = 0x000U;
static constexpr uint32_t IFR_PROT0_CLR         = 0x008U;
static constexpr uint32_t IFR_PROT0_RDY         = 0x00CU;
static constexpr uint32_t IFR_PROT_CONN_TX      = (1U << 25);
static constexpr uint32_t IFR_PROT_CONN_RX      = (1U << 26);

static constexpr uint32_t IFR_CONNSYS0_EN       = 0x1C0U;
static constexpr uint32_t IFR_CONNSYS0_CLR      = 0x1C8U;
static constexpr uint32_t IFR_CONNSYS0_RDY      = 0x1CCU;
static constexpr uint32_t IFR_CONNSYS_AHB_RX    = (1U << 0);
static constexpr uint32_t IFR_CONNSYS_AXI_TX    = (1U << 1);

static constexpr uint32_t SRCLKEN_RC_CFG1_OFF   = 0x004U;

/* =========================================================================
 * BL31 ATF SiP SMCs (MT6897)
 * ========================================================================= */

static constexpr uint64_t MTK_SIP_KERNEL_CONNSYS_CONTROL = 0xC2000534ULL;
static constexpr uint64_t MTK_SIP_KERNEL_WLAN_CONTROL    = 0xC2000535ULL;
static constexpr uint64_t MTK_SIP_KERNEL_BT_CONTROL      = 0xC2000536ULL;

enum ConnSmcOpId : uint64_t {
    SMC_CONNSYS_INIT_ATF_DATA         = 0,
    SMC_CONSYS_POLLING_CHIPID         = 10,
    SMC_CONNSYS_D_DIE_CFG             = 11,
    SMC_CONNSYS_SPI_MASTER_CFG        = 12,
    SMC_CONNSYS_A_DIE_CFG             = 13,
    SMC_CONNSYS_AFE_SW_PATCH          = 14,
    SMC_CONNSYS_AFE_WBG_CAL           = 15,
    SMC_CONNSYS_SUBSYS_PLL_INITIAL    = 16,
    SMC_CONNSYS_LOW_POWER_SETTING     = 17,
    SMC_CONSYS_CONNINFRA_WAKEUP       = 18,
    SMC_CONSYS_CONNINFRA_SLEEP        = 19,
    SMC_CONNSYS_SUBSYS_STATUS_UPDATE  = 24,
    SMC_CONNSYS_THERMAL_QUERY         = 25,
    SMC_CONNSYS_EMI_SET_REMAPPING_REG = 70,
};

enum ConnRadioType : uint32_t {
    CONNDRV_TYPE_BT   = 0,
    CONNDRV_TYPE_FM   = 1,
    CONNDRV_TYPE_GPS  = 2,
    CONNDRV_TYPE_WIFI = 3,
};

enum WlanSmcOpId : uint64_t {
    SMC_WLAN_PCCIF_ON         = 3,
    SMC_WLAN_PCCIF_OFF        = 4,
    SMC_WLAN_ENABLE_FWDL_MODE = 5,
};

/* =========================================================================
 * SPMI PMIF-M (0x1CC04000, SWINF_2) & MT6363 / MT6368 / MT6685 Registers
 * ========================================================================= */

static constexpr uint32_t PMIF_INIT_DONE        = 0x0000U;
static constexpr uint32_t PMIF_SWINF_2_ACC      = 0x0880U;
static constexpr uint32_t PMIF_SWINF_2_WDATA    = 0x0884U;
static constexpr uint32_t PMIF_SWINF_2_RDATA    = 0x0894U;
static constexpr uint32_t PMIF_SWINF_2_VLD_CLR  = 0x08A4U;
static constexpr uint32_t PMIF_SWINF_2_STA      = 0x08A8U;

static constexpr uint32_t PMIF_CMD_EXT_REG_LONG = 3U;
static constexpr uint32_t SWINF_FSM_IDLE        = 0U;
static constexpr uint32_t SWINF_FSM_WFVLDCLR    = 6U;

static inline uint32_t pmif_swinf_fsm(uint32_t sta) { return (sta >> 1) & 0x7U; }

static constexpr uint8_t  SPMI_SLVID_MT6363     = 0x4U;
static constexpr uint8_t  SPMI_SLVID_MT6368     = 0x5U;
static constexpr uint8_t  SPMI_SLVID_MT6685     = 0x9U;

static constexpr uint16_t PMIC_HWCID_L          = 0x0008U;
static constexpr uint16_t PMIC_HWCID_H          = 0x0009U;
static constexpr uint16_t MT6685_DCXO_DIGCLK_ELR = 0x07F4U; /* bit0: 1=52M, 0=26M */

/* MT6363 (slv=4): VRFIO18 (1.7V) & VCN13 (1.35V) + PMRC RC6/7/8/9 */
static constexpr uint16_t MT6363_VRFIO18_EN_LP    = 0x1BCDU;
static constexpr uint16_t MT6363_VRFIO18_MON      = 0x1BD1U;
static constexpr uint16_t MT6363_VRFIO18_RC67_EN  = 0x1BD2U; /* bit6=RC6, bit7=RC7 */
static constexpr uint16_t MT6363_VRFIO18_RC89_EN  = 0x1BD3U; /* bit0=RC8, bit1=RC9 */
static constexpr uint16_t MT6363_VRFIO18_SW_OP_EN = 0x1BD4U; /* bit7=SW_OP_EN, bit0=HW0 */
static constexpr uint16_t MT6363_VRFIO18_RC67_CFG = 0x1BD5U;
static constexpr uint16_t MT6363_VRFIO18_RC89_CFG = 0x1BD6U;
static constexpr uint16_t MT6363_VRFIO18_RC67_MOD = 0x1BD8U;
static constexpr uint16_t MT6363_VRFIO18_RC89_MOD = 0x1BD9U;
static constexpr uint16_t MT6363_VRFIO18_VOSEL    = 0x1F2CU; /* 0xB = 1.7V */

static constexpr uint16_t MT6363_VCN13_EN_LP      = 0x1D07U;
static constexpr uint16_t MT6363_VCN13_MON        = 0x1D0BU;
static constexpr uint16_t MT6363_VCN13_RC67_EN    = 0x1D14U; /* bit6=RC6, bit7=RC7 */
static constexpr uint16_t MT6363_VCN13_RC89_EN    = 0x1D15U; /* bit0=RC8, bit1=RC9 */
static constexpr uint16_t MT6363_VCN13_SW_OP_EN   = 0x1D16U; /* bit7=SW_OP_EN */
static constexpr uint16_t MT6363_VCN13_RC67_CFG   = 0x1D17U;
static constexpr uint16_t MT6363_VCN13_RC89_CFG   = 0x1D18U;
static constexpr uint16_t MT6363_VCN13_RC67_MOD   = 0x1D1AU;
static constexpr uint16_t MT6363_VCN13_RC89_MOD   = 0x1D1BU;
static constexpr uint16_t MT6363_VCN13_VOSEL      = 0x1B3FU;

/* MT6368 (slv=5): VCN33_1 (3.3V), VCN33_2 (3.3V), VANT18 (1.8V) + PMRC RC6/7/8/10 */
static constexpr uint16_t MT6368_VCN33_1_EN_LP    = 0x1CC0U;
static constexpr uint16_t MT6368_VCN33_1_MON      = 0x1CC4U;
static constexpr uint16_t MT6368_VCN33_1_RC7_EN   = 0x1CC5U; /* bit7 */
static constexpr uint16_t MT6368_VCN33_1_RC8_EN   = 0x1CC6U; /* bit0 */
static constexpr uint16_t MT6368_VCN33_1_SW_OP_EN = 0x1CC7U; /* bit7 */
static constexpr uint16_t MT6368_VCN33_1_RC7_CFG  = 0x1CC8U;
static constexpr uint16_t MT6368_VCN33_1_RC8_CFG  = 0x1CC9U;
static constexpr uint16_t MT6368_VCN33_1_RC7_MOD  = 0x1CCBU;
static constexpr uint16_t MT6368_VCN33_1_RC8_MOD  = 0x1CCCU;
static constexpr uint16_t MT6368_VCN33_1_VOSEL    = 0x1DB1U; /* 0xD = 3.3V */

static constexpr uint16_t MT6368_VCN33_2_EN_LP    = 0x1CCEU;
static constexpr uint16_t MT6368_VCN33_2_MON      = 0x1CD2U;
static constexpr uint16_t MT6368_VCN33_2_RC8_EN   = 0x1CD4U; /* bit0 */
static constexpr uint16_t MT6368_VCN33_2_SW_OP_EN = 0x1CD5U; /* bit7 */
static constexpr uint16_t MT6368_VCN33_2_RC8_CFG  = 0x1CD7U;
static constexpr uint16_t MT6368_VCN33_2_RC8_MOD  = 0x1CDAU;
static constexpr uint16_t MT6368_VCN33_2_VOSEL    = 0x1DB5U; /* 0xD = 3.3V */

static constexpr uint16_t MT6368_VANT18_EN_LP     = 0x1C23U;
static constexpr uint16_t MT6368_VANT18_MON       = 0x1C27U;
static constexpr uint16_t MT6368_VANT18_RC6_EN    = 0x1C28U; /* bit6 */
static constexpr uint16_t MT6368_VANT18_RC10_EN   = 0x1C29U; /* bit2 */
static constexpr uint16_t MT6368_VANT18_SW_OP_EN  = 0x1C2AU; /* bit7 */
static constexpr uint16_t MT6368_VANT18_RC6_CFG   = 0x1C2BU;
static constexpr uint16_t MT6368_VANT18_RC10_CFG  = 0x1C2CU;
static constexpr uint16_t MT6368_VANT18_RC6_MOD   = 0x1C2EU;
static constexpr uint16_t MT6368_VANT18_RC10_MOD  = 0x1C2FU;
static constexpr uint16_t MT6368_VANT18_VOSEL     = 0x1E09U; /* 0xC = 1.8V */

/* =========================================================================
 * Global Driver State
 * ========================================================================= */

struct ConnSysState {
    IOLock       *lock;
    MappedRegion  gpio;
    MappedRegion  ifrbus;
    MappedRegion  iocfg_rt;
    MappedRegion  spm;
    MappedRegion  srclken_rc;
    MappedRegion  spmi_pmif;
    MappedRegion  connsys;  /* 0x18000000 .. 0x188FFFFF */
    MappedRegion  emi;      /* 0x8CC00000, 13 MiB */
    MappedRegion  dma_pool; /* 0x8D900000, 20 MiB */

    bool          mapped;
    bool          conninfra_up;
    bool          wifi_mcu_up;
    bool          bt_up;
    bool          is_rc_mode;
    uint32_t      clock_type;
    uint32_t      adie_hw_ver;
    uint32_t      boot_mode;
};

static ConnSysState g_cs = {};

/* =========================================================================
 * SPMI PMIF-M Read / Write Helpers (SWINF_2)
 * ========================================================================= */

static bool spmi_wait_fsm(uint32_t want_state, int timeout_us) {
    for (int waited = 0; waited < timeout_us; waited += 10) {
        uint32_t sta = cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA);
        if (pmif_swinf_fsm(sta) == want_state) return true;
        delay(10);
    }
    return false;
}

static bool spmi_read8(uint8_t slvid, uint16_t addr, uint8_t *out_val) {
    if (!g_cs.spmi_pmif.va || !out_val) return false;

    uint32_t sta = cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA);
    if (pmif_swinf_fsm(sta) == SWINF_FSM_WFVLDCLR) {
        cs_w32(g_cs.spmi_pmif.va, PMIF_SWINF_2_VLD_CLR, 1U);
    }
    if (!spmi_wait_fsm(SWINF_FSM_IDLE, 20000)) {
        kprintf("[tb375fc-connsys] SPMI read IDLE timeout slv=%u addr=0x%04x sta=0x%08x\n",
                slvid, addr, cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA));
        return false;
    }

    uint32_t cmd = (PMIF_CMD_EXT_REG_LONG << 30) |
                   ((uint32_t)(slvid & 0xFU) << 24) |
                   (uint32_t)addr;
    cs_w32(g_cs.spmi_pmif.va, PMIF_SWINF_2_ACC, cmd);

    if (!spmi_wait_fsm(SWINF_FSM_WFVLDCLR, 20000)) {
        kprintf("[tb375fc-connsys] SPMI read WFVLDCLR timeout slv=%u addr=0x%04x sta=0x%08x\n",
                slvid, addr, cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA));
        return false;
    }

    uint32_t rdata = cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_RDATA);
    cs_w32(g_cs.spmi_pmif.va, PMIF_SWINF_2_VLD_CLR, 1U);
    *out_val = (uint8_t)(rdata & 0xFFU);
    return true;
}

static bool spmi_write8(uint8_t slvid, uint16_t addr, uint8_t val) {
    if (!g_cs.spmi_pmif.va) return false;

    uint32_t sta = cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA);
    if (pmif_swinf_fsm(sta) == SWINF_FSM_WFVLDCLR) {
        cs_w32(g_cs.spmi_pmif.va, PMIF_SWINF_2_VLD_CLR, 1U);
    }
    if (!spmi_wait_fsm(SWINF_FSM_IDLE, 20000)) {
        kprintf("[tb375fc-connsys] SPMI write IDLE timeout slv=%u addr=0x%04x sta=0x%08x\n",
                slvid, addr, cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA));
        return false;
    }

    cs_w32(g_cs.spmi_pmif.va, PMIF_SWINF_2_WDATA, (uint32_t)val);
    uint32_t cmd = (PMIF_CMD_EXT_REG_LONG << 30) |
                   (1U << 29) |
                   ((uint32_t)(slvid & 0xFU) << 24) |
                   (uint32_t)addr;
    cs_w32(g_cs.spmi_pmif.va, PMIF_SWINF_2_ACC, cmd);
    return spmi_wait_fsm(SWINF_FSM_IDLE, 20000);
}

static bool spmi_update8(uint8_t slvid, uint16_t addr, uint8_t mask, uint8_t val) {
    uint8_t cur = 0;
    if (!spmi_read8(slvid, addr, &cur)) return false;
    uint8_t next = (uint8_t)((cur & (uint8_t)~mask) | (val & mask));
    return spmi_write8(slvid, addr, next);
}

/* =========================================================================
 * Stage 0: Safe Read-Only Diagnostic Probe (zero writes!)
 * ========================================================================= */

static void connsys_probe_readonly(Tb375fcConnSysProbeOut *out, bool log_kprintf) {
    memset(out, 0, sizeof(*out));

    out->spm_pwr_cfg_en   = cs_r32(g_cs.spm.va, SPM_POWERON_CFG_EN);
    out->spm_conn_pwr_con = cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON);
    out->ifr_prot0_en     = cs_r32(g_cs.ifrbus.va, IFR_PROT0_EN);
    out->ifr_prot0_rdy    = cs_r32(g_cs.ifrbus.va, IFR_PROT0_RDY);
    out->ifr_cprot_en     = cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_EN);
    out->ifr_cprot_rdy    = cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_RDY);
    out->rc_central_cfg1  = g_cs.srclken_rc.va ? cs_r32(g_cs.srclken_rc.va, SRCLKEN_RC_CFG1_OFF) : 0U;
    out->is_rc_mode       = (out->rc_central_cfg1 & 1U) ? 1U : 0U;
    g_cs.is_rc_mode       = (out->is_rc_mode != 0U);

    out->pmif_init_done   = cs_r32(g_cs.spmi_pmif.va, PMIF_INIT_DONE);
    out->pmif_swinf2_sta  = cs_r32(g_cs.spmi_pmif.va, PMIF_SWINF_2_STA);

    auto read_hwcid = [](uint8_t slv, uint16_t *dst) -> bool {
        uint8_t lo = 0, hi = 0;
        bool ok = spmi_read8(slv, PMIC_HWCID_L, &lo) && spmi_read8(slv, PMIC_HWCID_H, &hi);
        *dst = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
        return ok;
    };
    if (read_hwcid(SPMI_SLVID_MT6363, &out->hwcid_mt6363)) out->spmi_ok_mask |= (1U << 0);
    if (read_hwcid(SPMI_SLVID_MT6368, &out->hwcid_mt6368)) out->spmi_ok_mask |= (1U << 1);
    if (read_hwcid(SPMI_SLVID_MT6685, &out->hwcid_mt6685)) out->spmi_ok_mask |= (1U << 2);
    if (spmi_read8(SPMI_SLVID_MT6685, MT6685_DCXO_DIGCLK_ELR, &out->dcxo_digclk_elr)) {
        out->spmi_ok_mask |= (1U << 3);
    }
    /* CONNSYS_CLOCK_SCHEMATIC_26M_COTMS = 0, 52M_COTMS = 1 */
    out->clock_type_detected = (out->dcxo_digclk_elr & 0x1U) ? 1U : 0U;
    g_cs.clock_type = out->clock_type_detected;

    spmi_read8(SPMI_SLVID_MT6363, MT6363_VRFIO18_EN_LP,    &out->vrfio18_en);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VRFIO18_MON,      &out->vrfio18_mon);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VRFIO18_SW_OP_EN, &out->vrfio18_swop);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VRFIO18_VOSEL,    &out->vrfio18_vosel);

    spmi_read8(SPMI_SLVID_MT6363, MT6363_VCN13_EN_LP,      &out->vcn13_en);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VCN13_MON,        &out->vcn13_mon);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VCN13_SW_OP_EN,   &out->vcn13_swop);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VCN13_VOSEL,      &out->vcn13_vosel);

    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_1_EN_LP,    &out->vcn33_1_en);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_1_MON,      &out->vcn33_1_mon);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_1_VOSEL,    &out->vcn33_1_vosel);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_2_EN_LP,    &out->vcn33_2_en);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_2_MON,      &out->vcn33_2_mon);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_2_VOSEL,    &out->vcn33_2_vosel);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VANT18_EN_LP,     &out->vant18_en);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VANT18_MON,       &out->vant18_mon);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VANT18_VOSEL,     &out->vant18_vosel);

    if (g_cs.gpio.va) {
        out->gpio_mode24 = cs_r32(g_cs.gpio.va, GPIO_MODE24_OFF);
        out->gpio_mode25 = cs_r32(g_cs.gpio.va, GPIO_MODE25_OFF);
    }
    if (g_cs.iocfg_rt.va) {
        out->iocfg_drv3  = cs_r32(g_cs.iocfg_rt.va, IOCFG_RT_DRV_CFG3_OFF);
        out->iocfg_pupd0 = cs_r32(g_cs.iocfg_rt.va, IOCFG_RT_PUPD_CFG0);
    }

    bool conn_powered = ((out->spm_conn_pwr_con & (PWR_ON_BIT | PWR_ACK_BIT | PWR_RST_B_BIT)) ==
                         (PWR_ON_BIT | PWR_ACK_BIT | PWR_RST_B_BIT)) &&
                        ((out->spm_conn_pwr_con & (PWR_ISO_BIT | PWR_CLK_DIS_BIT)) == 0U);
    bool bus_open     = ((out->ifr_prot0_en & (IFR_PROT_CONN_TX | IFR_PROT_CONN_RX)) == 0U) &&
                        ((out->ifr_cprot_en & (IFR_CONNSYS_AHB_RX | IFR_CONNSYS_AXI_TX)) == 0U);
    if (conn_powered && bus_open && g_cs.connsys.va) {
        out->conn_hw_ver = cs_r32(g_cs.connsys.va, OFF_CONN_CFG_IP_VER);
    }
    out->adie_hw_ver = g_cs.adie_hw_ver;

    if (g_cs.emi.va) {
        io_copy_from(out->emi_wmmcu_date, g_cs.emi.va + EMI_WMMCU_DATE_OFFSET, 16U);
        io_copy_from(out->emi_ro_wifi_head, g_cs.emi.va + EMI_RO_WIFI_OFFSET, 16U);
        io_copy_from(&out->beif_key, g_cs.emi.va + BEIF_EMI_OFFSET, 4U);
        io_copy_from(&out->beif_ver, g_cs.emi.va + BEIF_EMI_OFFSET + 4U, 4U);
    }

    out->state_flags = (g_cs.conninfra_up ? 1U : 0U) |
                       (g_cs.wifi_mcu_up  ? 2U : 0U) |
                       (g_cs.bt_up        ? 4U : 0U);

    if (log_kprintf) {
        kprintf("[tb375fc-connsys] RO: SPM(cfg=0x%08x pwr=0x%08x) IFR(p0=0x%08x/0x%08x c0=0x%08x/0x%08x) RC=0x%08x(rc=%u)\n",
                out->spm_pwr_cfg_en, out->spm_conn_pwr_con,
                out->ifr_prot0_en, out->ifr_prot0_rdy,
                out->ifr_cprot_en, out->ifr_cprot_rdy,
                out->rc_central_cfg1, out->is_rc_mode);
        kprintf("[tb375fc-connsys] RO: SPMI(ok=0x%x) 6363=0x%04x 6368=0x%04x 6685=0x%04x DCXO_ELR=0x%02x(clk=%s)\n",
                out->spmi_ok_mask, out->hwcid_mt6363, out->hwcid_mt6368, out->hwcid_mt6685,
                out->dcxo_digclk_elr, out->clock_type_detected ? "52M" : "26M");
        kprintf("[tb375fc-connsys] RO: LDO VRFIO18(en=%02x mon=%02x sw=%02x vs=%02x) VCN13(en=%02x mon=%02x sw=%02x vs=%02x) V33_1(%02x/%02x/%02x) V33_2(%02x/%02x/%02x) VANT(%02x/%02x/%02x)\n",
                out->vrfio18_en, out->vrfio18_mon, out->vrfio18_swop, out->vrfio18_vosel,
                out->vcn13_en, out->vcn13_mon, out->vcn13_swop, out->vcn13_vosel,
                out->vcn33_1_en, out->vcn33_1_mon, out->vcn33_1_vosel,
                out->vcn33_2_en, out->vcn33_2_mon, out->vcn33_2_vosel,
                out->vant18_en, out->vant18_mon, out->vant18_vosel);
        kprintf("[tb375fc-connsys] RO: PINMUX(m24=0x%08x m25=0x%08x drv3=0x%08x pupd0=0x%08x) CONN_VER=0x%08x EMI_RO[0..1]=%08x/%08x BEIF=%08x/%08x\n",
                out->gpio_mode24, out->gpio_mode25, out->iocfg_drv3, out->iocfg_pupd0,
                out->conn_hw_ver, out->emi_ro_wifi_head[0], out->emi_ro_wifi_head[1],
                out->beif_key, out->beif_ver);
    }
}

/* =========================================================================
 * Stage 1: PMIC + Pinmux + SPM CONN + BL31 ATF ConnInfra & MT6637 A-Die
 * ========================================================================= */

static bool pmic_enable_common_rails(void) {
    bool ok = true;
    /* MT6363 (slv=4) VRFIO18 = 1.7V (vosel 0xB in table3) for MT6637 A-Die */
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_VOSEL, 0x0FU, 0x0BU);
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_SW_OP_EN, 0x80U, 0x80U);
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_EN_LP, 0x03U, 0x01U);

    /* MT6363 (slv=4) VCN13 (preserve factory 1.35V VOSEL/VOCAL calibration), SW ON */
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_SW_OP_EN, 0x80U, 0x80U);
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_EN_LP, 0x03U, 0x01U);
    delay(2000);
    return ok;
}

static bool pmic_configure_low_power_and_radio(bool enable_wifi, bool keep_sw_on) {
    bool ok = true;
    if (g_cs.is_rc_mode) {
        /* MT6363 VRFIO18 RC6/7/8/9: OP_MODE=0, OP_EN=1, OP_CFG=0 */
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_RC67_MOD, 0xC0U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_RC67_EN,  0xC0U, 0xC0U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_RC67_CFG, 0xC0U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_RC89_MOD, 0x03U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_RC89_EN,  0x03U, 0x03U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_RC89_CFG, 0x03U, 0x00U);

        /* MT6363 VCN13 RC6/7/8/9: OP_MODE=0, OP_EN=1, OP_CFG=0 */
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_RC67_MOD, 0xC0U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_RC67_EN,  0xC0U, 0xC0U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_RC67_CFG, 0xC0U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_RC89_MOD, 0x03U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_RC89_EN,  0x03U, 0x03U);
        ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_RC89_CFG, 0x03U, 0x00U);

        /* MT6368 VCN33_1 RC7/8, VCN33_2 RC8, VANT18 RC6/10 */
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_RC7_MOD, 0x80U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_RC7_EN,  0x80U, 0x80U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_RC7_CFG, 0x80U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_RC8_MOD, 0x01U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_RC8_EN,  0x01U, 0x01U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_RC8_CFG, 0x01U, 0x00U);

        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_RC8_MOD, 0x01U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_RC8_EN,  0x01U, 0x01U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_RC8_CFG, 0x01U, 0x00U);

        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_RC6_MOD,  0x40U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_RC6_EN,   0x40U, 0x40U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_RC6_CFG,  0x40U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_RC10_MOD, 0x04U, 0x00U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_RC10_EN,  0x04U, 0x04U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_RC10_CFG, 0x04U, 0x00U);
    }

    if (keep_sw_on || !g_cs.is_rc_mode) {
        /* Force SW enable (normal mode) on radio rails for reliable bring-up */
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_VOSEL,    0x0FU, 0x0DU); /* 3.3V */
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_SW_OP_EN, 0x80U, 0x80U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_EN_LP,    0x03U, 0x01U);

        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_VOSEL,     0x0FU, 0x0CU); /* 1.8V */
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_SW_OP_EN,  0x80U, 0x80U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_EN_LP,     0x03U, 0x01U);

        if (enable_wifi) {
            ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_VOSEL,    0x0FU, 0x0DU); /* 3.3V */
            ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_SW_OP_EN, 0x80U, 0x80U);
            ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_EN_LP,    0x03U, 0x01U);
        }
    }
    delay(1000);
    return ok;
}

static void connsys_set_if_pinmux(bool enable) {
    if (!g_cs.gpio.va || !g_cs.iocfg_rt.va) return;
    if (enable) {
        /* D-Die <-> MT6637 A-Die interface pinmux (Aux1), drive=001, PUPD=0 */
        cs_upd32(g_cs.gpio.va, GPIO_MODE24_OFF, 0x77777777U, 0x11111111U);
        cs_upd32(g_cs.gpio.va, GPIO_MODE25_OFF, 0x00000777U, 0x00000111U);
        cs_upd32(g_cs.iocfg_rt.va, IOCFG_RT_DRV_CFG3_OFF, 0x001FF000U, 0x00049000U);
        cs_upd32(g_cs.iocfg_rt.va, IOCFG_RT_PUPD_CFG0, 0x0000000AU, 0x00000000U);
    } else {
        cs_upd32(g_cs.gpio.va, GPIO_MODE24_OFF, 0x77777777U, 0x00000000U);
        cs_upd32(g_cs.gpio.va, GPIO_MODE25_OFF, 0x00000777U, 0x00000000U);
        cs_upd32(g_cs.iocfg_rt.va, IOCFG_RT_PUPD_CFG0, 0x0000000AU, 0x0000000AU);
    }
}

static bool spm_power_on_conn_domain(void) {
    /* Unlock SPM clock/power writes with key 0x0B160001 (mask 0xFFFF0001) */
    cs_upd32(g_cs.spm.va, SPM_POWERON_CFG_EN, 0xFFFF0001U, 0x0B160001U);

    uint32_t v = cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON);
    bool already_on = ((v & (PWR_ON_BIT | PWR_ON_2ND_BIT | PWR_ACK_BIT | PWR_ACK_2ND_BIT | PWR_RST_B_BIT)) ==
                       (PWR_ON_BIT | PWR_ON_2ND_BIT | PWR_ACK_BIT | PWR_ACK_2ND_BIT | PWR_RST_B_BIT)) &&
                      ((v & (PWR_ISO_BIT | PWR_CLK_DIS_BIT)) == 0U);
    if (!already_on) {
        /* 1. Primary power on & poll ACK (bit30) */
        cs_upd32(g_cs.spm.va, SPM_CONN_PWR_CON, PWR_ON_BIT, PWR_ON_BIT);
        bool ack1 = false;
        for (int i = 0; i < 40; ++i) {
            if (cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON) & PWR_ACK_BIT) { ack1 = true; break; }
            delay(500);
        }
        if (!ack1) {
            kprintf("[tb375fc-connsys] SPM CONN primary ACK timeout: 0x%08x\n",
                    cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON));
            return false;
        }

        /* 2. Secondary power on & poll ACK_2ND (bit31) */
        cs_upd32(g_cs.spm.va, SPM_CONN_PWR_CON, PWR_ON_2ND_BIT, PWR_ON_2ND_BIT);
        bool ack2 = false;
        for (int i = 0; i < 40; ++i) {
            if (cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON) & PWR_ACK_2ND_BIT) { ack2 = true; break; }
            delay(500);
        }
        if (!ack2) {
            kprintf("[tb375fc-connsys] SPM CONN secondary ACK timeout: 0x%08x\n",
                    cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON));
            return false;
        }

        /* 3. Enable bus clock, clear ISO, deassert S/W reset */
        cs_upd32(g_cs.spm.va, SPM_CONN_PWR_CON, PWR_CLK_DIS_BIT, 0U);
        delay(5);
        cs_upd32(g_cs.spm.va, SPM_CONN_PWR_CON, PWR_ISO_BIT, 0U);
        cs_upd32(g_cs.spm.va, SPM_CONN_PWR_CON, PWR_RST_B_BIT, PWR_RST_B_BIT);
        delay(500);
    }

    /* 4. Exact MT6897 ifrbus_ao sleep-protect release sequence:
     *    a) AP2CONN AHB RX: W1C 0x1C8[0], poll 0x1CC[0] == 0
     *    b) AP2CONN AHB TX: W1C 0x008[25], poll 0x00C[25] == 0
     *    c) CONN2AP AXI RX: W1C 0x008[26]
     *    d) CONN2AP AXI TX: W1C 0x1C8[1]
     *    e) delay 5 ms
     */
    cs_w32(g_cs.ifrbus.va, IFR_CONNSYS0_CLR, IFR_CONNSYS_AHB_RX);
    bool rx_ok = false;
    for (int i = 0; i < 100; ++i) {
        if ((cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_RDY) & IFR_CONNSYS_AHB_RX) == 0U) { rx_ok = true; break; }
        delay(500);
    }
    if (!rx_ok) {
        kprintf("[tb375fc-connsys] IFRBUS AP2CONN AHB RX release timeout: rdy14=0x%08x\n",
                cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_RDY));
        return false;
    }

    cs_w32(g_cs.ifrbus.va, IFR_PROT0_CLR, IFR_PROT_CONN_TX);
    bool tx_ok = false;
    for (int i = 0; i < 100; ++i) {
        if ((cs_r32(g_cs.ifrbus.va, IFR_PROT0_RDY) & IFR_PROT_CONN_TX) == 0U) { tx_ok = true; break; }
        delay(500);
    }
    if (!tx_ok) {
        kprintf("[tb375fc-connsys] IFRBUS AP2CONN AHB TX release timeout: rdy0=0x%08x\n",
                cs_r32(g_cs.ifrbus.va, IFR_PROT0_RDY));
        return false;
    }

    cs_w32(g_cs.ifrbus.va, IFR_PROT0_CLR, IFR_PROT_CONN_RX);
    cs_w32(g_cs.ifrbus.va, IFR_CONNSYS0_CLR, IFR_CONNSYS_AXI_TX);
    IOSleep(5);

    return true;
}

static int32_t connsys_stage1_bringup(const Tb375fcConnSysStage1In &in,
                                      Tb375fcConnSysStage1Out *out) {
    memset(out, 0, sizeof(*out));
    uint32_t radio_mask = in.radio_mask ? in.radio_mask : (1U << CONNDRV_TYPE_WIFI);
    uint32_t radio_idx  = (radio_mask & (1U << CONNDRV_TYPE_WIFI)) ? CONNDRV_TYPE_WIFI : CONNDRV_TYPE_BT;
    uint32_t clk_type   = (in.force_clock_type == 0xFFFFFFFFU) ? g_cs.clock_type : (in.force_clock_type & 3U);
    bool keep_sw_on     = ((in.flags & 1U) == 0U); /* default keep SW ON */

    out->clock_type_used = clk_type;
    out->is_rc_mode      = g_cs.is_rc_mode ? 1U : 0U;

    if (g_cs.conninfra_up) {
        out->spm_conn_pwr_con = cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON);
        out->ifr_prot0_en     = cs_r32(g_cs.ifrbus.va, IFR_PROT0_EN);
        out->ifr_cprot_en     = cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_EN);
        out->conn_hw_ver      = cs_r32(g_cs.connsys.va, OFF_CONN_CFG_IP_VER);
        out->adie_hw_ver      = g_cs.adie_hw_ver;
        out->sysram_adie_id   = cs_r32(g_cs.connsys.va, OFF_SYSRAM_ADIE_ID);
        out->status = 0;
        return 0;
    }

    /* Step 1: BL31 INIT_ATF_DATA(platform_config=0, clock_type) */
    SmcRes r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_INIT_ATF_DATA, 0ULL, clk_type);
    out->smc_init_atf = static_cast<int64_t>(r.a0);

    /* Step 2: PMIC common rails (MT6363 VRFIO18 1.7V + VCN13 1.35V) */
    if (!pmic_enable_common_rails()) {
        out->status = -2; out->failed_step = 2; return -2;
    }

    /* Step 3: D-Die <-> MT6637 A-Die Pinmux */
    connsys_set_if_pinmux(true);

    /* Step 4: SPM CONN power domain + ifrbus_ao sleep-protect release */
    if (!spm_power_on_conn_domain()) {
        out->spm_conn_pwr_con = cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON);
        out->ifr_prot0_en     = cs_r32(g_cs.ifrbus.va, IFR_PROT0_EN);
        out->ifr_cprot_en     = cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_EN);
        out->status = -4; out->failed_step = 4; return -4;
    }
    out->spm_conn_pwr_con = cs_r32(g_cs.spm.va, SPM_CONN_PWR_CON);
    out->ifr_prot0_en     = cs_r32(g_cs.ifrbus.va, IFR_PROT0_EN);
    out->ifr_cprot_en     = cs_r32(g_cs.ifrbus.va, IFR_CONNSYS0_EN);

    /* Step 5: BL31 POLLING_CHIPID & verify CONN_HW_VER (0x02050400) */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONSYS_POLLING_CHIPID);
    out->smc_poll_chipid = static_cast<int64_t>(r.a0);
    out->conn_hw_ver = cs_r32(g_cs.connsys.va, OFF_CONN_CFG_IP_VER);
    if (out->smc_poll_chipid != 0) {
        out->status = -5; out->failed_step = 5; return -5;
    }

    /* Step 6: BL31 EMI_SET_REMAPPING_REG */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_EMI_SET_REMAPPING_REG);
    out->smc_emi_remap = static_cast<int64_t>(r.a0);

    /* Step 7: BL31 D_DIE_CFG */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_D_DIE_CFG);
    out->smc_ddie_cfg = static_cast<int64_t>(r.a0);
    if (out->smc_ddie_cfg != 0) {
        out->status = -7; out->failed_step = 7; return -7;
    }

    /* Step 8: BL31 SPI_MASTER_CFG(0, 0) */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SPI_MASTER_CFG, 0ULL, 0ULL);
    out->smc_spi_mst = static_cast<int64_t>(r.a0);
    if (out->smc_spi_mst != 0) {
        out->status = -8; out->failed_step = 8; return -8;
    }

    /* Step 9: BL31 A_DIE_CFG(0, 0) -> returns a1 = MT6637 A-Die HW version */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_A_DIE_CFG, 0ULL, 0ULL);
    out->smc_adie_cfg   = static_cast<int64_t>(r.a0);
    out->adie_hw_ver    = static_cast<uint32_t>(r.a1);
    g_cs.adie_hw_ver    = out->adie_hw_ver;
    out->sysram_adie_id = cs_r32(g_cs.connsys.va, OFF_SYSRAM_ADIE_ID);
    if (out->smc_adie_cfg != 0) {
        out->status = -9; out->failed_step = 9; return -9;
    }

    /* Step 10: AFE SW patch, WBG cal, Subsys PLL init */
    cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_AFE_SW_PATCH);
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_AFE_WBG_CAL);
    out->smc_afe_cal = static_cast<int64_t>(r.a0);
    if (out->smc_afe_cal != 0) {
        out->status = -10; out->failed_step = 10; return -10;
    }

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SUBSYS_PLL_INITIAL);
    out->smc_subsys_pll = static_cast<int64_t>(r.a0);
    if (out->smc_subsys_pll != 0) {
        out->status = -11; out->failed_step = 11; return -11;
    }

    /* Step 11: Update subsys status & low-power setting */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SUBSYS_STATUS_UPDATE, 1ULL, radio_idx);
    out->smc_status_upd = static_cast<int64_t>(r.a0);

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_LOW_POWER_SETTING, 0ULL, radio_mask);
    out->smc_low_pwr = static_cast<int64_t>(r.a0);

    /* Step 12: PMIC RC / radio rails (VCN33_1, VCN33_2, VANT18) */
    bool want_wifi = (radio_mask & (1U << CONNDRV_TYPE_WIFI)) != 0U;
    pmic_configure_low_power_and_radio(want_wifi, keep_sw_on);

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_THERMAL_QUERY);
    out->smc_thermal = static_cast<int64_t>(r.a0);

    g_cs.conninfra_up = true;
    kprintf("[tb375fc-connsys] Stage 1 UP! clk=%u rc=%u CONN=0x%08x ADIE=0x%08x(sysram=0x%08x) therm=%lld\n",
            out->clock_type_used, out->is_rc_mode, out->conn_hw_ver,
            out->adie_hw_ver, out->sysram_adie_id, out->smc_thermal);
    out->status = 0;
    return 0;
}

/* =========================================================================
 * Stage 3: Wi-Fi soc7_0 WMMCU Power-On (wf_pwr_on_consys_mcu) + WFDMA Probe
 * ========================================================================= */

static int32_t connsys_stage3_wifi_on(const Tb375fcConnSysWifiOnIn &in,
                                      Tb375fcConnSysWifiOnOut *out) {
    memset(out, 0, sizeof(*out));

    if (!g_cs.conninfra_up && ((in.flags & 1U) == 0U)) {
        Tb375fcConnSysStage1In s1_in = { (1U << CONNDRV_TYPE_WIFI), 0xFFFFFFFFU, 0U, 0U };
        Tb375fcConnSysStage1Out s1_out = {};
        int32_t rc = connsys_stage1_bringup(s1_in, &s1_out);
        if (rc != 0) {
            out->status = -100 + rc;
            out->failed_step = 1;
            return out->status;
        }
    }
    if (!g_cs.conninfra_up) {
        out->status = -1; out->failed_step = 1; return -1;
    }

    uintptr_t cs = g_cs.connsys.va;

    /* Ensure Wi-Fi subsys radio status & MT6368 VCN33_1/VCN33_2/VANT18 active */
    pmic_configure_low_power_and_radio(true, true);
    cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SUBSYS_STATUS_UPDATE, 1ULL, CONNDRV_TYPE_WIFI);

    /* 1. wake_up_conninfra_off(): 0x1806_01A4[0]=1, wait 200us, poll IP_VER & CONN_INFRA_RDY */
    cs_upd32(cs, OFF_HOST_CSR_WAKE_WF, 1U, 1U);
    delay(200);

    {
        bool ip_ok = false;
        for (int i = 0; i <= 10; ++i) {
            out->conn_ip_ver = cs_r32(cs, OFF_CONN_CFG_IP_VER);
            if (out->conn_ip_ver == EXPECTED_CONN_HW_VER) { ip_ok = true; break; }
            delay(1000);
        }
        if (!ip_ok) {
            out->status = -2; out->failed_step = 2; goto fail_release;
        }
    }

    {
        bool rdy_ok = false;
        for (int i = 0; i <= 20; ++i) {
            out->conn_rdy_reg = cs_r32(cs, OFF_CFG_ON_PWRCTRL1);
            if (out->conn_rdy_reg & (1U << 16)) { rdy_ok = true; break; }
            delay(500);
        }
        if (!rdy_ok) {
            out->status = -3; out->failed_step = 3; goto fail_release;
        }
    }

    /* 2. PTA clock on */
    cs_w32(cs, OFF_CLKGEN_COEX0_SET, 0x01010101U);
    cs_w32(cs, OFF_CLKGEN_COEX1_SET, 0x00000101U);

    /* 3. Assert WM CPU SW reset: 0x1800_0120[0] = 0 */
    cs_upd32(cs, OFF_RGU_WF_CPU_RST_B, 1U, 0U);

    /* 4. Turn off conn_infra <-> wfsys bus sleep protect: 0x1800_1440[0] = 0 */
    cs_upd32(cs, OFF_CFG_ON_WF_SLP_CTL, 1U, 0U);

    /* 5. Turn on wfsys_top_on: 0x1800_0010 [31:16]=0x5746, [7]=1 */
    cs_upd32(cs, OFF_RGU_WF_PWR_CTL, 0xFFFF0080U, WFSYS_ON_TOP_KEY | (1U << 7));

    /* 6. Poll wfsys_rgu_off_hreset_rst_b: 0x1806_0A10[30] == 1 */
    {
        bool ok = false;
        for (int i = 0; i <= 20; ++i) {
            out->pwr_states = cs_r32(cs, OFF_HOST_CSR_PWR_STA);
            if (out->pwr_states & (1U << 30)) { ok = true; break; }
            delay(500);
        }
        if (!ok) { out->status = -6; out->failed_step = 6; goto fail_release; }
    }

    /* 7. Poll conn2wf / wf2conn sleep protect off: 0x1800_1444[29,31] == 0 */
    {
        bool ok = false;
        for (int i = 0; i <= 100; ++i) {
            out->wf_slp_sta = cs_r32(cs, OFF_CFG_ON_WF_SLP_STA);
            if ((out->wf_slp_sta & ((1U << 29) | (1U << 31))) == 0U) { ok = true; break; }
            delay(500);
        }
        if (!ok) { out->status = -7; out->failed_step = 7; goto fail_release; }
    }

    /* 8. Poll WFDMA2CONN AXI TX/RX sleep protect off: 0x184C_300C[23,21] == 0 */
    {
        bool ok = false;
        for (int i = 0; i <= 100; ++i) {
            out->slpprot_on_sta = cs_r32(cs, OFF_WF_SLPPROT_STA);
            if ((out->slpprot_on_sta & ((1U << 23) | (1U << 21))) == 0U) { ok = true; break; }
            delay(500);
        }
        if (!ok) { out->status = -8; out->failed_step = 8; goto fail_release; }
    }

    /* 9. Poll WF_TOP_CFG_IP_VERSION (0x184B_0010) == 0x02040200 (MT6897) */
    {
        bool ok = false;
        for (int i = 0; i <= 20; ++i) {
            out->wf_ip_ver = cs_r32(cs, OFF_WF_TOP_IP_VER);
            if (out->wf_ip_ver == EXPECTED_WF_IP_VER) { ok = true; break; }
            delay(500);
        }
        if (!ok) { out->status = -9; out->failed_step = 9; goto fail_release; }
    }

    /* 10. Configure WFSYS bus timeout & debug_ctrl_ao remap */
    cs_upd32(cs, OFF_WF_BUSHANGCR, 0xFFU, 0x03U);
    cs_upd32(cs, OFF_WF_BUSHANGCR, (1U << 28) | (1U << 31), (1U << 28) | (1U << 31));
    cs_w32(cs, OFF_WF_AP2WF_REMAP1, 0x810F0000U);
    cs_upd32(cs, OFF_WF_REMAP0_CTRL0, (1U << 3), (1U << 3));
    cs_upd32(cs, OFF_WF_REMAP0_CTRL0, (1U << 9), (1U << 9));
    cs_upd32(cs, OFF_WF_REMAP0_CTRL0, (1U << 9), 0U);
    cs_upd32(cs, OFF_WF_REMAP0_CTRL0, 0xFFFF0000U, 0x04F50000U);
    cs_upd32(cs, OFF_WF_REMAP0_CTRL3, (1U << 8), (1U << 8));
    cs_upd32(cs, OFF_WF_REMAP0_CTRL0, (1U << 2) | (1U << 3) | (1U << 4), (1U << 2) | (1U << 3) | (1U << 4));

    /* 11. Configure monflg mailbox monitor (0x1806_0B00[0]=1, 0x1806_0B04[4:0]=0xC) */
    cs_upd32(cs, OFF_HOST_CSR_MON_EN, 1U, 1U);
    cs_upd32(cs, OFF_HOST_CSR_MON_SEL, 0x1FU, 0x0CU);

    /* 12. De-assert WM CPU SW reset: 0x1800_0120[0] = 1 */
    cs_upd32(cs, OFF_RGU_WF_CPU_RST_B, 1U, 1U);

    /* 13. Poll CONNSYS ROM idle loop marker: 0x1806_0B10 == 0x1D1E (up to 1000 ms) */
    {
        bool ok = false;
        for (int i = 0; i <= 1000; ++i) {
            out->monflg_out = cs_r32(cs, OFF_HOST_CSR_MON_OUT);
            if (out->monflg_out == CONNSYS_ROM_DONE) { ok = true; break; }
            IOSleep(1);
        }
        if (!ok) {
            out->status = -13;
            out->failed_step = 13;
            goto fail_release;
        }
    }

    /* 14. Release conn_infra off force-on & call WLAN PCCIF_ON SMC */
    cs_upd32(cs, OFF_HOST_CSR_WAKE_WF, 1U, 0U);

    {
        SmcRes r = cs_smc(MTK_SIP_KERNEL_WLAN_CONTROL, SMC_WLAN_PCCIF_ON);
        out->smc_pccif_on = static_cast<int64_t>(r.a0);
    }
    if (in.flags & 2U) {
        SmcRes r = cs_smc(MTK_SIP_KERNEL_WLAN_CONTROL, SMC_WLAN_ENABLE_FWDL_MODE);
        out->smc_fwdl_mode = static_cast<int64_t>(r.a0);
    }

    /* 15. Snapshot live WFDMA0 & Host CSR registers */
    out->wf_lpctl      = cs_r32(cs, OFF_HOST_CSR_LPCTL);
    out->conn_on_misc  = cs_r32(cs, OFF_HOST_CSR_MISC);
    out->wfdma_int_sta = cs_r32(cs, OFF_WFDMA0_INT_STA);
    out->wfdma_int_ena = cs_r32(cs, OFF_WFDMA0_INT_ENA);
    out->wfdma_glo_cfg = cs_r32(cs, OFF_WFDMA0_GLO_CFG);

    g_cs.wifi_mcu_up = true;
    kprintf("[tb375fc-connsys] Stage 3 Wi-Fi soc7_0 MCU UP! WF_IP=0x%08x MON=0x%08x PCCIF=%lld WFDMA_GLO=0x%08x LPCTL=0x%08x MISC=0x%08x\n",
            out->wf_ip_ver, out->monflg_out, out->smc_pccif_on,
            out->wfdma_glo_cfg, out->wf_lpctl, out->conn_on_misc);
    out->status = 0;
    return 0;

fail_release:
    cs_upd32(cs, OFF_HOST_CSR_WAKE_WF, 1U, 0U);
    kprintf("[tb375fc-connsys] Stage 3 Wi-Fi FAILED at step %u (rc=%d) ip=0x%08x rdy=0x%08x pwr=0x%08x slp=0x%08x prot=0x%08x wf_ip=0x%08x mon=0x%08x\n",
            out->failed_step, out->status, out->conn_ip_ver, out->conn_rdy_reg,
            out->pwr_states, out->wf_slp_sta, out->slpprot_on_sta,
            out->wf_ip_ver, out->monflg_out);
    return out->status;
}

} // namespace

#include "Tb375fcWifiEngine.h"

/* =========================================================================
 * IOService & Root-Only IOUserClient ("Tb375fcConnSys")
 * ========================================================================= */

class Tb375fcConnSys : public IOService {
    OSDeclareDefaultStructors(Tb375fcConnSys)
public:
    bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
    IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type,
                           OSDictionary *properties, IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};

class Tb375fcConnSysUserClient : public IOUserClient {
    OSDeclareDefaultStructors(Tb375fcConnSysUserClient)
public:
    task_t fTask;

    bool initWithTask(task_t owningTask, void *securityToken, UInt32 type,
                      OSDictionary *properties) APPLE_KEXT_OVERRIDE;
    IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
                            IOExternalMethodDispatch *dispatch, OSObject *target,
                            void *reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(Tb375fcConnSys, IOService)
OSDefineMetaClassAndStructors(Tb375fcConnSysUserClient, IOUserClient)

bool Tb375fcConnSys::start(IOService *provider) {
    if (!IOService::start(provider)) return false;
    setName(TB375FC_CONNSYS_SERVICE_NAME);
    setProperty("NativeSandboxGrantAll", kOSBooleanTrue);
    registerService();
    return true;
}

IOReturn Tb375fcConnSys::newUserClient(task_t owningTask, void *securityID, UInt32 type,
                                       OSDictionary *properties, IOUserClient **handler) {
    /* Strictly root-only (uid 0) for ConnSys bring-up & RO UFS firmware extraction */
    if (kauth_cred_getuid(kauth_cred_get()) != 0U) {
        return kIOReturnNotPrivileged;
    }
    Tb375fcConnSysUserClient *uc = new Tb375fcConnSysUserClient;
    if (!uc) return kIOReturnNoMemory;
    if (!uc->initWithTask(owningTask, securityID, type, properties) ||
        !uc->attach(this) || !uc->start(this)) {
        uc->release();
        return kIOReturnError;
    }
    *handler = uc;
    return kIOReturnSuccess;
}

bool Tb375fcConnSysUserClient::initWithTask(task_t owningTask, void *securityToken,
                                            UInt32 type, OSDictionary *properties) {
    if (!IOUserClient::initWithTask(owningTask, securityToken, type, properties)) return false;
    fTask = owningTask;
    return true;
}

IOReturn Tb375fcConnSysUserClient::clientClose(void) {
    terminate();
    return kIOReturnSuccess;
}

IOReturn Tb375fcConnSysUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
                                                  IOExternalMethodDispatch *dispatch, OSObject *target,
                                                  void *reference) {
    (void)dispatch; (void)target; (void)reference;
    if (!g_cs.mapped || !g_cs.lock) return kIOReturnNotReady;

    switch (selector) {
    case CONNSYS_SEL_RO_PROBE: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysProbeOut))
            return kIOReturnBadArgument;
        auto *out = static_cast<Tb375fcConnSysProbeOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        connsys_probe_readonly(out, false);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_STAGE1_UP: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcConnSysStage1In) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysStage1Out))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcConnSysStage1In *>(args->structureInput);
        auto *out = static_cast<Tb375fcConnSysStage1Out *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        connsys_stage1_bringup(*in, out);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_STAGE3_WIFI_ON: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcConnSysWifiOnIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysWifiOnOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcConnSysWifiOnIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcConnSysWifiOnOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        connsys_stage3_wifi_on(*in, out);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_UFS_READ_RO: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcConnSysUfsReadIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysUfsReadOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcConnSysUfsReadIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcConnSysUfsReadOut *>(args->structureOutput);
        out->rc = -1;
        out->bytes_read = 0;

        if (!in->user_buf || in->count == 0U || in->count > TB375FC_UFS_MAX_BLOCKS || in->lun > 7U)
            return kIOReturnBadArgument;
        uint32_t nbytes = in->count * TB375FC_UFS_BLOCK;

        uint8_t *kbuf = static_cast<uint8_t *>(IOMallocData(nbytes));
        if (!kbuf) return kIOReturnNoMemory;

        int rc = 0;
        if (in->lun == 2U) {
            for (int tries = 0; tries < 100; ++tries) {
                rc = tb375fc_ufs_read_blocks(in->lba, in->count, kbuf);
                if (rc != static_cast<int>(TB375FC_UFS_BUSY)) break;
                IOSleep(1);
            }
        } else {
            for (uint32_t b = 0; b < in->count; ++b) {
                for (int tries = 0; tries < 100; ++tries) {
                    rc = tb375fc_ufs_read_lun_block(static_cast<uint8_t>(in->lun),
                                                    in->lba + b, kbuf + b * TB375FC_UFS_BLOCK);
                    if (rc != static_cast<int>(TB375FC_UFS_BUSY)) break;
                    IOSleep(1);
                }
                if (rc != 0) break;
            }
        }

        if (rc == 0) {
            IOMemoryDescriptor *udesc = IOMemoryDescriptor::withAddressRange(
                static_cast<mach_vm_address_t>(in->user_buf), nbytes, kIODirectionInOut, fTask);
            if (!udesc || udesc->prepare() != kIOReturnSuccess) {
                if (udesc) udesc->release();
                IOFreeData(kbuf, nbytes);
                return kIOReturnVMError;
            }
            IOByteCount written = udesc->writeBytes(0, kbuf, nbytes);
            udesc->complete();
            udesc->release();
            out->rc = (written == nbytes) ? 0 : -2;
            out->bytes_read = static_cast<uint32_t>(written);
        } else {
            out->rc = rc;
        }
        IOFreeData(kbuf, nbytes);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_EMI_RW:
    case CONNSYS_SEL_DMA_RW: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcConnSysMemRwIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysMemRwOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcConnSysMemRwIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcConnSysMemRwOut *>(args->structureOutput);
        out->rc = -1;
        out->bytes_done = 0;

        const MappedRegion &reg = (selector == CONNSYS_SEL_EMI_RW) ? g_cs.emi : g_cs.dma_pool;
        if (!reg.va || !in->user_buf || in->size == 0U || in->size > (4U * 1024U * 1024U))
            return kIOReturnBadArgument;
        if (static_cast<uint64_t>(in->offset) + static_cast<uint64_t>(in->size) > reg.size)
            return kIOReturnBadArgument;

        const uint32_t chunkMax = 65536U;
        uint32_t allocSz = (in->size < chunkMax) ? in->size : chunkMax;
        uint8_t *kbuf = static_cast<uint8_t *>(IOMallocData(allocSz));
        if (!kbuf) return kIOReturnNoMemory;

        IOMemoryDescriptor *udesc = IOMemoryDescriptor::withAddressRange(
            static_cast<mach_vm_address_t>(in->user_buf), in->size,
            in->is_write ? kIODirectionOut : kIODirectionIn, fTask);
        if (!udesc || udesc->prepare() != kIOReturnSuccess) {
            if (udesc) udesc->release();
            IOFreeData(kbuf, allocSz);
            return kIOReturnVMError;
        }

        uint32_t done = 0;
        while (done < in->size) {
            uint32_t n = in->size - done;
            if (n > allocSz) n = allocSz;
            if (in->is_write) {
                if (udesc->readBytes(done, kbuf, n) != n) break;
                io_copy_to(reg.va + in->offset + done, kbuf, n, reg.is_wc);
            } else {
                io_copy_from(kbuf, reg.va + in->offset + done, n);
                if (udesc->writeBytes(done, kbuf, n) != n) break;
            }
            done += n;
        }

        udesc->complete();
        udesc->release();
        IOFreeData(kbuf, allocSz);

        out->rc = (done == in->size) ? 0 : -2;
        out->bytes_done = done;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_REG_OP: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcConnSysRegOpIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysRegOpOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcConnSysRegOpIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcConnSysRegOpOut *>(args->structureOutput);
        out->rc = -1;
        out->value = 0;

        IOLockLock(g_cs.lock);
        if (in->kind == CONNSYS_REG_MMIO_READ32 || in->kind == CONNSYS_REG_MMIO_WRITE32) {
            /* Restrict raw MMIO diag tool to safe top-level ConnInfra/WFDMA window (0x18000000..0x180FFFFC);
             * >= 0x18100000 requires AP2WF remap/wakeup and unguarded pokes can hang the AXI bus. */
            if (in->addr >= static_cast<uint32_t>(PHYS_CONNSYS_BASE) &&
                in->addr <= static_cast<uint32_t>(PHYS_CONNSYS_BASE + 0x100000U - 4U) &&
                (in->addr & 3U) == 0U && g_cs.connsys.va) {
                uint32_t off = in->addr - static_cast<uint32_t>(PHYS_CONNSYS_BASE);
                if (in->kind == CONNSYS_REG_MMIO_READ32) {
                    out->value = cs_r32(g_cs.connsys.va, off);
                } else {
                    cs_w32(g_cs.connsys.va, off, in->value);
                }
                out->rc = 0;
            }
        } else if (in->kind == CONNSYS_REG_SPMI_READ8) {
            uint8_t v = 0;
            if (spmi_read8(static_cast<uint8_t>(in->spmi_slvid), static_cast<uint16_t>(in->addr), &v)) {
                out->value = v;
                out->rc = 0;
            }
        } else if (in->kind == CONNSYS_REG_SPMI_WRITE8) {
            if (spmi_write8(static_cast<uint8_t>(in->spmi_slvid), static_cast<uint16_t>(in->addr),
                            static_cast<uint8_t>(in->value))) {
                out->rc = 0;
            }
        }
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_SMC_CALL: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcConnSysSmcIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcConnSysSmcOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcConnSysSmcIn *>(args->structureInput);
        if (in->fid != MTK_SIP_KERNEL_CONNSYS_CONTROL &&
            in->fid != MTK_SIP_KERNEL_WLAN_CONTROL &&
            in->fid != MTK_SIP_KERNEL_BT_CONTROL) {
            return kIOReturnNotPermitted;
        }
        auto *out = static_cast<Tb375fcConnSysSmcOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        SmcRes r = cs_smc(in->fid, in->x1, in->x2, in->x3, in->x4, in->x5);
        IOLockUnlock(g_cs.lock);
        out->a0 = r.a0; out->a1 = r.a1; out->a2 = r.a2; out->a3 = r.a3;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_WIFI_UP: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcWifiUpOut))
            return kIOReturnBadArgument;
        auto *out = static_cast<Tb375fcWifiUpOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        wifi_cmd_up(out);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_WIFI_SCAN: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcWifiScanIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcWifiScanOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcWifiScanIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcWifiScanOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        wifi_cmd_scan(*in, out);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_WIFI_JOIN: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcWifiJoinIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcWifiJoinOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcWifiJoinIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcWifiJoinOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        wifi_cmd_join(*in, out);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_WIFI_STATUS: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcWifiStatusOut))
            return kIOReturnBadArgument;
        auto *out = static_cast<Tb375fcWifiStatusOut *>(args->structureOutput);
        wifi_cmd_status(out);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case CONNSYS_SEL_WIFI_DOWN: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcWifiDownOut))
            return kIOReturnBadArgument;
        auto *out = static_cast<Tb375fcWifiDownOut *>(args->structureOutput);
        IOLockLock(g_cs.lock);
        wifi_cmd_down(out);
        IOLockUnlock(g_cs.lock);
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    default:
        return kIOReturnUnsupported;
    }
}

/* =========================================================================
 * Asynchronous Boot Worker & Public Entry Point
 * ========================================================================= */

static void connSysWorker(void *, wait_result_t) {
    kprintf("[tb375fc-connsys] worker starting (mode=%u)\n", g_cs.boot_mode);
    g_cs.lock = IOLockAlloc();
    if (!g_cs.lock) {
        (void)thread_terminate(current_thread());
        return;
    }

    bool ok = true;
    ok &= mapRegion(PHYS_GPIO_BASE,   0x1000ULL, false, &g_cs.gpio);
    ok &= mapRegion(PHYS_IFRBUS_AO,   0x1000ULL, false, &g_cs.ifrbus);
    ok &= mapRegion(PHYS_IOCFG_RT,    0x1000ULL, false, &g_cs.iocfg_rt);
    ok &= mapRegion(PHYS_SPM_BASE,    0x1000ULL, false, &g_cs.spm);
    ok &= mapRegion(PHYS_SRCLKEN_RC,  0x1000ULL, false, &g_cs.srclken_rc);
    ok &= mapRegion(PHYS_SPMI_PMIF_M, 0x1000ULL, false, &g_cs.spmi_pmif);
    ok &= mapRegion(PHYS_CONNSYS_BASE, CONNSYS_WIN_SIZE, false, &g_cs.connsys);
    ok &= mapRegion(PHYS_CONSYS_EMI_BASE, CONSYS_EMI_SIZE, true, &g_cs.emi);
    ok &= mapRegion(PHYS_WIFI_DMA_POOL, WIFI_DMA_POOL_SIZE, true, &g_cs.dma_pool);

    if (!ok) {
        kprintf("[tb375fc-connsys] ERROR: MMIO/EMI/DMA mapping failed!\n");
        (void)thread_terminate(current_thread());
        return;
    }
    g_cs.mapped = true;

    Tb375fcConnSysProbeOut probe = {};
    IOLockLock(g_cs.lock);
    connsys_probe_readonly(&probe, true);
    IOLockUnlock(g_cs.lock);

    Tb375fcConnSys *svc = new Tb375fcConnSys;
    if (svc && svc->init()) {
        if (svc->attach(IOService::getPlatform()) && svc->start(IOService::getPlatform())) {
            kprintf("[tb375fc-connsys] IOService \"%s\" published (root-only IOUserClient, emi_wc=%u dma_wc=%u)\n",
                    TB375FC_CONNSYS_SERVICE_NAME, g_cs.emi.is_wc ? 1U : 0U, g_cs.dma_pool.is_wc ? 1U : 0U);
        } else {
            svc->release();
        }
    }

    if (g_cs.boot_mode == 2U) {
        Tb375fcConnSysStage1In s1_in = { (1U << CONNDRV_TYPE_WIFI), 0xFFFFFFFFU, 0U, 0U };
        Tb375fcConnSysStage1Out s1_out = {};
        IOLockLock(g_cs.lock);
        connsys_stage1_bringup(s1_in, &s1_out);
        IOLockUnlock(g_cs.lock);
    } else if (g_cs.boot_mode == 4U) {
        Tb375fcConnSysWifiOnIn wf_in = { 0U, 0U };
        Tb375fcConnSysWifiOnOut wf_out = {};
        IOLockLock(g_cs.lock);
        connsys_stage3_wifi_on(wf_in, &wf_out);
        IOLockUnlock(g_cs.lock);
    } else if (g_cs.boot_mode == 5U) {
        Tb375fcWifiUpOut up_out = {};
        IOLockLock(g_cs.lock);
        wifi_cmd_up(&up_out);
        IOLockUnlock(g_cs.lock);
    }

    (void)thread_terminate(current_thread());
}

extern "C" void tb375fc_connsys_init(void) {
    static bool started = false;
    if (started) return;
    started = true;

    uint32_t mode = 0;
    if (!PE_parse_boot_argn("connsys", &mode, sizeof(mode)) || mode == 0U) {
        return;
    }
    g_cs.boot_mode = mode;

    thread_t th = THREAD_NULL;
    if (kernel_thread_start(connSysWorker, nullptr, &th) == KERN_SUCCESS && th != THREAD_NULL) {
        thread_deallocate(th);
    }
}
