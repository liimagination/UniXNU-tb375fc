/*
 * Tb375fcConnSys.cpp — Native MT6897 (Dimensity 8300) Onboard ConnSys
 *                      Wi-Fi (soc7_0 / Connac 2.x) & Bluetooth (BGF / BEIF)
 *                      XNU Kernel Bring-Up Foundation Driver
 *
 * Target: Lenovo Xiaoxin Pad Pro 12.7 (2025) / TB375FC (MT6897 + MT6637 A-Die)
 * Scope:  Strictly offline-prepared in ram-expansion/kernel/connsys/.
 *         NOT linked into Boot 152. Default inert (connsys=0).
 *
 * Architecture Overview:
 *   MT6897 integrates Wi-Fi 6 (soc7_0 / Connac 2.x AXI WFDMA) and Bluetooth 5.x
 *   (BGFSYS N10 MCU + BEIF lock-free EMI ring) inside the onboard ConnSys subsystem
 *   at 0x18000000 with an external MT6637 RF A-Die. Crucially, MediaTek's EL3 BL31
 *   ATF firmware implements the majority of D-Die, RFSPI, A-Die, EMI remapping,
 *   BGF power-on, and WLAN PCCIF sequences behind standard ARM SMCCC SiP SMCs:
 *     - MTK_SIP_KERNEL_CONNSYS_CONTROL = 0xC2000534
 *     - MTK_SIP_KERNEL_WLAN_CONTROL    = 0xC2000535
 *     - MTK_SIP_KERNEL_BT_CONTROL      = 0xC2000536
 *   And Preloader/LK already reserves & MPU-protects both shared EMI pools:
 *     - consys_emi_reserved            = 0x8cc00000 (13 MiB; BT BEIF @ +0xBA0000)
 *     - wifi-reserve-memory_dma        = 0x8d900000 (20 MiB WFDMA coherent pool)
 *
 * Boot-arg control (default OFF = 0):
 *   connsys=0 (default): Completely inert; touches zero MMIO, SPMI, or SMC.
 *   connsys=1          : Safe read-only diagnostic probe (SPM CONN_PWR_CON,
 *                        ifrbus_ao bus-protect, SPMI PMIC MT6363/MT6368/MT6685
 *                        HWCID & LDO states, EMI/BEIF magic). Writes nothing!
 *   connsys=2          : Stage 1 ConnInfra bring-up: enable MT6363 VRFIO18/VCN13
 *                        via SPMI SWINF_2, power on MT6897_POWER_DOMAIN_CONN,
 *                        run BL31 ATF D-Die / SPI / A-Die / AFE / PLL sequence,
 *                        verify CONN_HW_VER (0x02050400) & MT6637 A-Die ID.
 *   connsys=3          : Stage 2 Native Bluetooth BGF bring-up: Stage 1 +
 *                        MT6368 VCN33_1 + BL31 BT PWR_ON_{TOP,MID,END} SMCs +
 *                        verify BGF_IP_VERSION (0x02080000) & ROM ready (0x1D1E) +
 *                        initialize 32 KiB BEIF H2C/C2H HCI ring @ 0x8D7A0000.
 *   connsys=4          : Stage 3 Native Wi-Fi soc7_0 probe: Stage 1 +
 *                        MT6368 VCN33_1/VCN33_2/VANT18 + BL31 WLAN PCCIF_ON +
 *                        probe AXI WFDMA / Host CSR registers & 20 MiB DMA pool.
 *
 * Clean-room XNU implementation derived from TB375FC device tree (live-fdt.dtb)
 * and MT6897 hardware register specifications.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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

void tb375fc_connsys_init(void);
int  tb375fc_bt_beif_send_hci(const uint8_t *pkt, uint32_t len);
int  tb375fc_bt_beif_recv_hci(uint8_t *out_buf, uint32_t max_len);
}

/* =========================================================================
 * Low-level MMIO & ARM SMCCC64 Helpers
 * ========================================================================= */

static inline uint32_t cs_r32(vm_offset_t base, uint32_t off) {
    uint32_t v = *(volatile uint32_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}

static inline void cs_w32(vm_offset_t base, uint32_t off, uint32_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint32_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
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
 * TB375FC / MT6897 Physical Memory Map (verified from live-fdt.dtb)
 * ========================================================================= */

[[maybe_unused]] static constexpr uint64_t PHYS_INFRACFG_AO      = 0x10001000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_IFRBUS_AO        = 0x1002c000ULL; /* size 0x1000, scpsys ifr-bus */
static constexpr uint64_t PHYS_SPM_BASE         = 0x1c001000ULL; /* size 0x1000, scpsys */
static constexpr uint64_t PHYS_SPMI_PMIF_M      = 0x1cc04000ULL; /* size 0x1000, SPMI-M */
[[maybe_unused]] static constexpr uint64_t PHYS_SPMI_MST_M       = 0x1cc01000ULL; /* size 0x1000, SPMI-M master */

[[maybe_unused]] static constexpr uint64_t PHYS_CONN_RGU_ON      = 0x18000000ULL; /* size 0x1000 */
[[maybe_unused]] static constexpr uint64_t PHYS_CONN_CFG_ON      = 0x18001000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_CONN_CFG         = 0x18011000ULL; /* size 0x1000, CONN_HW_VER */
[[maybe_unused]] static constexpr uint64_t PHYS_CONN_DBG_CTL     = 0x18023000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_CONN_BGF_SWIRQ   = 0x1803f000ULL; /* size 0x1000, +0x10/+0x14 */
static constexpr uint64_t PHYS_CONN_SYSRAM_SWCR = 0x18053000ULL; /* size 0x1000, SW CR @ +0x800 */
static constexpr uint64_t PHYS_CONN_HOST_CSR    = 0x18060000ULL; /* size 0x1000 */
static constexpr uint64_t PHYS_BGFSYS_BASE      = 0x18800000ULL; /* size 0x1000, SW_DBG_CTL @ +0x16C */
static constexpr uint64_t PHYS_BGF_REG_INFO     = 0x18812000ULL; /* size 0x1000, BGF_IP_VER @ +0x0 */
static constexpr uint64_t PHYS_WF_WFDMA_DMA0    = 0x18024000ULL; /* Connac 2.x WFDMA host DMA0 */

/* LK-reserved & MPU-configured shared DRAM pools (live-fdt.dtb) */
static constexpr uint64_t PHYS_CONSYS_EMI_BASE  = 0x8cc00000ULL; /* 13 MiB (0x00d00000) */
[[maybe_unused]] static constexpr uint32_t CONSYS_EMI_SIZE       = 0x00d00000U;
static constexpr uint32_t BEIF_EMI_OFFSET       = 0x00ba0000U;   /* 0x8D7A0000 */
static constexpr uint32_t BEIF_EMI_SIZE         = 32U * 1024U;   /* 32 KiB */
static constexpr uint64_t PHYS_WIFI_DMA_POOL    = 0x8d900000ULL; /* 20 MiB (0x01400000) */
static constexpr uint32_t WIFI_DMA_POOL_SIZE    = 0x01400000U;

/* Hardware IDs & Magic Constants */
static constexpr uint32_t EXPECTED_CONN_HW_VER  = 0x02050400U;
static constexpr uint32_t EXPECTED_BGF_IP_VER   = 0x02080000U;
static constexpr uint32_t BGF_MCU_ROM_READY     = 0x00001D1EU;
static constexpr uint32_t BGF_SW_DBG_CTL_OFF    = 0x016CU;
[[maybe_unused]] static constexpr uint32_t ADIE_MT6637_MASK      = 0xFFFF0000U;
[[maybe_unused]] static constexpr uint32_t ADIE_MT6637_PREFIX    = 0x66370000U;

/* GIC SPI Numbers (live-fdt.dtb; GIC IRQ = SPI + 32) */
[[maybe_unused]] static constexpr uint32_t SPI_BT_BGF2AP_BTIF    = 0x37cU; /* IRQ 924 */
[[maybe_unused]] static constexpr uint32_t SPI_BT_BGF2AP_SW      = 0x266U; /* IRQ 646 */
[[maybe_unused]] static constexpr uint32_t SPI_BT_BGF2AP_WAKE    = 0x37bU; /* IRQ 923 */
static constexpr uint32_t SPI_WIFI_WFDMA        = 0x379U; /* IRQ 921 */
static constexpr uint32_t SPI_WIFI_CONN2AP_SW   = 0x265U; /* IRQ 645 */

/* =========================================================================
 * BL31 ATF SiP SMC Definitions (MT6897)
 * ========================================================================= */

static constexpr uint64_t MTK_SIP_KERNEL_CONNSYS_CONTROL = 0xC2000534ULL;
static constexpr uint64_t MTK_SIP_KERNEL_WLAN_CONTROL    = 0xC2000535ULL;
static constexpr uint64_t MTK_SIP_KERNEL_BT_CONTROL      = 0xC2000536ULL;

/* ConnSys ATF OPIDs (MTK_SIP_KERNEL_CONNSYS_CONTROL) */
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
    SMC_CONNSYS_SPI_READ              = 20,
    SMC_CONNSYS_SPI_WRITE             = 21,
    SMC_CONNSYS_SPI_UPDATE_BITS       = 22,
    SMC_CONNSYS_SPI_CLOCK_SWITCH      = 23,
    SMC_CONNSYS_SUBSYS_STATUS_UPDATE  = 24,
    SMC_CONNSYS_THERMAL_QUERY         = 25,
    SMC_CONNSYS_EMI_SET_REMAPPING_REG = 70,
    SMC_CONNSYS_ADIE_CLK_ENABLE       = 90,
};

/* ConnSys Subsystem Radio Indices */
enum ConnRadioType : uint32_t {
    CONNDRV_TYPE_BT   = 0,
    CONNDRV_TYPE_FM   = 1,
    CONNDRV_TYPE_GPS  = 2,
    CONNDRV_TYPE_WIFI = 3,
};

/* BT ATF OPIDs (MTK_SIP_KERNEL_BT_CONTROL) */
enum BtSmcOpId : uint64_t {
    SMC_BT_PWR_ON_TOP_CONSYS_MCU      = 1,
    SMC_BT_PWR_ON_MID_CONSYS_MCU      = 2,
    SMC_BT_PWR_ON_END_CONSYS_MCU      = 3,
    SMC_BT_PWR_OFF_TOP_CONSYS_MCU     = 4,
    SMC_BT_PWR_OFF_MID_CONSYS_MCU     = 5,
    SMC_BT_PWR_OFF_END_CONSYS_MCU     = 6,
    SMC_BT_CONN_INFRA_FORCE_ON_OFF    = 8,
    SMC_BT_GET_SW_IRQ_STATUS          = 11,
    SMC_BT_FW_OWN_CLR_FIRST           = 13,
    SMC_BT_FW_OWN_CLR_SECOND          = 14,
    SMC_BT_FW_OWN_SET_FIRST           = 15,
    SMC_BT_FW_OWN_SET_SECOND          = 16,
    SMC_BT_SET_BEIF_REG               = 22,
};

/* WLAN ATF OPIDs (MTK_SIP_KERNEL_WLAN_CONTROL) */
enum WlanSmcOpId : uint64_t {
    SMC_WLAN_PCCIF_ON                 = 3,
    SMC_WLAN_PCCIF_OFF                = 4,
    SMC_WLAN_ENABLE_FWDL_MODE         = 5,
};

/* =========================================================================
 * MT6897 SPMI PMIF-M (0x1cc04000, mt6xxx_regs, AP soc_chan = 2 -> SWINF_2)
 * ========================================================================= */

static constexpr uint32_t PMIF_INIT_DONE          = 0x0000U;
static constexpr uint32_t PMIF_SWINF_2_ACC        = 0x0880U;
static constexpr uint32_t PMIF_SWINF_2_WDATA      = 0x0884U;
static constexpr uint32_t PMIF_SWINF_2_RDATA      = 0x0894U;
static constexpr uint32_t PMIF_SWINF_2_VLD_CLR    = 0x08A4U;
static constexpr uint32_t PMIF_SWINF_2_STA        = 0x08A8U;

static constexpr uint32_t PMIF_CMD_EXT_REG_LONG   = 3U;
static constexpr uint32_t SWINF_FSM_IDLE          = 0U;
static constexpr uint32_t SWINF_FSM_WFVLDCLR      = 6U;

static inline uint32_t pmif_swinf_fsm(uint32_t sta) {
    return (sta >> 1) & 0x7U;
}

/* SPMI Slave IDs (TB375FC live-fdt.dtb) */
static constexpr uint8_t SPMI_SLVID_MT6363        = 0x4U;
static constexpr uint8_t SPMI_SLVID_MT6368        = 0x5U;
static constexpr uint8_t SPMI_SLVID_MT6685        = 0x9U;

/* PMIC Common HWCID registers */
static constexpr uint16_t PMIC_HWCID_L            = 0x0008U;
static constexpr uint16_t PMIC_HWCID_H            = 0x0009U;

/* MT6363 (slvid=4) ConnSys LDOs: VRFIO18 (1.7V for MT6637) & VCN13 (1.35V) */
static constexpr uint16_t MT6363_VRFIO18_EN_LP    = 0x1bcdU; /* bit0=EN, bit1=LP */
[[maybe_unused]] static constexpr uint16_t MT6363_VRFIO18_MON      = 0x1bd1U;
static constexpr uint16_t MT6363_VRFIO18_SW_OP_EN = 0x1bd4U; /* bit7 */
static constexpr uint16_t MT6363_VRFIO18_VOSEL    = 0x1f2cU; /* mask 0x0f; 0xB=1.7V, 0xC=1.8V */

static constexpr uint16_t MT6363_VCN13_EN_LP      = 0x1d07U; /* bit0=EN, bit1=LP */
[[maybe_unused]] static constexpr uint16_t MT6363_VCN13_MON        = 0x1d0bU;
static constexpr uint16_t MT6363_VCN13_SW_OP_EN   = 0x1d16U; /* bit7 */
static constexpr uint16_t MT6363_VCN13_VOSEL      = 0x1b3fU; /* mask 0x0f; 0x4=1.30V */

/* MT6368 (slvid=5) ConnSys LDOs: VCN33_1 (3.3V), VCN33_2 (3.3V), VANT18 (1.8V) */
static constexpr uint16_t MT6368_VCN33_1_EN_LP    = 0x1cc0U; /* bit0=EN, bit1=LP */
[[maybe_unused]] static constexpr uint16_t MT6368_VCN33_1_MON      = 0x1cc4U;
static constexpr uint16_t MT6368_VCN33_1_SW_OP_EN = 0x1cc7U; /* bit7 */
static constexpr uint16_t MT6368_VCN33_1_VOSEL    = 0x1db1U; /* mask 0x0f; 0xD=3.3V */

static constexpr uint16_t MT6368_VCN33_2_EN_LP    = 0x1cceU; /* bit0=EN, bit1=LP */
[[maybe_unused]] static constexpr uint16_t MT6368_VCN33_2_MON      = 0x1cd2U;
static constexpr uint16_t MT6368_VCN33_2_SW_OP_EN = 0x1cd5U; /* bit7 */
static constexpr uint16_t MT6368_VCN33_2_VOSEL    = 0x1db5U; /* mask 0x0f; 0xD=3.3V */

static constexpr uint16_t MT6368_VANT18_EN_LP     = 0x1c23U; /* bit0=EN, bit1=LP */
[[maybe_unused]] static constexpr uint16_t MT6368_VANT18_MON       = 0x1c27U;
static constexpr uint16_t MT6368_VANT18_SW_OP_EN  = 0x1c2aU; /* bit7 */
static constexpr uint16_t MT6368_VANT18_VOSEL     = 0x1e09U; /* mask 0x0f; 0xC=1.8V */

/* =========================================================================
 * MT6897 SPM & ifrbus_ao (MT6897_POWER_DOMAIN_CONN)
 * ========================================================================= */

static constexpr uint32_t SPM_CONN_PWR_CON        = 0x0E04U;
static constexpr uint32_t PWR_RST_B_BIT           = (1U << 0);
static constexpr uint32_t PWR_ISO_BIT             = (1U << 1);
static constexpr uint32_t PWR_ON_BIT              = (1U << 2);
static constexpr uint32_t PWR_ON_2ND_BIT          = (1U << 3);
static constexpr uint32_t PWR_CLK_DIS_BIT         = (1U << 4);
static constexpr uint32_t PWR_ACK_BIT             = (1U << 30);
static constexpr uint32_t PWR_ACK_2ND_BIT         = (1U << 31);

/* ifrbus_ao (0x1002c000) bus protection offsets for CONN */
static constexpr uint32_t IFR_PROT0_EN            = 0x000U;
[[maybe_unused]] static constexpr uint32_t IFR_PROT0_SET           = 0x004U;
static constexpr uint32_t IFR_PROT0_CLR           = 0x008U;
static constexpr uint32_t IFR_PROT0_RDY           = 0x00CU;
static constexpr uint32_t IFR_PROT_CONN_BIT1      = (1U << 25);
static constexpr uint32_t IFR_PROT_CONN_BIT2      = (1U << 26);

static constexpr uint32_t IFR_CONNSYS0_EN         = 0x1C0U;
[[maybe_unused]] static constexpr uint32_t IFR_CONNSYS0_SET        = 0x1C4U;
static constexpr uint32_t IFR_CONNSYS0_CLR        = 0x1C8U;
static constexpr uint32_t IFR_CONNSYS0_RDY        = 0x1CCU;
static constexpr uint32_t IFR_CONNSYS_BIT1        = (1U << 1);
static constexpr uint32_t IFR_CONNSYS_BIT2        = (1U << 0);

/* =========================================================================
 * Bluetooth BEIF Lock-Free Shared Memory Ring (0x8D7A0000, 32 KiB)
 * ========================================================================= */

static constexpr uint32_t BEIF_HEADER_KEY         = 0x55665566U;
static constexpr uint32_t BEIF_HEADER_VERSION     = 0U;
static constexpr uint32_t BEIF_GUARD_PATTERN      = 0xDEADBEEFU;
static constexpr uint32_t BEIF_GUARD_SIZE         = 32U;

struct __attribute__((packed)) BeifHeader {
    uint32_t key;
    uint32_t version;
    uint32_t host_ctrl_offset;
    uint32_t fw_ctrl_offset;
    uint32_t h2c_buf_offset;
    uint32_t h2c_buf_size;
    uint32_t c2h_buf_offset;
    uint32_t c2h_buf_size;
};

struct __attribute__((packed)) BeifCtrl {
    uint32_t tx_push;
    uint32_t rx_pop;
    uint32_t dir;
    uint32_t mode;
    uint32_t burst_size;
    uint32_t packet_size;
    uint32_t status;
    uint32_t guardp;
};

/* =========================================================================
 * Driver State
 * ========================================================================= */

struct ConnSysState {
    vm_offset_t spm;
    vm_offset_t ifrbus;
    vm_offset_t spmi_pmif;
    vm_offset_t conn_cfg;
    vm_offset_t conn_sysram;
    vm_offset_t conn_host_csr;
    vm_offset_t bgf_base;
    vm_offset_t bgf_info;
    vm_offset_t bgf_swirq;
    vm_offset_t wf_wfdma;
    vm_offset_t beif_emi;

    bool        conninfra_up;
    bool        bt_up;
    bool        beif_ready;
    uint32_t    adie_hw_ver;
    uint32_t    h2c_off;
    uint32_t    h2c_size;
    uint32_t    c2h_off;
    uint32_t    c2h_size;
};

static ConnSysState g_cs = {};

/* =========================================================================
 * SPMI PMIF-M Read / Write Helpers (SWINF_2)
 * ========================================================================= */

static bool spmi_wait_fsm(uint32_t want_state, int timeout_us) {
    for (int waited = 0; waited < timeout_us; waited += 10) {
        uint32_t sta = cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA);
        if (pmif_swinf_fsm(sta) == want_state)
            return true;
        delay(10);
    }
    return false;
}

static bool spmi_read8(uint8_t slvid, uint16_t addr, uint8_t *out_val) {
    if (!g_cs.spmi_pmif || !out_val) return false;

    uint32_t sta = cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA);
    if (pmif_swinf_fsm(sta) == SWINF_FSM_WFVLDCLR) {
        cs_w32(g_cs.spmi_pmif, PMIF_SWINF_2_VLD_CLR, 1U);
    }
    if (!spmi_wait_fsm(SWINF_FSM_IDLE, 20000)) {
        kprintf("[tb375fc-connsys] SPMI read IDLE timeout slv=%u addr=0x%04x sta=0x%08x\n",
                slvid, addr, cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA));
        return false;
    }

    uint32_t cmd = (PMIF_CMD_EXT_REG_LONG << 30) |
                   ((uint32_t)(slvid & 0xFU) << 24) |
                   (0U << 16) | /* bc = len - 1 = 0 */
                   (uint32_t)addr;
    cs_w32(g_cs.spmi_pmif, PMIF_SWINF_2_ACC, cmd);

    if (!spmi_wait_fsm(SWINF_FSM_WFVLDCLR, 20000)) {
        kprintf("[tb375fc-connsys] SPMI read WFVLDCLR timeout slv=%u addr=0x%04x sta=0x%08x\n",
                slvid, addr, cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA));
        return false;
    }

    uint32_t rdata = cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_RDATA);
    cs_w32(g_cs.spmi_pmif, PMIF_SWINF_2_VLD_CLR, 1U);
    *out_val = (uint8_t)(rdata & 0xFFU);
    return true;
}

static bool spmi_write8(uint8_t slvid, uint16_t addr, uint8_t val) {
    if (!g_cs.spmi_pmif) return false;

    uint32_t sta = cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA);
    if (pmif_swinf_fsm(sta) == SWINF_FSM_WFVLDCLR) {
        cs_w32(g_cs.spmi_pmif, PMIF_SWINF_2_VLD_CLR, 1U);
    }
    if (!spmi_wait_fsm(SWINF_FSM_IDLE, 20000)) {
        kprintf("[tb375fc-connsys] SPMI write IDLE timeout slv=%u addr=0x%04x sta=0x%08x\n",
                slvid, addr, cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA));
        return false;
    }

    cs_w32(g_cs.spmi_pmif, PMIF_SWINF_2_WDATA, (uint32_t)val);
    uint32_t cmd = (PMIF_CMD_EXT_REG_LONG << 30) |
                   (1U << 29) | /* write bit */
                   ((uint32_t)(slvid & 0xFU) << 24) |
                   (0U << 16) |
                   (uint32_t)addr;
    cs_w32(g_cs.spmi_pmif, PMIF_SWINF_2_ACC, cmd);
    return spmi_wait_fsm(SWINF_FSM_IDLE, 20000);
}

static bool spmi_update8(uint8_t slvid, uint16_t addr, uint8_t mask, uint8_t val) {
    uint8_t cur = 0;
    if (!spmi_read8(slvid, addr, &cur)) return false;
    uint8_t next = (uint8_t)((cur & (uint8_t)~mask) | (val & mask));
    return spmi_write8(slvid, addr, next);
}

/* =========================================================================
 * Stage 0: Safe Read-Only Diagnostic Probe (connsys=1)
 * ========================================================================= */

static void connsys_probe_readonly(void) {
    uint32_t pwr_con   = cs_r32(g_cs.spm, SPM_CONN_PWR_CON);
    uint32_t prot0_en  = cs_r32(g_cs.ifrbus, IFR_PROT0_EN);
    uint32_t prot0_rdy = cs_r32(g_cs.ifrbus, IFR_PROT0_RDY);
    uint32_t cprot_en  = cs_r32(g_cs.ifrbus, IFR_CONNSYS0_EN);
    uint32_t cprot_rdy = cs_r32(g_cs.ifrbus, IFR_CONNSYS0_RDY);
    uint32_t pmif_init = cs_r32(g_cs.spmi_pmif, PMIF_INIT_DONE);
    uint32_t pmif_sta  = cs_r32(g_cs.spmi_pmif, PMIF_SWINF_2_STA);

    kprintf("[tb375fc-connsys] RO probe: SPM_CONN_PWR_CON=0x%08x (ON=%u ACK=%u ISO=%u RST_B=%u)\n",
            pwr_con,
            (pwr_con & PWR_ON_BIT) ? 1U : 0U,
            (pwr_con & PWR_ACK_BIT) ? 1U : 0U,
            (pwr_con & PWR_ISO_BIT) ? 1U : 0U,
            (pwr_con & PWR_RST_B_BIT) ? 1U : 0U);
    kprintf("[tb375fc-connsys] RO probe: IFRBUS prot0(en=0x%08x rdy=0x%08x) connsys0(en=0x%08x rdy=0x%08x)\n",
            prot0_en, prot0_rdy, cprot_en, cprot_rdy);
    kprintf("[tb375fc-connsys] RO probe: SPMI-M init_done=0x%08x swinf2_sta=0x%08x (fsm=%u)\n",
            pmif_init, pmif_sta, pmif_swinf_fsm(pmif_sta));

    /* Probe MT6363 (slv=4), MT6368 (slv=5), MT6685 (slv=9) read-only */
    const uint8_t slvs[3] = { SPMI_SLVID_MT6363, SPMI_SLVID_MT6368, SPMI_SLVID_MT6685 };
    const char *names[3]  = { "MT6363", "MT6368", "MT6685" };
    for (int i = 0; i < 3; i++) {
        uint8_t lo = 0, hi = 0;
        bool ok_l = spmi_read8(slvs[i], PMIC_HWCID_L, &lo);
        bool ok_h = spmi_read8(slvs[i], PMIC_HWCID_H, &hi);
        kprintf("[tb375fc-connsys] RO probe: PMIC %s (slv=%u) hwcid=0x%02x%02x (ok=%u/%u)\n",
                names[i], slvs[i], hi, lo, ok_h ? 1U : 0U, ok_l ? 1U : 0U);
    }

    uint8_t vrf_en = 0, vrf_vs = 0, vcn13_en = 0, vcn13_vs = 0;
    uint8_t v33_1 = 0, v33_2 = 0, vant = 0;
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VRFIO18_EN_LP, &vrf_en);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VRFIO18_VOSEL, &vrf_vs);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VCN13_EN_LP, &vcn13_en);
    spmi_read8(SPMI_SLVID_MT6363, MT6363_VCN13_VOSEL, &vcn13_vs);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_1_EN_LP, &v33_1);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VCN33_2_EN_LP, &v33_2);
    spmi_read8(SPMI_SLVID_MT6368, MT6368_VANT18_EN_LP, &vant);
    kprintf("[tb375fc-connsys] RO probe: LDOs VRFIO18(en=0x%02x vs=0x%02x) VCN13(en=0x%02x vs=0x%02x) "
            "VCN33_1=0x%02x VCN33_2=0x%02x VANT18=0x%02x\n",
            vrf_en, vrf_vs, vcn13_en, vcn13_vs, v33_1, v33_2, vant);

    /* Only read 0x18011000 if MTCMOS CONN is already ON & unprotected */
    bool conn_powered = ((pwr_con & (PWR_ON_BIT | PWR_ACK_BIT | PWR_RST_B_BIT)) ==
                         (PWR_ON_BIT | PWR_ACK_BIT | PWR_RST_B_BIT)) &&
                        ((pwr_con & (PWR_ISO_BIT | PWR_CLK_DIS_BIT)) == 0U);
    bool bus_open     = ((prot0_en & (IFR_PROT_CONN_BIT1 | IFR_PROT_CONN_BIT2)) == 0U) &&
                        ((cprot_en & (IFR_CONNSYS_BIT1 | IFR_CONNSYS_BIT2)) == 0U);
    if (conn_powered && bus_open && g_cs.conn_cfg) {
        uint32_t hw_ver = cs_r32(g_cs.conn_cfg, 0x0);
        kprintf("[tb375fc-connsys] RO probe: CONN_HW_VER(0x18011000)=0x%08x (expect 0x%08x)\n",
                hw_ver, EXPECTED_CONN_HW_VER);
    } else {
        kprintf("[tb375fc-connsys] RO probe: CONN domain off/protected (powered=%u bus_open=%u) — skipped 0x18011000 read safely\n",
                conn_powered ? 1U : 0U, bus_open ? 1U : 0U);
    }

    if (g_cs.beif_emi) {
        uint32_t key = cs_r32(g_cs.beif_emi, 0x0);
        uint32_t ver = cs_r32(g_cs.beif_emi, 0x4);
        kprintf("[tb375fc-connsys] RO probe: BEIF EMI(0x%llx) key=0x%08x ver=0x%08x\n",
                PHYS_CONSYS_EMI_BASE + BEIF_EMI_OFFSET, key, ver);
    }
}

/* =========================================================================
 * Stage 1: MT6897 Power Domain CONN + PMIC Rails + BL31 ConnInfra Bring-Up
 * ========================================================================= */

static bool spm_power_on_conn_domain(void) {
    uint32_t v = cs_r32(g_cs.spm, SPM_CONN_PWR_CON);
    bool already_on = ((v & (PWR_ON_BIT | PWR_ON_2ND_BIT | PWR_ACK_BIT | PWR_ACK_2ND_BIT | PWR_RST_B_BIT)) ==
                       (PWR_ON_BIT | PWR_ON_2ND_BIT | PWR_ACK_BIT | PWR_ACK_2ND_BIT | PWR_RST_B_BIT)) &&
                      ((v & (PWR_ISO_BIT | PWR_CLK_DIS_BIT)) == 0U);
    if (!already_on) {
        v |= PWR_ON_BIT;
        cs_w32(g_cs.spm, SPM_CONN_PWR_CON, v);
        v |= PWR_ON_2ND_BIT;
        cs_w32(g_cs.spm, SPM_CONN_PWR_CON, v);

        bool acked = false;
        for (int i = 0; i < 1000; i++) {
            uint32_t cur = cs_r32(g_cs.spm, SPM_CONN_PWR_CON);
            if ((cur & (PWR_ACK_BIT | PWR_ACK_2ND_BIT)) == (PWR_ACK_BIT | PWR_ACK_2ND_BIT)) {
                acked = true;
                break;
            }
            delay(10);
        }
        if (!acked) {
            kprintf("[tb375fc-connsys] SPM CONN PWR_ACK timeout! con=0x%08x\n",
                    cs_r32(g_cs.spm, SPM_CONN_PWR_CON));
            return false;
        }

        v = cs_r32(g_cs.spm, SPM_CONN_PWR_CON);
        v &= ~PWR_CLK_DIS_BIT;
        cs_w32(g_cs.spm, SPM_CONN_PWR_CON, v);
        v &= ~PWR_ISO_BIT;
        cs_w32(g_cs.spm, SPM_CONN_PWR_CON, v);
        v |= PWR_RST_B_BIT;
        cs_w32(g_cs.spm, SPM_CONN_PWR_CON, v);
        delay(50);
    }

    /* Release ifrbus_ao bus protections in reverse order of mtk-scpsys-mt6897.c */
    struct BpStep { uint32_t clr_off; uint32_t rdy_off; uint32_t mask; };
    static const BpStep steps[4] = {
        { IFR_CONNSYS0_CLR, IFR_CONNSYS0_RDY, IFR_CONNSYS_BIT2 },
        { IFR_PROT0_CLR,    IFR_PROT0_RDY,    IFR_PROT_CONN_BIT2 },
        { IFR_CONNSYS0_CLR, IFR_CONNSYS0_RDY, IFR_CONNSYS_BIT1 },
        { IFR_PROT0_CLR,    IFR_PROT0_RDY,    IFR_PROT_CONN_BIT1 },
    };
    for (int i = 0; i < 4; i++) {
        cs_w32(g_cs.ifrbus, steps[i].clr_off, steps[i].mask);
        for (int t = 0; t < 1000; t++) {
            if ((cs_r32(g_cs.ifrbus, steps[i].rdy_off) & steps[i].mask) == 0U)
                break;
            delay(10);
        }
    }

    kprintf("[tb375fc-connsys] SPM CONN powered ON: con=0x%08x prot0=0x%08x connsys0=0x%08x\n",
            cs_r32(g_cs.spm, SPM_CONN_PWR_CON),
            cs_r32(g_cs.ifrbus, IFR_PROT0_EN),
            cs_r32(g_cs.ifrbus, IFR_CONNSYS0_EN));
    return true;
}

static bool pmic_enable_common_rails(void) {
    /*
     * MT6897 + MT6637 A-Die common rails on MT6363 (slv=4):
     *   - VRFIO18: 1.7V (vosel=0xB in table3), SW_OP_EN bit7=1, EN=1, LP=0
     *   - VCN13  : 1.35V (vosel=0x4 base + factory vocal), SW_OP_EN bit7=1, EN=1, LP=0
     */
    bool ok = true;
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_VOSEL, 0x0FU, 0x0BU);
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_SW_OP_EN, 0x80U, 0x80U);
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VRFIO18_EN_LP, 0x03U, 0x01U);

    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_SW_OP_EN, 0x80U, 0x80U);
    ok &= spmi_update8(SPMI_SLVID_MT6363, MT6363_VCN13_EN_LP, 0x03U, 0x01U);
    delay(2000);

    kprintf("[tb375fc-connsys] PMIC common rails (MT6363 VRFIO18/VCN13) ok=%u\n", ok ? 1U : 0U);
    return ok;
}

static bool pmic_enable_radio_rails(bool enable_wifi) {
    /*
     * MT6368 (slv=5):
     *   - VCN33_1 (3.3V, vosel=0xD): required for BT & Wi-Fi
     *   - VCN33_2 (3.3V, vosel=0xD): required for Wi-Fi
     *   - VANT18  (1.8V, vosel=0xC): external FEM/antenna switch supply
     */
    bool ok = true;
    ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_VOSEL, 0x0FU, 0x0DU);
    ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_SW_OP_EN, 0x80U, 0x80U);
    ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_1_EN_LP, 0x03U, 0x01U);

    ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_VOSEL, 0x0FU, 0x0CU);
    ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_SW_OP_EN, 0x80U, 0x80U);
    ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VANT18_EN_LP, 0x03U, 0x01U);

    if (enable_wifi) {
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_VOSEL, 0x0FU, 0x0DU);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_SW_OP_EN, 0x80U, 0x80U);
        ok &= spmi_update8(SPMI_SLVID_MT6368, MT6368_VCN33_2_EN_LP, 0x03U, 0x01U);
    }
    delay(1000);
    kprintf("[tb375fc-connsys] PMIC radio rails (MT6368 VCN33_1/VANT18%s) ok=%u\n",
            enable_wifi ? "/VCN33_2" : "", ok ? 1U : 0U);
    return ok;
}

static bool connsys_stage1_bringup(uint32_t radio_mask) {
    if (g_cs.conninfra_up) return true;

    if (!pmic_enable_common_rails()) return false;
    if (!spm_power_on_conn_domain()) return false;

    /* 1. Initialize BL31 ConnSys ATF state (platform_config=0, clock_type=1 -> 26M co-clock) */
    SmcRes r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_INIT_ATF_DATA, 0, 1);
    kprintf("[tb375fc-connsys] ATF INIT_ATF_DATA -> a0=0x%llx\n", r.a0);

    /* 2. Poll ConnInfra D-Die Chip ID via BL31 */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONSYS_POLLING_CHIPID);
    uint32_t hw_ver = cs_r32(g_cs.conn_cfg, 0x0);
    kprintf("[tb375fc-connsys] ATF POLLING_CHIPID -> a0=0x%llx CONN_HW_VER=0x%08x\n", r.a0, hw_ver);
    if ((int64_t)r.a0 != 0) return false;

    /* 3. Set EMI remapping registers (MPU already configured in LK) */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_EMI_SET_REMAPPING_REG);
    kprintf("[tb375fc-connsys] ATF EMI_SET_REMAPPING -> a0=0x%llx\n", r.a0);

    /* 4. Configure D-Die, RFSPI master, and probe/configure MT6637 A-Die */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_D_DIE_CFG);
    kprintf("[tb375fc-connsys] ATF D_DIE_CFG -> a0=0x%llx\n", r.a0);
    if ((int64_t)r.a0 != 0) return false;

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SPI_MASTER_CFG, 0, radio_mask);
    kprintf("[tb375fc-connsys] ATF SPI_MASTER_CFG -> a0=0x%llx\n", r.a0);
    if ((int64_t)r.a0 != 0) return false;

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_A_DIE_CFG, 0, radio_mask);
    g_cs.adie_hw_ver = (uint32_t)r.a1;
    kprintf("[tb375fc-connsys] ATF A_DIE_CFG -> a0=0x%llx A_DIE_HW_VER=0x%08x\n",
            r.a0, g_cs.adie_hw_ver);
    if ((int64_t)r.a0 != 0) return false;

    /* 5. AFE WBG calibration, SW patch, subsys PLL, and low-power settings */
    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_AFE_WBG_CAL);
    kprintf("[tb375fc-connsys] ATF AFE_WBG_CAL -> a0=0x%llx\n", r.a0);

    cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_AFE_SW_PATCH);

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SUBSYS_PLL_INITIAL);
    kprintf("[tb375fc-connsys] ATF SUBSYS_PLL_INITIAL -> a0=0x%llx\n", r.a0);
    if ((int64_t)r.a0 != 0) return false;

    r = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_LOW_POWER_SETTING, 0, radio_mask);
    kprintf("[tb375fc-connsys] ATF LOW_POWER_SETTING -> a0=0x%llx\n", r.a0);

    SmcRes th = cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_THERMAL_QUERY);
    kprintf("[tb375fc-connsys] Stage 1 ConnInfra UP! adie=0x%08x thermal_raw=0x%llx\n",
            g_cs.adie_hw_ver, th.a0);
    g_cs.conninfra_up = true;
    return true;
}

/* =========================================================================
 * Stage 2: Native Bluetooth BGF Power-On & BEIF Lock-Free HCI Ring
 * ========================================================================= */

static bool bt_beif_reset_ring(void) {
    if (!g_cs.beif_emi) return false;

    const uint32_t hdr_sz  = (uint32_t)sizeof(BeifHeader);
    const uint32_t ctrl_sz = (uint32_t)sizeof(BeifCtrl);
    const uint32_t buf_sz  = (BEIF_EMI_SIZE - hdr_sz - (ctrl_sz + BEIF_GUARD_SIZE) * 2U) / 2U;
    if (buf_sz < 1024U) return false;

    g_cs.h2c_off  = hdr_sz + 2U * ctrl_sz;
    g_cs.h2c_size = buf_sz;
    g_cs.c2h_off  = g_cs.h2c_off + g_cs.h2c_size + BEIF_GUARD_SIZE;
    g_cs.c2h_size = buf_sz;

    /* Populate BeifHeader at base of 0x8D7A0000 */
    cs_w32(g_cs.beif_emi, 0x00U, BEIF_HEADER_KEY);
    cs_w32(g_cs.beif_emi, 0x04U, BEIF_HEADER_VERSION);
    cs_w32(g_cs.beif_emi, 0x08U, hdr_sz);
    cs_w32(g_cs.beif_emi, 0x0CU, hdr_sz + ctrl_sz);
    cs_w32(g_cs.beif_emi, 0x10U, g_cs.h2c_off);
    cs_w32(g_cs.beif_emi, 0x14U, g_cs.h2c_size);
    cs_w32(g_cs.beif_emi, 0x18U, g_cs.c2h_off);
    cs_w32(g_cs.beif_emi, 0x1CU, g_cs.c2h_size);

    /* Host control block (tx_push, rx_pop, ..., guardp) */
    for (uint32_t off = 0; off < ctrl_sz; off += 4U) {
        cs_w32(g_cs.beif_emi, hdr_sz + off, 0U);
        cs_w32(g_cs.beif_emi, hdr_sz + ctrl_sz + off, 0U);
    }
    cs_w32(g_cs.beif_emi, hdr_sz + 28U, BEIF_GUARD_PATTERN);
    cs_w32(g_cs.beif_emi, hdr_sz + ctrl_sz + 28U, BEIF_GUARD_PATTERN);

    /* Zero H2C & C2H ring buffers and write 32-byte guard patterns */
    for (uint32_t i = 0; i < g_cs.h2c_size; i += 4U) {
        cs_w32(g_cs.beif_emi, g_cs.h2c_off + i, 0U);
        cs_w32(g_cs.beif_emi, g_cs.c2h_off + i, 0U);
    }
    for (uint32_t i = 0; i < BEIF_GUARD_SIZE; i += 4U) {
        cs_w32(g_cs.beif_emi, g_cs.h2c_off + g_cs.h2c_size + i, BEIF_GUARD_PATTERN);
        cs_w32(g_cs.beif_emi, g_cs.c2h_off + g_cs.c2h_size + i, BEIF_GUARD_PATTERN);
    }

    g_cs.beif_ready = true;
    kprintf("[tb375fc-connsys] BEIF HCI ring ready @ 0x%llx (h2c=+0x%x/%uB c2h=+0x%x/%uB)\n",
            PHYS_CONSYS_EMI_BASE + BEIF_EMI_OFFSET,
            g_cs.h2c_off, g_cs.h2c_size, g_cs.c2h_off, g_cs.c2h_size);
    return true;
}

extern "C" int tb375fc_bt_beif_send_hci(const uint8_t *pkt, uint32_t len) {
    if (!g_cs.beif_ready || !pkt || len == 0 || len >= g_cs.h2c_size) return -1;

    const uint32_t host_off = (uint32_t)sizeof(BeifHeader);
    const uint32_t fw_off   = host_off + (uint32_t)sizeof(BeifCtrl);

    uint32_t push = cs_r32(g_cs.beif_emi, host_off + 0U);
    uint32_t pop  = cs_r32(g_cs.beif_emi, fw_off   + 4U);
    uint32_t space = (push >= pop) ? (g_cs.h2c_size - (push - pop) - 1U) : (pop - push - 1U);
    if (space < len) return -2;

    volatile uint8_t *ring = (volatile uint8_t *)(g_cs.beif_emi + g_cs.h2c_off);
    for (uint32_t i = 0; i < len; i++) {
        ring[(push + i) % g_cs.h2c_size] = pkt[i];
    }
    __asm__ volatile("dsb st" ::: "memory");
    cs_w32(g_cs.beif_emi, host_off + 0U, (push + len) % g_cs.h2c_size);

    /* Trigger BGF MCU BEIF doorbell IRQ via BL31 ATF */
    cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_SET_BEIF_REG);
    return (int)len;
}

extern "C" int tb375fc_bt_beif_recv_hci(uint8_t *out_buf, uint32_t max_len) {
    if (!g_cs.beif_ready || !out_buf || max_len == 0) return -1;

    const uint32_t host_off = (uint32_t)sizeof(BeifHeader);
    const uint32_t fw_off   = host_off + (uint32_t)sizeof(BeifCtrl);

    uint32_t push = cs_r32(g_cs.beif_emi, fw_off   + 0U);
    uint32_t pop  = cs_r32(g_cs.beif_emi, host_off + 4U);
    uint32_t avail = (push >= pop) ? (push - pop) : (g_cs.c2h_size - pop + push);
    if (avail == 0) return 0;

    uint32_t take = (avail < max_len) ? avail : max_len;
    volatile uint8_t *ring = (volatile uint8_t *)(g_cs.beif_emi + g_cs.c2h_off);
    for (uint32_t i = 0; i < take; i++) {
        out_buf[i] = ring[(pop + i) % g_cs.c2h_size];
    }
    __asm__ volatile("dsb ld" ::: "memory");
    cs_w32(g_cs.beif_emi, host_off + 4U, (pop + take) % g_cs.c2h_size);
    return (int)take;
}

static bool connsys_stage2_bt_bringup(void) {
    if (!connsys_stage1_bringup(1U << CONNDRV_TYPE_BT)) return false;
    if (!pmic_enable_radio_rails(false)) return false;

    cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SUBSYS_STATUS_UPDATE, 1, CONNDRV_TYPE_BT);
    cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_CONN_INFRA_FORCE_ON_OFF, 1);

    /* Power on BGF top & mid via BL31 ATF */
    SmcRes r = cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_PWR_ON_TOP_CONSYS_MCU);
    kprintf("[tb375fc-connsys] BT ATF PWR_ON_TOP -> a0=0x%llx\n", r.a0);
    if ((int64_t)r.a0 < 0) goto fail;

    r = cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_PWR_ON_MID_CONSYS_MCU);
    kprintf("[tb375fc-connsys] BT ATF PWR_ON_MID -> a0=0x%llx\n", r.a0);
    if ((int64_t)r.a0 < 0) goto fail;

    {
        uint32_t bgf_ver = cs_r32(g_cs.bgf_info, 0x0);
        kprintf("[tb375fc-connsys] BGF_IP_VERSION(0x18812000)=0x%08x (expect 0x%08x)\n",
                bgf_ver, EXPECTED_BGF_IP_VER);

        /* Poll BGF MCU ROM idle marker (0x1D1E) */
        uint32_t mcu_dbg = 0;
        bool rom_ready = false;
        for (int i = 0; i < 400; i++) {
            mcu_dbg = cs_r32(g_cs.bgf_base, BGF_SW_DBG_CTL_OFF);
            if (mcu_dbg == BGF_MCU_ROM_READY) {
                rom_ready = true;
                break;
            }
            delay(5000);
        }
        kprintf("[tb375fc-connsys] BGF_MCU_CFG_SW_DBG_CTL=0x%08x (rom_ready=%u)\n",
                mcu_dbg, rom_ready ? 1U : 0U);
        if (!rom_ready) goto fail;
    }

    r = cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_PWR_ON_END_CONSYS_MCU);
    kprintf("[tb375fc-connsys] BT ATF PWR_ON_END -> a0=0x%llx\n", r.a0);
    if ((int64_t)r.a0 < 0) goto fail;

    cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_CONN_INFRA_FORCE_ON_OFF, 0);

    if (!bt_beif_reset_ring()) return false;
    g_cs.bt_up = true;
    kprintf("[tb375fc-connsys] Stage 2 BT BGF UP & BEIF ready! (awaiting soc7_0_ram_{mcu,bt}_1_1_hdr.bin load)\n");
    return true;

fail:
    cs_smc(MTK_SIP_KERNEL_BT_CONTROL, SMC_BT_CONN_INFRA_FORCE_ON_OFF, 0);
    return false;
}

/* =========================================================================
 * Stage 3: Native Wi-Fi soc7_0 (Connac 2.x AXI WFDMA) Probe
 * ========================================================================= */

static bool connsys_stage3_wifi_probe(void) {
    if (!connsys_stage1_bringup(1U << CONNDRV_TYPE_WIFI)) return false;
    if (!pmic_enable_radio_rails(true)) return false;

    cs_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_SUBSYS_STATUS_UPDATE, 1, CONNDRV_TYPE_WIFI);

    SmcRes r = cs_smc(MTK_SIP_KERNEL_WLAN_CONTROL, SMC_WLAN_PCCIF_ON);
    kprintf("[tb375fc-connsys] WLAN ATF PCCIF_ON -> a0=0x%llx\n", r.a0);

    uint32_t wfdma_glo = cs_r32(g_cs.wf_wfdma, 0x208U); /* WPDMA_GLO_CFG */
    uint32_t host_lp   = cs_r32(g_cs.conn_host_csr, 0x10U); /* BN0_LPCTL */
    kprintf("[tb375fc-connsys] Stage 3 WLAN probe: WFDMA_GLO_CFG=0x%08x HOST_LPCTL=0x%08x "
            "DMA_POOL=0x%llx/%uMiB (SPI=%u/%u)\n",
            wfdma_glo, host_lp, PHYS_WIFI_DMA_POOL, WIFI_DMA_POOL_SIZE >> 20,
            SPI_WIFI_WFDMA, SPI_WIFI_CONN2AP_SW);
    return true;
}

/* =========================================================================
 * Public Entry Point (Boot-Arg Gated, Default Inert)
 * ========================================================================= */

extern "C" void tb375fc_connsys_init(void) {
    uint32_t mode = 0;
    if (!PE_parse_boot_argn("connsys", &mode, sizeof(mode)) || mode == 0) {
        return; /* Strictly inert by default */
    }

    kprintf("[tb375fc-connsys] init mode=%u (1=RO-probe 2=conninfra 3=BT-BGF+BEIF 4=WiFi-probe)\n", mode);

    g_cs.spm           = ml_io_map(PHYS_SPM_BASE, 0x1000);
    g_cs.ifrbus        = ml_io_map(PHYS_IFRBUS_AO, 0x1000);
    g_cs.spmi_pmif     = ml_io_map(PHYS_SPMI_PMIF_M, 0x1000);
    g_cs.conn_cfg      = ml_io_map(PHYS_CONN_CFG, 0x1000);
    g_cs.conn_sysram   = ml_io_map(PHYS_CONN_SYSRAM_SWCR, 0x1000);
    g_cs.conn_host_csr = ml_io_map(PHYS_CONN_HOST_CSR, 0x1000);
    g_cs.bgf_base      = ml_io_map(PHYS_BGFSYS_BASE, 0x1000);
    g_cs.bgf_info      = ml_io_map(PHYS_BGF_REG_INFO, 0x1000);
    g_cs.bgf_swirq     = ml_io_map(PHYS_CONN_BGF_SWIRQ, 0x1000);
    g_cs.wf_wfdma      = ml_io_map(PHYS_WF_WFDMA_DMA0, 0x1000);
    g_cs.beif_emi      = ml_io_map(PHYS_CONSYS_EMI_BASE + BEIF_EMI_OFFSET, BEIF_EMI_SIZE);

    if (!g_cs.spm || !g_cs.ifrbus || !g_cs.spmi_pmif) {
        kprintf("[tb375fc-connsys] ERROR: ml_io_map failed\n");
        return;
    }

    connsys_probe_readonly();
    if (mode == 1) return;

    if (mode == 2) {
        connsys_stage1_bringup(1U << CONNDRV_TYPE_BT);
        return;
    }
    if (mode == 3) {
        connsys_stage2_bt_bringup();
        return;
    }
    if (mode == 4) {
        connsys_stage3_wifi_probe();
        return;
    }
}
