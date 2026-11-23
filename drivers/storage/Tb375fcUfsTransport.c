/*
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 * TB375FC / MT6897 strictly read-only UFS host transport (ufshc @ 0x112b0000).
 * Brings up UFSHCI after LK's ufs_aio_exit() (HCE=0) and performs only SCSI READ(10) (0x28).
 */
#include "Tb375fcUfsTransport.h"
#include <stddef.h>

extern uintptr_t ml_io_map(uintptr_t pa, uintptr_t size);
extern uint64_t kvtophys(uintptr_t va);

#define UFS_PA       0x112b0000u
#define UFSPHY_PA    0x112a0000u
#define UFSCFG_AO    0x112b8000u
#define UFSCFG_PDN   0x112bb000u
#define SPMI_PMIF_PA 0x1cc04000u
#define BLOCK  TB375FC_UFS_BLOCK
#define CAP    0x00u
#define VER    0x08u
#define AHIT   0x18u
#define IS     0x20u
#define IE     0x24u
#define HCS    0x30u
#define HCE    0x34u
#define BA     0x50u
#define BAU    0x54u
#define DBR    0x58u
#define CLR    0x5cu
#define RSR    0x60u
#define CNR    0x64u
#define TMDBR  0x78u
#define UIC_UE (1u << 2)
#define FATAL  ((1u << 11) | (1u << 12) | (1u << 16) | (1u << 17))
#define RESP   512u
#define PRDT   1024u
#define LUN    2u
#define ALIGN_DMA __attribute__((aligned(16384)))

static uint32_t list[256] ALIGN_DMA;
static uint8_t command[2048] ALIGN_DMA;
static uint8_t bounce[BLOCK * TB375FC_UFS_MAX_BLOCKS] ALIGN_DMA;
static uint32_t bounce_xfer_bytes = BLOCK;

static uint32_t io_lock, enabled, error_code = TB375FC_UFS_NOT_READY;
static uint32_t write_win_locked = 0, write_win_first = 0, write_win_blocks = 0;
static uint8_t  write_win_lun = 0xffu;
/* Hard floor: never allow writes below verified macOS RO roots + hashes (LBA 54,211,840). */
#define TB375FC_UFS_WRITE_HARD_FLOOR_LBA 54220000u

static int write_range_allowed(uint8_t lun, uint32_t lba, uint32_t count)
{
    if (!__atomic_load_n(&write_win_locked, __ATOMIC_ACQUIRE)) return 0;
    if (lun != write_win_lun || !count || count > TB375FC_UFS_MAX_BLOCKS) return 0;
    if (lba < TB375FC_UFS_WRITE_HARD_FLOOR_LBA) return 0;
    if (lba < write_win_first) return 0;
    uint32_t off = lba - write_win_first;
    if (off >= write_win_blocks || count > write_win_blocks - off) return 0;
    return 1;
}
static uint64_t frequency;
static uintptr_t hci, phy_base, ao_base, pdn_base, pmif_base;
static uint64_t list_pa, command_pa, bounce_pa;
static struct tb375fc_ufs_diag g_diag;
static uint32_t last_ocs, last_resp0, last_scsi_status, last_sense_key, last_asc, last_ascq;

static uint64_t ticks(void)
{
    uint64_t n;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(n));
    return n;
}

static void delay_us(uint32_t us)
{
    uint64_t wait_ticks = ((uint64_t)us * frequency + 999999ULL) / 1000000ULL;
    uint64_t target = ticks() + (wait_ticks ? wait_ticks : 1ULL);
    while (ticks() < target) {
        __asm__ volatile("yield" ::: "memory");
    }
}

static int try_lock(uint32_t *lock)
{
    return !__atomic_exchange_n(lock, 1u, __ATOMIC_ACQUIRE);
}

static void unlock(uint32_t *lock)
{
    __atomic_store_n(lock, 0u, __ATOMIC_RELEASE);
}

static void zero(void *p, size_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static void cache_dma(void *p, size_t n)
{
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    uintptr_t step = (uintptr_t)4u << ((ctr >> 16) & 15u);
    uintptr_t end = (uintptr_t)p + n;
    for (uintptr_t a = (uintptr_t)p & ~(step - 1u); a < end; a += step)
        __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");
}

static uint32_t rd(unsigned r)
{
    return *(volatile uint32_t *)(hci + r);
}

static void wr(unsigned r, uint32_t v)
{
    *(volatile uint32_t *)(hci + r) = v;
    __asm__ volatile("dsb sy" ::: "memory");
}

static uint32_t phy_rd(unsigned r)
{
    return *(volatile uint32_t *)(phy_base + r);
}

static void phy_wr(unsigned r, uint32_t v)
{
    *(volatile uint32_t *)(phy_base + r) = v;
    __asm__ volatile("dsb sy" ::: "memory");
}

static uint32_t pmif_rd(unsigned r)
{
    return *(volatile uint32_t *)(pmif_base + r);
}

static void pmif_wr(unsigned r, uint32_t v)
{
    *(volatile uint32_t *)(pmif_base + r) = v;
    __asm__ volatile("dsb sy" ::: "memory");
}

static int spmi_wait_idle(void)
{
    if (!pmif_base) return 0;
    uint64_t deadline = ticks() + frequency / 200u;
    do {
        uint32_t sta = pmif_rd(0x08a8u);
        uint32_t fsm = (sta >> 1) & 7u;
        if (fsm == 6u) {
            pmif_wr(0x08a4u, 1u);
            continue;
        }
        if (fsm == 0u) return 1;
    } while (ticks() < deadline);
    return 0;
}

static int spmi_read_byte(uint8_t sid, uint16_t addr)
{
    if (!spmi_wait_idle()) return -1;
    uint32_t cmd = (3u << 30) | ((uint32_t)(sid & 0xfu) << 24) | (uint32_t)addr;
    pmif_wr(0x0880u, cmd);
    uint64_t deadline = ticks() + frequency / 200u;
    do {
        uint32_t sta = pmif_rd(0x08a8u);
        uint32_t fsm = (sta >> 1) & 7u;
        if ((sta >> 18) & 1u) return -2;
        if (fsm == 6u) {
            uint32_t val = pmif_rd(0x0894u) & 0xffu;
            pmif_wr(0x08a4u, 1u);
            return (int)val;
        }
    } while (ticks() < deadline);
    return -1;
}

static int spmi_write_byte(uint8_t sid, uint16_t addr, uint8_t val)
{
    if (!spmi_wait_idle()) return -1;
    pmif_wr(0x0884u, (uint32_t)val);
    uint32_t cmd = (3u << 30) | (1u << 29) | ((uint32_t)(sid & 0xfu) << 24) | (uint32_t)addr;
    pmif_wr(0x0880u, cmd);
    return spmi_wait_idle() ? 0 : -1;
}

static void ensure_ufs_pmic_rails(int record_initial)
{
    if (!pmif_base) {
        if (record_initial) {
            g_diag.vemc_0 = -99;
            g_diag.vufs12_0 = -99;
            g_diag.vufs18_0 = -99;
        }
        g_diag.vemc_1 = -99;
        return;
    }
    uint32_t init0 = pmif_rd(0x0000u);
    uint32_t inf_en = pmif_rd(0x0024u);
    g_diag.pmif_init = ((init0 & 0xffffu) << 16) | (inf_en & 0xffffu);
    if ((init0 & 0x8000u) == 0u || (inf_en & 0x40u) == 0u) {
        if (record_initial) {
            g_diag.vemc_0 = -98;
            g_diag.vufs12_0 = -98;
            g_diag.vufs18_0 = -98;
        }
        g_diag.vemc_1 = -98;
        return;
    }

    int32_t v_vemc = spmi_read_byte(4u, 0x1bf3u);
    int32_t v_12 = spmi_read_byte(4u, 0x1c01u);
    int32_t v_18 = spmi_read_byte(4u, 0x1c0fu);
    if (record_initial) {
        g_diag.vemc_0 = v_vemc;
        g_diag.vufs12_0 = v_12;
        g_diag.vufs18_0 = v_18;
    }

    static const uint16_t rail_addrs[3] = { 0x1bf3u, 0x1c01u, 0x1c0fu };
    int32_t vals[3] = { v_vemc, v_12, v_18 };
    for (unsigned i = 0; i < 3u; i++) {
        int32_t v = vals[i];
        if (v >= 0 && (((uint32_t)v & 1u) == 0u || ((uint32_t)v & 2u) != 0u)) {
            uint8_t next_v = (uint8_t)(((uint32_t)v | 1u) & ~2u);
            if (spmi_write_byte(4u, rail_addrs[i], next_v) == 0) {
                delay_us(2000u);
            }
        }
    }
    g_diag.vemc_1 = spmi_read_byte(4u, 0x1bf3u);
}

static void ufs_phy_set_active(void)
{
    if (!phy_base) return;
    uint32_t v = phy_rd(0x008cu);
    phy_wr(0x008cu, (v | (1u << 11)) & ~((1u << 10) | (1u << 9) | (1u << 8)));

    v = phy_rd(0xb044u);
    phy_wr(0xb044u, (v | (1u << 18)) & ~((1u << 20) | (1u << 19) | (1u << 17)));

    v = phy_rd(0xa0acu);
    phy_wr(0xa0acu, (v | (1u << 1)) & ~(1u << 0));

    int wake_lane1 = ((g_diag.phy_b144_0 & (1u << 17)) != 0u ||
                      (g_diag.phy_a19c_0 & (1u << 18)) != 0u ||
                      (phy_rd(0xb144u) & (1u << 17)) != 0u ||
                      (phy_rd(0xa19cu) & (1u << 18)) != 0u);
    if (wake_lane1) {
        v = phy_rd(0xb144u);
        phy_wr(0xb144u, (v | (1u << 18)) & ~((1u << 20) | (1u << 19) | (1u << 17)));

        v = phy_rd(0xa1acu);
        phy_wr(0xa1acu, (v | (1u << 1)) & ~(1u << 0));
    }

    delay_us(10u);

    v = phy_rd(0xa09cu);
    phy_wr(0xa09cu, v & ~(1u << 18));
    if (wake_lane1) {
        v = phy_rd(0xa19cu);
        phy_wr(0xa19cu, v & ~(1u << 18));
    }

    g_diag.phy_8c_1 = phy_rd(0x008cu);
    g_diag.phy_9c_1 = phy_rd(0xa09cu);
    g_diag.phy_ac_1 = phy_rd(0xa0acu);
    g_diag.phy_b044_1 = phy_rd(0xb044u);
}

static uint64_t mtk_ufs_smc(uint64_t cmd, uint64_t v1, uint64_t v2)
{
    register uint64_t x0 __asm__("x0") = 0xC2000276ULL;
    register uint64_t x1 __asm__("x1") = cmd;
    register uint64_t x2 __asm__("x2") = v1;
    register uint64_t x3 __asm__("x3") = v2;
    register uint64_t x4 __asm__("x4") = 0;
    register uint64_t x5 __asm__("x5") = 0;
    register uint64_t x6 __asm__("x6") = 0;
    register uint64_t x7 __asm__("x7") = 0;
    __asm__ volatile(
        "smc #0"
        : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
        : "r"(x4), "r"(x5), "r"(x6), "r"(x7)
        : "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
          "x16", "x17", "cc", "memory"
    );
    return x0;
}

static int wait_bits(unsigned r, uint32_t mask, uint32_t want, uint64_t deadline)
{
    do {
        if ((rd(r) & mask) == want) return 1;
    } while (ticks() < deadline);
    return 0;
}

static void absorb_uic_error(void)
{
    if (rd(IS) & UIC_UE) {
        for (unsigned i = 0; i < 5; i++) (void)rd(0x38u + 4u * i);
        wr(IS, UIC_UE);
    }
}

static int dma_extent(void *p, size_t n, uint64_t *pa)
{
    *pa = kvtophys((uintptr_t)p);
    return *pa && !(*pa & (BLOCK - 1u)) &&
        kvtophys((uintptr_t)p + n - 1u) == *pa + n - 1u &&
        ((rd(CAP) & (1u << 24)) || !((*pa + n - 1u) >> 32));
}

static int fail(uint32_t why)
{
    __atomic_store_n(&error_code, why, __ATOMIC_RELAXED);
    return -(int)why;
}

static int dme(unsigned op, unsigned attr, uint32_t value, uint32_t *out)
{
    uint64_t limit = ticks() + frequency / 10u;
    while (!(rd(HCS) & 8u)) if (ticks() > limit) return -1;
    wr(IS, 1u << 10);
    wr(0x94u, attr << 16);
    wr(0x98u, 0);
    wr(0x9cu, value);
    wr(0x90u, op);
    limit = ticks() + frequency / 10u;
    while (!(rd(IS) & (1u << 10))) if (ticks() > limit) return -2;
    int result = (int)(rd(0x98u) & 255u);
    if (out) *out = rd(0x9cu);
    wr(IS, 1u << 10);
    return result;
}

static int submit_utrd(uint32_t dd, uint32_t prdt_entries, uint64_t timeout_ticks)
{
    if (dd == 1u) {
        /* Defense-in-depth: re-verify WRITE(10) opcode, LUN, LBA and block count directly from UPIU CDB */
        uint8_t cdb_op = command[16];
        uint8_t cdb_lun = command[2];
        uint32_t cdb_lba = get32(command + 18);
        uint32_t cdb_cnt = ((uint32_t)command[23] << 8) | (uint32_t)command[24];
        if (cdb_op != 0x2Au || !write_range_allowed(cdb_lun, cdb_lba, cdb_cnt)) {
            return -(int)TB375FC_UFS_POLICY;
        }
    }
    if (rd(HCE) != 1u || (rd(HCS) & 3u) != 3u || (rd(RSR) & 1u) != 1u || (rd(DBR) & 1u) != 0u) {
        return -(int)TB375FC_UFS_STATE;
    }
    absorb_uic_error();
    uint32_t cur_is = rd(IS);
    if (cur_is & ~FATAL) wr(IS, cur_is & ~FATAL);

    list[0] = (1u << 28) | ((dd & 3u) << 25) | (1u << 24);
    list[1] = 0u;
    list[2] = 0x0fu;
    list[3] = 0u;
    list[4] = (uint32_t)command_pa;
    list[5] = (uint32_t)(command_pa >> 32);
    list[6] = ((RESP / 4u) << 16) | (RESP / 4u);
    list[7] = ((PRDT / 4u) << 16) | prdt_entries;

    cache_dma(list, sizeof(list));
    cache_dma(command, sizeof(command));
    if (prdt_entries) cache_dma(bounce, bounce_xfer_bytes);

    wr(DBR, 1u);
    uint64_t deadline = ticks() + timeout_ticks;
    if (!wait_bits(DBR, 1u, 0u, deadline)) {
        wr(CLR, ~1u);
        (void)wait_bits(DBR, 1u, 0u, ticks() + frequency / 20u);
        return -1;
    }

    if (rd(CNR) & 1u) wr(CNR, 1u);
    if (rd(IS) & 1u) wr(IS, 1u);

    cache_dma(list, sizeof(list));
    cache_dma(command, sizeof(command));
    if (prdt_entries) cache_dma(bounce, bounce_xfer_bytes);

    const uint8_t *q = command + RESP;
    last_ocs = list[2] & 0xffu;
    last_resp0 = get32(q);
    last_scsi_status = q[7];
    if ((q[34] & 0x70u) == 0x70u) {
        last_sense_key = q[36] & 0x0fu;
        last_asc = q[46];
        last_ascq = q[47];
    } else {
        last_sense_key = q[34] & 0x0fu;
        last_asc = q[44];
        last_ascq = q[45];
    }
    return (int)last_ocs;
}

void tb375fc_ufs_get_last_cmd_diag(uint32_t out[6])
{
    if (!out) return;
    out[0] = last_ocs;
    out[1] = last_resp0;
    out[2] = last_scsi_status;
    out[3] = last_sense_key;
    out[4] = last_asc;
    out[5] = last_ascq;
}

static int try_hce_and_link_startup(unsigned attempts, uint32_t *out_uarg2, uint32_t *out_tries)
{
    wr(0x0140u, rd(0x0140u) | 0x80u); /* REG_UFS_XOUFS_CTRL */
    wr(0x0144u, 1u);                  /* REG_UFS_REFCLK_CTRL: REFCLK_REQUEST */
    uint64_t ref_limit = ticks() + (frequency * 3ULL) / 1000ULL;
    do {
        uint32_t rc = rd(0x0144u);
        if (((rc & 2u) >> 1) == (rc & 1u)) break;
    } while (ticks() < ref_limit);
    delay_us(100u);
    g_diag.refclk_144_1 = rd(0x0144u);

    if (rd(HCE) & 1u) {
        wr(HCE, 0u);
        (void)wait_bits(HCE, 1u, 0u, ticks() + frequency / 20u);
        delay_us(200u);
    }

    wr(IE, 0u);
    wr(AHIT, 0u);
    wr(0x0140u, rd(0x0140u) | 0x80u);
    wr(HCE, 1u);
    int hce_ok = wait_bits(HCE, 1u, 1u, ticks() + frequency / 5u);
    delay_us(1000u);
    g_diag.hce_after = rd(HCE);
    g_diag.hcs_after_hce = rd(HCS);
    if (!hce_ok) return -9;

    wr(IE, 0u);
    wr(AHIT, 0u);
    wr(IS, rd(IS));
    (void)wait_bits(HCS, 8u, 8u, ticks() + frequency / 10u);

    g_diag.dme_lpm = dme(2u, 0xd0a8u, 0u, 0);
    g_diag.dme_lcc = dme(2u, 0x155eu, 0u, 0);
    uint32_t spc = 0;
    int spc_rc = dme(1u, 0xd0a6u, 0u, &spc);
    if (spc_rc == 0) spc_rc = dme(2u, 0xd0a6u, spc & ~(1u << 6), 0);
    g_diag.dme_spc = spc_rc;

    uint32_t last_uarg2 = 0x100u;
    uint32_t tries_done = 0u;
    for (unsigned attempt = 0; attempt < attempts; attempt++) {
        tries_done = attempt + 1u;
        (void)wait_bits(HCS, 8u, 8u, ticks() + frequency / 10u);
        wr(IS, rd(IS));
        wr(0x94u, 0u);
        wr(0x98u, 0u);
        wr(0x9cu, 0u);
        wr(0x90u, 0x16u); /* UIC_CMD_DME_LINK_STARTUP */
        int uccs_ok = wait_bits(IS, 1u << 10, 1u << 10, ticks() + frequency / 2u);
        last_uarg2 = uccs_ok ? (rd(0x98u) & 0xffu) : 0x100u;
        wr(IS, 1u << 10);
        (void)wait_bits(HCS, 3u, 3u, ticks() + frequency / 10u);
        absorb_uic_error();
        wr(IS, rd(IS) & ((1u << 7) | (1u << 8) | (1u << 10) | UIC_UE));
        if (last_uarg2 == 0u && (rd(HCS) & 3u) == 3u) {
            if (out_uarg2) *out_uarg2 = last_uarg2;
            if (out_tries) *out_tries = tries_done;
            return 0;
        }
        delay_us(10000u);
    }
    if (out_uarg2) *out_uarg2 = last_uarg2;
    if (out_tries) *out_tries = tries_done;
    return -10;
}

int tb375fc_ufs_init(struct tb375fc_ufs_diag *diag)
{
    if (__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) {
        if (diag) *diag = g_diag;
        return 0;
    }

    zero(&g_diag, sizeof(g_diag));
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    if (!frequency) frequency = 13000000ULL;

    hci = ml_io_map(UFS_PA, 0x3000u);
    if (!hci) {
        if (diag) *diag = g_diag;
        return fail(TB375FC_UFS_NOT_READY);
    }

    g_diag.cap = rd(CAP);
    g_diag.ver = rd(VER);
    g_diag.hcs_before = rd(HCS);
    g_diag.hce_before = rd(HCE);
    g_diag.ba_before = rd(BA);
    g_diag.bau_before = rd(BAU);
    g_diag.dbr_before = rd(DBR);
    g_diag.rsr_before = rd(RSR);

    phy_base = ml_io_map(UFSPHY_PA, 0x10000u);
    ao_base = ml_io_map(UFSCFG_AO, 0x1000u);
    pdn_base = ml_io_map(UFSCFG_PDN, 0x1000u);
    pmif_base = ml_io_map(SPMI_PMIF_PA, 0x1000u);
    if (ao_base) {
        g_diag.ao_cg = *(volatile uint32_t *)(ao_base + 0x00u);
        g_diag.ao_rst = *(volatile uint32_t *)(ao_base + 0x44u);
    }
    if (pdn_base) {
        g_diag.pdn_cg = *(volatile uint32_t *)(pdn_base + 0x00u);
        g_diag.pdn_rst = *(volatile uint32_t *)(pdn_base + 0x44u);
    }
    g_diag.xoufs_140_0 = rd(0x0140u);
    g_diag.refclk_144_0 = rd(0x0144u);
    g_diag.opt_160_0 = rd(0x0160u);
    g_diag.extreg_2100 = rd(0x2100u);
    g_diag.mphyctrl_2200 = rd(0x2200u);
    g_diag.ipver_2240 = rd(0x2240u);
    if (phy_base) {
        g_diag.phy_8c_0 = phy_rd(0x008cu);
        g_diag.phy_9c_0 = phy_rd(0xa09cu);
        g_diag.phy_ac_0 = phy_rd(0xa0acu);
        g_diag.phy_b044_0 = phy_rd(0xb044u);
        g_diag.phy_a19c_0 = phy_rd(0xa19cu);
        g_diag.phy_b144_0 = phy_rd(0xb144u);
    }

    if (!dma_extent(list, sizeof(list), &list_pa) ||
        !dma_extent(command, sizeof(command), &command_pa) ||
        !dma_extent(bounce, sizeof(bounce), &bounce_pa)) {
        if (diag) *diag = g_diag;
        return fail(TB375FC_UFS_DMA);
    }

    if ((rd(HCE) & 1u) == 0u || (rd(HCS) & 3u) != 3u) {
        ensure_ufs_pmic_rails(1);

        /* Stage A: MMIO-only wake */
        ufs_phy_set_active();
        g_diag.stage_used = 1u;
        int rc_a = try_hce_and_link_startup(2u, &g_diag.stage_a_uarg2, &g_diag.link_tries);
        g_diag.stage_a_hcs = rd(HCS);
        g_diag.link_uarg2 = g_diag.stage_a_uarg2;

        if (rc_a != 0) {
            /* Stage B: Full Linux ufs-mediatek.c + MTK_SIP_UFS_CONTROL cold bring-up */
            g_diag.stage_used = 2u;
            g_diag.smc_sram = (int32_t)mtk_ufs_smc(1u << 5, 1u, 0u);
            g_diag.smc_devpwr = (int32_t)mtk_ufs_smc(1u << 7, 1u, 0x0400u);
            g_diag.smc_va09 = (int32_t)mtk_ufs_smc(1u << 0, 1u, 0u);
            g_diag.smc_mtcmos0 = (int32_t)mtk_ufs_smc(1u << 9, 0u, 0u);
            g_diag.smc_mtcmos1 = (int32_t)mtk_ufs_smc(1u << 9, 1u, 0u);
            ensure_ufs_pmic_rails(0);
            ufs_phy_set_active();
            g_diag.smc_refpre = (int32_t)mtk_ufs_smc(1u << 3, 1u, 0u);
            wr(0x0144u, 1u);
            {
                uint64_t ref_limit = ticks() + (frequency * 3ULL) / 1000ULL;
                do {
                    uint32_t rc = rd(0x0144u);
                    if (((rc & 2u) >> 1) == (rc & 1u)) break;
                } while (ticks() < ref_limit);
                delay_us(100u);
            }
            g_diag.smc_refpost = (int32_t)mtk_ufs_smc(1u << 3, 1u, 1u);
            g_diag.smc_rst0 = (int32_t)mtk_ufs_smc(1u << 1, 0u, 0u);
            wr(HCE, 0u);
            (void)wait_bits(HCE, 1u, 0u, ticks() + frequency / 20u);
            delay_us(20u);
            g_diag.smc_rst1 = (int32_t)mtk_ufs_smc(1u << 1, 1u, 0u);
            delay_us(15000u);
            if (pdn_base) {
                *(volatile uint32_t *)(pdn_base + 0x48u) = 0x7u;
                __asm__ volatile("dsb sy" ::: "memory");
                delay_us(110u);
                *(volatile uint32_t *)(pdn_base + 0x4cu) = 0x7u;
                __asm__ volatile("dsb sy" ::: "memory");
                delay_us(50u);
            }
            ufs_phy_set_active();
            int rc_b = try_hce_and_link_startup(3u, &g_diag.link_uarg2, &g_diag.link_tries);
            if (rc_b == -9) {
                g_diag.hcs_after_link = rd(HCS);
                if (diag) *diag = g_diag;
                return fail(TB375FC_UFS_HCE);
            }
        }
        g_diag.hcs_after_link = rd(HCS);
        if ((g_diag.hcs_after_link & 3u) != 3u) {
            if (diag) *diag = g_diag;
            return fail(TB375FC_UFS_LINK);
        }
    } else {
        g_diag.hce_after = rd(HCE);
        g_diag.hcs_after_hce = rd(HCS);
        g_diag.hcs_after_link = rd(HCS);
    }

    if (rd(RSR) & 1u) {
        (void)wait_bits(DBR, 0xffffffffu, 0u, ticks() + frequency / 10u);
        wr(RSR, 0u);
        (void)wait_bits(RSR, 1u, 0u, ticks() + frequency / 20u);
    }
    zero(list, sizeof(list));
    cache_dma(list, sizeof(list));
    wr(IE, 0u);
    wr(AHIT, 0u);
    wr(IS, rd(IS));
    wr(BA, (uint32_t)list_pa);
    wr(BAU, (uint32_t)(list_pa >> 32));
    wr(RSR, 1u);
    int rsr_ok = wait_bits(RSR, 1u, 1u, ticks() + frequency / 10u);
    g_diag.rsr_after = rd(RSR);
    if (!rsr_ok) {
        if (diag) *diag = g_diag;
        return fail(TB375FC_UFS_RSR);
    }

    /* 1. NOP OUT -> NOP IN */
    int nop_ok = 0;
    for (unsigned t = 0; t < 20u; t++) {
        g_diag.nop_tries = t + 1u;
        zero(command, sizeof(command));
        command[0] = 0x00u;
        command[3] = 0x01u;
        int rc = submit_utrd(0u, 0u, frequency / 5u);
        g_diag.nop_rc = rc;
        if (rc == 0 && (command[RESP + 0] & 0x3fu) == 0x20u) {
            nop_ok = 1;
            break;
        }
        delay_us(5000u);
    }
    if (!nop_ok) {
        if (diag) *diag = g_diag;
        return fail(TB375FC_UFS_NOP);
    }

    /* 2. SET_FLAG fDeviceInit (0x01) + poll READ_FLAG until cleared */
    zero(command, sizeof(command));
    command[0] = 0x16u;
    command[3] = 0x02u;
    command[5] = 0x81u;
    command[12] = 0x06u;
    command[13] = 0x01u;
    int set_rc = submit_utrd(0u, 0u, frequency / 2u);
    g_diag.fdevinit_rc = set_rc;
    if (set_rc != 0 || (command[RESP + 0] & 0x3fu) != 0x36u || command[RESP + 6] != 0x00u) {
        if (set_rc == 0) g_diag.fdevinit_rc = -100 - (int)command[RESP + 6];
        if (diag) *diag = g_diag;
        return fail(TB375FC_UFS_FDEVINIT);
    }

    int fdev_done = 0;
    for (unsigned p = 0; p < 200u; p++) {
        g_diag.fdevinit_polls = p + 1u;
        zero(command, sizeof(command));
        command[0] = 0x16u;
        command[3] = 0x02u;
        command[5] = 0x01u;
        command[12] = 0x05u;
        command[13] = 0x01u;
        int rc = submit_utrd(0u, 0u, frequency / 2u);
        g_diag.fdevinit_rc = rc;
        if (rc == 0 && (command[RESP + 0] & 0x3fu) == 0x36u &&
            command[RESP + 6] == 0x00u && (command[RESP + 23] & 1u) == 0u) {
            fdev_done = 1;
            break;
        }
        delay_us(5000u);
    }
    if (!fdev_done) {
        if (diag) *diag = g_diag;
        return fail(TB375FC_UFS_FDEVINIT);
    }

    /* 3. TEST UNIT READY on LUN2 to absorb initial CHECK CONDITION / UNIT ATTENTION */
    for (unsigned t = 0; t < 6u; t++) {
        g_diag.tur_tries = t + 1u;
        zero(command, sizeof(command));
        command[0] = 0x01u;
        command[1] = 0x00u;
        command[2] = LUN;
        command[3] = 0x03u;
        command[16] = 0x00u;
        int rc = submit_utrd(0u, 0u, frequency / 2u);
        g_diag.tur_rc = (rc == 0) ? (int32_t)command[RESP + 7] : rc;
        if (rc == 0 && (command[RESP + 0] & 0x3fu) == 0x21u && command[RESP + 7] == 0x00u) {
            break;
        }
        delay_us(5000u);
    }

    __atomic_store_n(&error_code, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&enabled, 1u, __ATOMIC_RELEASE);
    if (diag) *diag = g_diag;
    return 0;
}

static int transfer_read_blocks(uint8_t lun, uint32_t lba, uint32_t count, uint64_t deadline)
{
    if (!count || count > TB375FC_UFS_MAX_BLOCKS) return fail(TB375FC_UFS_POLICY);
    uint32_t nbytes = count * BLOCK;
    bounce_xfer_bytes = nbytes;
    for (unsigned attempt = 0; attempt < 3u; attempt++) {
        if (ticks() >= deadline) return fail(TB375FC_UFS_TIMEOUT);
        zero(list, sizeof(list));
        zero(command, sizeof(command));
        command[0] = 0x01u;
        command[1] = 0x40u;
        command[2] = lun;
        command[3] = 0x04u;
        put32(command + 12, nbytes);
        command[16] = 0x28u; /* SCSI READ(10) — strictly read-only */
        put32(command + 18, lba);
        command[23] = (uint8_t)(count >> 8);
        command[24] = (uint8_t)(count & 0xffu);
        uint32_t *prdt = (uint32_t *)(command + PRDT);
        prdt[0] = (uint32_t)bounce_pa;
        prdt[1] = (uint32_t)(bounce_pa >> 32);
        prdt[3] = nbytes - 1u;

        uint64_t now = ticks();
        uint64_t rem = (deadline > now) ? (deadline - now) : 1ULL;
        int rc = submit_utrd(2u, 1u, rem);
        if (rc < 0) return fail(rc == -1 ? TB375FC_UFS_TIMEOUT : TB375FC_UFS_STATE);
        const uint8_t *q = command + RESP;
        if (rc == 0 && (q[0] & 0x3fu) == 0x21u && q[2] == lun && q[3] == 0x04u &&
            q[6] == 0u && q[7] == 0u && (q[1] & 0x60u) == 0u && get32(q + 12) == 0u) {
            return 0;
        }
        /* Retry transient UNIT ATTENTION (0x02) or BUSY (0x08) */
        if (rc == 0 && (q[7] == 0x02u || q[7] == 0x08u)) {
            delay_us(2000u);
            continue;
        }
        return fail(TB375FC_UFS_RESPONSE);
    }
    return fail(TB375FC_UFS_RESPONSE);
}

static int change_pwr_mode(uint32_t mode)
{
    wr(IS, 1u << 4);
    int e = dme(2, 0x1571, mode, 0);
    if (e) return 100 + e;
    uint64_t limit = ticks() + frequency / 2u;
    while (!(rd(IS) & (1u << 4))) if (ticks() > limit) return -3;
    wr(IS, 1u << 4);
    uint32_t upmcrs = (rd(HCS) >> 8) & 7u;
    return upmcrs == 1 ? 0 : 200 + (int)upmcrs;
}

static int power_mode_hs(uint32_t lanes_tx, uint32_t lanes_rx, uint32_t gear)
{
    static const unsigned sync[] = { 0x1552, 0x1554, 0x1556, 0x15d0, 0x15d6 };
    static const unsigned user[] = { 0x15b0, 0x15b1, 0x15b2, 0x15b3, 0x15b4, 0x15b5, 0xd041, 0xd042, 0xd043 };
    static const uint32_t userv[] = { 8191, 65535, 32767, 8191, 65535, 32767, 8191, 65535, 32767 };
    int e;
#define S(a, v) do { if ((e = dme(2, (a), (v), 0))) return ((a) << 8) | (e & 255); } while (0)
    for (unsigned i = 0; i < 5; i++) {
        uint32_t v = 0;
        if (!dme(1, sync[i], 0, &v) && v < 0x48) S(sync[i], 0x48);
    }
    S(0x1569, 1); S(0x1568, 1); S(0x1584, 1); S(0x1583, 1);
    S(0x1560, lanes_tx); S(0x1580, lanes_rx); S(0x156a, 2); S(0x15d4, 3);
    for (unsigned i = 0; i < 9; i++) S(user[i], userv[i]);
    if ((e = change_pwr_mode(0x44))) return 0x10000 | (e & 0xffff);
    S(0x15d4, gear >= 4 ? 1 : 3);
    S(0x1583, gear); S(0x1580, lanes_rx); S(0x1584, 1);
    S(0x1568, gear); S(0x1560, lanes_tx); S(0x1569, 1); S(0x156a, 2);
    for (unsigned i = 0; i < 9; i++) S(user[i], userv[i]);
    if ((e = change_pwr_mode(0x11))) return 0x20000 | (e & 0xffff);
#undef S
    return 0;
}

int tb375fc_ufs_link_hs(int32_t out[6])
{
    static const unsigned attrs[] = { 0x1571, 0x1568, 0x1583, 0x1560, 0x1580, 0x156a, 0x1561, 0x1581, 0x1587 };
    uint32_t v[9] = {0}, peer = 0;
    if (!__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return -(int)error_code;
    uint64_t limit = ticks() + frequency;
    while (!try_lock(&io_lock)) if (ticks() > limit) return -(int)TB375FC_UFS_BUSY;
    int r = 0;
    for (unsigned i = 0; i < 9 && !r; i++) if (dme(1, attrs[i], 0, &v[i])) r = -10 - (int)i;
    if (!r && dme(3, 0x1587, 0, &peer)) r = -20;
    if (!r && v[0] == 0x55 && v[6] && v[7] && v[6] <= 2 && v[7] <= 2 && v[8] && peer) {
        uint32_t gear = v[8] < peer ? v[8] : peer;
        if (gear > 5) gear = 5;
        r = power_mode_hs(v[6], v[7], gear);
    } else if (!r && v[0] != 0x55) {
        r = 1;
    }
    for (unsigned i = 0; i < 6; i++) (void)dme(1, attrs[i], 0, &v[i]);
    wr(IS, rd(IS) & ((1u << 10) | (1u << 4) | UIC_UE));
    absorb_uic_error();
    unlock(&io_lock);
    out[0] = r; out[1] = (int32_t)v[0]; out[2] = (int32_t)v[1]; out[3] = (int32_t)v[2];
    out[4] = (int32_t)v[3]; out[5] = (int32_t)v[4];
    return r < 0 ? r : 0;
}

int tb375fc_ufs_read_lun_blocks(uint8_t lun, uint32_t lba, uint32_t count, void *out)
{
    if (!out || !count || count > TB375FC_UFS_MAX_BLOCKS) return -(int)TB375FC_UFS_POLICY;
    if (!__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return -(int)error_code;
    if (!try_lock(&io_lock)) return -(int)TB375FC_UFS_BUSY;
    int result = transfer_read_blocks(lun, lba, count, ticks() + frequency);
    if (!result) {
        uint8_t *p = (uint8_t *)out;
        uint32_t nbytes = count * BLOCK;
        for (uint32_t i = 0; i < nbytes; i++) p[i] = bounce[i];
    }
    unlock(&io_lock);
    return result;
}

int tb375fc_ufs_read_lun_block(uint8_t lun, uint32_t lba, void *out)
{
    return tb375fc_ufs_read_lun_blocks(lun, lba, 1u, out);
}

int tb375fc_ufs_read_blocks(uint32_t lba, uint32_t count, void *out)
{
    return tb375fc_ufs_read_lun_blocks(LUN, lba, count, out);
}

int tb375fc_ufs_read_block(uint32_t lba, void *out)
{
    return tb375fc_ufs_read_lun_blocks(LUN, lba, 1u, out);
}

int tb375fc_ufs_set_write_window(uint8_t lun, uint32_t first_lba, uint32_t blocks)
{
    if (lun != LUN || !blocks || first_lba < TB375FC_UFS_WRITE_HARD_FLOOR_LBA) {
        return -(int)TB375FC_UFS_POLICY;
    }
    if (first_lba > 0xffffffffu - blocks) {
        return -(int)TB375FC_UFS_POLICY;
    }
    if (__atomic_load_n(&write_win_locked, __ATOMIC_ACQUIRE)) {
        return -(int)TB375FC_UFS_POLICY; /* Write-once: never re-aim after boot lock */
    }
    write_win_lun = lun;
    write_win_first = first_lba;
    write_win_blocks = blocks;
    __atomic_store_n(&write_win_locked, 1u, __ATOMIC_RELEASE);
    return 0;
}

static int transfer_write_blocks(uint8_t lun, uint32_t lba, uint32_t count, int fua, uint64_t deadline)
{
    if (!write_range_allowed(lun, lba, count)) return fail(TB375FC_UFS_POLICY);
    uint32_t nbytes = count * BLOCK;
    bounce_xfer_bytes = nbytes;
    for (unsigned attempt = 0; attempt < 3u; attempt++) {
        if (ticks() >= deadline) return fail(TB375FC_UFS_TIMEOUT);
        zero(list, sizeof(list));
        zero(command, sizeof(command));
        command[0] = 0x01u; /* UPIU COMMAND */
        command[1] = 0x20u; /* W (Host-to-Target write) */
        command[2] = lun;
        command[3] = 0x05u; /* task tag */
        put32(command + 12, nbytes);
        command[16] = 0x2Au; /* SCSI WRITE(10) */
        command[17] = fua ? 0x08u : 0x00u; /* FUA bit */
        put32(command + 18, lba);
        command[23] = (uint8_t)(count >> 8);
        command[24] = (uint8_t)(count & 0xffu);
        uint32_t *prdt = (uint32_t *)(command + PRDT);
        prdt[0] = (uint32_t)bounce_pa;
        prdt[1] = (uint32_t)(bounce_pa >> 32);
        prdt[3] = nbytes - 1u;

        uint64_t now = ticks();
        uint64_t rem = (deadline > now) ? (deadline - now) : 1ULL;
        int rc = submit_utrd(1u, 1u, rem);
        if (rc < 0) return fail(rc == -1 ? TB375FC_UFS_TIMEOUT : (rc == -(int)TB375FC_UFS_POLICY ? TB375FC_UFS_POLICY : TB375FC_UFS_STATE));
        const uint8_t *q = command + RESP;
        if (rc == 0 && (q[0] & 0x3fu) == 0x21u && q[2] == lun && q[3] == 0x05u &&
            q[6] == 0u && q[7] == 0u && (q[1] & 0x60u) == 0u && get32(q + 12) == 0u) {
            return 0;
        }
        if (rc == 0 && (q[7] == 0x02u || q[7] == 0x08u)) {
            delay_us(2000u);
            continue;
        }
        return fail(TB375FC_UFS_RESPONSE);
    }
    return fail(TB375FC_UFS_RESPONSE);
}

static int transfer_sync_cache(uint8_t lun, uint64_t deadline)
{
    if (!__atomic_load_n(&write_win_locked, __ATOMIC_ACQUIRE) || lun != write_win_lun) {
        return fail(TB375FC_UFS_POLICY);
    }
    for (unsigned attempt = 0; attempt < 3u; attempt++) {
        if (ticks() >= deadline) return fail(TB375FC_UFS_TIMEOUT);
        zero(list, sizeof(list));
        zero(command, sizeof(command));
        command[0] = 0x01u; /* UPIU COMMAND */
        command[1] = 0x00u; /* no data transfer */
        command[2] = lun;
        command[3] = 0x06u;
        command[16] = 0x35u; /* SCSI SYNCHRONIZE CACHE(10) */

        uint64_t now = ticks();
        uint64_t rem = (deadline > now) ? (deadline - now) : 1ULL;
        int rc = submit_utrd(0u, 0u, rem);
        if (rc < 0) return fail(rc == -1 ? TB375FC_UFS_TIMEOUT : TB375FC_UFS_STATE);
        const uint8_t *q = command + RESP;
        if (rc == 0 && (q[0] & 0x3fu) == 0x21u && q[2] == lun && q[3] == 0x06u &&
            q[6] == 0u && q[7] == 0u) {
            return 0;
        }
        if (rc == 0 && (q[7] == 0x02u || q[7] == 0x08u)) {
            delay_us(2000u);
            continue;
        }
        return fail(TB375FC_UFS_RESPONSE);
    }
    return fail(TB375FC_UFS_RESPONSE);
}

int tb375fc_ufs_write_blocks(uint32_t lba, uint32_t count, const void *in, int fua)
{
    if (!in || !write_range_allowed(LUN, lba, count)) return -(int)TB375FC_UFS_POLICY;
    if (!__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return -(int)error_code;
    if (!try_lock(&io_lock)) return -(int)TB375FC_UFS_BUSY;
    const uint8_t *p = (const uint8_t *)in;
    uint32_t nbytes = count * BLOCK;
    for (uint32_t i = 0; i < nbytes; i++) bounce[i] = p[i];
    int result = transfer_write_blocks(LUN, lba, count, fua, ticks() + frequency * 2ULL);
    unlock(&io_lock);
    return result;
}

int tb375fc_ufs_sync_cache(void)
{
    if (!__atomic_load_n(&enabled, __ATOMIC_ACQUIRE)) return -(int)error_code;
    if (!__atomic_load_n(&write_win_locked, __ATOMIC_ACQUIRE)) return -(int)TB375FC_UFS_POLICY;
    if (!try_lock(&io_lock)) return -(int)TB375FC_UFS_BUSY;
    int result = transfer_sync_cache(LUN, ticks() + frequency * 2ULL);
    unlock(&io_lock);
    return result;
}
