/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/*
 * Tb375fcWifiEngine.h — Clean-Room MT6897 soc7_0 (Connac 2.x AXI WFDMA0)
 *                       Native Wi-Fi 6 STA Engine & en0 Data Plane Bridge
 *
 * Included by Tb375fcConnSys.cpp inside the anonymous namespace after
 * low-level MMIO/Stage1/Stage3 helpers are defined.
 */
#ifndef TB375FC_WIFI_ENGINE_H
#define TB375FC_WIFI_ENGINE_H

#include "Tb375fcWifiCrypto.h"

/* Bridge symbols exported by Tb375fcDummyEth.cpp */
extern "C" {
typedef int (*tb375fc_eth_tx_cb_t)(const uint8_t *frame, uint32_t len);
extern void tb375fc_eth_register_tx(tb375fc_eth_tx_cb_t cb);
extern int  tb375fc_eth_rx_deliver(const uint8_t *frame, uint32_t len);
extern void tb375fc_eth_set_link(bool up, uint64_t speed_bps);
extern void read_random(void *buffer, unsigned int numBytes);
}

namespace {

/* =========================================================================
 * WFDMA0 & DMASHDL Register Offsets (relative to PHYS_CONNSYS_BASE 0x18000000)
 * ========================================================================= */

static constexpr uint32_t OFF_DMASHDL_BASE        = 0x026000U;
static constexpr uint32_t OFF_WFDMA0_RST          = 0x024100U;
static constexpr uint32_t OFF_WFDMA0_INT_STA_R    = 0x024200U;
static constexpr uint32_t OFF_WFDMA0_INT_ENA_R    = 0x024204U;
static constexpr uint32_t OFF_WFDMA0_GLO_CFG_R    = 0x024208U;
static constexpr uint32_t OFF_WFDMA0_RST_DTX_PTR  = 0x02420CU;
static constexpr uint32_t OFF_WFDMA0_RST_DRX_PTR  = 0x024280U;
static constexpr uint32_t OFF_WFDMA0_INT_RX_PRI   = 0x024298U;

static constexpr uint32_t OFF_WFDMA0_TX0          = 0x024300U; /* Data0 */
static constexpr uint32_t OFF_WFDMA0_TX1          = 0x024310U; /* Data1 */
static constexpr uint32_t OFF_WFDMA0_TX2          = 0x024320U; /* Data2 / Prio */
static constexpr uint32_t OFF_WFDMA0_TX15         = 0x0243F0U; /* CMD + Mgmt */
static constexpr uint32_t OFF_WFDMA0_TX16         = 0x024400U; /* FWDL */
static constexpr uint32_t OFF_WFDMA0_RX0          = 0x024500U; /* Band0 Data RX */
static constexpr uint32_t OFF_WFDMA0_RX1          = 0x024510U; /* Band1 Data RX */
static constexpr uint32_t OFF_WFDMA0_RX2          = 0x024520U; /* Events + Mgmt RX */
static constexpr uint32_t OFF_WFDMA0_RX3          = 0x024530U; /* TX Free Done RX */

static constexpr uint32_t OFF_WFDMA0_TX0_EXT      = 0x024600U;
static constexpr uint32_t OFF_WFDMA0_TX1_EXT      = 0x024604U;
static constexpr uint32_t OFF_WFDMA0_TX2_EXT      = 0x024608U;
static constexpr uint32_t OFF_WFDMA0_TX8_EXT      = 0x024620U;
static constexpr uint32_t OFF_WFDMA0_TX9_EXT      = 0x024624U;
static constexpr uint32_t OFF_WFDMA0_TX10_EXT     = 0x024628U;
static constexpr uint32_t OFF_WFDMA0_TX14_EXT     = 0x024638U;
static constexpr uint32_t OFF_WFDMA0_TX15_EXT     = 0x02463CU;
static constexpr uint32_t OFF_WFDMA0_TX16_EXT     = 0x024640U;
static constexpr uint32_t OFF_WFDMA0_RX0_EXT      = 0x024680U; /* +4*r for RX0..7 */

static constexpr uint32_t OFF_BN0_LPCTL           = 0x060010U;
static constexpr uint32_t OFF_BN0_IRQ_STA         = 0x060014U;
static constexpr uint32_t OFF_BN0_IRQ_ENA         = 0x060018U;
static constexpr uint32_t OFF_CONN_ON_MISC_R      = 0x0600F0U;

/* =========================================================================
 * DMA Pool Layout inside PHYS_WIFI_DMA_POOL (0x8D900000, 20 MiB WC mapped)
 * ========================================================================= */

static constexpr uint32_t WF_TX0_SZ               = 256U;
static constexpr uint32_t WF_TX1_SZ               = 16U;
static constexpr uint32_t WF_TX2_SZ               = 16U;
static constexpr uint32_t WF_TX15_SZ              = 64U;
static constexpr uint32_t WF_TX16_SZ              = 16U;
static constexpr uint32_t WF_RX0_SZ               = 256U;
static constexpr uint32_t WF_RX1_SZ               = 64U;
static constexpr uint32_t WF_RX2_SZ               = 128U;
static constexpr uint32_t WF_RX3_SZ               = 128U;

static constexpr uint32_t DMA_OFF_TX0_RING        = 0x000000U; /* 4 KiB */
static constexpr uint32_t DMA_OFF_TX1_RING        = 0x001000U;
static constexpr uint32_t DMA_OFF_TX2_RING        = 0x002000U;
static constexpr uint32_t DMA_OFF_TX15_RING       = 0x003000U; /* 1 KiB */
static constexpr uint32_t DMA_OFF_TX16_RING       = 0x004000U;
static constexpr uint32_t DMA_OFF_RX0_RING        = 0x008000U; /* 4 KiB */
static constexpr uint32_t DMA_OFF_RX1_RING        = 0x009000U; /* 1 KiB */
static constexpr uint32_t DMA_OFF_RX2_RING        = 0x00A000U; /* 2 KiB */
static constexpr uint32_t DMA_OFF_RX3_RING        = 0x00B000U; /* 2 KiB */

static constexpr uint32_t DMA_OFF_TX15_BUFS       = 0x010000U; /* 64 * 2 KiB = 128 KiB */
static constexpr uint32_t WF_TX15_BUF_STRIDE      = 0x000800U;

static constexpr uint32_t DMA_OFF_TX0_BUFS        = 0x040000U; /* 256 * 2 KiB = 512 KiB */
static constexpr uint32_t WF_TX0_BUF_STRIDE       = 0x000800U;

static constexpr uint32_t DMA_OFF_RX0_BUFS        = 0x100000U; /* 256 * 4 KiB = 1 MiB */
static constexpr uint32_t DMA_OFF_RX1_BUFS        = 0x200000U; /* 64 * 4 KiB = 256 KiB */
static constexpr uint32_t DMA_OFF_RX2_BUFS        = 0x240000U; /* 128 * 4 KiB = 512 KiB */
static constexpr uint32_t DMA_OFF_RX3_BUFS        = 0x2C0000U; /* 128 * 4 KiB = 512 KiB */
static constexpr uint32_t WF_RX_BUF_STRIDE        = 0x001000U;
static constexpr uint32_t WF_RX_BUF_MAX_LEN       = 3840U;

static constexpr uint8_t  WF_OWN_MAC[6]           = { 0x02, 0x37, 0x5F, 0xC0, 0x00, 0x01 };
static constexpr uint8_t  WF_BSS_IDX              = 0U;
static constexpr uint8_t  WF_AP_STA_IDX           = 0U;
static constexpr uint8_t  WF_AP_WLAN_IDX          = 1U;
static constexpr uint8_t  WF_BMC_WLAN_IDX         = 30U; /* 0x1E */

/* WFDMA0 16-byte DMA descriptor (matches TXD_STRUCT / RXD_STRUCT) */
struct WfdmaDesc {
    uint32_t sdp0;
    uint32_t ctrl; /* [31]=DMADONE, [30]=LS0, [29:16]=SDL0, [15]=Burst, [14]=LS1, [13:0]=SDL1 */
    uint32_t sdp1;
    uint32_t info; /* ext high bits / rxinfo */
};

/* Connac 2.x 32-byte MAC TXD */
struct Connac2xTxd {
    uint32_t dw[8];
};

/* Connac 2.x 32-byte Cut-Through TXP Append (CONNAC_APPEND) */
struct Connac2xTxp {
    uint16_t msdu_id[4];
    uint32_t ptr0;
    uint16_t len0;
    uint16_t len1;
    uint32_t ptr1;
    uint32_t ptr2;
    uint16_t len2;
    uint16_t len3;
    uint32_t ptr3;
};
static_assert(sizeof(Connac2xTxd) == 32, "Connac2xTxd must be 32 bytes");
static_assert(sizeof(Connac2xTxp) == 32, "Connac2xTxp must be 32 bytes");

/* Firmware Command IDs (non-unified Connac2x gen4m) */
static constexpr uint8_t CID_INIT_FW_START        = 0x42U;
static constexpr uint8_t CID_BASIC_CONFIG         = 0x02U;
static constexpr uint8_t CID_SCAN_REQ_V2          = 0x03U;
static constexpr uint8_t CID_ADD_REMOVE_KEY       = 0x07U;
static constexpr uint8_t CID_SET_DOMAIN_INFO      = 0x0FU;
static constexpr uint8_t CID_BSS_ACTIVATE_CTRL    = 0x11U;
static constexpr uint8_t CID_SET_BSS_INFO         = 0x12U;
static constexpr uint8_t CID_UPDATE_STA_RECORD    = 0x13U;
static constexpr uint8_t CID_REMOVE_STA_RECORD    = 0x14U;
static constexpr uint8_t CID_INDICATE_PM_CONNECTED= 0x16U;
static constexpr uint8_t CID_INDICATE_PM_ABORT    = 0x17U;
static constexpr uint8_t CID_CH_PRIVILEGE         = 0x1CU;
static constexpr uint8_t CID_UPDATE_WMM_PARMS     = 0x1DU;
static constexpr uint8_t CID_GET_NIC_CAP_V2       = 0x8AU;

/* Firmware Event IDs */
static constexpr uint8_t EID_SCAN_DONE            = 0x0DU;
static constexpr uint8_t EID_TX_DONE              = 0x0FU;
static constexpr uint8_t EID_CH_PRIVILEGE         = 0x10U;
static constexpr uint8_t EID_BSS_BCN_TIMEOUT      = 0x13U;
static constexpr uint8_t EID_ADD_PKEY_DONE        = 0x24U;
static constexpr uint8_t EID_DEBUG_MSG            = 0x27U;
static constexpr uint8_t EID_NIC_CAP_V2           = 0xECU;

/* =========================================================================
 * Driver State
 * ========================================================================= */

struct WifiState {
    IOLock   *tx_lock;
    IOLock   *state_lock;
    bool      initialized;
    bool      fw_ready;
    bool      rx_thread_running;

    uint32_t  state; /* Tb375fcWifiState */
    uint8_t   cmd_seq;
    uint16_t  mgmt_seq;
    uint8_t   ch_token;

    /* Ring software cursors */
    uint32_t  tx0_cidx;
    uint32_t  tx15_cidx;
    uint32_t  rx0_idx;
    uint32_t  rx1_idx;
    uint32_t  rx2_idx;
    uint32_t  rx3_idx;

    /* TX0 256-slot cut-through MSDU token allocator (bit=1 means free) */
    uint64_t  tok_free[4];
    uint32_t  tok_alloc_cursor;
    uint32_t  free_token_count;

    /* Sync flags toggled by RX2 thread */
    volatile uint32_t evt_nic_cap_seen;
    uint32_t          nic_cap_len;
    uint32_t          adie_ver_cap;
    volatile uint32_t evt_scan_done_cnt;
    volatile uint32_t evt_ch_grant_cnt;
    volatile uint8_t  evt_ch_grant_tok;
    volatile uint8_t  evt_ch_grant_status;
    volatile uint32_t evt_pkey_done_cnt;

    /* Auth / Assoc response handshake */
    volatile uint32_t auth_rx_cnt;
    volatile uint16_t auth_rx_status;
    volatile uint16_t auth_rx_seq;
    volatile uint32_t assoc_rx_cnt;
    volatile uint16_t assoc_rx_status;
    volatile uint16_t assoc_rx_aid;

    /* Scan results table */
    uint32_t              scan_count;
    Tb375fcWifiScanEntry  scan_list[TB375FC_WIFI_MAX_SCAN_APS];

    /* Active connection info */
    bool      connected;
    bool      need_wpa2;
    bool      use_sha256_kdf;
    bool      keys_installed;
    bool      tx_encrypt;
    uint8_t   bssid[6];
    char      ssid[36];
    uint8_t   ssid_len;
    uint8_t   channel;
    uint8_t   band;
    int8_t    rssi;
    uint8_t   security;
    uint8_t   group_cipher;
    uint8_t   ap_flags;
    uint16_t  beacon_interval;
    uint16_t  assoc_id;
    uint8_t   last_deauth_reason;

    /* WPA2-PSK 4-Way Handshake State */
    uint8_t   pmk[32];
    uint8_t   snonce[32];
    uint8_t   anonce[32];
    uint8_t   ptk[48]; /* KCK[16] | KEK[16] | TK[16] */
    uint8_t   rsn_ie[32];
    uint8_t   rsn_ie_len;
    uint8_t   eapol_stage; /* 0..4 */
    uint64_t  last_replay_ctr;
    bool      has_replay_ctr;

    /* Counters */
    uint64_t  tx_packets;
    uint64_t  tx_bytes;
    uint64_t  tx_dropped;
    uint64_t  tx_freed;
    uint64_t  rx_packets;
    uint64_t  rx_bytes;
    uint64_t  rx_eapol;
    uint64_t  rx_mgmt;
    uint64_t  rx_events;
    uint64_t  rx_errors;

    /* Software TX rate ladder driven by RX3 HW ACK feedback (stat==0 vs stat!=0) */
    uint8_t   tx_rate_idx;
    uint8_t   tx_consec_ok;
    uint8_t   tx_consec_fail;
    uint8_t   tx_log_budget;
    uint8_t   rx3_log_budget;

    /* Scratch buffers exclusively owned by single RX poller thread (keeps kernel stack tiny) */
    uint8_t   rx_tmp_buf[4096];
    uint8_t   rx_eth_tmp[1600];
};

static WifiState g_wf = {};

static inline uint16_t wf_le16(const uint8_t *p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
static inline uint16_t wf_be16(const uint8_t *p) {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}
static inline uint32_t wf_le32(const uint8_t *p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
static inline uint64_t wf_be64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
    return v;
}
static inline void wf_put_le16(uint8_t *p, uint16_t v) {
    p[0] = uint8_t(v & 0xFFU); p[1] = uint8_t(v >> 8);
}
static inline void wf_put_be16(uint8_t *p, uint16_t v) {
    p[0] = uint8_t(v >> 8); p[1] = uint8_t(v & 0xFFU);
}
static inline void wf_put_le32(uint8_t *p, uint32_t v) {
    p[0] = uint8_t(v & 0xFFU); p[1] = uint8_t((v >> 8) & 0xFFU);
    p[2] = uint8_t((v >> 16) & 0xFFU); p[3] = uint8_t(v >> 24);
}

static inline uintptr_t wf_dma_va(uint32_t off) {
    return g_cs.dma_pool.va + off;
}
static inline uint64_t wf_dma_pa(uint32_t off) {
    return PHYS_WIFI_DMA_POOL + off;
}

static inline void wf_write_desc(uint32_t ring_off, uint32_t idx,
                                 uint64_t buf_pa, uint32_t len, bool is_tx) {
    WfdmaDesc d;
    d.sdp0 = static_cast<uint32_t>(buf_pa & 0xFFFFFFFFULL);
    d.ctrl = (is_tx ? (1U << 30) : 0U) | ((len & 0x3FFFU) << 16); /* DMADONE=0 */
    d.sdp1 = 0;
    d.info = static_cast<uint32_t>((buf_pa >> 32) & 0xFU);
    auto *dst = reinterpret_cast<volatile uint32_t *>(wf_dma_va(ring_off + idx * 16U));
    dst[0] = d.sdp0;
    dst[2] = d.sdp1;
    dst[3] = d.info;
    __asm__ volatile("dsb st" ::: "memory");
    dst[1] = d.ctrl;
    __asm__ volatile("dsb sy" ::: "memory");
}

/* =========================================================================
 * Low-Level TX15 (CMD & 802.11 Mgmt) Submission
 * ========================================================================= */

static bool wf_tx15_submit(const void *pkt, uint32_t len) {
    if (!g_wf.initialized || !pkt || len == 0 || len > WF_TX15_BUF_STRIDE) return false;
    IOLockLock(g_wf.tx_lock);
    uint32_t idx = g_wf.tx15_cidx;
    uint32_t buf_off = DMA_OFF_TX15_BUFS + idx * WF_TX15_BUF_STRIDE;
    io_copy_to(wf_dma_va(buf_off), pkt, len, g_cs.dma_pool.is_wc);
    wf_write_desc(DMA_OFF_TX15_RING, idx, wf_dma_pa(buf_off), len, true);
    g_wf.tx15_cidx = (idx + 1U) % WF_TX15_SZ;
    cs_w32(g_cs.connsys.va, OFF_WFDMA0_TX15 + 0x8U, g_wf.tx15_cidx);
    IOLockUnlock(g_wf.tx_lock);
    return true;
}

static uint8_t wf_send_init_cmd(uint8_t cid, const void *payload, uint32_t plen) {
    uint8_t buf[WF_TX15_BUF_STRIDE];
    uint32_t total = 32U + 12U + plen; /* 32B TXD + 12B INIT_WIFI_CMD + payload */
    if (total > sizeof(buf)) return 0;
    memset(buf, 0, 44U);
    /* TXD DW0: total | PKT_FORMAT=2(CMD), DW1: HF=1(CMD) */
    wf_put_le32(buf + 0, (total & 0xFFFFU) | (2U << 23));
    wf_put_le32(buf + 4, (1U << 16));
    uint8_t seq = ++g_wf.cmd_seq;
    wf_put_le16(buf + 32, static_cast<uint16_t>(total - 32U));
    wf_put_le16(buf + 34, 0x8000U); /* PQ_ID = P1_Q0 */
    buf[36] = cid;
    buf[37] = 0xA0U; /* CMD_PACKET_TYPE_ID */
    buf[38] = 0x00U;
    buf[39] = seq;
    if (plen > 0 && payload) memcpy(buf + 44, payload, plen);
    wf_tx15_submit(buf, total);
    return seq;
}

static uint8_t wf_send_cmd(uint8_t cid, const void *payload, uint32_t plen,
                           uint8_t set_query = 1U, uint8_t ext_cid = 0U, uint8_t ext_opt = 0U) {
    uint8_t buf[WF_TX15_BUF_STRIDE];
    uint32_t total = 32U + 32U + plen; /* 32B TXD + 32B WIFI_CMD + payload */
    if (total > sizeof(buf)) return 0;
    memset(buf, 0, 64U);
    wf_put_le32(buf + 0, (total & 0xFFFFU) | (2U << 23));
    wf_put_le32(buf + 4, (1U << 16));
    uint8_t seq = ++g_wf.cmd_seq;
    wf_put_le16(buf + 32, static_cast<uint16_t>(total - 32U));
    wf_put_le16(buf + 34, 0x8000U);
    buf[36] = cid;
    buf[37] = 0xA0U; /* CMD_PACKET_TYPE_ID */
    buf[38] = set_query;
    buf[39] = seq;
    buf[41] = ext_cid;
    buf[42] = 0U; /* H2N */
    buf[43] = ext_opt;
    if (plen > 0 && payload) memcpy(buf + 64, payload, plen);
    wf_tx15_submit(buf, total);
    return seq;
}

static bool wf_send_mgmt_frame(const uint8_t *frame, uint32_t flen, uint8_t wlan_idx, uint8_t band) {
    if (!frame || flen < 24U || 32U + flen > WF_TX15_BUF_STRIDE) return false;
    uint8_t buf[WF_TX15_BUF_STRIDE];
    uint16_t fc = wf_le16(frame);
    uint32_t ftype = (fc >> 2) & 0x3U;
    uint32_t fsub  = (fc >> 4) & 0xFU;
    uint32_t total = 32U + flen;
    uint32_t eth_off_w = (8U + 24U) >> 1; /* (pse_hdr=8 + mac_hdr=24)/2 = 16 */
    uint32_t rate_code = (band == 2U) ? 0x004BU : 0x0000U; /* OFDM 6M (5G) or CCK 1M (2.4G) */
    uint8_t pid = static_cast<uint8_t>(32U + ((++g_wf.mgmt_seq) & 0x3FU));

    Connac2xTxd txd = {};
    txd.dw[0] = (total & 0xFFFFU) | (eth_off_w << 16) | (2U << 23) | (0U << 25);
    txd.dw[1] = (wlan_idx & 0x3FFU) | (1U << 10) | (12U << 11) | (2U << 16) | (0U << 24) | (1U << 31);
    txd.dw[2] = fsub | (ftype << 4) | (31U << 16) | (1U << 31); /* FR=1 desc, life=31 */
    txd.dw[3] = (15U << 11) | (1U << 28); /* retry=15, BA_DISABLE=1 */
    txd.dw[5] = pid | (1U << 9); /* TXS_TO_MCU=1 */
    txd.dw[6] = (rate_code << 16);
    txd.dw[7] = (fsub << 16) | (ftype << 20);

    memcpy(buf, &txd, 32U);
    memcpy(buf + 32U, frame, flen);
    return wf_tx15_submit(buf, total);
}

/* =========================================================================
 * TX0 Cut-Through Ethernet II Data Submission & Token Allocator
 * ========================================================================= */

static int wf_alloc_tx0_token_locked(void) {
    for (uint32_t k = 0; k < WF_TX0_SZ; ++k) {
        uint32_t tok = (g_wf.tok_alloc_cursor + k) & (WF_TX0_SZ - 1U);
        uint32_t w = tok >> 6;
        uint64_t bit = 1ULL << (tok & 63U);
        if (g_wf.tok_free[w] & bit) {
            g_wf.tok_free[w] &= ~bit;
            g_wf.tok_alloc_cursor = (tok + 1U) & (WF_TX0_SZ - 1U);
            if (g_wf.free_token_count > 0) g_wf.free_token_count--;
            return static_cast<int>(tok);
        }
    }
    return -1;
}

static void wf_free_tx0_token(uint32_t tok) {
    if (tok >= WF_TX0_SZ) return;
    IOLockLock(g_wf.tx_lock);
    uint32_t w = tok >> 6;
    uint64_t bit = 1ULL << (tok & 63U);
    if ((g_wf.tok_free[w] & bit) == 0ULL) {
        g_wf.tok_free[w] |= bit;
        g_wf.free_token_count++;
        g_wf.tx_freed++;
    }
    IOLockUnlock(g_wf.tx_lock);
}

static int wf_tx0_send_eth(const uint8_t *eth_frame, uint32_t len, bool force_unencrypted) {
    if (!g_wf.initialized || !eth_frame || len < 14U || len > 1600U) return -1;
    IOLockLock(g_wf.tx_lock);

    /* Ensure TX0 ring has room */
    uint32_t didx = cs_r32(g_cs.connsys.va, OFF_WFDMA0_TX0 + 0xCU) & 0xFFFU;
    uint32_t next_cidx = (g_wf.tx0_cidx + 1U) & (WF_TX0_SZ - 1U);
    if (next_cidx == didx) {
        g_wf.tx_dropped++;
        IOLockUnlock(g_wf.tx_lock);
        return -2;
    }

    int tok = wf_alloc_tx0_token_locked();
    if (tok < 0) {
        g_wf.tx_dropped++;
        IOLockUnlock(g_wf.tx_lock);
        return -3;
    }

    uint32_t slot_off = DMA_OFF_TX0_BUFS + static_cast<uint32_t>(tok) * WF_TX0_BUF_STRIDE;
    uint64_t slot_pa  = wf_dma_pa(slot_off);
    uint64_t pkt_pa   = slot_pa + 64ULL;

    bool prot = (!force_unencrypted) && g_wf.tx_encrypt;
    uint16_t etype = wf_be16(eth_frame + 12);
    bool is_eapol = (etype == 0x888EU);
    bool is_mc = (eth_frame[0] & 1U) != 0U;
    uint32_t tx_bytes = 32U + len;
    uint32_t eth_off_w = (14U - 2U + 8U) >> 1; /* 10 */

    /* Select proven FR=1 rate code:
     * - EAPOL & broadcast/multicast (DHCP/ARP/ND6): basic OFDM 6M (5G) / CCK 1M (2.4G)
     * - Unicast data: adaptive rate ladder stepped by RX3 HW ACK status (up to HT MCS7 65 Mbps) */
    static const uint16_t kRate5G[6] = { 0x004BU, 0x0049U, 0x004CU, 0x0084U, 0x0086U, 0x0087U };
    static const uint16_t kRate2G[6] = { 0x0000U, 0x004BU, 0x0049U, 0x0083U, 0x0085U, 0x0087U };
    uint8_t max_ridx = (g_wf.ap_flags & 2U) ? 5U : 2U;
    uint8_t ridx = g_wf.tx_rate_idx;
    if (ridx > max_ridx) ridx = max_ridx;
    uint32_t rate_code;
    if (is_eapol || is_mc) {
        rate_code = (g_wf.band == 2U) ? 0x004BU : 0x0000U;
    } else {
        rate_code = (g_wf.band == 2U) ? kRate5G[ridx] : kRate2G[ridx];
    }
    bool want_txs = is_eapol || (g_wf.tx_log_budget > 0U);

    Connac2xTxd txd = {};
    txd.dw[0] = (tx_bytes & 0xFFFFU) | (eth_off_w << 16) | (0U << 23) | (0U << 25); /* PKT_FORMAT=0(Cut-Through), Q=AC0 */
    txd.dw[1] = (WF_AP_WLAN_IDX & 0x3FFU) | (1U << 10) | (1U << 15) | (0U << 16) | (0U << 20) | (0U << 24) | (1U << 31);
                /* WLAN=1, VTA=1, ETHERNET_II=1, HF=0(NON_802_11), TID=0, OMAC=0, LONG=1 */
    txd.dw[2] = (1U << 31); /* TX_TIME=0(unlimited), FR=1(desc fixed rate) */
    txd.dw[3] = (prot ? (1U << 1) : 0U) | (30U << 11) | (1U << 28); /* PF, retry=30, BA_DISABLE=1 */
    if (want_txs) {
        uint8_t pid = static_cast<uint8_t>(128U + ((g_wf.tx_packets) & 0x7FU));
        txd.dw[5] = pid | (1U << 9); /* TXS_TO_MCU=1 */
    }
    txd.dw[6] = (rate_code << 16) | (1U << 10); /* fixed rate + SPE_IDX_SEL=1 (proven on soc7_0) */

    Connac2xTxp txp = {};
    txp.msdu_id[0] = static_cast<uint16_t>((tok & 0x7FFF) | 0x8000U); /* TXD_MSDU_ID_VLD */
    txp.ptr0 = static_cast<uint32_t>(pkt_pa & 0xFFFFFFFFULL);
    txp.len0 = static_cast<uint16_t>((len & 0xFFFU) | (((pkt_pa >> 20) & 0x7000ULL)) | 0x8000U); /* TXD_LEN_ML_V2 */

    uint8_t hdr[64];
    memcpy(hdr, &txd, 32U);
    memcpy(hdr + 32U, &txp, 32U);
    io_copy_to(wf_dma_va(slot_off), hdr, 64U, g_cs.dma_pool.is_wc);
    io_copy_to(wf_dma_va(slot_off + 64U), eth_frame, len, g_cs.dma_pool.is_wc);

    wf_write_desc(DMA_OFF_TX0_RING, g_wf.tx0_cidx, slot_pa, 64U, true);
    g_wf.tx0_cidx = next_cidx;
    cs_w32(g_cs.connsys.va, OFF_WFDMA0_TX0 + 0x8U, g_wf.tx0_cidx);

    g_wf.tx_packets++;
    g_wf.tx_bytes += len;
    if (g_wf.tx_log_budget > 0U) {
        g_wf.tx_log_budget--;
        kprintf("[tb375fc-wifi] TX0 #%llu len=%u etype=0x%04x prot=%u rate=0x%04x tok=%d\n",
                g_wf.tx_packets, len, etype, prot ? 1U : 0U, rate_code, tok);
    }
    IOLockUnlock(g_wf.tx_lock);
    return 0;
}

static int wf_eth_tx_bridge_cb(const uint8_t *frame, uint32_t len) {
    if (!g_wf.connected) return -1;
    if (g_wf.need_wpa2 && !g_wf.keys_installed) return -1;
    return wf_tx0_send_eth(frame, len, false);
}

/* =========================================================================
 * Firmware Commands (BSS, STA, ChPriv, Key, PM)
 * ========================================================================= */

static void wf_cmd_bss_activate(bool active) {
    uint8_t buf[12] = {};
    buf[0] = WF_BSS_IDX;
    buf[1] = active ? 1U : 0U;
    buf[2] = 0U; /* NETWORK_TYPE_AIS */
    buf[3] = 0U; /* OwnMacIdx */
    memcpy(buf + 4, WF_OWN_MAC, 6U);
    buf[10] = WF_BMC_WLAN_IDX;
    buf[11] = 0U;
    wf_send_cmd(CID_BSS_ACTIVATE_CTRL, buf, sizeof(buf), 1U);
}

static void wf_cmd_ch_priv(uint8_t action, uint8_t tok, uint8_t ch, uint8_t band, uint32_t max_ms) {
    uint8_t buf[24] = {};
    buf[0] = WF_BSS_IDX;
    buf[1] = tok;
    buf[2] = action; /* 0=REQ, 1=ABORT */
    buf[3] = ch;
    buf[4] = 0U; /* SCN */
    buf[5] = band; /* 1=2.4G, 2=5G */
    buf[6] = 0U; /* CW_20_40MHZ */
    buf[9] = 0U; /* CH_REQ_TYPE_JOIN */
    buf[10] = 0xFFU; /* ENUM_BAND_AUTO */
    wf_put_le32(buf + 12, max_ms);
    wf_send_cmd(CID_CH_PRIVILEGE, buf, sizeof(buf), 1U);
}

static void wf_cmd_update_sta_record(const uint8_t bssid[6], uint8_t state, uint16_t assoc_id,
                                     uint8_t band, bool qos, bool ht) {
    uint8_t buf[204];
    memset(buf, 0, sizeof(buf));
    buf[0] = WF_AP_STA_IDX;
    buf[1] = 0x41U; /* STA_TYPE_LEGACY_AP = BIT(0)|BIT(6) */
    memcpy(buf + 2, bssid, 6U);
    wf_put_le16(buf + 8, assoc_id);
    wf_put_le16(buf + 10, 10U); /* listen interval */
    buf[12] = WF_BSS_IDX;
    uint8_t phy_set = (band == 2U) ? 0x08U : 0x03U; /* OFDM(5G) or BG(2.4G) */
    if (ht) phy_set |= 0x10U; /* PHY_TYPE_BIT_HT = BIT(4) */
    buf[13] = phy_set;
    wf_put_le16(buf + 14, (band == 2U) ? 0x3FC0U : 0x3FCFU); /* desired non-HT rates */
    wf_put_le16(buf + 16, (band == 2U) ? 0x0540U : 0x000FU); /* BSS basic rates */
    buf[18] = qos ? 1U : 0U;
    buf[20] = state; /* 1..3 */
    buf[21] = ht ? 0xFFU : 0x00U; /* ucMcsSet */
    buf[23] = 1U; /* ucVersion = 1 */
    if (ht) {
        buf[24] = 0xFFU; /* aucRxMcsBitmask[0]: MCS 0..7 */
        buf[25] = 0xFFU; /* aucRxMcsBitmask[1]: MCS 8..15 */
        wf_put_le16(buf + 40, 0x016EU); /* u2HtCapInfo: SGI20, RX_STBC1, SMPS_DIS, HT20 */
        buf[48] = 0x1BU; /* ucAmpduParam */
    }
    buf[50] = 100U; /* ucRCPI (~ -60 dBm so FW Rate Adaptation starts with valid rates) */
    buf[54] = WF_AP_WLAN_IDX;
    buf[55] = 0xFFU; /* WTBL_RESERVED_ENTRY */
    buf[68] = 3U; /* RTS_POLICY_LEGACY */
    wf_put_le16(buf + 74, (band == 2U) ? 0x004BU : 0x0000U); /* default fixed rate */
    wf_put_le16(buf + 96, 0xFFFFU); /* u2PfmuId = invalid */
    wf_send_cmd(CID_UPDATE_STA_RECORD, buf, sizeof(buf), 1U);
}

static void wf_cmd_remove_sta_record(void) {
    uint8_t buf[4] = { 0U, WF_AP_STA_IDX, WF_BSS_IDX, 0U };
    wf_send_cmd(CID_REMOVE_STA_RECORD, buf, sizeof(buf), 1U);
}

static void wf_cmd_set_bss_info(bool connected, const uint8_t bssid[6], const char *ssid, uint8_t ssid_len,
                                uint8_t ch, uint8_t band, bool qos, bool ht, bool wpa2) {
    uint8_t buf[116];
    memset(buf, 0, sizeof(buf));
    buf[0] = WF_BSS_IDX;
    buf[1] = connected ? 0U : 1U; /* MEDIA_STATE_CONNECTED=0, DISCONNECTED=1 */
    buf[2] = 0U; /* OP_MODE_INFRASTRUCTURE */
    if (ssid_len > 32U) ssid_len = 32U;
    buf[3] = ssid_len;
    if (ssid && ssid_len) memcpy(buf + 4, ssid, ssid_len);
    if (bssid) memcpy(buf + 36, bssid, 6U);
    buf[42] = qos ? 1U : 0U;
    wf_put_le16(buf + 44, (band == 2U) ? 0x3FC0U : 0x3FCFU);
    wf_put_le16(buf + 46, (band == 2U) ? 0x0540U : 0x000FU);
    buf[48] = connected ? WF_AP_STA_IDX : 0xFFU;
    wf_put_le16(buf + 50, (band == 2U) ? 0x004BU : 0x0000U);
    buf[52] = (band == 2U) ? 3U : 1U; /* PHY_TYPE_OFDM_INDEX=3 or ERP=1 */
    buf[53] = wpa2 ? 7U : 0U;         /* AUTH_MODE_WPA2_PSK=7, AUTH_MODE_OPEN=0 */
    buf[54] = wpa2 ? 6U : 1U;         /* ENUM_ENCRYPTION3_ENABLED=6, DISABLED=1 */
    uint8_t phy_set = (band == 2U) ? 0x08U : 0x03U;
    if (ht) phy_set |= 0x10U;
    buf[55] = phy_set;
    buf[58] = WF_BMC_WLAN_IDX;

    /* rBssRlmParam @ offset 68 (22 bytes):
     * 68:ucBssIndex, 69:ucRfBand, 70:ucPrimaryChannel, 71:ucRfSco,
     * 72:ucErpProt, 73:ucHtProt, 74:ucGfOp, 75:ucTxRifs,
     * 76..77:u2HtOpInfo3, 78..79:u2HtOpInfo2, 80:ucHtOpInfo1,
     * 81:ucUseShortPreamble, 82:ucUseShortSlotTime,
     * 83:ucVhtChannelWidth, 84:ucVhtS1, 85:ucVhtS2,
     * 86..87:u2VhtBasicMcsSet, 88:ucTxNss, 89:ucRxNss */
    buf[68] = WF_BSS_IDX;
    buf[69] = band;
    buf[70] = ch;
    buf[71] = 0U; /* SCN (20MHz) */
    buf[80] = 0U;
    buf[81] = (band == 1U) ? 1U : 0U;
    buf[82] = 1U;
    buf[83] = 0U;
    buf[84] = 0U;
    buf[85] = 0U;
    wf_put_le16(buf + 86, 0xFFFFU); /* u2VhtBasicMcsSet: none */
    buf[88] = 1U; /* TxNss */
    buf[89] = 1U; /* RxNss */

    buf[90] = 0xFFU; /* ucDBDCBand = ENUM_BAND_AUTO */
    buf[91] = 0U;    /* ucWmmSet = 0 */
    buf[92] = 0U;
    buf[93] = 1U;    /* ucNss = 1 */
    wf_send_cmd(CID_SET_BSS_INFO, buf, sizeof(buf), 1U);
}

static void wf_cmd_update_wmm_parms(bool qos) {
    /* struct CMD_UPDATE_WMM_PARMS (44B): 4 * AC_QUE_PARMS(10B packed) + BssIdx + IsQBSS + WmmSet + Rsv */
    uint8_t buf[44] = {};
    static const uint16_t cwmin[4] = { 15U, 15U,  7U,  3U }; /* AC0(BE/BK), AC1, AC2(VI), AC3(VO) */
    static const uint16_t cwmax[4] = { 1023U, 1023U, 15U, 7U };
    static const uint16_t txop[4]  = {  0U,  0U, 94U, 47U };
    static const uint16_t aifsn[4] = {  3U,  7U,  2U,  2U };
    for (int i = 0; i < 4; ++i) {
        uint8_t *p = buf + i * 10;
        wf_put_le16(p + 0, cwmin[i]);
        wf_put_le16(p + 2, cwmax[i]);
        wf_put_le16(p + 4, txop[i]);
        wf_put_le16(p + 6, aifsn[i]);
        p[8] = 0U; /* ucGuardTime */
        p[9] = 0U; /* ucIsACMSet */
    }
    buf[40] = WF_BSS_IDX;
    buf[41] = qos ? 1U : 0U;
    buf[42] = 0U; /* ucWmmSet = 0 */
    buf[43] = 0U;
    wf_send_cmd(CID_UPDATE_WMM_PARMS, buf, sizeof(buf), 1U);
}

static void wf_cmd_pm_connected(uint16_t aid, uint16_t bcn_int) {
    uint8_t buf[12] = {};
    buf[0] = WF_BSS_IDX;
    buf[1] = 1U; /* DTIM period */
    wf_put_le16(buf + 2, aid);
    wf_put_le16(buf + 4, bcn_int ? bcn_int : 100U);
    wf_send_cmd(CID_INDICATE_PM_CONNECTED, buf, sizeof(buf), 1U);
}

static void wf_cmd_pm_abort(void) {
    uint8_t buf[4] = { WF_BSS_IDX, 0, 0, 0 };
    wf_send_cmd(CID_INDICATE_PM_ABORT, buf, sizeof(buf), 1U);
}

static void wf_cmd_add_key(bool pairwise, uint8_t key_id, uint8_t cipher_id,
                           const uint8_t *key_mat, uint8_t key_len, const uint8_t peer_mac[6]) {
    /* struct CMD_802_11_KEY (64 bytes) */
    uint8_t buf[64] = {};
    buf[0] = 1U;                          /* ucAddRemove = Add */
    buf[1] = pairwise ? 1U : 0U;          /* ucTxKey */
    buf[2] = pairwise ? 1U : 0U;          /* ucKeyType: 1=Pairwise, 0=Group */
    buf[3] = 0U;                          /* ucIsAuthenticator = 0 (STA) */
    memcpy(buf + 4, peer_mac, 6U);        /* aucPeerAddr */
    buf[10] = WF_BSS_IDX;
    buf[11] = cipher_id;                  /* 4=CIPHER_SUITE_CCMP, 2=TKIP */
    buf[12] = key_id;
    buf[13] = key_len;
    buf[14] = pairwise ? WF_AP_WLAN_IDX : WF_BMC_WLAN_IDX;
    buf[15] = 0U;                         /* ucMgmtProtection */
    if (key_len > 32U) key_len = 32U;
    memcpy(buf + 16, key_mat, key_len);
    /* TKIP RX/TX MIC key swap for STA (if 32-byte TKIP GTK):
     * IEEE 802.11i GTK KDE puts TxMIC at [16..23] and RxMIC at [24..31];
     * MTK WTBL expects RxMIC at [16..23] and TxMIC at [24..31] for STA! */
    if (cipher_id == 2U && key_len == 32U) {
        memcpy(buf + 16 + 16, key_mat + 24, 8U);
        memcpy(buf + 16 + 24, key_mat + 16, 8U);
    }
    wf_send_cmd(CID_ADD_REMOVE_KEY, buf, sizeof(buf), 1U);
}

/* =========================================================================
 * WPA2-PSK 4-Way & Group Key EAPOL Handshake Handler (runs in RX thread)
 * ========================================================================= */

static void wf_compute_eapol_mic(uint16_t key_ver, const uint8_t kck[16],
                                 const uint8_t *eapol, uint32_t eapol_len, uint8_t out_mic[16]) {
    if (key_ver == 3U) {
        tb375fc_wcrypto::aes128_cmac(kck, eapol, eapol_len, out_mic);
    } else if (key_ver == 2U) {
        uint8_t digest[20];
        tb375fc_wcrypto::hmac_sha1(kck, 16, eapol, eapol_len, digest);
        memcpy(out_mic, digest, 16);
    } else {
        /* Fallback HMAC-SHA1-128 */
        uint8_t digest[20];
        tb375fc_wcrypto::hmac_sha1(kck, 16, eapol, eapol_len, digest);
        memcpy(out_mic, digest, 16);
    }
}

static bool wf_send_eapol_reply(uint16_t key_info, const uint8_t replay_ctr[8],
                                const uint8_t nonce[32], const uint8_t *key_data, uint16_t kd_len,
                                bool encrypted) {
    /* Ethernet II (14B) + EAPOL-Key (99B) + kd_len */
    uint32_t eapol_len = 99U + kd_len;
    uint32_t eth_len = 14U + eapol_len;
    if (eth_len > 512U) return false;
    uint8_t pkt[512];
    memset(pkt, 0, eth_len);

    memcpy(pkt + 0, g_wf.bssid, 6U);
    memcpy(pkt + 6, WF_OWN_MAC, 6U);
    pkt[12] = 0x88U; pkt[13] = 0x8EU;

    uint8_t *ep = pkt + 14;
    ep[0] = 0x01U; /* Protocol Version = 802.1X-2001 */
    ep[1] = 0x03U; /* Packet Type = Key */
    wf_put_be16(ep + 2, static_cast<uint16_t>(95U + kd_len));
    ep[4] = 0x02U; /* Descriptor Type = RSN (2) */
    wf_put_be16(ep + 5, key_info);
    wf_put_be16(ep + 7, 0x0000U); /* Key Length = 0 in M2/M4/G2 */
    memcpy(ep + 9, replay_ctr, 8U);
    if (nonce) memcpy(ep + 17, nonce, 32U);
    /* [49..64] Key IV = 0, [65..72] Key RSC = 0, [73..80] Reserved = 0, [81..96] MIC = 0 */
    wf_put_be16(ep + 97, kd_len);
    if (kd_len && key_data) memcpy(ep + 99, key_data, kd_len);

    uint16_t key_ver = key_info & 0x7U;
    uint8_t mic[16];
    wf_compute_eapol_mic(key_ver, g_wf.ptk, ep, eapol_len, mic);
    memcpy(ep + 81, mic, 16U);

    return wf_tx0_send_eth(pkt, eth_len, !encrypted) == 0;
}

static bool wf_parse_gtk_kde(const uint8_t *kd, size_t kd_len,
                             uint8_t *out_gtk, uint8_t *out_len, uint8_t *out_kid) {
    size_t pos = 0;
    while (pos + 2U <= kd_len) {
        uint8_t tag = kd[pos];
        if (tag == 0xDDU && pos + 2U <= kd_len) {
            uint8_t elen = kd[pos + 1];
            if (elen == 0 || pos + 2U + elen > kd_len) break;
            /* KDE OUI 00-0F-AC, Data Type 0x01 = GTK */
            if (elen >= 6U && kd[pos + 2] == 0x00 && kd[pos + 3] == 0x0F &&
                kd[pos + 4] == 0xAC && kd[pos + 5] == 0x01) {
                uint8_t kid = kd[pos + 6] & 0x3U;
                uint8_t glen = static_cast<uint8_t>(elen - 6U);
                if (glen == 16U || glen == 32U) {
                    memcpy(out_gtk, kd + pos + 8, glen);
                    *out_len = glen;
                    *out_kid = kid;
                    return true;
                }
            }
            pos += 2U + elen;
        } else if (tag == 0x00U) {
            pos++;
        } else {
            uint8_t elen = kd[pos + 1];
            if (pos + 2U + elen > kd_len) break;
            pos += 2U + elen;
        }
    }
    return false;
}

static void wf_handle_eapol_rx(const uint8_t *eth, uint32_t eth_len) {
    g_wf.rx_eapol++;
    if (!g_wf.need_wpa2 || eth_len < 14U + 99U) return;
    const uint8_t *ep = eth + 14;
    uint32_t ep_len = eth_len - 14U;
    if (ep[1] != 0x03U) return; /* Not EAPOL-Key */
    uint16_t body_len = wf_be16(ep + 2);
    if (4U + body_len > ep_len) return;
    uint32_t full_eapol_len = 4U + body_len;

    uint8_t desc_type = ep[4];
    if (desc_type != 0x02U && desc_type != 0xFEU) return;
    uint16_t key_info = wf_be16(ep + 5);
    uint16_t key_ver  = key_info & 0x7U;
    bool is_pairwise  = (key_info & (1U << 3)) != 0;
    bool is_install   = (key_info & (1U << 6)) != 0;
    bool is_ack       = (key_info & (1U << 7)) != 0;
    bool is_mic       = (key_info & (1U << 8)) != 0;
    bool is_secure    = (key_info & (1U << 9)) != 0;
    bool is_encrypted = (key_info & (1U << 12)) != 0;
    uint16_t kd_len   = wf_be16(ep + 97);
    if (99U + kd_len > full_eapol_len) return;

    const uint8_t *replay_ctr = ep + 9;
    uint64_t rctr = wf_be64(replay_ctr);

    if (is_pairwise && is_ack && !is_mic && !is_install) {
        /* ===== EAPOL Message 1/4 ===== */
        memcpy(g_wf.anonce, ep + 17, 32U);
        read_random(g_wf.snonce, 32U);
        /* Mix mach_absolute_time into SNonce in case read_random is early */
        uint64_t t = mach_absolute_time();
        for (int i = 0; i < 8; ++i) g_wf.snonce[i] ^= uint8_t((t >> (i * 8)) & 0xFFU);

        bool sha256_kdf = (key_ver == 3U) || g_wf.use_sha256_kdf;
        tb375fc_wcrypto::derive_ptk(g_wf.pmk, g_wf.bssid, WF_OWN_MAC,
                                    g_wf.anonce, g_wf.snonce, sha256_kdf, g_wf.ptk);
        g_wf.eapol_stage = 1U;
        g_wf.last_replay_ctr = rctr;
        g_wf.has_replay_ctr = true;

        /* Send Message 2/4: Pairwise=1, MIC=1, same key_ver */
        uint16_t m2_info = static_cast<uint16_t>(key_ver | (1U << 3) | (1U << 8));
        wf_send_eapol_reply(m2_info, replay_ctr, g_wf.snonce,
                            g_wf.rsn_ie, g_wf.rsn_ie_len, false);
        g_wf.eapol_stage = 2U;
        kprintf("[tb375fc-wifi] EAPOL M1 rx (ver=%u) -> M2 sent (rsn_len=%u)\n",
                key_ver, g_wf.rsn_ie_len);
    } else if (is_pairwise && is_ack && is_mic && is_install) {
        /* ===== EAPOL Message 3/4 ===== */
        /* Verify MIC over message with MIC field zeroed */
        uint8_t verify_buf[512];
        if (full_eapol_len > sizeof(verify_buf)) return;
        memcpy(verify_buf, ep, full_eapol_len);
        memset(verify_buf + 81, 0, 16U);
        uint8_t calc_mic[16];
        wf_compute_eapol_mic(key_ver, g_wf.ptk, verify_buf, full_eapol_len, calc_mic);
        if (memcmp(calc_mic, ep + 81, 16U) != 0) {
            kprintf("[tb375fc-wifi] EAPOL M3 MIC MISMATCH (wrong passphrase?)\n");
            return;
        }
        g_wf.eapol_stage = 3U;

        /* Unwrap GTK KDE if encrypted */
        uint8_t gtk[32] = {};
        uint8_t gtk_len = 0, gtk_kid = 1;
        bool got_gtk = false;
        if (is_encrypted && kd_len >= 24U && (kd_len & 7U) == 0 && kd_len <= 256U) {
            uint8_t plain[256];
            if (tb375fc_wcrypto::aes128_key_unwrap(g_wf.ptk + 16, ep + 99, kd_len, plain)) {
                got_gtk = wf_parse_gtk_kde(plain, kd_len - 8U, gtk, &gtk_len, &gtk_kid);
            } else {
                kprintf("[tb375fc-wifi] EAPOL M3 AES key-unwrap FAILED\n");
            }
        }

        /* Send Message 4/4 (unencrypted before PTK install per 802.11i) */
        uint16_t m4_info = static_cast<uint16_t>(key_ver | (1U << 3) | (1U << 8) | (1U << 9));
        wf_send_eapol_reply(m4_info, replay_ctr, nullptr, nullptr, 0U, false);
        IOSleep(10);

        /* Install Pairwise Key (TK = PTK[32..47], CCMP-128 = 4) into WTBL wlan_idx=1 */
        wf_cmd_add_key(true, 0U, 4U, g_wf.ptk + 32, 16U, g_wf.bssid);
        /* Install Group Key (GTK) into WTBL bmc_wlan_idx=30 */
        if (got_gtk) {
            uint8_t gcipher = (gtk_len == 32U) ? 2U : 4U; /* TKIP(2) or CCMP(4) */
            wf_cmd_add_key(false, gtk_kid, gcipher, gtk, gtk_len, g_wf.bssid);
        }
        g_wf.tx_encrypt = true;
        g_wf.keys_installed = true;
        g_wf.eapol_stage = 4U;
        g_wf.state = WIFI_STATE_CONNECTED;
        kprintf("[tb375fc-wifi] EAPOL M3->M4 COMPLETE! PTK(CCMP) + GTK(kid=%u len=%u) installed!\n",
                gtk_kid, gtk_len);
    } else if (!is_pairwise && is_ack && is_mic && is_secure && g_wf.keys_installed) {
        /* ===== Group Key Rekey (G1 -> G2) ===== */
        uint8_t verify_buf[512];
        if (full_eapol_len > sizeof(verify_buf)) return;
        memcpy(verify_buf, ep, full_eapol_len);
        memset(verify_buf + 81, 0, 16U);
        uint8_t calc_mic[16];
        wf_compute_eapol_mic(key_ver, g_wf.ptk, verify_buf, full_eapol_len, calc_mic);
        if (memcmp(calc_mic, ep + 81, 16U) != 0) return;

        uint8_t gtk[32] = {};
        uint8_t gtk_len = 0, gtk_kid = 1;
        bool got_gtk = false;
        if (is_encrypted && kd_len >= 24U && (kd_len & 7U) == 0 && kd_len <= 256U) {
            uint8_t plain[256];
            if (tb375fc_wcrypto::aes128_key_unwrap(g_wf.ptk + 16, ep + 99, kd_len, plain)) {
                got_gtk = wf_parse_gtk_kde(plain, kd_len - 8U, gtk, &gtk_len, &gtk_kid);
            }
        }
        uint16_t g2_info = static_cast<uint16_t>(key_ver | (1U << 8) | (1U << 9));
        wf_send_eapol_reply(g2_info, replay_ctr, nullptr, nullptr, 0U, true);
        if (got_gtk) {
            uint8_t gcipher = (gtk_len == 32U) ? 2U : 4U;
            wf_cmd_add_key(false, gtk_kid, gcipher, gtk, gtk_len, g_wf.bssid);
            kprintf("[tb375fc-wifi] GTK rekeyed (kid=%u len=%u)\n", gtk_kid, gtk_len);
        }
    }
}

/* =========================================================================
 * RX Parsers: RX3 (TX Free Done), RX2 (Events & Mgmt), RX0/RX1 (Data)
 * ========================================================================= */

static void wf_handle_msdu_report(const uint8_t *buf, uint32_t len) {
    if (len < 8U) return;
    uint32_t dw0 = wf_le32(buf + 0);
    uint32_t dw1 = wf_le32(buf + 4);
    uint32_t ver = (dw1 >> 16) & 0x7U;
    uint32_t total_cnt = (ver == 3U) ? ((dw0 >> 16) & 0x3FFU) : ((dw0 >> 16) & 0x7FU);
    if (total_cnt > 128U) total_cnt = 128U;

    uint32_t max_words = (len - 8U) / 4U;
    uint32_t idx = 0, got = 0;
    uint32_t last_p1 = 0U;
    while (got < total_cnt) {
        uint32_t tok = 0xFFFFU;
        if (ver == 0U) {
            if ((idx >> 1) >= max_words) break;
            uint32_t w = wf_le32(buf + 8U + (idx >> 1) * 4U);
            tok = (idx & 1U) ? (w >> 16) : (w & 0xFFFFU);
        } else if (ver == 1U) {
            if (idx >= max_words) break;
            tok = wf_le32(buf + 8U + idx * 4U) & 0xFFFFU;
        } else if (ver == 2U) {
            if (idx >= max_words) break;
            tok = wf_le32(buf + 8U + idx * 4U) & 0x7FFFU;
        } else { /* ver == 3 */
            if (idx >= max_words) break;
            uint32_t w = wf_le32(buf + 8U + idx * 4U);
            if ((w & (1U << 31)) != 0U) { /* Pair=1 info word */
                last_p1 = w;
                idx++;
                continue;
            }
            tok = (w >> 16) & 0x7FFFU;
            uint32_t stat = (w >> 13) & 3U;
            if (stat == 0U) {
                g_wf.tx_consec_fail = 0U;
                if (++g_wf.tx_consec_ok >= 8U) {
                    g_wf.tx_consec_ok = 0U;
                    uint8_t max_ridx = (g_wf.ap_flags & 2U) ? 5U : 2U;
                    if (g_wf.tx_rate_idx < max_ridx) g_wf.tx_rate_idx++;
                }
            } else {
                g_wf.tx_consec_ok = 0U;
                if (++g_wf.tx_consec_fail >= 2U) {
                    g_wf.tx_consec_fail = 0U;
                    if (g_wf.tx_rate_idx > 0U) g_wf.tx_rate_idx--;
                }
            }
            if (g_wf.rx3_log_budget > 0U) {
                g_wf.rx3_log_budget--;
                kprintf("[tb375fc-wifi] RX3 v3 tok=%u stat=%u h=%u cnt=%u ridx=%u p1=0x%08x\n",
                        tok, stat, (w >> 15) & 1U, w & 0x1FFFU, g_wf.tx_rate_idx, last_p1);
            }
        }
        idx++;
        got++;
        if (tok < WF_TX0_SZ) wf_free_tx0_token(tok);
    }
}

static uint32_t wf_rxd_payload_offset(const uint8_t *buf, uint32_t rx_bytes, int8_t *out_rssi) {
    uint32_t dw1 = wf_le32(buf + 4);
    uint32_t dw2 = wf_le32(buf + 8);
    uint32_t gv  = (dw1 >> 11) & 0x1FU;
    uint32_t hoff = ((dw2 >> 14) & 0x3U) * 2U;
    uint32_t off = 24U;
    if (gv & (1U << 3)) off += 16U; /* Group 4 */
    if (gv & (1U << 0)) off += 16U; /* Group 1 */
    if (gv & (1U << 1)) off += 8U;  /* Group 2 */
    if (gv & (1U << 2)) off += 8U;  /* Group 3 */
    if (gv & (1U << 4)) {
        if (out_rssi && off + 28U <= rx_bytes) {
            uint32_t rcpi_w = wf_le32(buf + off + 24U); /* Group5 u4RxVector[6] */
            uint8_t rcpi0 = uint8_t(rcpi_w & 0xFFU);
            uint8_t rcpi1 = uint8_t((rcpi_w >> 8) & 0xFFU);
            uint8_t rcpi = (rcpi0 > rcpi1) ? rcpi0 : rcpi1;
            if (rcpi > 0 && rcpi <= 220U) *out_rssi = static_cast<int8_t>(int(rcpi / 2U) - 110);
        }
        off += 64U; /* Group 5 (16 DW = 64B) */
    }
    off += hoff;
    return off;
}

static void wf_parse_beacon_or_probe(const uint8_t *f, uint32_t flen, uint8_t hw_ch, int8_t rssi) {
    if (flen < 36U) return;
    const uint8_t *bssid = f + 16;
    uint16_t bcn_int = wf_le16(f + 32);
    uint16_t cap     = wf_le16(f + 34);

    char ssid[36] = {};
    uint8_t ssid_len = 0;
    uint8_t ds_ch = hw_ch;
    uint8_t sec = (cap & 0x0010U) ? WIFI_SEC_WEP : WIFI_SEC_OPEN;
    uint8_t group_cipher = 0;
    uint8_t flags = 0;

    uint32_t pos = 36U;
    while (pos + 2U <= flen) {
        uint8_t eid  = f[pos];
        uint8_t elen = f[pos + 1];
        if (pos + 2U + elen > flen) break;
        const uint8_t *val = f + pos + 2U;

        if (eid == 0U && elen <= 32U) {
            ssid_len = elen;
            if (elen > 0) memcpy(ssid, val, elen);
            ssid[elen] = '\0';
        } else if (eid == 3U && elen >= 1U) {
            ds_ch = val[0];
        } else if (eid == 45U) {
            flags |= (1U << 1); /* HT */
        } else if (eid == 191U) {
            flags |= (1U << 2); /* VHT */
        } else if (eid == 255U && elen >= 1U && val[0] == 35U) {
            flags |= (1U << 3); /* HE */
        } else if (eid == 221U && elen >= 6U &&
                   val[0] == 0x00 && val[1] == 0x50 && val[2] == 0xF2 && val[3] == 0x02) {
            flags |= (1U << 0); /* WMM */
        } else if (eid == 221U && elen >= 6U &&
                   val[0] == 0x00 && val[1] == 0x50 && val[2] == 0xF2 && val[3] == 0x01) {
            if (sec == WIFI_SEC_WEP) sec = WIFI_SEC_WPA_TKIP;
        } else if (eid == 48U && elen >= 10U) {
            /* Parse RSN IE: Version(2) + Group(4) + PairwiseCnt(2) + Pairwise(4*n) + AkmCnt(2) + Akm(4*m) + RsnCap(2) */
            group_cipher = val[5]; /* 2=TKIP, 4=CCMP */
            uint16_t pc = wf_le16(val + 6);
            uint32_t p = 8U + pc * 4U;
            bool has_psk = false, has_psk_sha256 = false, has_sae = false;
            if (p + 2U <= elen) {
                uint16_t ac = wf_le16(val + p);
                p += 2U;
                for (uint16_t k = 0; k < ac && p + 4U <= elen; ++k, p += 4U) {
                    if (val[p] == 0x00 && val[p+1] == 0x0F && val[p+2] == 0xAC) {
                        if (val[p+3] == 2U) has_psk = true;
                        else if (val[p+3] == 6U) has_psk_sha256 = true;
                        else if (val[p+3] == 8U) has_sae = true;
                    }
                }
                if (p + 2U <= elen) {
                    uint16_t rsn_cap = wf_le16(val + p);
                    if (rsn_cap & (1U << 6)) flags |= (1U << 4); /* MFPR */
                }
            }
            if (has_psk) sec = WIFI_SEC_WPA2_PSK;
            else if (has_psk_sha256) sec = WIFI_SEC_WPA2_PSK_SHA256;
            else if (has_sae) sec = WIFI_SEC_WPA3_SAE;
            else sec = WIFI_SEC_OTHER;
        }
        pos += 2U + elen;
    }

    uint8_t ch = (ds_ch != 0) ? ds_ch : hw_ch;
    uint8_t band = (ch > 14U) ? 2U : 1U;

    IOLockLock(g_wf.state_lock);
    int slot = -1;
    for (uint32_t i = 0; i < g_wf.scan_count; ++i) {
        if (memcmp(g_wf.scan_list[i].bssid, bssid, 6U) == 0) { slot = static_cast<int>(i); break; }
    }
    if (slot < 0 && g_wf.scan_count < TB375FC_WIFI_MAX_SCAN_APS) {
        slot = static_cast<int>(g_wf.scan_count++);
        memset(&g_wf.scan_list[slot], 0, sizeof(Tb375fcWifiScanEntry));
    }
    if (slot >= 0) {
        Tb375fcWifiScanEntry &e = g_wf.scan_list[slot];
        memcpy(e.bssid, bssid, 6U);
        e.channel = ch;
        e.band = band;
        if (rssi != 0 || e.rssi == 0) e.rssi = (rssi != 0) ? rssi : -65;
        e.security = sec;
        e.group_cipher = group_cipher;
        e.flags |= flags;
        e.cap_info = cap;
        e.beacon_interval = bcn_int;
        if (ssid_len > 0) {
            e.ssid_len = ssid_len;
            memcpy(e.ssid, ssid, ssid_len);
            e.ssid[ssid_len] = '\0';
        }
    }
    IOLockUnlock(g_wf.state_lock);
}

static void wf_process_rx2_buf(const uint8_t *buf, uint32_t buf_len) {
    if (buf_len < 36U) return;
    uint32_t dw0 = wf_le32(buf + 0);
    uint32_t dw1 = wf_le32(buf + 4);
    uint32_t rx_bytes = dw0 & 0xFFFFU;
    if (rx_bytes < 28U || rx_bytes > buf_len) rx_bytes = buf_len;
    uint32_t sw_type = (dw0 >> 16) & 0xFFFFU;

    /* Firmware Event packet: DW0[31:16] & 0x380F == 0x3800 && DW1 == 0 */
    if ((sw_type & 0x380FU) == 0x3800U && dw1 == 0U) {
        g_wf.rx_events++;
        uint16_t elen = wf_le16(buf + 24);
        uint8_t  eid  = buf[28];
        const uint8_t *pl = buf + 36;
        uint32_t plen = (elen > 12U && 24U + elen <= buf_len) ? (elen - 12U) : 0U;

        if (eid == EID_DEBUG_MSG) return; /* Quietly drop periodic PWR debug strings */
        if (eid == EID_NIC_CAP_V2) {
            g_wf.nic_cap_len = plen;
            /* Payload starts with u4TagNum(4) + rsv(4), then [u4Tag(4), u4Len(4), val...] */
            uint32_t p = 8U;
            while (p + 8U <= plen) {
                uint32_t tag = wf_le32(pl + p);
                uint32_t tlen = wf_le32(pl + p + 4);
                if (tlen < 8U || p + tlen > plen) break;
                if (tag == 0x14U && tlen >= 12U) g_wf.adie_ver_cap = wf_le32(pl + p + 8);
                p += tlen;
            }
            g_wf.evt_nic_cap_seen++;
        } else if (eid == EID_SCAN_DONE) {
            g_wf.evt_scan_done_cnt++;
        } else if (eid == EID_CH_PRIVILEGE && plen >= 4U) {
            g_wf.evt_ch_grant_tok = pl[1];
            g_wf.evt_ch_grant_status = pl[2];
            g_wf.evt_ch_grant_cnt++;
        } else if (eid == EID_TX_DONE && plen >= 16U) {
            uint8_t  pid    = pl[0];
            uint8_t  status = pl[1];
            uint16_t sn     = wf_le16(pl + 2);
            uint8_t  widx   = pl[4];
            uint8_t  txcnt  = pl[5];
            uint16_t txrate = wf_le16(pl + 6);
            uint8_t  bw     = pl[12];
            uint8_t  flush  = pl[14];
            if (pid >= 128U || status != 0U) {
                kprintf("[tb375fc-wifi] TX_DONE pid=%u widx=%u status=%u sn=%u cnt=%u rate=0x%04x bw=%u flush=%u\n",
                        pid, widx, status, sn, txcnt, txrate, bw, flush);
            }
        } else if (eid == EID_ADD_PKEY_DONE) {
            g_wf.evt_pkey_done_cnt++;
            kprintf("[tb375fc-wifi] FW EVENT_ID_ADD_PKEY_DONE!\n");
        } else if (eid == EID_BSS_BCN_TIMEOUT && plen >= 2U) {
            kprintf("[tb375fc-wifi] FW BEACON TIMEOUT bss=%u reason=%u\n", pl[0], pl[1]);
        }
        return;
    }

    /* 802.11 Management / Control frame on RX2 */
    int8_t rssi = 0;
    uint32_t off = wf_rxd_payload_offset(buf, rx_bytes, &rssi);
    if (off + 24U > rx_bytes) return;
    const uint8_t *f = buf + off;
    uint32_t flen = rx_bytes - off;
    uint16_t fc = wf_le16(f);
    uint8_t ftype = uint8_t((fc >> 2) & 0x3U);
    uint8_t fsub  = uint8_t((fc >> 4) & 0xFU);
    uint8_t hw_ch = uint8_t((wf_le32(buf + 12) >> 8) & 0xFFU);

    g_wf.rx_mgmt++;
    if (ftype != 0U) return; /* Only Mgmt */

    if (fsub == 8U || fsub == 5U) {
        /* Beacon or Probe Response */
        wf_parse_beacon_or_probe(f, flen, hw_ch, rssi);
        if (g_wf.connected && memcmp(f + 16, g_wf.bssid, 6U) == 0 && rssi != 0) {
            g_wf.rssi = rssi;
        }
    } else if (fsub == 0xBU && flen >= 30U) {
        /* Authentication Response addressed to us */
        if (memcmp(f + 4, WF_OWN_MAC, 6U) == 0) {
            g_wf.auth_rx_seq    = wf_le16(f + 26);
            g_wf.auth_rx_status = wf_le16(f + 28);
            g_wf.auth_rx_cnt++;
            kprintf("[tb375fc-wifi] RX AuthResp seq=%u status=%u\n", g_wf.auth_rx_seq, g_wf.auth_rx_status);
        }
    } else if ((fsub == 0x1U || fsub == 0x3U) && flen >= 30U) {
        /* Association / Reassociation Response addressed to us */
        if (memcmp(f + 4, WF_OWN_MAC, 6U) == 0) {
            g_wf.assoc_rx_status = wf_le16(f + 26);
            g_wf.assoc_rx_aid    = wf_le16(f + 28) & 0x3FFFU;
            g_wf.assoc_rx_cnt++;
            kprintf("[tb375fc-wifi] RX AssocResp status=%u aid=%u\n", g_wf.assoc_rx_status, g_wf.assoc_rx_aid);
        }
    } else if ((fsub == 0xCU || fsub == 0xAU) && flen >= 26U) {
        /* Deauth (0xC) or Disassoc (0xA) from our connected AP */
        if (g_wf.connected && memcmp(f + 16, g_wf.bssid, 6U) == 0 &&
            (memcmp(f + 4, WF_OWN_MAC, 6U) == 0 || f[4] == 0xFFU)) {
            uint16_t reason = wf_le16(f + 24);
            g_wf.last_deauth_reason = static_cast<uint8_t>(reason & 0xFFU);
            kprintf("[tb375fc-wifi] RX %s from AP reason=%u\n",
                    (fsub == 0xCU) ? "DEAUTH" : "DISASSOC", reason);
        }
    }
}

static void wf_process_rx_data_buf(const uint8_t *buf, uint32_t buf_len) {
    if (buf_len < 28U) return;
    uint32_t dw0 = wf_le32(buf + 0);
    uint32_t dw1 = wf_le32(buf + 4);
    uint32_t dw2 = wf_le32(buf + 8);
    uint32_t rx_bytes = dw0 & 0xFFFFU;
    uint32_t pkt_type = (dw0 >> 27) & 0x1FU;

    if (pkt_type == 3U) { /* TX Free Done */
        wf_handle_msdu_report(buf, (rx_bytes <= buf_len) ? rx_bytes : buf_len);
        return;
    }
    if (rx_bytes < 28U || rx_bytes > buf_len) return;
    if (dw1 & ((1U << 27) | (1U << 25) | (1U << 24))) { /* FCS / ICV / CLM error */
        g_wf.rx_errors++;
        if (g_wf.rx_errors <= 8ULL) {
            kprintf("[tb375fc-wifi] RX0 err #%llu dw0=0x%08x dw1=0x%08x dw2=0x%08x\n",
                    g_wf.rx_errors, dw0, dw1, dw2);
        }
        return;
    }

    int8_t rssi = 0;
    uint32_t off = wf_rxd_payload_offset(buf, rx_bytes, &rssi);
    if (off >= rx_bytes) return;
    if (rssi != 0 && g_wf.connected) g_wf.rssi = rssi;

    const uint8_t *p = buf + off;
    uint32_t plen = rx_bytes - off;

    uint8_t *eth_tmp = g_wf.rx_eth_tmp;
    const uint8_t *eth = nullptr;
    uint32_t eth_len = 0;

    if (dw2 & (1U << 13)) {
        /* Hardware Header Translation: already 802.3 Ethernet II! */
        if (plen < 14U || plen > 1600U) return;
        eth = p;
        eth_len = plen;
    } else {
        /* Raw 802.11 Data frame -> translate LLC/SNAP to Ethernet II */
        if (plen < 32U) return;
        uint16_t fc = wf_le16(p);
        uint8_t ftype = uint8_t((fc >> 2) & 0x3U);
        uint8_t fsub  = uint8_t((fc >> 4) & 0xFU);
        if (ftype != 2U || (fsub & 4U) != 0U) return; /* Not Data or Null Data */
        uint32_t hlen = 24U;
        bool to_ds = (fc & (1U << 8)) != 0;
        bool fr_ds = (fc & (1U << 9)) != 0;
        if (to_ds && fr_ds) hlen += 6U;
        if (fsub & 8U) hlen += 2U; /* QoS */
        if (fc & (1U << 15)) hlen += 4U; /* HTC */
        /* Skip CCMP IV (8B) if HW left Protected bit set? No, HW strips IV when decrypting */
        if (hlen + 8U > plen) return;
        const uint8_t *llc = p + hlen;
        if (llc[0] != 0xAAU || llc[1] != 0xAAU || llc[2] != 0x03U) return;
        uint32_t pay_len = plen - hlen - 8U;
        if (14U + pay_len > sizeof(g_wf.rx_eth_tmp)) return;
        memcpy(eth_tmp + 0, p + 4, 6U);                    /* DA = Addr1 */
        memcpy(eth_tmp + 6, fr_ds ? (p + 16) : (p + 10), 6U); /* SA = Addr3(FromDS) or Addr2 */
        eth_tmp[12] = llc[6];
        eth_tmp[13] = llc[7];
        if (pay_len) memcpy(eth_tmp + 14, llc + 8, pay_len);
        eth = eth_tmp;
        eth_len = 14U + pay_len;
    }

    uint16_t etype = wf_be16(eth + 12);
    if (etype == 0x888EU) {
        wf_handle_eapol_rx(eth, eth_len);
        return;
    }

    if (g_wf.connected && (!g_wf.need_wpa2 || g_wf.keys_installed)) {
        g_wf.rx_packets++;
        g_wf.rx_bytes += eth_len;
        if (g_wf.rx_packets <= 8ULL) {
            kprintf("[tb375fc-wifi] RX0 #%llu len=%u etype=0x%04x ht=%u\n",
                    g_wf.rx_packets, eth_len, etype, (dw2 & (1U << 13)) ? 1U : 0U);
        }
        (void)tb375fc_eth_rx_deliver(eth, eth_len);
    }
}

/* =========================================================================
 * Background RX Poller Thread (RX0, RX1, RX2, RX3)
 * ========================================================================= */

static uint32_t wf_drain_rx_ring(uint32_t mmio_off, uint32_t ring_dma_off,
                                 uint32_t bufs_dma_off, uint32_t ring_sz,
                                 uint32_t *sw_idx, uint32_t kind) {
    uint32_t didx = cs_r32(g_cs.connsys.va, mmio_off + 0xCU) & 0xFFFU;
    if (didx >= ring_sz) return 0U;
    uint32_t processed = 0U;
    uint8_t *local_buf = g_wf.rx_tmp_buf;

    while (*sw_idx != didx && processed < 64U) {
        uint32_t idx = *sw_idx;
        uint32_t buf_off = bufs_dma_off + idx * WF_RX_BUF_STRIDE;

        /* Read RX descriptor ctrl to get actual DMA length */
        __asm__ volatile("dsb ld" ::: "memory");
        uint32_t ctrl = *reinterpret_cast<volatile uint32_t *>(wf_dma_va(ring_dma_off + idx * 16U + 4U));
        uint32_t dma_len = (ctrl >> 16) & 0x3FFFU;
        if (dma_len == 0U || dma_len > WF_RX_BUF_MAX_LEN) dma_len = WF_RX_BUF_MAX_LEN;

        io_copy_from(local_buf, wf_dma_va(buf_off), dma_len);

        if (kind == 2U) {
            wf_process_rx2_buf(local_buf, dma_len);
        } else if (kind == 3U) {
            wf_handle_msdu_report(local_buf, dma_len);
        } else {
            wf_process_rx_data_buf(local_buf, dma_len);
        }

        /* Recycle descriptor back to hardware */
        wf_write_desc(ring_dma_off, idx, wf_dma_pa(buf_off), WF_RX_BUF_MAX_LEN, false);
        *sw_idx = (idx + 1U) % ring_sz;
        processed++;
    }

    if (processed > 0U) {
        uint32_t new_cidx = (*sw_idx + ring_sz - 1U) % ring_sz;
        cs_w32(g_cs.connsys.va, mmio_off + 0x8U, new_cidx);
    }
    return processed;
}

static void wf_rx_poller_thread(void *, wait_result_t) {
    kprintf("[tb375fc-wifi] RX poller thread started\n");
    while (g_wf.rx_thread_running) {
        uint32_t work = 0;
        work += wf_drain_rx_ring(OFF_WFDMA0_RX2, DMA_OFF_RX2_RING, DMA_OFF_RX2_BUFS, WF_RX2_SZ, &g_wf.rx2_idx, 2U);
        work += wf_drain_rx_ring(OFF_WFDMA0_RX3, DMA_OFF_RX3_RING, DMA_OFF_RX3_BUFS, WF_RX3_SZ, &g_wf.rx3_idx, 3U);
        work += wf_drain_rx_ring(OFF_WFDMA0_RX0, DMA_OFF_RX0_RING, DMA_OFF_RX0_BUFS, WF_RX0_SZ, &g_wf.rx0_idx, 0U);
        work += wf_drain_rx_ring(OFF_WFDMA0_RX1, DMA_OFF_RX1_RING, DMA_OFF_RX1_BUFS, WF_RX1_SZ, &g_wf.rx1_idx, 1U);
        if (work == 0U) {
            IOSleep(1);
        }
    }
    (void)thread_terminate(current_thread());
}

/* =========================================================================
 * Hardware Init: Driver Own + DMASHDL + WFDMA0 + Firmware Boot + BSS Active
 * ========================================================================= */

static bool wf_acquire_driver_own(void) {
    uintptr_t cs = g_cs.connsys.va;
    if ((cs_r32(cs, OFF_BN0_LPCTL) & 0x4U) == 0U) return true;
    cs_w32(cs, OFF_BN0_LPCTL, 0x2U); /* PCIE_LPCR_HOST_CLR_OWN */
    for (int i = 0; i < 200; ++i) {
        if ((cs_r32(cs, OFF_BN0_LPCTL) & 0x4U) == 0U) return true;
        IOSleep(1);
    }
    return false;
}

static void wf_dmashdl_init_soc7_0(void) {
    uintptr_t cs = g_cs.connsys.va;
    const uint32_t b = OFF_DMASHDL_BASE;
    cs_w32(cs, b + 0x1CU, 0x00180001U); /* PSE_MAX=0x18, PLE_MAX=0x1 */
    cs_w32(cs, b + 0x10U, 0xFFF00000U); /* Groups 0..3 refill enabled, 4..15 disabled */
    cs_w32(cs, b + 0x20U, 0x02000010U); /* Group 0 */
    cs_w32(cs, b + 0x24U, 0x02000010U); /* Group 1 */
    cs_w32(cs, b + 0x28U, 0x02000010U); /* Group 2 */
    cs_w32(cs, b + 0x2CU, 0x01FF0010U); /* Group 3 */
    for (uint32_t off = 0x30U; off <= 0x58U; off += 4U) cs_w32(cs, b + off, 0U);
    cs_w32(cs, b + 0x5CU, 0x00300030U); /* Group 15 (CMD) */
    cs_w32(cs, b + 0x60U, 0x21112000U);
    cs_w32(cs, b + 0x64U, 0x21112000U);
    cs_w32(cs, b + 0x68U, 0x11111000U);
    cs_w32(cs, b + 0x6CU, 0x11111000U);
    cs_w32(cs, b + 0x70U, 0x76543210U);
    cs_w32(cs, b + 0x74U, 0xFEDCBA98U);
    cs_w32(cs, b + 0x0CU, 0x003E1000U); /* slot_arbiter_en = 0 */
    cs_w32(cs, b + 0x08U, 0x70048007U); /* ACK_CNT_TH=4, GUP_ACT_MAP=0x8007 (0,1,2,15) */
}

static void wf_init_wfdma0_rings(void) {
    uintptr_t cs = g_cs.connsys.va;

    /* 1. Disable DMA & wait idle */
    cs_w32(cs, OFF_WFDMA0_GLO_CFG_R, 0x5430BC20U);
    cs_w32(cs, OFF_WFDMA0_INT_ENA_R, 0U);
    for (int i = 0; i < 50; ++i) {
        if ((cs_r32(cs, OFF_WFDMA0_GLO_CFG_R) & 0xAU) == 0U) break;
        IOSleep(1);
    }

    /* 2. Manual prefetch EXT_CTRL in exact soc7_0 order */
    uint32_t ext = 0x00000004U;
    for (uint32_t r = 0; r < 8U; ++r) { cs_w32(cs, OFF_WFDMA0_RX0_EXT + r * 4U, ext); ext += 0x00400000U; }
    cs_w32(cs, OFF_WFDMA0_TX0_EXT,  ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX1_EXT,  ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX2_EXT,  ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX15_EXT, ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX16_EXT, ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX8_EXT,  ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX9_EXT,  ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX10_EXT, ext); ext += 0x00400000U;
    cs_w32(cs, OFF_WFDMA0_TX14_EXT, ext);

    /* 3. Zero descriptor rings in DMA pool */
    uint8_t zpage[4096] = {};
    for (uint32_t off = 0; off < 0xC000U; off += 4096U) {
        io_copy_to(wf_dma_va(off), zpage, 4096U, g_cs.dma_pool.is_wc);
    }

    /* 4. Populate RX descriptors */
    for (uint32_t i = 0; i < WF_RX0_SZ; ++i)
        wf_write_desc(DMA_OFF_RX0_RING, i, wf_dma_pa(DMA_OFF_RX0_BUFS + i * WF_RX_BUF_STRIDE), WF_RX_BUF_MAX_LEN, false);
    for (uint32_t i = 0; i < WF_RX1_SZ; ++i)
        wf_write_desc(DMA_OFF_RX1_RING, i, wf_dma_pa(DMA_OFF_RX1_BUFS + i * WF_RX_BUF_STRIDE), WF_RX_BUF_MAX_LEN, false);
    for (uint32_t i = 0; i < WF_RX2_SZ; ++i)
        wf_write_desc(DMA_OFF_RX2_RING, i, wf_dma_pa(DMA_OFF_RX2_BUFS + i * WF_RX_BUF_STRIDE), WF_RX_BUF_MAX_LEN, false);
    for (uint32_t i = 0; i < WF_RX3_SZ; ++i)
        wf_write_desc(DMA_OFF_RX3_RING, i, wf_dma_pa(DMA_OFF_RX3_BUFS + i * WF_RX_BUF_STRIDE), WF_RX_BUF_MAX_LEN, false);

    /* 5. Program ring base/cnt/cidx */
    auto cfg_ring = [&](uint32_t mmio, uint32_t dma_off, uint32_t cnt, uint32_t cidx) {
        cs_w32(cs, mmio + 0x0U, static_cast<uint32_t>(wf_dma_pa(dma_off) & 0xFFFFFFFFULL));
        cs_w32(cs, mmio + 0x4U, cnt);
        cs_w32(cs, mmio + 0x8U, cidx);
    };
    cfg_ring(OFF_WFDMA0_TX0,  DMA_OFF_TX0_RING,  WF_TX0_SZ,  0U);
    cfg_ring(OFF_WFDMA0_TX1,  DMA_OFF_TX1_RING,  WF_TX1_SZ,  0U);
    cfg_ring(OFF_WFDMA0_TX2,  DMA_OFF_TX2_RING,  WF_TX2_SZ,  0U);
    cfg_ring(OFF_WFDMA0_TX15, DMA_OFF_TX15_RING, WF_TX15_SZ, 0U);
    cfg_ring(OFF_WFDMA0_TX16, DMA_OFF_TX16_RING, WF_TX16_SZ, 0U);
    cfg_ring(OFF_WFDMA0_RX0,  DMA_OFF_RX0_RING,  WF_RX0_SZ,  WF_RX0_SZ - 1U);
    cfg_ring(OFF_WFDMA0_RX1,  DMA_OFF_RX1_RING,  WF_RX1_SZ,  WF_RX1_SZ - 1U);
    cfg_ring(OFF_WFDMA0_RX2,  DMA_OFF_RX2_RING,  WF_RX2_SZ,  WF_RX2_SZ - 1U);
    cfg_ring(OFF_WFDMA0_RX3,  DMA_OFF_RX3_RING,  WF_RX3_SZ,  WF_RX3_SZ - 1U);

    cs_w32(cs, OFF_WFDMA0_RST_DTX_PTR, 0xFFFFFFFFU);
    cs_w32(cs, OFF_WFDMA0_RST_DRX_PTR, 0xFFFFFFFFU);
    cs_upd32(cs, OFF_WFDMA0_INT_RX_PRI, 0xF0U, 0xF0U);
    cs_w32(cs, OFF_WFDMA0_RX0 + 0x8U, WF_RX0_SZ - 1U);
    cs_w32(cs, OFF_WFDMA0_RX1 + 0x8U, WF_RX1_SZ - 1U);
    cs_w32(cs, OFF_WFDMA0_RX2 + 0x8U, WF_RX2_SZ - 1U);
    cs_w32(cs, OFF_WFDMA0_RX3 + 0x8U, WF_RX3_SZ - 1U);

    /* 6. Enable WFDMA0 TX & RX DMA + Host CSR IRQ */
    cs_w32(cs, OFF_BN0_IRQ_ENA, 1U);
    cs_w32(cs, OFF_WFDMA0_INT_STA_R, 0xFFFFFFFFU);
    cs_w32(cs, OFF_WFDMA0_GLO_CFG_R, 0x5430BC75U);

    g_wf.tx0_cidx  = 0;
    g_wf.tx15_cidx = 0;
    g_wf.rx0_idx   = 0;
    g_wf.rx1_idx   = 0;
    g_wf.rx2_idx   = 0;
    g_wf.rx3_idx   = 0;
    for (int i = 0; i < 4; ++i) g_wf.tok_free[i] = ~0ULL;
    g_wf.tok_alloc_cursor = 0;
    g_wf.free_token_count = WF_TX0_SZ;
}

static int32_t wifi_cmd_up(Tb375fcWifiUpOut *out) {
    memset(out, 0, sizeof(*out));
    memcpy(out->mac, WF_OWN_MAC, 6U);

    if (!g_wf.tx_lock)    g_wf.tx_lock    = IOLockAlloc();
    if (!g_wf.state_lock) g_wf.state_lock = IOLockAlloc();
    if (!g_wf.tx_lock || !g_wf.state_lock) {
        out->rc = -1; out->failed_step = 1; return -1;
    }

    if (g_wf.fw_ready && g_wf.initialized) {
        out->rc = 0;
        out->conn_on_misc = cs_r32(g_cs.connsys.va, OFF_CONN_ON_MISC_R);
        out->wf_lpctl     = cs_r32(g_cs.connsys.va, OFF_BN0_LPCTL);
        out->nic_cap_len  = g_wf.nic_cap_len;
        out->adie_ver     = g_wf.adie_ver_cap;
        return 0;
    }

    /* Step 1: Ensure Stage 1 + Stage 3 soc7_0 MCU are up */
    if (!g_cs.wifi_mcu_up) {
        Tb375fcConnSysWifiOnIn win = { 0U, 0U };
        Tb375fcConnSysWifiOnOut wout = {};
        int32_t rc = connsys_stage3_wifi_on(win, &wout);
        if (rc != 0) {
            out->rc = -10; out->failed_step = 1; return out->rc;
        }
    }

    /* Step 2: Acquire Driver Own */
    if (!wf_acquire_driver_own()) {
        out->rc = -2; out->failed_step = 2;
        out->wf_lpctl = cs_r32(g_cs.connsys.va, OFF_BN0_LPCTL);
        return out->rc;
    }

    /* Step 3: DMASHDL + WFDMA0 ring init */
    wf_dmashdl_init_soc7_0();
    wf_init_wfdma0_rings();
    g_wf.initialized = true;

    /* Step 4: Start RX poller kernel thread */
    if (!g_wf.rx_thread_running) {
        g_wf.rx_thread_running = true;
        thread_t th = THREAD_NULL;
        if (kernel_thread_start(wf_rx_poller_thread, nullptr, &th) == KERN_SUCCESS && th != THREAD_NULL) {
            thread_deallocate(th);
        } else {
            g_wf.rx_thread_running = false;
            out->rc = -4; out->failed_step = 4; return out->rc;
        }
    }

    /* Step 5: Boot LK-preloaded Wi-Fi 6 RAM firmware via INIT_CMD_ID_FW_IMAGE_START (0x42) */
    uint32_t misc = cs_r32(g_cs.connsys.va, OFF_CONN_ON_MISC_R);
    if ((misc & 0x3U) != 0x3U) {
        uint8_t start_pl[8] = {}; /* u4Override=0, u4Address=0 (LK preloaded RO EMI) */
        wf_send_init_cmd(CID_INIT_FW_START, start_pl, sizeof(start_pl));
        bool ready = false;
        for (int i = 0; i < 600; ++i) {
            misc = cs_r32(g_cs.connsys.va, OFF_CONN_ON_MISC_R);
            if ((misc & 0x3U) == 0x3U) { ready = true; break; }
            IOSleep(10);
        }
        if (!ready) {
            out->rc = -5; out->failed_step = 5;
            out->conn_on_misc = misc;
            return out->rc;
        }
    }
    IOSleep(100);

    /* Step 6: Query NIC Capability V2 */
    uint32_t prev_cap = g_wf.evt_nic_cap_seen;
    wf_send_cmd(CID_GET_NIC_CAP_V2, nullptr, 0U, 0U);
    for (int i = 0; i < 200; ++i) {
        if (g_wf.evt_nic_cap_seen != prev_cap) break;
        IOSleep(10);
    }

    /* Step 7: Basic Config + Activate BSS0 (en0 MAC); FW default domain enables 2.4G+5G */
    uint8_t basic_cfg[16] = {};
    wf_send_cmd(CID_BASIC_CONFIG, basic_cfg, sizeof(basic_cfg), 1U);
    IOSleep(20);
    wf_cmd_bss_activate(true);
    IOSleep(30);

    /* Register en0 TX bridge callback */
    tb375fc_eth_register_tx(wf_eth_tx_bridge_cb);

    g_wf.fw_ready = true;
    g_wf.state = WIFI_STATE_READY;
    out->rc = 0;
    out->conn_on_misc = cs_r32(g_cs.connsys.va, OFF_CONN_ON_MISC_R);
    out->wf_lpctl     = cs_r32(g_cs.connsys.va, OFF_BN0_LPCTL);
    out->nic_cap_len  = g_wf.nic_cap_len;
    out->adie_ver     = g_wf.adie_ver_cap;
    kprintf("[tb375fc-wifi] Wi-Fi 6 soc7_0 READY! MISC=0x%x NIC_CAP=%uB ADIE=0x%08x MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
            out->conn_on_misc, out->nic_cap_len, out->adie_ver,
            WF_OWN_MAC[0], WF_OWN_MAC[1], WF_OWN_MAC[2], WF_OWN_MAC[3], WF_OWN_MAC[4], WF_OWN_MAC[5]);
    return 0;
}

/* =========================================================================
 * Scan (2.4 GHz + 5 GHz)
 * ========================================================================= */

static int32_t wifi_cmd_scan(const Tb375fcWifiScanIn &in, Tb375fcWifiScanOut *out) {
    memset(out, 0, sizeof(*out));
    if (!g_wf.fw_ready) {
        Tb375fcWifiUpOut up = {};
        if (wifi_cmd_up(&up) != 0) { out->rc = -1; return -1; }
    }

    if ((in.flags & 1U) == 0U) {
        static const uint8_t ch2g[] = { 1,6,11,2,3,4,5,7,8,9,10,12,13 };
        static const uint8_t ch5g[] = { 36,40,44,48,149,153,157,161,165 };
        uint8_t pl[1192];
        memset(pl, 0, sizeof(pl));
        pl[0] = static_cast<uint8_t>((g_wf.cmd_seq + 1U) & 0xFFU);
        pl[1] = WF_BSS_IDX;
        pl[2] = 1U;    /* SCAN_TYPE_ACTIVE_SCAN */
        pl[3] = 1U;    /* SCAN_REQ_SSID_WILDCARD */
        pl[4] = 0U;
        pl[5] = 2U;    /* 2 probe reqs per ch */
        pl[6] = 0x02U; /* DIS_DBDC_SCAN */
        pl[7] = 1U;    /* version 1 */
        uint16_t dwell = (in.dwell_ms >= 20U && in.dwell_ms <= 250U) ? static_cast<uint16_t>(in.dwell_ms) : 60U;
        wf_put_le16(pl + 154, dwell);
        pl[158] = 4U; /* SCAN_CHANNEL_SPECIFIED */

        uint8_t cnt = 0;
        for (size_t i = 0; i < sizeof(ch2g) && cnt < 32U; ++i) {
            pl[160 + cnt * 2] = 1U;
            pl[160 + cnt * 2 + 1] = ch2g[i];
            cnt++;
        }
        if ((in.flags & 2U) == 0U) {
            for (size_t i = 0; i < sizeof(ch5g) && cnt < 32U; ++i) {
                pl[160 + cnt * 2] = 2U;
                pl[160 + cnt * 2 + 1] = ch5g[i];
                cnt++;
            }
        }
        pl[159] = cnt;
        wf_put_le16(pl + 828, 30U); /* min dwell */
        memset(pl + 1112, 0xFF, 6U); /* wildcard BSSID */

        IOLockLock(g_wf.state_lock);
        g_wf.scan_count = 0;
        IOLockUnlock(g_wf.state_lock);

        uint32_t prev_done = g_wf.evt_scan_done_cnt;
        uint32_t prev_state = g_wf.state;
        if (!g_wf.connected) g_wf.state = WIFI_STATE_SCANNING;
        wf_send_cmd(CID_SCAN_REQ_V2, pl, sizeof(pl), 1U);

        for (int i = 0; i < 600; ++i) {
            if (g_wf.evt_scan_done_cnt != prev_done) break;
            IOSleep(10);
        }
        if (!g_wf.connected) g_wf.state = prev_state;
    }

    IOLockLock(g_wf.state_lock);
    out->count = g_wf.scan_count;
    if (out->count > TB375FC_WIFI_MAX_SCAN_APS) out->count = TB375FC_WIFI_MAX_SCAN_APS;
    if (out->count > 0) {
        memcpy(out->entries, g_wf.scan_list, out->count * sizeof(Tb375fcWifiScanEntry));
    }
    IOLockUnlock(g_wf.state_lock);
    out->rc = 0;
    return 0;
}

/* =========================================================================
 * Disconnect & Join (OPEN / WPA2-PSK CCMP-128 / WPA2-PSK-SHA256)
 * ========================================================================= */

static void wf_disconnect_internal(bool send_deauth) {
    if (g_wf.connected && send_deauth) {
        /* Send 802.11 Deauth (reason=3 leaving BSS) */
        uint8_t deauth[26];
        wf_put_le16(deauth + 0, 0x00C0U);
        wf_put_le16(deauth + 2, 0U);
        memcpy(deauth + 4, g_wf.bssid, 6U);
        memcpy(deauth + 10, WF_OWN_MAC, 6U);
        memcpy(deauth + 16, g_wf.bssid, 6U);
        wf_put_le16(deauth + 22, static_cast<uint16_t>((++g_wf.mgmt_seq) << 4));
        wf_put_le16(deauth + 24, 3U);
        wf_send_mgmt_frame(deauth, sizeof(deauth), WF_AP_WLAN_IDX, g_wf.band ? g_wf.band : 1U);
        IOSleep(15);
    }
    g_wf.connected = false;
    g_wf.keys_installed = false;
    g_wf.tx_encrypt = false;
    g_wf.eapol_stage = 0;
    g_wf.assoc_id = 0;
    if (g_wf.fw_ready) {
        wf_cmd_pm_abort();
        wf_cmd_set_bss_info(false, g_wf.bssid, "", 0, 1, 1, false, false, false);
        wf_cmd_remove_sta_record();
        g_wf.state = WIFI_STATE_READY;
    }
}

static uint32_t wf_build_assoc_req(uint8_t *buf, const uint8_t bssid[6],
                                   const char *ssid, uint8_t ssid_len,
                                   uint8_t band, bool wpa2, uint8_t group_cipher,
                                   bool sha256_akm, bool wmm, bool ht,
                                   uint8_t *out_rsn_ie, uint8_t *out_rsn_len) {
    uint32_t p = 0;
    /* 24B 802.11 Header */
    wf_put_le16(buf + p, 0x0000U); p += 2; /* AssocReq */
    wf_put_le16(buf + p, 0x0000U); p += 2;
    memcpy(buf + p, bssid, 6U); p += 6;
    memcpy(buf + p, WF_OWN_MAC, 6U); p += 6;
    memcpy(buf + p, bssid, 6U); p += 6;
    wf_put_le16(buf + p, static_cast<uint16_t>((++g_wf.mgmt_seq) << 4)); p += 2;

    /* Fixed 4B: CapInfo + ListenInterval */
    uint16_t cap = 0x0001U; /* ESS */
    if (wpa2) cap |= 0x0010U; /* Privacy */
    if (band == 1U) cap |= 0x0420U; /* ShortPreamble | ShortSlot */
    wf_put_le16(buf + p, cap); p += 2;
    wf_put_le16(buf + p, 10U); p += 2;

    /* SSID IE (0) */
    buf[p++] = 0U; buf[p++] = ssid_len;
    memcpy(buf + p, ssid, ssid_len); p += ssid_len;

    /* Supported Rates (1) + Extended Supported Rates (50) */
    if (band == 2U) {
        static const uint8_t r5g[] = { 0x8C, 0x12, 0x98, 0x24, 0xB0, 0x48, 0x60, 0x6C };
        buf[p++] = 1U; buf[p++] = sizeof(r5g);
        memcpy(buf + p, r5g, sizeof(r5g)); p += sizeof(r5g);
    } else {
        static const uint8_t r2g[]  = { 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24 };
        static const uint8_t er2g[] = { 0x30, 0x48, 0x60, 0x6C };
        buf[p++] = 1U; buf[p++] = sizeof(r2g);
        memcpy(buf + p, r2g, sizeof(r2g)); p += sizeof(r2g);
        buf[p++] = 50U; buf[p++] = sizeof(er2g);
        memcpy(buf + p, er2g, sizeof(er2g)); p += sizeof(er2g);
    }

    /* RSN IE (48) for WPA2-PSK */
    *out_rsn_len = 0;
    if (wpa2) {
        uint8_t gc = (group_cipher == 2U) ? 2U : 4U; /* TKIP(2) or CCMP(4) */
        uint8_t akm = sha256_akm ? 6U : 2U;          /* PSK-SHA256(6) or PSK(2) */
        uint8_t rsn[22] = {
            48, 20,
            0x01, 0x00,                         /* Version 1 */
            0x00, 0x0F, 0xAC, gc,               /* Group Data Cipher Suite */
            0x01, 0x00,                         /* Pairwise Cipher Count = 1 */
            0x00, 0x0F, 0xAC, 0x04,             /* Pairwise = CCMP-128 */
            0x01, 0x00,                         /* AKM Count = 1 */
            0x00, 0x0F, 0xAC, akm,              /* AKM = PSK / PSK-SHA256 */
            0x00, 0x00                          /* RSN Capabilities */
        };
        memcpy(buf + p, rsn, sizeof(rsn)); p += sizeof(rsn);
        memcpy(out_rsn_ie, rsn, sizeof(rsn));
        *out_rsn_len = sizeof(rsn);
    }

    /* HT Capabilities IE (45) — 20MHz 2SS MCS0..15, SGI20, RX-STBC1 */
    if (ht) {
        uint8_t ht_ie[28] = {};
        ht_ie[0] = 45U; ht_ie[1] = 26U;
        wf_put_le16(ht_ie + 2, 0x016EU); /* HT Cap Info */
        ht_ie[4] = 0x1BU;                /* A-MPDU Params */
        ht_ie[5] = 0xFFU;                /* MCS 0..7 */
        ht_ie[6] = 0xFFU;                /* MCS 8..15 */
        memcpy(buf + p, ht_ie, sizeof(ht_ie)); p += sizeof(ht_ie);
    }

    /* WMM Information Element (221) */
    if (wmm) {
        static const uint8_t wmm_ie[9] = { 221, 7, 0x00, 0x50, 0xF2, 0x02, 0x00, 0x01, 0x00 };
        memcpy(buf + p, wmm_ie, sizeof(wmm_ie)); p += sizeof(wmm_ie);
    }

    return p;
}

static int32_t wifi_cmd_join(const Tb375fcWifiJoinIn &in, Tb375fcWifiJoinOut *out) {
    memset(out, 0, sizeof(*out));
    if (!g_wf.fw_ready) {
        Tb375fcWifiUpOut up = {};
        if (wifi_cmd_up(&up) != 0) { out->rc = -1; out->failed_step = 1; return -1; }
    }

    uint8_t target_ssid_len = 0;
    while (target_ssid_len < 32U && in.ssid[target_ssid_len] != '\0') target_ssid_len++;
    bool bssid_given = false;
    for (int i = 0; i < 6; ++i) if (in.bssid[i] != 0) bssid_given = true;
    if (target_ssid_len == 0 && !bssid_given) {
        out->rc = -2; out->failed_step = 2; return -2;
    }

    /* Step 1: Locate target AP in scan cache (auto-scan if not found) */
    auto find_ap = [&](Tb375fcWifiScanEntry *found) -> bool {
        IOLockLock(g_wf.state_lock);
        int best = -1;
        for (uint32_t i = 0; i < g_wf.scan_count; ++i) {
            const Tb375fcWifiScanEntry &e = g_wf.scan_list[i];
            if (bssid_given && memcmp(e.bssid, in.bssid, 6U) != 0) continue;
            if (!bssid_given) {
                if (e.ssid_len != target_ssid_len || memcmp(e.ssid, in.ssid, target_ssid_len) != 0) continue;
            }
            if (in.channel != 0 && e.channel != in.channel) continue;
            if (in.band != 0 && e.band != in.band) continue;
            if (best < 0 || e.rssi > g_wf.scan_list[best].rssi) best = static_cast<int>(i);
        }
        bool ok = (best >= 0);
        if (ok) *found = g_wf.scan_list[best];
        IOLockUnlock(g_wf.state_lock);
        return ok;
    };

    Tb375fcWifiScanEntry ap = {};
    if (!find_ap(&ap) && (in.flags & 1U) == 0U) {
        Tb375fcWifiScanIn sin = { 0U, 60U };
        Tb375fcWifiScanOut *sout = static_cast<Tb375fcWifiScanOut *>(IOMallocData(sizeof(Tb375fcWifiScanOut)));
        if (sout) {
            wifi_cmd_scan(sin, sout);
            IOFreeData(sout, sizeof(Tb375fcWifiScanOut));
        }
        find_ap(&ap);
    }

    if (ap.channel == 0) {
        if (bssid_given && in.channel != 0) {
            memcpy(ap.bssid, in.bssid, 6U);
            ap.channel = in.channel;
            ap.band = in.band ? in.band : ((in.channel > 14U) ? 2U : 1U);
            ap.ssid_len = target_ssid_len;
            memcpy(ap.ssid, in.ssid, target_ssid_len);
            ap.security = (in.passphrase[0] != '\0') ? WIFI_SEC_WPA2_PSK : WIFI_SEC_OPEN;
            ap.group_cipher = 4U;
            ap.flags = 0x03U; /* WMM + HT */
            ap.beacon_interval = 100U;
        } else {
            out->rc = -3; out->failed_step = 3; /* AP not found */
            return out->rc;
        }
    }

    /* Step 2: Precompute PMK if WPA2-PSK (takes ~5ms before RF handshake) */
    uint8_t pass_len = 0;
    while (pass_len < 64U && in.passphrase[pass_len] != '\0') pass_len++;
    bool want_wpa2 = (pass_len > 0) ||
                     (ap.security == WIFI_SEC_WPA2_PSK || ap.security == WIFI_SEC_WPA2_PSK_SHA256);
    if (want_wpa2 && pass_len == 0) {
        out->rc = -4; out->failed_step = 4; /* Passphrase required */
        return out->rc;
    }

    wf_disconnect_internal(true);

    memcpy(g_wf.bssid, ap.bssid, 6U);
    memcpy(g_wf.ssid, ap.ssid, ap.ssid_len);
    g_wf.ssid[ap.ssid_len] = '\0';
    g_wf.ssid_len = ap.ssid_len;
    g_wf.channel = ap.channel;
    g_wf.band = ap.band;
    g_wf.rssi = ap.rssi;
    g_wf.security = ap.security;
    g_wf.group_cipher = ap.group_cipher ? ap.group_cipher : 4U;
    g_wf.ap_flags = ap.flags;
    g_wf.beacon_interval = ap.beacon_interval ? ap.beacon_interval : 100U;
    g_wf.need_wpa2 = want_wpa2;
    g_wf.use_sha256_kdf = (ap.security == WIFI_SEC_WPA2_PSK_SHA256);
    g_wf.keys_installed = false;
    g_wf.tx_encrypt = false;
    g_wf.eapol_stage = 0;
    g_wf.has_replay_ctr = false;
    g_wf.last_deauth_reason = 0;
    g_wf.tx_rate_idx = 2U; /* Start at OFDM 54M (5G) / 24M (2.4G), step up to HT MCS7 (65M) */
    g_wf.tx_consec_ok = 0U;
    g_wf.tx_consec_fail = 0U;
    g_wf.tx_log_budget = 12U;
    g_wf.rx3_log_budget = 12U;

    if (want_wpa2) {
        if (pass_len == 64U) {
            /* Raw 64-hex PMK support */
            bool all_hex = true;
            uint8_t raw_pmk[32];
            for (int i = 0; i < 32 && all_hex; ++i) {
                auto hval = [](char c) -> int {
                    if (c >= '0' && c <= '9') return c - '0';
                    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                    return -1;
                };
                int hi = hval(in.passphrase[i * 2]), lo = hval(in.passphrase[i * 2 + 1]);
                if (hi < 0 || lo < 0) all_hex = false;
                else raw_pmk[i] = static_cast<uint8_t>((hi << 4) | lo);
            }
            if (all_hex) memcpy(g_wf.pmk, raw_pmk, 32U);
            else tb375fc_wcrypto::pbkdf2_sha1(reinterpret_cast<const uint8_t *>(in.passphrase), pass_len,
                                              reinterpret_cast<const uint8_t *>(g_wf.ssid), g_wf.ssid_len,
                                              4096U, g_wf.pmk, 32U);
        } else {
            tb375fc_wcrypto::pbkdf2_sha1(reinterpret_cast<const uint8_t *>(in.passphrase), pass_len,
                                          reinterpret_cast<const uint8_t *>(g_wf.ssid), g_wf.ssid_len,
                                          4096U, g_wf.pmk, 32U);
        }
    }

    memcpy(out->bssid, ap.bssid, 6U);
    out->channel = ap.channel;
    out->band = ap.band;
    out->rssi = ap.rssi;
    out->security = ap.security;

    /* Step 3: Request Channel Privilege */
    uint8_t tok = ++g_wf.ch_token;
    if (tok == 0) tok = ++g_wf.ch_token;
    uint32_t prev_grant = g_wf.evt_ch_grant_cnt;
    wf_cmd_ch_priv(0U, tok, ap.channel, ap.band, 5000U);
    bool granted = false;
    for (int i = 0; i < 200; ++i) {
        if (g_wf.evt_ch_grant_cnt != prev_grant) { granted = true; break; }
        IOSleep(10);
    }
    if (!granted) {
        out->rc = -5; out->failed_step = 5;
        return out->rc;
    }

    /* Step 4: Create STA Record at state 1 */
    bool use_qos = (ap.flags & 1U) != 0U;
    bool use_ht  = (ap.flags & 2U) != 0U;
    wf_cmd_update_sta_record(ap.bssid, 1U, 0U, ap.band, false, false);
    IOSleep(20);

    /* Step 5: 802.11 Open Authentication */
    g_wf.state = WIFI_STATE_AUTH;
    bool auth_ok = false;
    for (int attempt = 0; attempt < 4 && !auth_ok; ++attempt) {
        uint8_t auth[30];
        wf_put_le16(auth + 0, 0x00B0U);
        wf_put_le16(auth + 2, 0U);
        memcpy(auth + 4, ap.bssid, 6U);
        memcpy(auth + 10, WF_OWN_MAC, 6U);
        memcpy(auth + 16, ap.bssid, 6U);
        wf_put_le16(auth + 22, static_cast<uint16_t>((++g_wf.mgmt_seq) << 4));
        wf_put_le16(auth + 24, 0U); /* Open System */
        wf_put_le16(auth + 26, 1U); /* Seq 1 */
        wf_put_le16(auth + 28, 0U); /* Status 0 */
        uint32_t prev_a = g_wf.auth_rx_cnt;
        wf_send_mgmt_frame(auth, sizeof(auth), WF_AP_WLAN_IDX, ap.band);
        for (int i = 0; i < 40; ++i) {
            if (g_wf.auth_rx_cnt != prev_a) {
                out->status_code = g_wf.auth_rx_status;
                if (g_wf.auth_rx_status == 0U) auth_ok = true;
                break;
            }
            IOSleep(10);
        }
        if (g_wf.auth_rx_cnt != prev_a && g_wf.auth_rx_status != 0U) break;
    }
    if (!auth_ok) {
        wf_cmd_ch_priv(1U, tok, ap.channel, ap.band, 0U);
        wf_cmd_remove_sta_record();
        g_wf.state = WIFI_STATE_READY;
        out->rc = -6; out->failed_step = 6;
        return out->rc;
    }

    /* Step 6: 802.11 Association Request */
    g_wf.state = WIFI_STATE_ASSOC;
    uint8_t assoc_buf[256];
    uint32_t assoc_len = wf_build_assoc_req(assoc_buf, ap.bssid, g_wf.ssid, g_wf.ssid_len,
                                            ap.band, want_wpa2, g_wf.group_cipher,
                                            g_wf.use_sha256_kdf, use_qos, use_ht,
                                            g_wf.rsn_ie, &g_wf.rsn_ie_len);
    bool assoc_ok = false;
    for (int attempt = 0; attempt < 4 && !assoc_ok; ++attempt) {
        uint32_t prev_as = g_wf.assoc_rx_cnt;
        wf_send_mgmt_frame(assoc_buf, assoc_len, WF_AP_WLAN_IDX, ap.band);
        for (int i = 0; i < 50; ++i) {
            if (g_wf.assoc_rx_cnt != prev_as) {
                out->status_code = g_wf.assoc_rx_status;
                if (g_wf.assoc_rx_status == 0U) {
                    g_wf.assoc_id = g_wf.assoc_rx_aid;
                    out->assoc_id = g_wf.assoc_id;
                    assoc_ok = true;
                }
                break;
            }
            IOSleep(10);
        }
        if (g_wf.assoc_rx_cnt != prev_as && g_wf.assoc_rx_status != 0U) break;
    }
    if (!assoc_ok) {
        wf_cmd_ch_priv(1U, tok, ap.channel, ap.band, 0U);
        wf_cmd_remove_sta_record();
        g_wf.state = WIFI_STATE_READY;
        out->rc = -7; out->failed_step = 7;
        return out->rc;
    }

    /* Step 7: Exact upstream gen4m join order: WMM_PARMS -> SET_BSS_INFO(CONNECTED) ->
     * UPDATE_STA_RECORD(STATE_3) -> PM_CONNECTED -> CH_PRIV(ABORT).
     * (SET_BSS_INFO MUST precede UPDATE_STA_RECORD(3) so FW initializes WTBL/RA on the active band!) */
    wf_cmd_update_wmm_parms(use_qos);
    IOSleep(2);
    wf_cmd_set_bss_info(true, ap.bssid, g_wf.ssid, g_wf.ssid_len,
                        ap.channel, ap.band, use_qos, use_ht, want_wpa2);
    IOSleep(5);
    wf_cmd_update_sta_record(ap.bssid, 3U, g_wf.assoc_id, ap.band, use_qos, use_ht);
    IOSleep(5);
    wf_cmd_pm_connected(g_wf.assoc_id, g_wf.beacon_interval);
    IOSleep(2);
    wf_cmd_ch_priv(1U, tok, ap.channel, ap.band, 0U);
    g_wf.connected = true;

    /* Step 8: If WPA2-PSK, wait for automatic 4-Way Handshake completion (up to 6s) */
    if (want_wpa2) {
        if (!g_wf.keys_installed) g_wf.state = WIFI_STATE_EAPOL;
        for (int i = 0; i < 600; ++i) {
            if (g_wf.keys_installed) break;
            IOSleep(10);
        }
        out->eapol_stage = g_wf.eapol_stage;
        out->keys_installed = g_wf.keys_installed ? 1U : 0U;
        if (!g_wf.keys_installed) {
            kprintf("[tb375fc-wifi] WPA2 4-way handshake timeout at stage=%u (eapol_rx=%llu)\n",
                    g_wf.eapol_stage, g_wf.rx_eapol);
            out->rc = -8; out->failed_step = 8;
            return out->rc;
        }
    }
    g_wf.state = WIFI_STATE_CONNECTED;

    /* Step 9: Pulse en0 link DOWN -> UP so macOS IPConfiguration immediately triggers DHCP! */
    tb375fc_eth_set_link(false, 0ULL);
    IOSleep(80);
    tb375fc_eth_set_link(true, 150000000ULL);

    kprintf("[tb375fc-wifi] JOINED \"%s\" (%02x:%02x:%02x:%02x:%02x:%02x) ch=%u band=%u aid=%u sec=%u keys=%u -> en0 UP!\n",
            g_wf.ssid, ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5],
            ap.channel, ap.band, g_wf.assoc_id, ap.security, g_wf.keys_installed ? 1U : 0U);
    out->rc = 0;
    return 0;
}

static void wifi_cmd_status(Tb375fcWifiStatusOut *out) {
    memset(out, 0, sizeof(*out));
    out->state              = g_wf.state;
    out->fw_ready           = g_wf.fw_ready ? 1U : 0U;
    out->connected          = g_wf.connected ? 1U : 0U;
    out->keys_installed     = g_wf.keys_installed ? 1U : 0U;
    memcpy(out->own_mac, WF_OWN_MAC, 6U);
    memcpy(out->bssid, g_wf.bssid, 6U);
    memcpy(out->ssid, g_wf.ssid, 36U);
    out->channel            = g_wf.channel;
    out->band               = g_wf.band;
    out->rssi               = g_wf.rssi;
    out->security           = g_wf.security;
    out->assoc_id           = g_wf.assoc_id;
    out->eapol_stage        = g_wf.eapol_stage;
    out->last_deauth_reason = g_wf.last_deauth_reason;
    out->tx_packets         = g_wf.tx_packets;
    out->tx_bytes           = g_wf.tx_bytes;
    out->tx_dropped         = g_wf.tx_dropped;
    out->tx_freed           = g_wf.tx_freed;
    out->rx_packets         = g_wf.rx_packets;
    out->rx_bytes           = g_wf.rx_bytes;
    out->rx_eapol           = g_wf.rx_eapol;
    out->rx_mgmt            = g_wf.rx_mgmt;
    out->rx_events          = g_wf.rx_events;
    out->rx_errors          = g_wf.rx_errors;
    out->free_tokens        = g_wf.free_token_count;
    if (g_cs.wifi_mcu_up && g_cs.connsys.va) {
        uintptr_t cs = g_cs.connsys.va;
        out->tx0_cidx     = cs_r32(cs, OFF_WFDMA0_TX0  + 0x8U);
        out->tx0_didx     = cs_r32(cs, OFF_WFDMA0_TX0  + 0xCU);
        out->tx15_cidx    = cs_r32(cs, OFF_WFDMA0_TX15 + 0x8U);
        out->tx15_didx    = cs_r32(cs, OFF_WFDMA0_TX15 + 0xCU);
        out->rx0_cidx     = cs_r32(cs, OFF_WFDMA0_RX0  + 0x8U);
        out->rx0_didx     = cs_r32(cs, OFF_WFDMA0_RX0  + 0xCU);
        out->rx2_cidx     = cs_r32(cs, OFF_WFDMA0_RX2  + 0x8U);
        out->rx2_didx     = cs_r32(cs, OFF_WFDMA0_RX2  + 0xCU);
        out->rx3_cidx     = cs_r32(cs, OFF_WFDMA0_RX3  + 0x8U);
        out->rx3_didx     = cs_r32(cs, OFF_WFDMA0_RX3  + 0xCU);
        out->conn_on_misc = cs_r32(cs, OFF_CONN_ON_MISC_R);
    }
}

static int32_t wifi_cmd_down(Tb375fcWifiDownOut *out) {
    memset(out, 0, sizeof(*out));
    out->prev_state = g_wf.state;
    wf_disconnect_internal(true);
    out->rc = 0;
    return 0;
}

} // namespace

#endif /* TB375FC_WIFI_ENGINE_H */
