/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/*
 * Tb375fcConnSysABI.h — Shared XNU Kernel <-> Userspace IOUserClient ABI
 *                       for MT6897 Onboard ConnSys (Wi-Fi 6 soc7_0 + BT BGF)
 */
#ifndef TB375FC_CONNSYS_ABI_H
#define TB375FC_CONNSYS_ABI_H

#include <stdint.h>

#define TB375FC_CONNSYS_SERVICE_NAME "Tb375fcConnSys"

enum Tb375fcConnSysSelector {
    CONNSYS_SEL_RO_PROBE       = 0,
    CONNSYS_SEL_STAGE1_UP      = 1,
    CONNSYS_SEL_STAGE3_WIFI_ON = 2,
    CONNSYS_SEL_UFS_READ_RO    = 3,
    CONNSYS_SEL_EMI_RW         = 4,
    CONNSYS_SEL_DMA_RW         = 5,
    CONNSYS_SEL_REG_OP         = 6,
    CONNSYS_SEL_SMC_CALL       = 7,
    CONNSYS_SEL_WIFI_UP        = 8,
    CONNSYS_SEL_WIFI_SCAN      = 9,
    CONNSYS_SEL_WIFI_JOIN      = 10,
    CONNSYS_SEL_WIFI_STATUS    = 11,
    CONNSYS_SEL_WIFI_DOWN      = 12,
};

/* SEL 0: Read-only hardware & EMI diagnostic snapshot */
struct Tb375fcConnSysProbeOut {
    uint32_t spm_pwr_cfg_en;
    uint32_t spm_conn_pwr_con;
    uint32_t ifr_prot0_en;
    uint32_t ifr_prot0_rdy;
    uint32_t ifr_cprot_en;
    uint32_t ifr_cprot_rdy;
    uint32_t rc_central_cfg1;   /* 0x1C00D004 bit0 = srclken_rc_en */
    uint32_t pmif_init_done;
    uint32_t pmif_swinf2_sta;

    uint16_t hwcid_mt6363;      /* slv=4 */
    uint16_t hwcid_mt6368;      /* slv=5 */
    uint16_t hwcid_mt6685;      /* slv=9 */
    uint8_t  dcxo_digclk_elr;   /* MT6685 0x7f4: bit0=1 -> 52M, 0 -> 26M */
    uint8_t  spmi_ok_mask;      /* bit0=6363, bit1=6368, bit2=6685, bit3=dcxo */

    uint8_t  vrfio18_en;
    uint8_t  vrfio18_mon;
    uint8_t  vrfio18_swop;
    uint8_t  vrfio18_vosel;

    uint8_t  vcn13_en;
    uint8_t  vcn13_mon;
    uint8_t  vcn13_swop;
    uint8_t  vcn13_vosel;

    uint8_t  vcn33_1_en;
    uint8_t  vcn33_1_mon;
    uint8_t  vcn33_1_vosel;
    uint8_t  vcn33_2_en;

    uint8_t  vcn33_2_mon;
    uint8_t  vcn33_2_vosel;
    uint8_t  vant18_en;
    uint8_t  vant18_mon;

    uint8_t  vant18_vosel;
    uint8_t  is_rc_mode;
    uint8_t  clock_type_detected; /* 0=26M_COTMS, 1=52M_COTMS */
    uint8_t  pad0;

    uint32_t gpio_mode24;
    uint32_t gpio_mode25;
    uint32_t iocfg_drv3;
    uint32_t iocfg_pupd0;

    uint32_t conn_hw_ver;       /* 0 if CONN domain off/protected */
    uint32_t adie_hw_ver;
    uint32_t beif_key;
    uint32_t beif_ver;

    uint8_t  emi_wmmcu_date[16]; /* 0x8cc00000 + 0x4954E0 */
    uint32_t emi_ro_wifi_head[4]; /* 0x8cc00000 + 0x380000 */
    uint32_t state_flags;       /* bit0=conninfra_up, bit1=wifi_mcu_up, bit2=wifi_fw_up */
    uint32_t pad1;
};

/* SEL 1: Stage 1 ConnInfra + MT6637 A-Die bring-up */
struct Tb375fcConnSysStage1In {
    uint32_t radio_mask;        /* e.g. (1u << 3) for Wi-Fi, (1u << 0) for BT */
    uint32_t force_clock_type;  /* 0xFFFFFFFF = auto-detect from DCXO_DIGCLK_ELR */
    uint32_t flags;             /* bit0=keep_sw_ldos_on (default 1 when 0) */
    uint32_t pad;
};

struct Tb375fcConnSysStage1Out {
    int32_t  status;            /* 0 = OK, negative = failed step */
    uint32_t failed_step;
    uint32_t clock_type_used;
    uint32_t is_rc_mode;
    uint32_t spm_conn_pwr_con;
    uint32_t ifr_prot0_en;
    uint32_t ifr_cprot_en;
    uint32_t conn_hw_ver;
    uint32_t adie_hw_ver;
    uint32_t sysram_adie_id;
    int64_t  smc_init_atf;
    int64_t  smc_poll_chipid;
    int64_t  smc_emi_remap;
    int64_t  smc_ddie_cfg;
    int64_t  smc_spi_mst;
    int64_t  smc_adie_cfg;
    int64_t  smc_afe_cal;
    int64_t  smc_subsys_pll;
    int64_t  smc_status_upd;
    int64_t  smc_low_pwr;
    int64_t  smc_thermal;
};

/* SEL 2: Stage 3 Wi-Fi soc7_0 MCU power-on + PCCIF_ON + WFDMA probe */
struct Tb375fcConnSysWifiOnIn {
    uint32_t flags;             /* bit0=skip_stage1_if_up, bit1=enable_fwdl_mode */
    uint32_t pad;
};

struct Tb375fcConnSysWifiOnOut {
    int32_t  status;            /* 0 = OK */
    uint32_t failed_step;
    uint32_t conn_ip_ver;       /* 0x18011000 expect 0x02050400 */
    uint32_t conn_rdy_reg;      /* 0x18001210 expect bit16=1 */
    uint32_t pwr_states;        /* 0x18060A10 expect bit30=1 */
    uint32_t wf_slp_sta;        /* 0x18001444 expect bit29=0 & bit31=0 */
    uint32_t slpprot_on_sta;    /* 0x184C300C expect bit23=0 & bit21=0 */
    uint32_t wf_ip_ver;         /* 0x184B0010 expect 0x02040200 */
    uint32_t monflg_out;        /* 0x18060B10 expect 0x00001D1E */
    uint32_t romcode_idx;       /* 0x184C1604 (diagnostic if monflg fails) */
    uint32_t wf_lpctl;          /* 0x18060010 */
    uint32_t conn_on_misc;      /* 0x180600F0 */
    uint32_t wfdma_glo_cfg;     /* 0x18024208 */
    uint32_t wfdma_int_sta;     /* 0x18024200 */
    uint32_t wfdma_int_ena;     /* 0x18024204 */
    uint32_t wfdma_pda_cfg;
    int64_t  smc_pccif_on;
    int64_t  smc_fwdl_mode;
};

/* SEL 3: Strictly READ-ONLY UFS LUN block reader */
struct Tb375fcConnSysUfsReadIn {
    uint64_t user_buf;
    uint32_t lba;
    uint32_t count;
    uint32_t lun;
    uint32_t pad;
};

struct Tb375fcConnSysUfsReadOut {
    int32_t  rc;
    uint32_t bytes_read;
};

/* SEL 4 & 5: Shared EMI (0x8cc00000, 13 MiB) and Wi-Fi DMA Pool (0x8d900000, 20 MiB) R/W */
struct Tb375fcConnSysMemRwIn {
    uint64_t user_buf;
    uint32_t offset;
    uint32_t size;
    uint32_t is_write;
    uint32_t pad;
};

struct Tb375fcConnSysMemRwOut {
    int32_t  rc;
    uint32_t bytes_done;
};

/* SEL 6: Bounded ConnSys MMIO or SPMI register op (root-only) */
enum Tb375fcConnSysRegKind {
    CONNSYS_REG_MMIO_READ32  = 0,
    CONNSYS_REG_MMIO_WRITE32 = 1,
    CONNSYS_REG_SPMI_READ8   = 2,
    CONNSYS_REG_SPMI_WRITE8  = 3,
};

struct Tb375fcConnSysRegOpIn {
    uint32_t kind;
    uint32_t addr;
    uint32_t value;
    uint32_t spmi_slvid;
};

struct Tb375fcConnSysRegOpOut {
    int32_t  rc;
    uint32_t value;
};

/* SEL 7: Bounded ConnSys / WLAN / BT BL31 ATF SMC call (root-only) */
struct Tb375fcConnSysSmcIn {
    uint64_t fid;
    uint64_t x1;
    uint64_t x2;
    uint64_t x3;
    uint64_t x4;
    uint64_t x5;
};

struct Tb375fcConnSysSmcOut {
    uint64_t a0;
    uint64_t a1;
    uint64_t a2;
    uint64_t a3;
};

/* =========================================================================
 * Native MT6897 Wi-Fi 6 (soc7_0) STA Control & Diagnostics (SEL 8..12)
 * ========================================================================= */

#define TB375FC_WIFI_MAX_SCAN_APS  48

enum Tb375fcWifiSecurity {
    WIFI_SEC_OPEN            = 0,
    WIFI_SEC_WEP             = 1,
    WIFI_SEC_WPA_TKIP        = 2,
    WIFI_SEC_WPA2_PSK        = 3,  /* CCMP-128, AKM 00-0F-AC:2 */
    WIFI_SEC_WPA2_PSK_SHA256 = 4,  /* CCMP-128, AKM 00-0F-AC:6 */
    WIFI_SEC_WPA3_SAE        = 5,
    WIFI_SEC_OTHER           = 6,
};

enum Tb375fcWifiState {
    WIFI_STATE_DOWN          = 0,
    WIFI_STATE_READY         = 1,  /* FW booted, BSS activated, idle */
    WIFI_STATE_SCANNING      = 2,
    WIFI_STATE_AUTH          = 3,
    WIFI_STATE_ASSOC         = 4,
    WIFI_STATE_EAPOL         = 5,
    WIFI_STATE_CONNECTED     = 6,
};

/* SEL 8: Full Wi-Fi bring-up (Stage1 + Stage3 + WFDMA0 + FW Ready + BSS Activate) */
struct Tb375fcWifiUpOut {
    int32_t  rc;
    uint32_t failed_step;
    uint32_t conn_on_misc;
    uint32_t wf_lpctl;
    uint32_t nic_cap_len;
    uint32_t adie_ver;
    uint8_t  mac[6];
    uint8_t  pad[2];
};

/* SEL 9: Scan 2.4GHz + 5GHz APs */
struct Tb375fcWifiScanEntry {
    uint8_t  bssid[6];
    uint8_t  channel;
    uint8_t  band;              /* 1 = 2.4GHz, 2 = 5GHz */
    int8_t   rssi;              /* dBm (negative) */
    uint8_t  security;          /* Tb375fcWifiSecurity */
    uint8_t  group_cipher;      /* 4=CCMP, 2=TKIP, 0=NONE */
    uint8_t  flags;             /* bit0=WMM, bit1=HT, bit2=VHT, bit3=HE, bit4=MFPR */
    uint16_t cap_info;
    uint16_t beacon_interval;
    uint8_t  ssid_len;
    uint8_t  pad[3];
    char     ssid[36];          /* NUL-terminated */
};

struct Tb375fcWifiScanIn {
    uint32_t flags;             /* bit0=cached_only (no new scan), bit1=2g_only */
    uint32_t dwell_ms;          /* 0 = default (60ms) */
};

struct Tb375fcWifiScanOut {
    int32_t  rc;
    uint32_t count;
    struct Tb375fcWifiScanEntry entries[TB375FC_WIFI_MAX_SCAN_APS];
};

/* SEL 10: Join an AP (OPEN or WPA2-PSK) and bridge onto en0 */
struct Tb375fcWifiJoinIn {
    char     ssid[36];
    char     passphrase[68];    /* empty string = OPEN network */
    uint8_t  bssid[6];          /* all-zero = auto-select strongest matching SSID */
    uint8_t  channel;           /* 0 = auto */
    uint8_t  band;              /* 0 = auto, 1 = 2.4G, 2 = 5G */
    uint32_t flags;             /* bit0=skip_autoscan */
};

struct Tb375fcWifiJoinOut {
    int32_t  rc;
    uint32_t failed_step;
    uint16_t status_code;       /* 802.11 Auth/Assoc status code if rejected */
    uint16_t assoc_id;
    uint8_t  bssid[6];
    uint8_t  channel;
    uint8_t  band;
    int8_t   rssi;
    uint8_t  security;
    uint8_t  eapol_stage;       /* 0=none, 1=M1, 2=M2, 3=M3, 4=M4+keys */
    uint8_t  keys_installed;
};

/* SEL 11: Live Wi-Fi status & packet counters */
struct Tb375fcWifiStatusOut {
    uint32_t state;             /* Tb375fcWifiState */
    uint32_t fw_ready;
    uint32_t connected;
    uint32_t keys_installed;
    uint8_t  own_mac[6];
    uint8_t  bssid[6];
    char     ssid[36];
    uint8_t  channel;
    uint8_t  band;
    int8_t   rssi;
    uint8_t  security;
    uint16_t assoc_id;
    uint8_t  eapol_stage;
    uint8_t  last_deauth_reason;
    uint64_t tx_packets;
    uint64_t tx_bytes;
    uint64_t tx_dropped;
    uint64_t tx_freed;
    uint64_t rx_packets;
    uint64_t rx_bytes;
    uint64_t rx_eapol;
    uint64_t rx_mgmt;
    uint64_t rx_events;
    uint64_t rx_errors;
    uint32_t tx0_cidx;
    uint32_t tx0_didx;
    uint32_t tx15_cidx;
    uint32_t tx15_didx;
    uint32_t rx0_cidx;
    uint32_t rx0_didx;
    uint32_t rx2_cidx;
    uint32_t rx2_didx;
    uint32_t rx3_cidx;
    uint32_t rx3_didx;
    uint32_t free_tokens;
    uint32_t conn_on_misc;
};

/* SEL 12: Disconnect from current AP */
struct Tb375fcWifiDownOut {
    int32_t  rc;
    uint32_t prev_state;
};

#endif /* TB375FC_CONNSYS_ABI_H */
