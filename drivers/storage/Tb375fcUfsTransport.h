/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#ifndef TB375FC_UFS_TRANSPORT_H
#define TB375FC_UFS_TRANSPORT_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define TB375FC_UFS_OK        0u
#define TB375FC_UFS_NOT_READY 1u
#define TB375FC_UFS_BUSY      2u
#define TB375FC_UFS_DMA       3u
#define TB375FC_UFS_POLICY    4u
#define TB375FC_UFS_STATE     5u
#define TB375FC_UFS_TIMEOUT   6u
#define TB375FC_UFS_RESPONSE  7u
#define TB375FC_UFS_RESTORE   8u
#define TB375FC_UFS_HCE       9u
#define TB375FC_UFS_LINK      10u
#define TB375FC_UFS_RSR       11u
#define TB375FC_UFS_NOP       12u
#define TB375FC_UFS_FDEVINIT  13u
#define TB375FC_UFS_BLOCK     4096u

struct tb375fc_ufs_diag {
    uint32_t cap;
    uint32_t ver;
    uint32_t hcs_before;
    uint32_t hce_before;
    uint32_t ba_before;
    uint32_t bau_before;
    uint32_t dbr_before;
    uint32_t rsr_before;
    uint32_t ao_cg;
    uint32_t ao_rst;
    uint32_t pdn_cg;
    uint32_t pdn_rst;
    uint32_t xoufs_140_0;
    uint32_t refclk_144_0;
    uint32_t opt_160_0;
    uint32_t extreg_2100;
    uint32_t mphyctrl_2200;
    uint32_t ipver_2240;
    uint32_t phy_8c_0;
    uint32_t phy_9c_0;
    uint32_t phy_ac_0;
    uint32_t phy_b044_0;
    uint32_t phy_a19c_0;
    uint32_t phy_b144_0;
    uint32_t phy_8c_1;
    uint32_t phy_9c_1;
    uint32_t phy_ac_1;
    uint32_t phy_b044_1;
    uint32_t pmif_init;
    int32_t  vemc_0;
    int32_t  vemc_1;
    int32_t  vufs12_0;
    int32_t  vufs18_0;
    uint32_t refclk_144_1;
    uint32_t stage_used;
    uint32_t stage_a_uarg2;
    uint32_t stage_a_hcs;
    int32_t  smc_sram;
    int32_t  smc_devpwr;
    int32_t  smc_va09;
    int32_t  smc_mtcmos0;
    int32_t  smc_mtcmos1;
    int32_t  smc_refpre;
    int32_t  smc_refpost;
    int32_t  smc_rst0;
    int32_t  smc_rst1;
    uint32_t hce_after;
    uint32_t hcs_after_hce;
    int32_t  dme_lpm;
    int32_t  dme_lcc;
    int32_t  dme_spc;
    uint32_t link_uarg2;
    uint32_t link_tries;
    uint32_t hcs_after_link;
    uint32_t rsr_after;
    int32_t  nop_rc;
    uint32_t nop_tries;
    int32_t  fdevinit_rc;
    uint32_t fdevinit_polls;
    int32_t  tur_rc;
    uint32_t tur_tries;
};

int tb375fc_ufs_init(struct tb375fc_ufs_diag *diag);
void tb375fc_ufs_get_last_cmd_diag(uint32_t out[6]);
int tb375fc_ufs_link_hs(int32_t out[6]);
#define TB375FC_UFS_MAX_BLOCKS 16u
int tb375fc_ufs_read_block(uint32_t lba, void *out);
int tb375fc_ufs_read_blocks(uint32_t lba, uint32_t count, void *out);
int tb375fc_ufs_read_lun_block(uint8_t lun, uint32_t lba, void *out);

/* Write-once bounded write window for P2 persistent Data / PersistOverlay.
 * Writes remain 100% disabled until explicitly unlocked once for a verified LUN2 range. */
int tb375fc_ufs_set_write_window(uint8_t lun, uint32_t first_lba, uint32_t blocks);
int tb375fc_ufs_write_blocks(uint32_t lba, uint32_t count, const void *in, int fua);
int tb375fc_ufs_sync_cache(void);

#ifdef __cplusplus
}
#endif
#endif
