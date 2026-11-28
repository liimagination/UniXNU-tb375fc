/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/*
 * tb375fc-connsys-ctl.c — Interactive Userspace Bring-Up & RO UFS Firmware
 *                         Extraction Tool for MT6897 Onboard ConnSys
 *
 * Talks to kernel IOService "Tb375fcConnSys" (root-only IOUserClient).
 */

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "Tb375fcConnSysABI.h"

static io_connect_t open_connsys(void) {
    CFMutableDictionaryRef match = IOServiceMatching(TB375FC_CONNSYS_SERVICE_NAME);
    if (!match) { fprintf(stderr, "IOServiceMatching failed\n"); return MACH_PORT_NULL; }
    io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, match);
    if (svc == IO_OBJECT_NULL) {
        fprintf(stderr, "Service \"%s\" not found (is connsys=1 passed in boot-args?)\n",
                TB375FC_CONNSYS_SERVICE_NAME);
        return MACH_PORT_NULL;
    }
    io_connect_t conn = MACH_PORT_NULL;
    kern_return_t kr = IOServiceOpen(svc, mach_task_self(), 0, &conn);
    IOObjectRelease(svc);
    if (kr != KERN_SUCCESS || conn == MACH_PORT_NULL) {
        fprintf(stderr, "IOServiceOpen failed: kr=0x%x (must run as root)\n", kr);
        return MACH_PORT_NULL;
    }
    return conn;
}

static int ufs_read_blocks(io_connect_t conn, uint32_t lun, uint32_t lba, uint32_t count, void *buf) {
    uint8_t *dst = (uint8_t *)buf;
    uint32_t done = 0;
    while (done < count) {
        uint32_t n = count - done;
        if (n > 16u) n = 16u;
        struct Tb375fcConnSysUfsReadIn in = {
            .user_buf = (uint64_t)(uintptr_t)(dst + done * 4096u),
            .lba      = lba + done,
            .count    = n,
            .lun      = lun,
            .pad      = 0,
        };
        struct Tb375fcConnSysUfsReadOut out = {0};
        size_t osz = sizeof(out);
        kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_UFS_READ_RO,
                                                     &in, sizeof(in), &out, &osz);
        if (kr != KERN_SUCCESS || out.rc != 0 || out.bytes_read != n * 4096u) {
            fprintf(stderr, "ufs_read_blocks(lun=%u lba=%u n=%u) failed: kr=0x%x rc=%d bytes=%u\n",
                    lun, lba + done, n, kr, out.rc, out.bytes_read);
            return -1;
        }
        done += n;
    }
    return 0;
}

static int ufs_read_bytes(io_connect_t conn, uint32_t lun, uint64_t byte_off, void *out_buf, size_t len) {
    uint8_t blk[65536];
    uint8_t *dst = (uint8_t *)out_buf;
    size_t pos = 0;
    while (pos < len) {
        uint64_t cur = byte_off + pos;
        uint32_t lba = (uint32_t)(cur / 4096ULL);
        uint32_t off_in_blk = (uint32_t)(cur % 4096ULL);
        size_t rem = len - pos;
        uint32_t max_bytes = 65536u - off_in_blk;
        size_t take = (rem < max_bytes) ? rem : max_bytes;
        uint32_t nblks = (uint32_t)((off_in_blk + take + 4095u) / 4096u);
        if (nblks > 16u) nblks = 16u;
        if (ufs_read_blocks(conn, lun, lba, nblks, blk) != 0) return -1;
        memcpy(dst + pos, blk + off_in_blk, take);
        pos += take;
    }
    return 0;
}

static int cmd_probe(io_connect_t conn) {
    struct Tb375fcConnSysProbeOut p = {0};
    size_t osz = sizeof(p);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_RO_PROBE, NULL, 0, &p, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "RO_PROBE failed: 0x%x\n", kr); return 1; }

    char date_str[17];
    for (int i = 0; i < 16; ++i) {
        uint8_t c = p.emi_wmmcu_date[i];
        date_str[i] = (c >= 32 && c < 127) ? (char)c : '.';
    }
    date_str[16] = '\0';

    printf("=== TB375FC MT6897 ConnSys RO Probe ===\n");
    printf("SPM       : PWR_CFG_EN=0x%08x CONN_PWR_CON=0x%08x (ON=%u ON2=%u ACK=%u ACK2=%u ISO=%u RST_B=%u)\n",
           p.spm_pwr_cfg_en, p.spm_conn_pwr_con,
           !!(p.spm_conn_pwr_con & (1u<<2)), !!(p.spm_conn_pwr_con & (1u<<3)),
           !!(p.spm_conn_pwr_con & (1u<<30)), !!(p.spm_conn_pwr_con & (1u<<31)),
           !!(p.spm_conn_pwr_con & (1u<<1)), !!(p.spm_conn_pwr_con & (1u<<0)));
    printf("IFRBUS_AO : prot0(en=0x%08x rdy=0x%08x) connsys0(en=0x%08x rdy=0x%08x)\n",
           p.ifr_prot0_en, p.ifr_prot0_rdy, p.ifr_cprot_en, p.ifr_cprot_rdy);
    printf("SRCLKEN_RC: CENTRAL_CFG1=0x%08x (is_rc_mode=%u)\n", p.rc_central_cfg1, p.is_rc_mode);
    printf("SPMI-M    : init_done=0x%08x swinf2_sta=0x%08x ok_mask=0x%x\n",
           p.pmif_init_done, p.pmif_swinf2_sta, p.spmi_ok_mask);
    printf("PMIC IDs  : MT6363=0x%04x MT6368=0x%04x MT6685=0x%04x DCXO_ELR=0x%02x (%s co-clock)\n",
           p.hwcid_mt6363, p.hwcid_mt6368, p.hwcid_mt6685, p.dcxo_digclk_elr,
           p.clock_type_detected ? "52M" : "26M");
    printf("MT6363 LDO: VRFIO18(en=%02x mon=%02x swop=%02x vosel=%02x) VCN13(en=%02x mon=%02x swop=%02x vosel=%02x)\n",
           p.vrfio18_en, p.vrfio18_mon, p.vrfio18_swop, p.vrfio18_vosel,
           p.vcn13_en, p.vcn13_mon, p.vcn13_swop, p.vcn13_vosel);
    printf("MT6368 LDO: VCN33_1(en=%02x mon=%02x vs=%02x) VCN33_2(en=%02x mon=%02x vs=%02x) VANT18(en=%02x mon=%02x vs=%02x)\n",
           p.vcn33_1_en, p.vcn33_1_mon, p.vcn33_1_vosel,
           p.vcn33_2_en, p.vcn33_2_mon, p.vcn33_2_vosel,
           p.vant18_en, p.vant18_mon, p.vant18_vosel);
    printf("PINMUX    : MODE24=0x%08x MODE25=0x%08x DRV3=0x%08x PUPD0=0x%08x\n",
           p.gpio_mode24, p.gpio_mode25, p.iocfg_drv3, p.iocfg_pupd0);
    printf("CONNSYS   : CONN_HW_VER=0x%08x ADIE_HW_VER=0x%08x state_flags=0x%x\n",
           p.conn_hw_ver, p.adie_hw_ver, p.state_flags);
    printf("EMI       : WMMCU_DATE=\"%s\" RO_WIFI[0..3]=%08x %08x %08x %08x BEIF=%08x/%08x\n",
           date_str, p.emi_ro_wifi_head[0], p.emi_ro_wifi_head[1],
           p.emi_ro_wifi_head[2], p.emi_ro_wifi_head[3], p.beif_key, p.beif_ver);
    return 0;
}

static int cmd_stage1(io_connect_t conn, int argc, char **argv) {
    struct Tb375fcConnSysStage1In in = {
        .radio_mask       = (argc >= 1) ? (uint32_t)strtoul(argv[0], NULL, 0) : (1u << 3),
        .force_clock_type = (argc >= 2) ? (uint32_t)strtoul(argv[1], NULL, 0) : 0xFFFFFFFFu,
        .flags            = (argc >= 3) ? (uint32_t)strtoul(argv[2], NULL, 0) : 0u,
        .pad              = 0,
    };
    struct Tb375fcConnSysStage1Out out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_STAGE1_UP,
                                                 &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "STAGE1_UP ioctl failed: 0x%x\n", kr); return 1; }
    printf("=== Stage 1 ConnInfra Bring-Up (status=%d failed_step=%u) ===\n", out.status, out.failed_step);
    printf("clk=%u is_rc=%u SPM_PWR=0x%08x IFR(p0=0x%08x c0=0x%08x)\n",
           out.clock_type_used, out.is_rc_mode, out.spm_conn_pwr_con, out.ifr_prot0_en, out.ifr_cprot_en);
    printf("CONN_HW_VER=0x%08x ADIE_HW_VER=0x%08x SYSRAM_ADIE=0x%08x\n",
           out.conn_hw_ver, out.adie_hw_ver, out.sysram_adie_id);
    printf("SMCs: init=%lld poll=%lld remap=%lld ddie=%lld spi=%lld adie=%lld afe=%lld pll=%lld upd=%lld lp=%lld therm=%lld\n",
           (long long)out.smc_init_atf, (long long)out.smc_poll_chipid, (long long)out.smc_emi_remap,
           (long long)out.smc_ddie_cfg, (long long)out.smc_spi_mst, (long long)out.smc_adie_cfg,
           (long long)out.smc_afe_cal, (long long)out.smc_subsys_pll, (long long)out.smc_status_upd,
           (long long)out.smc_low_pwr, (long long)out.smc_thermal);
    return out.status == 0 ? 0 : 1;
}

static int cmd_wifi_on(io_connect_t conn, int argc, char **argv) {
    struct Tb375fcConnSysWifiOnIn in = {
        .flags = (argc >= 1) ? (uint32_t)strtoul(argv[0], NULL, 0) : 0u,
        .pad   = 0,
    };
    struct Tb375fcConnSysWifiOnOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_STAGE3_WIFI_ON,
                                                 &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "STAGE3_WIFI_ON ioctl failed: 0x%x\n", kr); return 1; }
    printf("=== Stage 3 Wi-Fi soc7_0 MCU Power-On (status=%d failed_step=%u) ===\n", out.status, out.failed_step);
    printf("CONN_IP=0x%08x CONN_RDY=0x%08x PWR_STA=0x%08x WF_SLP=0x%08x SLPPROT=0x%08x\n",
           out.conn_ip_ver, out.conn_rdy_reg, out.pwr_states, out.wf_slp_sta, out.slpprot_on_sta);
    printf("WF_IP_VER=0x%08x (expect 0x02040200) MONFLG_OUT=0x%08x (expect 0x00001D1E)\n",
           out.wf_ip_ver, out.monflg_out);
    printf("PCCIF_ON=%lld FWDL_MODE=%lld WF_LPCTL=0x%08x CONN_ON_MISC=0x%08x\n",
           (long long)out.smc_pccif_on, (long long)out.smc_fwdl_mode, out.wf_lpctl, out.conn_on_misc);
    printf("WFDMA0: GLO_CFG=0x%08x INT_STA=0x%08x INT_ENA=0x%08x\n",
           out.wfdma_glo_cfg, out.wfdma_int_sta, out.wfdma_int_ena);
    return out.status == 0 ? 0 : 1;
}

/* =========================================================================
 * Strictly Read-Only UFS GPT & Android Dynamic Partition (super LP) Parser
 * ========================================================================= */

static int cmd_ufs_gpt(io_connect_t conn, uint32_t lun) {
    uint8_t hdr[4096];
    if (ufs_read_blocks(conn, lun, 1, 1, hdr) != 0) return 1;
    if (memcmp(hdr, "EFI PART", 8) != 0) {
        fprintf(stderr, "LUN%u LBA1 is not EFI PART (got %02x%02x%02x%02x)\n",
                lun, hdr[0], hdr[1], hdr[2], hdr[3]);
        return 1;
    }
    uint64_t part_lba = 0;
    uint32_t num_parts = 0, part_sz = 0;
    memcpy(&part_lba,  hdr + 72, 8);
    memcpy(&num_parts, hdr + 80, 4);
    memcpy(&part_sz,   hdr + 84, 4);
    printf("GPT LUN%u: part_lba=%llu num_parts=%u entry_sz=%u\n",
           (unsigned)lun, (unsigned long long)part_lba, num_parts, part_sz);
    if (part_sz < 128 || num_parts > 256) return 1;

    uint32_t total_bytes = num_parts * part_sz;
    uint32_t nblks = (total_bytes + 4095u) / 4096u;
    uint8_t *tbl = (uint8_t *)malloc(nblks * 4096u);
    if (!tbl) return 1;
    if (ufs_read_blocks(conn, lun, (uint32_t)part_lba, nblks, tbl) != 0) { free(tbl); return 1; }

    for (uint32_t i = 0; i < num_parts; ++i) {
        const uint8_t *e = tbl + i * part_sz;
        uint64_t t0 = 0, t1 = 0, first = 0, last = 0;
        memcpy(&t0, e, 8); memcpy(&t1, e + 8, 8);
        if (t0 == 0 && t1 == 0) continue;
        memcpy(&first, e + 32, 8); memcpy(&last, e + 40, 8);
        char name[40] = {0};
        for (int c = 0; c < 36; ++c) {
            uint16_t u = (uint16_t)(e[56 + c*2] | (e[56 + c*2 + 1] << 8));
            if (u == 0) break;
            name[c] = (u < 127) ? (char)u : '?';
        }
        uint64_t blocks = (last >= first) ? (last - first + 1) : 0;
        printf("  [%2u] %-24s lba=%9llu..%9llu (%7.2f MiB)\n",
               i, name, (unsigned long long)first, (unsigned long long)last,
               (double)blocks * 4096.0 / (1024.0 * 1024.0));
    }
    free(tbl);
    return 0;
}

struct __attribute__((packed)) LpGeometry {
    uint32_t magic;
    uint32_t struct_size;
    uint8_t  checksum[32];
    uint32_t metadata_max_size;
    uint32_t metadata_slot_count;
    uint32_t logical_block_size;
};

struct __attribute__((packed)) LpTableDesc {
    uint32_t offset;
    uint32_t num_entries;
    uint32_t entry_size;
};

struct __attribute__((packed)) LpHeader {
    uint32_t magic;
    uint16_t major_version;
    uint16_t minor_version;
    uint32_t header_size;
    uint8_t  header_checksum[32];
    uint32_t tables_size;
    uint8_t  tables_checksum[32];
    struct LpTableDesc partitions;
    struct LpTableDesc extents;
    struct LpTableDesc groups;
    struct LpTableDesc block_devices;
};

struct __attribute__((packed)) LpPartition {
    char     name[36];
    uint32_t attributes;
    uint32_t first_extent_index;
    uint32_t num_extents;
    uint32_t group_index;
};

struct __attribute__((packed)) LpExtent {
    uint64_t num_sectors;
    uint32_t target_type;
    uint64_t target_data;
    uint32_t target_source;
};

static int cmd_ufs_lp(io_connect_t conn, uint32_t super_lba) {
    uint8_t geo_blk[4096];
    if (ufs_read_blocks(conn, 2, super_lba + 1, 1, geo_blk) != 0) return 1;
    struct LpGeometry geo;
    memcpy(&geo, geo_blk, sizeof(geo));
    if (geo.magic != 0x616C4467u) {
        fprintf(stderr, "No LP geometry magic at lba %u (got 0x%08x)\n", super_lba + 1, geo.magic);
        return 1;
    }
    printf("LP Geometry: max_meta=%u slots=%u blk=%u\n",
           geo.metadata_max_size, geo.metadata_slot_count, geo.logical_block_size);

    uint32_t meta_blks = (geo.metadata_max_size + 4095u) / 4096u;
    if (meta_blks == 0 || meta_blks > 64) meta_blks = 16;
    uint8_t *meta = (uint8_t *)malloc(meta_blks * 4096u);
    if (!meta) return 1;
    /* Primary slot 0 metadata starts at super_byte_offset + 12288 = super_lba + 3 */
    if (ufs_read_blocks(conn, 2, super_lba + 3, meta_blks, meta) != 0) { free(meta); return 1; }

    struct LpHeader hdr;
    memcpy(&hdr, meta, sizeof(hdr));
    if (hdr.magic != 0x414C5030u) {
        fprintf(stderr, "No LP header magic at lba %u (got 0x%08x)\n", super_lba + 3, hdr.magic);
        free(meta);
        return 1;
    }
    printf("LP Header v%u.%u: hdr_sz=%u tbl_sz=%u parts=%u extents=%u\n",
           hdr.major_version, hdr.minor_version, hdr.header_size, hdr.tables_size,
           hdr.partitions.num_entries, hdr.extents.num_entries);

    const uint8_t *tbl_base = meta + hdr.header_size;
    for (uint32_t i = 0; i < hdr.partitions.num_entries; ++i) {
        struct LpPartition p;
        memcpy(&p, tbl_base + hdr.partitions.offset + i * hdr.partitions.entry_size, sizeof(p));
        char pname[37];
        memcpy(pname, p.name, 36);
        pname[36] = '\0';
        printf("  LP %-24s extents=%u:", pname, p.num_extents);
        for (uint32_t e = 0; e < p.num_extents; ++e) {
            struct LpExtent ext;
            memcpy(&ext, tbl_base + hdr.extents.offset + (p.first_extent_index + e) * hdr.extents.entry_size, sizeof(ext));
            uint64_t lba_start = (uint64_t)super_lba + (ext.target_data / 8ULL);
            uint64_t lba_cnt   = ext.num_sectors / 8ULL;
            printf(" [type=%u lba=%llu +%llu (%.2f MiB)]",
                   ext.target_type, (unsigned long long)lba_start,
                   (unsigned long long)lba_cnt, (double)ext.num_sectors * 512.0 / (1024.0 * 1024.0));
        }
        printf("\n");
    }
    free(meta);
    return 0;
}

static int cmd_ufs_fsinfo(io_connect_t conn, uint32_t part_lba) {
    uint8_t blk[4096];
    if (ufs_read_blocks(conn, 2, part_lba, 1, blk) != 0) return 1;
    uint16_t ext4_magic = (uint16_t)(blk[0x438] | (blk[0x439] << 8));
    uint32_t erofs_magic = 0;
    memcpy(&erofs_magic, blk + 0x400, 4);
    printf("Partition @ LBA %u: ext4_magic=0x%04x erofs_magic=0x%08x\n",
           part_lba, ext4_magic, erofs_magic);
    if (ext4_magic == 0xEF53u) {
        uint32_t log_bs = 0, inodes_pg = 0, b_pg = 0;
        uint16_t inode_sz = 0, desc_sz = 0;
        memcpy(&log_bs, blk + 0x418, 4);
        memcpy(&b_pg, blk + 0x420, 4);
        memcpy(&inodes_pg, blk + 0x428, 4);
        memcpy(&inode_sz, blk + 0x458, 2);
        memcpy(&desc_sz, blk + 0x4FE, 2);
        printf("  EXT4: blk_sz=%u b_pg=%u inodes_pg=%u inode_sz=%u desc_sz=%u\n",
               1024u << log_bs, b_pg, inodes_pg, inode_sz, desc_sz ? desc_sz : 32u);
    } else if (erofs_magic == 0xE0F5E1E2u) {
        uint8_t blkszbits = blk[0x40C];
        uint16_t root_nid = 0;
        uint32_t meta_blk = 0, blocks = 0, feat_incompat = 0;
        memcpy(&root_nid, blk + 0x40E, 2);
        memcpy(&blocks, blk + 0x424, 4);
        memcpy(&meta_blk, blk + 0x428, 4);
        memcpy(&feat_incompat, blk + 0x450, 4);
        printf("  EROFS: blk_sz=%u root_nid=%u meta_blkaddr=%u blocks=%u feat_incompat=0x%08x\n",
               1u << blkszbits, root_nid, meta_blk, blocks, feat_incompat);
    }
    return 0;
}

/* =========================================================================
 * Built-in Read-Only EXT4 + EROFS Directory Lister & File Extractor
 * ========================================================================= */

static int lz4_decompress_block(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_cap) {
    size_t sp = 0, dp = 0;
    while (sp < src_len) {
        uint8_t tok = src[sp++];
        size_t lit_len = tok >> 4;
        if (lit_len == 15) {
            uint8_t b;
            do { if (sp >= src_len) return -1; b = src[sp++]; lit_len += b; } while (b == 255);
        }
        if (sp + lit_len > src_len || dp + lit_len > dst_cap) return -1;
        memcpy(dst + dp, src + sp, lit_len);
        sp += lit_len; dp += lit_len;
        if (sp >= src_len) break;
        if (sp + 2 > src_len) return -1;
        uint16_t off = (uint16_t)(src[sp] | (src[sp + 1] << 8));
        sp += 2;
        if (off == 0 || off > dp) return -1;
        size_t mlen = (tok & 0x0Fu) + 4;
        if ((tok & 0x0Fu) == 15) {
            uint8_t b;
            do { if (sp >= src_len) return -1; b = src[sp++]; mlen += b; } while (b == 255);
        }
        if (dp + mlen > dst_cap) return -1;
        for (size_t i = 0; i < mlen; ++i) dst[dp + i] = dst[dp - off + i];
        dp += mlen;
    }
    return (int)dp;
}

/* EXT4 minimal RO reader (4K blocks, extent tree depth 0/1) */
struct Ext4Ctx {
    io_connect_t conn;
    uint64_t part_off;
    uint32_t blk_sz;
    uint32_t inodes_pg;
    uint32_t inode_sz;
    uint32_t desc_sz;
};

static int ext4_init(io_connect_t conn, uint32_t part_lba, struct Ext4Ctx *c) {
    uint8_t sb[4096];
    if (ufs_read_blocks(conn, 2, part_lba, 1, sb) != 0) return -1;
    uint16_t magic = (uint16_t)(sb[0x438] | (sb[0x439] << 8));
    if (magic != 0xEF53u) return -1;
    uint32_t log_bs = 0;
    uint16_t isz = 0, dsz = 0;
    memcpy(&log_bs, sb + 0x418, 4);
    memcpy(&c->inodes_pg, sb + 0x428, 4);
    memcpy(&isz, sb + 0x458, 2);
    memcpy(&dsz, sb + 0x4FE, 2);
    c->conn     = conn;
    c->part_off = (uint64_t)part_lba * 4096ULL;
    c->blk_sz   = 1024u << log_bs;
    c->inode_sz = isz ? isz : 128u;
    c->desc_sz  = dsz ? dsz : 32u;
    return 0;
}

static int ext4_read_inode(const struct Ext4Ctx *c, uint32_t ino, uint8_t *inode_buf) {
    if (ino == 0) return -1;
    uint32_t grp = (ino - 1) / c->inodes_pg;
    uint32_t idx = (ino - 1) % c->inodes_pg;
    uint64_t gdt_off = c->part_off + (c->blk_sz == 1024u ? 2048ULL : (uint64_t)c->blk_sz);
    uint8_t gd[64] = {0};
    if (ufs_read_bytes(c->conn, 2, gdt_off + (uint64_t)grp * c->desc_sz, gd, c->desc_sz < 64 ? c->desc_sz : 64) != 0)
        return -1;
    uint32_t itbl_lo = 0, itbl_hi = 0;
    memcpy(&itbl_lo, gd + 8, 4);
    if (c->desc_sz >= 64) memcpy(&itbl_hi, gd + 40, 4);
    uint64_t itbl_blk = ((uint64_t)itbl_hi << 32) | itbl_lo;
    uint64_t ino_off = c->part_off + itbl_blk * c->blk_sz + (uint64_t)idx * c->inode_sz;
    return ufs_read_bytes(c->conn, 2, ino_off, inode_buf, c->inode_sz > 256 ? 256 : c->inode_sz);
}

static int ext4_read_extent_node(const struct Ext4Ctx *c, const uint8_t *node, uint8_t *out, uint64_t file_sz) {
    uint16_t magic = 0, entries = 0, depth = 0;
    memcpy(&magic, node, 2);
    memcpy(&entries, node + 2, 2);
    memcpy(&depth, node + 6, 2);
    if (magic != 0xF30Au) return -1;
    if (depth == 0) {
        for (uint16_t i = 0; i < entries; ++i) {
            const uint8_t *ex = node + 12 + i * 12;
            uint32_t ee_block = 0, ee_start_lo = 0;
            uint16_t ee_len = 0, ee_start_hi = 0;
            memcpy(&ee_block, ex, 4);
            memcpy(&ee_len, ex + 4, 2);
            memcpy(&ee_start_hi, ex + 6, 2);
            memcpy(&ee_start_lo, ex + 8, 4);
            if (ee_len > 32768) continue; /* unwritten */
            uint64_t pblk = ((uint64_t)ee_start_hi << 32) | ee_start_lo;
            uint64_t foff = (uint64_t)ee_block * c->blk_sz;
            if (foff >= file_sz) continue;
            uint64_t nbytes = (uint64_t)ee_len * c->blk_sz;
            if (foff + nbytes > file_sz) nbytes = file_sz - foff;
            if (ufs_read_bytes(c->conn, 2, c->part_off + pblk * c->blk_sz, out + foff, (size_t)nbytes) != 0)
                return -1;
        }
        return 0;
    }
    uint8_t *child = (uint8_t *)malloc(c->blk_sz);
    if (!child) return -1;
    for (uint16_t i = 0; i < entries; ++i) {
        const uint8_t *ix = node + 12 + i * 12;
        uint32_t leaf_lo = 0;
        uint16_t leaf_hi = 0;
        memcpy(&leaf_lo, ix + 4, 4);
        memcpy(&leaf_hi, ix + 8, 2);
        uint64_t cblk = ((uint64_t)leaf_hi << 32) | leaf_lo;
        if (ufs_read_bytes(c->conn, 2, c->part_off + cblk * c->blk_sz, child, c->blk_sz) != 0) {
            free(child); return -1;
        }
        if (ext4_read_extent_node(c, child, out, file_sz) != 0) {
            free(child); return -1;
        }
    }
    free(child);
    return 0;
}

static uint8_t *ext4_read_file_data(const struct Ext4Ctx *c, uint32_t ino, uint64_t *sz_out, uint16_t *mode_out) {
    uint8_t inode[256] = {0};
    if (ext4_read_inode(c, ino, inode) != 0) return NULL;
    uint16_t mode = 0;
    uint32_t sz_lo = 0, sz_hi = 0, flags = 0;
    memcpy(&mode, inode + 0, 2);
    memcpy(&sz_lo, inode + 4, 4);
    memcpy(&flags, inode + 32, 4);
    if ((mode & 0xF000u) == 0x8000u) memcpy(&sz_hi, inode + 108, 4);
    uint64_t sz = ((uint64_t)sz_hi << 32) | sz_lo;
    if (mode_out) *mode_out = mode;
    if (sz_out) *sz_out = sz;
    if (sz == 0 || sz > 64ULL * 1024ULL * 1024ULL) return NULL;
    uint8_t *buf = (uint8_t *)calloc(1, (size_t)sz);
    if (!buf) return NULL;
    if (flags & 0x80000u) {
        if (ext4_read_extent_node(c, inode + 40, buf, sz) != 0) { free(buf); return NULL; }
    } else {
        /* Direct blocks 0..11 fallback */
        for (int b = 0; b < 12; ++b) {
            uint32_t pblk = 0;
            memcpy(&pblk, inode + 40 + b * 4, 4);
            if (!pblk) continue;
            uint64_t foff = (uint64_t)b * c->blk_sz;
            if (foff >= sz) break;
            size_t n = (sz - foff < c->blk_sz) ? (size_t)(sz - foff) : c->blk_sz;
            if (ufs_read_bytes(c->conn, 2, c->part_off + (uint64_t)pblk * c->blk_sz, buf + foff, n) != 0) {
                free(buf); return NULL;
            }
        }
    }
    return buf;
}

static uint32_t ext4_lookup(const struct Ext4Ctx *c, const char *path) {
    uint32_t cur_ino = 2; /* root inode */
    const char *p = path;
    while (*p == '/') ++p;
    while (*p) {
        const char *slash = strchr(p, '/');
        size_t clen = slash ? (size_t)(slash - p) : strlen(p);
        uint64_t dsz = 0;
        uint8_t *dbuf = ext4_read_file_data(c, cur_ino, &dsz, NULL);
        if (!dbuf) return 0;
        uint32_t next_ino = 0;
        size_t off = 0;
        while (off + 8 <= dsz) {
            uint32_t dino = 0; uint16_t rlen = 0; uint8_t nlen = dbuf[off + 6];
            memcpy(&dino, dbuf + off, 4);
            memcpy(&rlen, dbuf + off + 4, 2);
            if (rlen == 0) break;
            if (dino && nlen == clen && memcmp(dbuf + off + 8, p, clen) == 0) {
                next_ino = dino;
                break;
            }
            off += rlen;
        }
        free(dbuf);
        if (!next_ino) return 0;
        cur_ino = next_ino;
        if (!slash) break;
        p = slash + 1;
        while (*p == '/') ++p;
    }
    return cur_ino;
}

/* EROFS minimal RO reader (FLAT_PLAIN, FLAT_INLINE, and uncompressed/LZ4 blocks) */
struct ErofsCtx {
    io_connect_t conn;
    uint64_t part_off;
    uint32_t blk_sz;
    uint32_t blkszbits;
    uint32_t meta_blkaddr;
    uint16_t root_nid;
};

static int erofs_init(io_connect_t conn, uint32_t part_lba, struct ErofsCtx *c) {
    uint8_t sb[4096];
    if (ufs_read_blocks(conn, 2, part_lba, 1, sb) != 0) return -1;
    uint32_t magic = 0;
    memcpy(&magic, sb + 0x400, 4);
    if (magic != 0xE0F5E1E2u) return -1;
    c->conn      = conn;
    c->part_off  = (uint64_t)part_lba * 4096ULL;
    c->blkszbits = sb[0x40C];
    c->blk_sz    = 1u << c->blkszbits;
    memcpy(&c->root_nid, sb + 0x40E, 2);
    memcpy(&c->meta_blkaddr, sb + 0x428, 4);
    return 0;
}

struct ErofsInodeInfo {
    uint16_t mode;
    uint8_t  datalayout;
    uint8_t  version;
    uint32_t xattr_size;
    uint64_t size;
    uint32_t raw_blkaddr;
    uint64_t inode_off;
    uint32_t inode_sz;
};

static int erofs_read_inode(const struct ErofsCtx *c, uint64_t nid, struct ErofsInodeInfo *info) {
    uint64_t ioff = c->part_off + ((uint64_t)c->meta_blkaddr << c->blkszbits) + nid * 32ULL;
    uint8_t raw[64];
    if (ufs_read_bytes(c->conn, 2, ioff, raw, 64) != 0) return -1;
    uint16_t fmt = 0, xicnt = 0;
    memcpy(&fmt, raw, 2);
    memcpy(&xicnt, raw + 2, 2);
    info->version    = (uint8_t)(fmt & 1u);
    info->datalayout = (uint8_t)((fmt >> 1) & 7u);
    info->xattr_size = xicnt ? (12u + (xicnt - 1u) * 4u) : 0u;
    info->inode_off  = ioff;
    if (info->version == 0) {
        info->inode_sz = 32u;
        uint32_t sz32 = 0;
        memcpy(&info->mode, raw + 4, 2);
        memcpy(&sz32, raw + 8, 4);
        memcpy(&info->raw_blkaddr, raw + 16, 4);
        info->size = sz32;
    } else {
        info->inode_sz = 64u;
        memcpy(&info->mode, raw + 4, 2);
        memcpy(&info->size, raw + 8, 8);
        memcpy(&info->raw_blkaddr, raw + 16, 4);
    }
    return 0;
}

static uint8_t *erofs_read_data(const struct ErofsCtx *c, const struct ErofsInodeInfo *in, int *err_layout) {
    if (err_layout) *err_layout = in->datalayout;
    if (in->size == 0 || in->size > 32ULL * 1024ULL * 1024ULL) return NULL;
    uint8_t *buf = (uint8_t *)calloc(1, (size_t)in->size);
    if (!buf) return NULL;

    if (in->datalayout == 0 /* FLAT_PLAIN */) {
        if (ufs_read_bytes(c->conn, 2, c->part_off + (uint64_t)in->raw_blkaddr * c->blk_sz, buf, (size_t)in->size) != 0) {
            free(buf); return NULL;
        }
        return buf;
    }
    if (in->datalayout == 2 /* FLAT_INLINE */) {
        uint64_t full = (in->size / c->blk_sz) * c->blk_sz;
        uint64_t tail = in->size - full;
        if (full > 0) {
            if (ufs_read_bytes(c->conn, 2, c->part_off + (uint64_t)in->raw_blkaddr * c->blk_sz, buf, (size_t)full) != 0) {
                free(buf); return NULL;
            }
        }
        if (tail > 0) {
            uint64_t tail_off = in->inode_off + in->inode_sz + in->xattr_size;
            if (ufs_read_bytes(c->conn, 2, tail_off, buf + full, (size_t)tail) != 0) {
                free(buf); return NULL;
            }
        }
        return buf;
    }
    if (in->datalayout == 1 /* FLAT_COMPRESSION_LEGACY */ || in->datalayout == 3 /* FLAT_COMPRESSION */) {
        /* Read z_erofs_map_header (8 bytes) aligned to 4 after inode + xattr */
        uint64_t moff = (in->inode_off + in->inode_sz + in->xattr_size + 3ULL) & ~3ULL;
        uint8_t mhdr[8];
        if (ufs_read_bytes(c->conn, 2, moff, mhdr, 8) != 0) { free(buf); return NULL; }
        uint16_t advise = 0;
        memcpy(&advise, mhdr + 4, 2);
        bool compacted = (in->datalayout == 3) && !(advise & 1u); /* legacy=0 -> legacy 8B indexes */
        if (in->datalayout == 1 || (advise & 1u) == 0) {
            /* Legacy 8-byte z_erofs_vle_decompressed_index array starting at moff+8 (or compacted) */
            if (!compacted) {
                uint32_t lclusters = (uint32_t)((in->size + c->blk_sz - 1) / c->blk_sz);
                uint8_t *idx = (uint8_t *)malloc(lclusters * 8u);
                uint8_t *pblk_buf = (uint8_t *)malloc(c->blk_sz);
                if (!idx || !pblk_buf) { free(idx); free(pblk_buf); free(buf); return NULL; }
                if (ufs_read_bytes(c->conn, 2, moff + 8, idx, lclusters * 8u) != 0) {
                    free(idx); free(pblk_buf); free(buf); return NULL;
                }
                for (uint32_t l = 0; l < lclusters; ++l) {
                    uint16_t di_advise = 0, di_clusterofs = 0;
                    uint32_t di_u = 0;
                    memcpy(&di_advise, idx + l * 8, 2);
                    memcpy(&di_clusterofs, idx + l * 8 + 2, 2);
                    memcpy(&di_u, idx + l * 8 + 4, 4);
                    uint8_t type = (uint8_t)(di_advise & 3u);
                    if (type == 0 /* PLAIN */) {
                        uint64_t foff = (uint64_t)l * c->blk_sz + di_clusterofs;
                        size_t n = (in->size - foff < c->blk_sz) ? (size_t)(in->size - foff) : c->blk_sz;
                        ufs_read_bytes(c->conn, 2, c->part_off + (uint64_t)di_u * c->blk_sz, buf + foff, n);
                    } else if (type == 1 /* HEAD */) {
                        ufs_read_bytes(c->conn, 2, c->part_off + (uint64_t)di_u * c->blk_sz, pblk_buf, c->blk_sz);
                        /* LZ4 stream may be 0-padded at front (EROFS zero-padding) */
                        size_t off0 = 0;
                        while (off0 < c->blk_sz && pblk_buf[off0] == 0) ++off0;
                        uint64_t foff = (uint64_t)l * c->blk_sz + di_clusterofs;
                        size_t cap = (size_t)(in->size - foff);
                        if (cap > c->blk_sz * 2u) cap = c->blk_sz * 2u;
                        lz4_decompress_block(pblk_buf + off0, c->blk_sz - off0, buf + foff, cap);
                    }
                }
                free(idx); free(pblk_buf);
                return buf;
            }
        }
    }
    fprintf(stderr, "Unsupported EROFS datalayout=%u (size=%llu)\n",
            in->datalayout, (unsigned long long)in->size);
    free(buf);
    return NULL;
}

static int erofs_lookup(const struct ErofsCtx *c, const char *path, uint64_t *nid_out, struct ErofsInodeInfo *info_out) {
    uint64_t cur_nid = c->root_nid;
    struct ErofsInodeInfo cur;
    if (erofs_read_inode(c, cur_nid, &cur) != 0) return -1;
    const char *p = path;
    while (*p == '/') ++p;
    while (*p) {
        const char *slash = strchr(p, '/');
        size_t clen = slash ? (size_t)(slash - p) : strlen(p);
        uint8_t *dbuf = erofs_read_data(c, &cur, NULL);
        if (!dbuf) return -1;
        uint64_t next_nid = 0;
        bool found = false;
        for (uint64_t blk_off = 0; blk_off < cur.size && !found; blk_off += c->blk_sz) {
            uint64_t bsz = (cur.size - blk_off < c->blk_sz) ? (cur.size - blk_off) : c->blk_sz;
            if (bsz < 12) break;
            const uint8_t *b = dbuf + blk_off;
            uint16_t first_nameoff = 0;
            memcpy(&first_nameoff, b + 8, 2);
            uint32_t nents = first_nameoff / 12u;
            for (uint32_t i = 0; i < nents; ++i) {
                uint64_t nid = 0; uint16_t nameoff = 0;
                memcpy(&nid, b + i * 12, 8);
                memcpy(&nameoff, b + i * 12 + 8, 2);
                uint16_t next_nameoff = (i + 1 < nents) ? (uint16_t)(b[(i+1)*12 + 8] | (b[(i+1)*12 + 9] << 8)) : (uint16_t)bsz;
                if (nameoff >= bsz || next_nameoff > bsz || next_nameoff < nameoff) continue;
                size_t nlen = next_nameoff - nameoff;
                while (nlen > 0 && b[nameoff + nlen - 1] == 0) --nlen;
                if (nlen == clen && memcmp(b + nameoff, p, clen) == 0) {
                    next_nid = nid; found = true; break;
                }
            }
        }
        free(dbuf);
        if (!found) return -1;
        cur_nid = next_nid;
        if (erofs_read_inode(c, cur_nid, &cur) != 0) return -1;
        if (!slash) break;
        p = slash + 1;
        while (*p == '/') ++p;
    }
    if (nid_out) *nid_out = cur_nid;
    if (info_out) *info_out = cur;
    return 0;
}

static int cmd_ufs_ls_or_cat(io_connect_t conn, uint32_t part_lba, const char *path, const char *out_path) {
    struct Ext4Ctx e4;
    if (ext4_init(conn, part_lba, &e4) == 0) {
        uint32_t ino = ext4_lookup(&e4, path);
        if (!ino) { fprintf(stderr, "ext4: path not found: %s\n", path); return 1; }
        uint64_t sz = 0; uint16_t mode = 0;
        uint8_t *buf = ext4_read_file_data(&e4, ino, &sz, &mode);
        if (!buf) { fprintf(stderr, "ext4: read failed ino=%u\n", ino); return 1; }
        if (!out_path) {
            size_t off = 0;
            while (off + 8 <= sz) {
                uint32_t dino = 0; uint16_t rlen = 0; uint8_t nlen = buf[off + 6], ft = buf[off + 7];
                memcpy(&dino, buf + off, 4); memcpy(&rlen, buf + off + 4, 2);
                if (rlen == 0) break;
                if (dino && nlen > 0) {
                    printf("  [%u] type=%u %.*s\n", dino, ft, (int)nlen, (const char *)(buf + off + 8));
                }
                off += rlen;
            }
        } else {
            FILE *fp = fopen(out_path, "wb");
            if (!fp) { perror("fopen"); free(buf); return 1; }
            fwrite(buf, 1, (size_t)sz, fp);
            fclose(fp);
            printf("EXT4 extracted %s -> %s (%llu B)\n", path, out_path, (unsigned long long)sz);
        }
        free(buf);
        return 0;
    }

    struct ErofsCtx er;
    if (erofs_init(conn, part_lba, &er) == 0) {
        struct ErofsInodeInfo info;
        uint64_t nid = 0;
        if (erofs_lookup(&er, path, &nid, &info) != 0) {
            fprintf(stderr, "erofs: path not found: %s\n", path);
            return 1;
        }
        uint8_t *buf = erofs_read_data(&er, &info, NULL);
        if (!buf) {
            fprintf(stderr, "erofs: read failed nid=%llu layout=%u sz=%llu\n",
                    (unsigned long long)nid, info.datalayout, (unsigned long long)info.size);
            return 1;
        }
        if (!out_path) {
            for (uint64_t blk_off = 0; blk_off < info.size; blk_off += er.blk_sz) {
                uint64_t bsz = (info.size - blk_off < er.blk_sz) ? (info.size - blk_off) : er.blk_sz;
                if (bsz < 12) break;
                const uint8_t *b = buf + blk_off;
                uint16_t first_nameoff = 0;
                memcpy(&first_nameoff, b + 8, 2);
                uint32_t nents = first_nameoff / 12u;
                for (uint32_t i = 0; i < nents; ++i) {
                    uint64_t cnid = 0; uint16_t nameoff = 0; uint8_t ft = b[i * 12 + 10];
                    memcpy(&cnid, b + i * 12, 8);
                    memcpy(&nameoff, b + i * 12 + 8, 2);
                    uint16_t next_nameoff = (i + 1 < nents) ? (uint16_t)(b[(i+1)*12 + 8] | (b[(i+1)*12 + 9] << 8)) : (uint16_t)bsz;
                    if (nameoff >= bsz || next_nameoff > bsz || next_nameoff < nameoff) continue;
                    size_t nlen = next_nameoff - nameoff;
                    while (nlen > 0 && b[nameoff + nlen - 1] == 0) --nlen;
                    printf("  [nid=%llu] type=%u %.*s\n", (unsigned long long)cnid, ft, (int)nlen, (const char *)(b + nameoff));
                }
            }
        } else {
            FILE *fp = fopen(out_path, "wb");
            if (!fp) { perror("fopen"); free(buf); return 1; }
            fwrite(buf, 1, (size_t)info.size, fp);
            fclose(fp);
            printf("EROFS extracted %s -> %s (%llu B, layout=%u)\n",
                   path, out_path, (unsigned long long)info.size, info.datalayout);
        }
        free(buf);
        return 0;
    }

    fprintf(stderr, "LBA %u is neither EXT4 nor EROFS\n", part_lba);
    return 1;
}

static int cmd_emi_load_rom(io_connect_t conn, const char *path) {
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror("fopen"); return 1; }
    fseek(fp, 0, SEEK_END);
    long fsz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (fsz <= 48 || fsz > 4 * 1024 * 1024) {
        fprintf(stderr, "Bad ROM EMI file size: %ld\n", fsz);
        fclose(fp);
        return 1;
    }
    uint8_t *buf = (uint8_t *)malloc((size_t)fsz);
    if (!buf || fread(buf, 1, (size_t)fsz, fp) != (size_t)fsz) {
        fprintf(stderr, "Read failed\n");
        free(buf); fclose(fp); return 1;
    }
    fclose(fp);

    /* struct ROM_EMI_HEADER (48 bytes):
     *   [0..15]  aucBuildDate[16]
     *   [16..19] u4PLat
     *   [20..21] u2HwVer
     *   [22..23] u2SwVer
     *   [24..27] u4PatchAddr
     *   [28..31] u4PatchType
     *   [32..47] u4CRC[4]
     */
    char date_str[17] = {0};
    memcpy(date_str, buf, 16);
    uint32_t patch_addr = 0, patch_type = 0;
    memcpy(&patch_addr, buf + 24, 4);
    memcpy(&patch_type, buf + 28, 4);
    uint32_t emi_off = patch_addr & 0x00FFFFFFu;
    uint32_t payload_len = (uint32_t)fsz - 48u;
    printf("ROM_EMI_HEADER: date=\"%.16s\" patch_addr=0x%08x (emi_off=0x%06x) type=0x%x payload=%u B\n",
           date_str, patch_addr, emi_off, patch_type, payload_len);

    struct Tb375fcConnSysMemRwIn in = {
        .user_buf = (uint64_t)(uintptr_t)(buf + 48),
        .offset   = emi_off,
        .size     = payload_len,
        .is_write = 1,
        .pad      = 0,
    };
    struct Tb375fcConnSysMemRwOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_EMI_RW, &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS || out.rc != 0) {
        fprintf(stderr, "EMI payload write failed: kr=0x%x rc=%d done=%u\n", kr, out.rc, out.bytes_done);
        free(buf); return 1;
    }
    /* Write 16-byte date to WMMCU_MCU_ROM_EMI_DATE_ADDR (0x4954E0) */
    in.user_buf = (uint64_t)(uintptr_t)buf;
    in.offset   = 0x004954E0u;
    in.size     = 16u;
    osz = sizeof(out);
    kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_EMI_RW, &in, sizeof(in), &out, &osz);
    free(buf);
    printf("EMI ROM loaded OK (date_rc=%d)\n", (kr == KERN_SUCCESS) ? out.rc : -1);
    return 0;
}

static int cmd_mem_write_file(io_connect_t conn, uint32_t sel, uint32_t off, const char *path) {
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror("fopen"); return 1; }
    fseek(fp, 0, SEEK_END);
    long fsz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (fsz <= 0 || fsz > 4 * 1024 * 1024) {
        fprintf(stderr, "Bad file size: %ld\n", fsz);
        fclose(fp); return 1;
    }
    uint8_t *buf = (uint8_t *)malloc((size_t)fsz);
    if (!buf || fread(buf, 1, (size_t)fsz, fp) != (size_t)fsz) {
        fprintf(stderr, "Read failed\n");
        free(buf); fclose(fp); return 1;
    }
    fclose(fp);
    struct Tb375fcConnSysMemRwIn in = { (uint64_t)(uintptr_t)buf, off, (uint32_t)fsz, 1, 0 };
    struct Tb375fcConnSysMemRwOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, sel, &in, sizeof(in), &out, &osz);
    free(buf);
    if (kr != KERN_SUCCESS || out.rc != 0) {
        fprintf(stderr, "MEM_WRITE failed: kr=0x%x rc=%d done=%u\n", kr, out.rc, out.bytes_done);
        return 1;
    }
    printf("MEM_WRITE ok: off=0x%x bytes=%u\n", off, out.bytes_done);
    return 0;
}

static int cmd_mem_dump(io_connect_t conn, uint32_t sel, uint32_t off, uint32_t len) {
    if (len == 0 || len > 4096) len = 256;
    uint8_t *buf = (uint8_t *)calloc(1, len);
    if (!buf) return 1;
    struct Tb375fcConnSysMemRwIn in = { (uint64_t)(uintptr_t)buf, off, len, 0, 0 };
    struct Tb375fcConnSysMemRwOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, sel, &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS || out.rc != 0) {
        fprintf(stderr, "MEM_READ failed: kr=0x%x rc=%d\n", kr, out.rc);
        free(buf); return 1;
    }
    for (uint32_t i = 0; i < len; i += 16) {
        printf("%06x:", off + i);
        for (uint32_t j = 0; j < 16 && i + j < len; j += 4) {
            uint32_t w = 0;
            memcpy(&w, buf + i + j, (i + j + 4 <= len) ? 4 : (len - i - j));
            printf(" %08x", w);
        }
        printf("  ");
        for (uint32_t j = 0; j < 16 && i + j < len; ++j) {
            uint8_t c = buf[i + j];
            putchar((c >= 32 && c < 127) ? (char)c : '.');
        }
        putchar('\n');
    }
    free(buf);
    return 0;
}

static int cmd_reg(io_connect_t conn, uint32_t kind, uint32_t slv, uint32_t addr, uint32_t val) {
    struct Tb375fcConnSysRegOpIn in = { kind, addr, val, slv };
    struct Tb375fcConnSysRegOpOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_REG_OP, &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS || out.rc != 0) {
        fprintf(stderr, "REG_OP failed: kr=0x%x rc=%d\n", kr, out.rc);
        return 1;
    }
    printf("rc=0 val=0x%08x\n", out.value);
    return 0;
}

static int cmd_smc(io_connect_t conn, int argc, char **argv) {
    struct Tb375fcConnSysSmcIn in = {0};
    in.fid = strtoull(argv[0], NULL, 0);
    if (argc > 1) in.x1 = strtoull(argv[1], NULL, 0);
    if (argc > 2) in.x2 = strtoull(argv[2], NULL, 0);
    if (argc > 3) in.x3 = strtoull(argv[3], NULL, 0);
    struct Tb375fcConnSysSmcOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_SMC_CALL, &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "SMC failed: kr=0x%x\n", kr); return 1; }
    printf("SMC(0x%llx, %llu, %llu, %llu) -> a0=0x%llx (%lld) a1=0x%llx a2=0x%llx a3=0x%llx\n",
           (unsigned long long)in.fid, (unsigned long long)in.x1, (unsigned long long)in.x2, (unsigned long long)in.x3,
           (unsigned long long)out.a0, (long long)out.a0,
           (unsigned long long)out.a1, (unsigned long long)out.a2, (unsigned long long)out.a3);
    return 0;
}

static const char *sec_name(uint8_t s) {
    switch (s) {
    case WIFI_SEC_OPEN:            return "OPEN";
    case WIFI_SEC_WEP:             return "WEP";
    case WIFI_SEC_WPA_TKIP:        return "WPA-TKIP";
    case WIFI_SEC_WPA2_PSK:        return "WPA2-PSK";
    case WIFI_SEC_WPA2_PSK_SHA256: return "WPA2-SHA256";
    case WIFI_SEC_WPA3_SAE:        return "WPA3-SAE";
    default:                       return "OTHER";
    }
}

static const char *wf_state_name(uint32_t s) {
    switch (s) {
    case WIFI_STATE_DOWN:      return "DOWN";
    case WIFI_STATE_READY:     return "READY";
    case WIFI_STATE_SCANNING:  return "SCANNING";
    case WIFI_STATE_AUTH:      return "AUTH";
    case WIFI_STATE_ASSOC:     return "ASSOC";
    case WIFI_STATE_EAPOL:     return "EAPOL";
    case WIFI_STATE_CONNECTED: return "CONNECTED";
    default:                   return "UNKNOWN";
    }
}

static int cmd_wifi_up(io_connect_t conn) {
    struct Tb375fcWifiUpOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_WIFI_UP, NULL, 0, &out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "WIFI_UP ioctl failed: 0x%x\n", kr); return 1; }
    printf("=== MT6897 Wi-Fi 6 Up (rc=%d step=%u) ===\n", out.rc, out.failed_step);
    printf("MISC=0x%08x LPCTL=0x%08x NIC_CAP=%uB ADIE=0x%08x MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
           out.conn_on_misc, out.wf_lpctl, out.nic_cap_len, out.adie_ver,
           out.mac[0], out.mac[1], out.mac[2], out.mac[3], out.mac[4], out.mac[5]);
    return out.rc == 0 ? 0 : 1;
}

static int cmd_wifi_scan(io_connect_t conn, int argc, char **argv) {
    struct Tb375fcWifiScanIn in = {0};
    if (argc >= 1) in.flags = (uint32_t)strtoul(argv[0], NULL, 0);
    if (argc >= 2) in.dwell_ms = (uint32_t)strtoul(argv[1], NULL, 0);
    struct Tb375fcWifiScanOut *out = (struct Tb375fcWifiScanOut *)calloc(1, sizeof(*out));
    if (!out) return 1;
    size_t osz = sizeof(*out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_WIFI_SCAN, &in, sizeof(in), out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "WIFI_SCAN ioctl failed: 0x%x\n", kr); free(out); return 1; }
    printf("=== MT6897 Wi-Fi 6 Scan (rc=%d, %u APs) ===\n", out->rc, out->count);
    printf("%-18s %4s %4s %5s %-12s %-6s %s\n", "BSSID", "BAND", "CH", "RSSI", "SECURITY", "FLAGS", "SSID");
    for (uint32_t i = 0; i < out->count; ++i) {
        const struct Tb375fcWifiScanEntry *e = &out->entries[i];
        char fstr[16] = "";
        if (e->flags & 1) strcat(fstr, "Q");
        if (e->flags & 2) strcat(fstr, "N");
        if (e->flags & 4) strcat(fstr, "AC");
        if (e->flags & 8) strcat(fstr, "AX");
        printf("%02x:%02x:%02x:%02x:%02x:%02x  %4s %4u %4ddBm %-12s %-6s %s\n",
               e->bssid[0], e->bssid[1], e->bssid[2], e->bssid[3], e->bssid[4], e->bssid[5],
               (e->band == 2) ? "5G" : "2.4G", e->channel, (int)e->rssi,
               sec_name(e->security), fstr[0] ? fstr : "-",
               e->ssid_len ? e->ssid : "<hidden>");
    }
    int rc = out->rc == 0 ? 0 : 1;
    free(out);
    return rc;
}

static int cmd_wifi_join(io_connect_t conn, int argc, char **argv) {
    if (argc < 1) {
        fprintf(stderr, "Usage: wifi-join <ssid> [passphrase] [channel]\n");
        return 2;
    }
    struct Tb375fcWifiJoinIn in = {0};
    strncpy(in.ssid, argv[0], sizeof(in.ssid) - 1);
    if (argc >= 2 && strcmp(argv[1], "-") != 0) {
        strncpy(in.passphrase, argv[1], sizeof(in.passphrase) - 1);
    }
    if (argc >= 3) in.channel = (uint8_t)strtoul(argv[2], NULL, 0);

    struct Tb375fcWifiJoinOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_WIFI_JOIN, &in, sizeof(in), &out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "WIFI_JOIN ioctl failed: 0x%x\n", kr); return 1; }
    printf("=== MT6897 Wi-Fi 6 Join \"%s\" (rc=%d step=%u) ===\n", in.ssid, out.rc, out.failed_step);
    printf("BSSID=%02x:%02x:%02x:%02x:%02x:%02x ch=%u(%s) rssi=%ddBm sec=%s aid=%u status=%u eapol=%u keys=%u\n",
           out.bssid[0], out.bssid[1], out.bssid[2], out.bssid[3], out.bssid[4], out.bssid[5],
           out.channel, (out.band == 2) ? "5G" : "2.4G", (int)out.rssi, sec_name(out.security),
           out.assoc_id, out.status_code, out.eapol_stage, out.keys_installed);
    return out.rc == 0 ? 0 : 1;
}

static int cmd_wifi_status(io_connect_t conn) {
    struct Tb375fcWifiStatusOut st = {0};
    size_t osz = sizeof(st);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_WIFI_STATUS, NULL, 0, &st, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "WIFI_STATUS ioctl failed: 0x%x\n", kr); return 1; }
    printf("=== MT6897 Wi-Fi 6 Status ===\n");
    printf("State=%s fw_ready=%u connected=%u keys=%u eapol=%u last_deauth=%u MISC=0x%x\n",
           wf_state_name(st.state), st.fw_ready, st.connected, st.keys_installed,
           st.eapol_stage, st.last_deauth_reason, st.conn_on_misc);
    printf("MAC=%02x:%02x:%02x:%02x:%02x:%02x BSSID=%02x:%02x:%02x:%02x:%02x:%02x SSID=\"%s\" ch=%u(%s) rssi=%ddBm aid=%u sec=%s\n",
           st.own_mac[0], st.own_mac[1], st.own_mac[2], st.own_mac[3], st.own_mac[4], st.own_mac[5],
           st.bssid[0], st.bssid[1], st.bssid[2], st.bssid[3], st.bssid[4], st.bssid[5],
           st.ssid, st.channel, (st.band == 2) ? "5G" : "2.4G", (int)st.rssi, st.assoc_id, sec_name(st.security));
    printf("TX: pkts=%llu bytes=%llu freed=%llu drop=%llu free_tok=%u\n",
           (unsigned long long)st.tx_packets, (unsigned long long)st.tx_bytes,
           (unsigned long long)st.tx_freed, (unsigned long long)st.tx_dropped, st.free_tokens);
    printf("RX: pkts=%llu bytes=%llu eapol=%llu mgmt=%llu evts=%llu err=%llu\n",
           (unsigned long long)st.rx_packets, (unsigned long long)st.rx_bytes,
           (unsigned long long)st.rx_eapol, (unsigned long long)st.rx_mgmt,
           (unsigned long long)st.rx_events, (unsigned long long)st.rx_errors);
    printf("WFDMA0: TX0=%u/%u TX15=%u/%u RX0=%u/%u RX2=%u/%u RX3=%u/%u\n",
           st.tx0_cidx, st.tx0_didx, st.tx15_cidx, st.tx15_didx,
           st.rx0_cidx, st.rx0_didx, st.rx2_cidx, st.rx2_didx, st.rx3_cidx, st.rx3_didx);
    return 0;
}

static int cmd_wifi_down(io_connect_t conn) {
    struct Tb375fcWifiDownOut out = {0};
    size_t osz = sizeof(out);
    kern_return_t kr = IOConnectCallStructMethod(conn, CONNSYS_SEL_WIFI_DOWN, NULL, 0, &out, &osz);
    if (kr != KERN_SUCCESS) { fprintf(stderr, "WIFI_DOWN ioctl failed: 0x%x\n", kr); return 1; }
    printf("Wi-Fi disconnected (prev_state=%s, rc=%d)\n", wf_state_name(out.prev_state), out.rc);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,
            "Usage: tb375fc-connsys-ctl <cmd> [args]\n"
            "  wifi-up                               Bring up MT6897 Wi-Fi 6 FW + WFDMA0 + en0 bridge\n"
            "  wifi-scan [flags] [dwell_ms]          Scan 2.4GHz + 5GHz APs\n"
            "  wifi-join <ssid> [pass] [ch]          Join OPEN or WPA2-PSK AP (bridged to en0)\n"
            "  wifi-status                           Show live Wi-Fi & WFDMA0 TX/RX status\n"
            "  wifi-down                             Disconnect from AP\n"
            "  probe                                 Read-only ConnSys/PMIC/EMI diagnostic\n"
            "  stage1 [radio_mask] [clk] [flags]     ConnInfra + MT6637 A-Die bring-up\n"
            "  wifi-on [flags]                       Stage 3 soc7_0 WMMCU power-on + WFDMA\n"
            "  ufs-gpt [lun=2]                       Read-only GPT table dump\n"
            "  ufs-lp <super_lba>                    Read-only Android super LP metadata dump\n"
            "  ufs-fsinfo <part_lba>                 Inspect EXT4/EROFS superblock\n"
            "  ufs-ls <part_lba> <path>              List directory in EXT4/EROFS partition\n"
            "  ufs-cat <part_lba> <path> <out_file>  Extract file from EXT4/EROFS partition\n"
            "  ufs-read <lba> <cnt> <out> [lun=2]    Dump raw 4K blocks read-only\n"
            "  emi-dump <hex_off> <len>              Hexdump consys EMI (0x8cc00000+off)\n"
            "  emi-write <hex_off> <file>            Write file into consys EMI\n"
            "  emi-load-rom <file>                   Load soc7_0_ram_wmmcu_1_1_hdr.bin into EMI\n"
            "  dma-dump <hex_off> <len>              Hexdump Wi-Fi DMA pool (0x8d900000+off)\n"
            "  dma-write <hex_off> <file>            Write file into Wi-Fi DMA pool\n"
            "  reg-rd <hex_addr>                     Read32 ConnSys MMIO (0x18000000..0x188FFFFC)\n"
            "  reg-wr <hex_addr> <hex_val>           Write32 ConnSys MMIO\n"
            "  spmi-rd <slv> <hex_addr>              Read8 SPMI PMIC register\n"
            "  spmi-wr <slv> <hex_addr> <hex_val>    Write8 SPMI PMIC register\n"
            "  smc <fid> [x1] [x2] [x3]              Call BL31 ConnSys/WLAN/BT SMC\n");
        return 2;
    }

    io_connect_t conn = open_connsys();
    if (conn == MACH_PORT_NULL) return 1;

    int rc = 2;
    const char *cmd = argv[1];
    if (strcmp(cmd, "wifi-up") == 0) {
        rc = cmd_wifi_up(conn);
    } else if (strcmp(cmd, "wifi-scan") == 0) {
        rc = cmd_wifi_scan(conn, argc - 2, argv + 2);
    } else if (strcmp(cmd, "wifi-join") == 0) {
        rc = cmd_wifi_join(conn, argc - 2, argv + 2);
    } else if (strcmp(cmd, "wifi-status") == 0) {
        rc = cmd_wifi_status(conn);
    } else if (strcmp(cmd, "wifi-down") == 0) {
        rc = cmd_wifi_down(conn);
    } else if (strcmp(cmd, "probe") == 0) {
        rc = cmd_probe(conn);
    } else if (strcmp(cmd, "stage1") == 0) {
        rc = cmd_stage1(conn, argc - 2, argv + 2);
    } else if (strcmp(cmd, "wifi-on") == 0) {
        rc = cmd_wifi_on(conn, argc - 2, argv + 2);
    } else if (strcmp(cmd, "ufs-gpt") == 0) {
        uint32_t lun = (argc >= 3) ? (uint32_t)strtoul(argv[2], NULL, 0) : 2u;
        rc = cmd_ufs_gpt(conn, lun);
    } else if (strcmp(cmd, "ufs-lp") == 0 && argc >= 3) {
        rc = cmd_ufs_lp(conn, (uint32_t)strtoul(argv[2], NULL, 0));
    } else if (strcmp(cmd, "ufs-fsinfo") == 0 && argc >= 3) {
        rc = cmd_ufs_fsinfo(conn, (uint32_t)strtoul(argv[2], NULL, 0));
    } else if (strcmp(cmd, "ufs-ls") == 0 && argc >= 4) {
        rc = cmd_ufs_ls_or_cat(conn, (uint32_t)strtoul(argv[2], NULL, 0), argv[3], NULL);
    } else if (strcmp(cmd, "ufs-cat") == 0 && argc >= 5) {
        rc = cmd_ufs_ls_or_cat(conn, (uint32_t)strtoul(argv[2], NULL, 0), argv[3], argv[4]);
    } else if (strcmp(cmd, "ufs-read") == 0 && argc >= 5) {
        uint32_t lba = (uint32_t)strtoul(argv[2], NULL, 0);
        uint32_t cnt = (uint32_t)strtoul(argv[3], NULL, 0);
        uint32_t lun = (argc >= 6) ? (uint32_t)strtoul(argv[5], NULL, 0) : 2u;
        uint8_t *buf = (uint8_t *)malloc((size_t)cnt * 4096u);
        if (buf && ufs_read_blocks(conn, lun, lba, cnt, buf) == 0) {
            FILE *fp = fopen(argv[4], "wb");
            if (fp) { fwrite(buf, 1, (size_t)cnt * 4096u, fp); fclose(fp); rc = 0; }
        }
        free(buf);
    } else if (strcmp(cmd, "emi-dump") == 0 && argc >= 4) {
        rc = cmd_mem_dump(conn, CONNSYS_SEL_EMI_RW,
                          (uint32_t)strtoul(argv[2], NULL, 0), (uint32_t)strtoul(argv[3], NULL, 0));
    } else if (strcmp(cmd, "emi-write") == 0 && argc >= 4) {
        rc = cmd_mem_write_file(conn, CONNSYS_SEL_EMI_RW,
                                (uint32_t)strtoul(argv[2], NULL, 0), argv[3]);
    } else if (strcmp(cmd, "emi-load-rom") == 0 && argc >= 3) {
        rc = cmd_emi_load_rom(conn, argv[2]);
    } else if (strcmp(cmd, "dma-dump") == 0 && argc >= 4) {
        rc = cmd_mem_dump(conn, CONNSYS_SEL_DMA_RW,
                          (uint32_t)strtoul(argv[2], NULL, 0), (uint32_t)strtoul(argv[3], NULL, 0));
    } else if (strcmp(cmd, "dma-write") == 0 && argc >= 4) {
        rc = cmd_mem_write_file(conn, CONNSYS_SEL_DMA_RW,
                                (uint32_t)strtoul(argv[2], NULL, 0), argv[3]);
    } else if (strcmp(cmd, "reg-rd") == 0 && argc >= 3) {
        rc = cmd_reg(conn, CONNSYS_REG_MMIO_READ32, 0, (uint32_t)strtoul(argv[2], NULL, 0), 0);
    } else if (strcmp(cmd, "reg-wr") == 0 && argc >= 4) {
        rc = cmd_reg(conn, CONNSYS_REG_MMIO_WRITE32, 0,
                     (uint32_t)strtoul(argv[2], NULL, 0), (uint32_t)strtoul(argv[3], NULL, 0));
    } else if (strcmp(cmd, "spmi-rd") == 0 && argc >= 4) {
        rc = cmd_reg(conn, CONNSYS_REG_SPMI_READ8,
                     (uint32_t)strtoul(argv[2], NULL, 0), (uint32_t)strtoul(argv[3], NULL, 0), 0);
    } else if (strcmp(cmd, "spmi-wr") == 0 && argc >= 5) {
        rc = cmd_reg(conn, CONNSYS_REG_SPMI_WRITE8,
                     (uint32_t)strtoul(argv[2], NULL, 0), (uint32_t)strtoul(argv[3], NULL, 0),
                     (uint32_t)strtoul(argv[4], NULL, 0));
    } else if (strcmp(cmd, "smc") == 0 && argc >= 3) {
        rc = cmd_smc(conn, argc - 2, argv + 2);
    } else {
        fprintf(stderr, "Unknown or incomplete command: %s\n", cmd);
    }

    IOServiceClose(conn);
    return rc;
}
