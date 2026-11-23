/* SPDX-License-Identifier: MIT */
/*
 * Novatek NT36532 SPI Touch Protocol, Firmware Downloader, Coordinate Decoder,
 * and Multi-Touch-to-macOS-Pointer Gesture Engine.
 *
 * Every protocol detail is verified against TB375FC stock nt36532.ko.
 */
#include "Nvt36532Protocol.h"
#include "Mt6897TouchBoard.h"

#include <string.h>

static inline void msleep_wrap(const nvt_bus_t *bus, uint32_t ms) {
    if (bus && bus->msleep) bus->msleep(bus->ctx, ms);
}

static inline uint32_t rd_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint16_t rd_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ---- Low-level SPI framing (verified from CTP_SPI_READ / CTP_SPI_WRITE) ---- */

int nvt_set_page(const nvt_bus_t *bus, uint32_t addr) {
    if (!bus || !bus->xfer) return -1;
    /* nt36532.ko L11e0..L11f4: [0xFF, (addr >> 15) & 0xFF, (addr >> 7) & 0xFF] */
    uint8_t cmd[3];
    cmd[0] = 0xFFu;
    cmd[1] = (uint8_t)((addr >> 15) & 0xFFu);
    cmd[2] = (uint8_t)((addr >> 7) & 0xFFu);
    for (int retry = 0; retry < 5; ++retry) {
        if (bus->xfer(bus->ctx, cmd, NULL, sizeof(cmd)) == 0) return 0;
    }
    return -5;
}

int nvt_read_bytes(const nvt_bus_t *bus, uint32_t addr, uint8_t *out, size_t len) {
    if (!bus || !bus->xfer || !out || len == 0 || len > 256u) return -1;
    if (nvt_set_page(bus, addr) != 0) return -5;

    uint8_t tx[256u + 2u];
    uint8_t rx[256u + 2u];
    memset(tx, 0, len + 2u);
    /* CTP_SPI_READ: byte 0 = (addr & 0x7F) [MSB=0 for READ], byte 1 = dummy */
    tx[0] = (uint8_t)(addr & 0x7Fu);

    for (int retry = 0; retry < 5; ++retry) {
        memset(rx, 0, len + 2u);
        if (bus->xfer(bus->ctx, tx, rx, len + 2u) == 0) {
            memcpy(out, rx + 2u, len);
            return 0;
        }
    }
    return -5;
}

int nvt_write_bytes(const nvt_bus_t *bus, uint32_t addr, const uint8_t *data, size_t len) {
    if (!bus || !bus->xfer || !data || len == 0 || len > NVT_FW_CHUNK_BYTES) return -1;
    if (nvt_set_page(bus, addr) != 0) return -5;

    uint8_t tx[NVT_FW_CHUNK_BYTES + 1u];
    /* CTP_SPI_WRITE: byte 0 = (addr & 0x7F) | 0x80 [MSB=1 for WRITE] */
    tx[0] = (uint8_t)((addr & 0x7Fu) | 0x80u);
    memcpy(tx + 1u, data, len);

    for (int retry = 0; retry < 5; ++retry) {
        if (bus->xfer(bus->ctx, tx, NULL, len + 1u) == 0) return 0;
    }
    return -5;
}

int nvt_write_u8(const nvt_bus_t *bus, uint32_t addr, uint8_t val) {
    return nvt_write_bytes(bus, addr, &val, 1u);
}

/* ---- Bootloader Reset & Chip Trim ID Probe ---- */

int nvt_bootloader_reset(const nvt_bus_t *bus, uint32_t swrst_addr, uint32_t spi_rd_fast_addr) {
    if (!bus || !swrst_addr) return -1;
    /* nt36532.ko L0xed0..0xefc: write 0x69 to swrst_n8_addr, delay 5ms, clear spi_rd_fast_addr */
    int rc = nvt_write_u8(bus, swrst_addr, 0x69u);
    if (rc != 0) return rc;
    msleep_wrap(bus, 5u);
    if (spi_rd_fast_addr != 0u) {
        (void)nvt_write_u8(bus, spi_rd_fast_addr, 0x00u);
    }
    return 0;
}

int nvt_identify(const nvt_bus_t *bus, nvt_chip_id_t *out) {
    if (!bus || !out) return -1;
    memset(out, 0, sizeof(*out));

    /* Exact 3 address tuples from nt36532.ko file offset 0x13860..0x13880 */
    static const struct {
        uint32_t trim_addr;
        uint32_t swrst_addr;
        uint32_t spi_rd_fast_addr;
    } kTuples[3] = {
        { NVT36532_TRIM_ID_ADDR, NVT36532_SWRST_N8_ADDR, NVT36532_SPI_RD_FAST_ADDR },
        { 0x0003F004u,           0x0003F0FEu,            0x0003F135u               },
        { 0x0001F64Eu,           0x0001F01Au,            0x00000000u               },
    };

    for (int t = 0; t < 3; ++t) {
        for (int attempt = 0; attempt < 5; ++attempt) {
            if (nvt_bootloader_reset(bus, kTuples[t].swrst_addr, kTuples[t].spi_rd_fast_addr) != 0) {
                msleep_wrap(bus, 10u);
                continue;
            }
            uint8_t buf[6] = {0};
            /* nt36532.ko reads trim_id_addr twice back-to-back */
            (void)nvt_read_bytes(bus, kTuples[t].trim_addr, buf, 6u);
            if (nvt_read_bytes(bus, kTuples[t].trim_addr, buf, 6u) != 0) {
                msleep_wrap(bus, 10u);
                continue;
            }
            memcpy(out->trim, buf, 6u);
            out->trim_addr        = kTuples[t].trim_addr;
            out->swrst_addr       = kTuples[t].swrst_addr;
            out->spi_rd_fast_addr = kTuples[t].spi_rd_fast_addr;

            /* trim_id_table[2]: id = [xx, xx, xx, 0x32, 0x65, 0x03], mask = [0,0,0,1,1,1] */
            if (buf[3] == 0x32u && buf[4] == 0x65u && buf[5] == 0x03u) {
                out->matched_nt36532 = 1u;
                return 0;
            }
            msleep_wrap(bus, 10u);
        }
    }
    return -19; /* ENODEV */
}

/* ---- Firmware Image Parser & SRAM Downloader ---- */

int nvt_fw_parse(const uint8_t *fw, size_t fw_len, nvt_fw_plan_t *out) {
    if (!fw || fw_len < 4096u || !out) return -22;
    memset(out, 0, sizeof(*out));

    /* Scan backwards on 4 KiB boundaries for "NVT" or "MOD" tail marker (nt36532.ko L6364..63c4). */
    size_t fw_need = 0;
    for (size_t off = (fw_len & ~4095ULL); off >= 4096u; off -= 4096u) {
        const uint8_t *tag = fw + off - 3u;
        if ((tag[0] == 'N' && tag[1] == 'V' && tag[2] == 'T') ||
            (tag[0] == 'M' && tag[1] == 'O' && tag[2] == 'D')) {
            fw_need = off;
            break;
        }
    }
    if (fw_need < 4096u) return -22;

    /* Checksum check at fw_need - 4096 (nt36532.ko L64c0..64e0): byte[0] + byte[1] == 0xFF. */
    uint8_t c0 = fw[fw_need - 4096u];
    uint8_t c1 = fw[fw_need - 4095u];
    if ((uint8_t)(c0 + c1) != 0xFFu) return -8;

    out->fw_need_write_size = fw_need;
    out->ilm_dlm_crc_hw = 1u; /* NT36532_hw_info: hw_crc = 2 */

    /* Header layout (nt36532.ko L64e4..6860):
     *   +0x00: ILM { u32 bin_addr, u32 sram_addr, u32 size }
     *   +0x0C: DLM { u32 bin_addr, u32 sram_addr, u32 size }
     *   +0x18: ILM CRC32
     *   +0x1C: DLM CRC32
     *   +0x20: bit1 = ilm_dlm_num_flag (cascade/extended header flag)
     *   +0x28: bit4 = has_overlay, bits[3:0] = overlay count
     *   +0x30+: extra 16-byte partition entries { u32 sram_addr, u32 size, u32 bin_addr, u32 crc }
     */
    uint32_t ilm_bin_addr = rd_le32(fw + 0x00u);
    uint8_t  ext_flag     = (fw[0x20u] >> 1) & 1u;
    uint8_t  ovl_info     = fw[0x28u];
    uint8_t  ovl_cnt      = (ovl_info & 0x10u) ? (ovl_info & 0x0Fu) : 0u;

    uint32_t extra_cnt = 0;
    uint32_t tbl_off   = 0x30u;
    if (!ext_flag) {
        if (ilm_bin_addr >= 0x31u) {
            extra_cnt = ((ilm_bin_addr - 0x31u) >> 4) + 1u;
            tbl_off = 0x30u;
        }
    } else {
        if (ilm_bin_addr >= 0x62u) {
            uint32_t half = ilm_bin_addr >> 1;
            if (half > 0x40u) half = 0x40u;
            extra_cnt = ((half - 0x31u) >> 4) + 1u;
        }
        extra_cnt += 1u;
        tbl_off = 0x30u;
    }

    uint32_t total = 2u + extra_cnt + ovl_cnt;
    if (total > NVT_MAX_PARTITIONS) total = NVT_MAX_PARTITIONS;
    out->partition_count = (uint8_t)total;

    /* Part 0: ILM, Part 1: DLM */
    for (uint32_t i = 0; i < 2u; ++i) {
        nvt_fw_partition_t *p = &out->parts[i];
        memcpy(p->name, (i == 0u) ? "ILM" : "DLM", 4u);
        p->bin_addr  = rd_le32(fw + i * 12u + 0u);
        p->sram_addr = rd_le32(fw + i * 12u + 4u);
        p->size      = rd_le32(fw + i * 12u + 8u);
        p->crc       = rd_le32(fw + 0x18u + i * 4u);
        if (p->size > 0u && ((size_t)p->bin_addr + (size_t)p->size + 1u > fw_len)) {
            return -22;
        }
    }

    /* Extra partitions (2 .. 2 + extra_cnt - 1) & overlay partitions */
    for (uint32_t i = 2u; i < total; ++i) {
        uint32_t off = tbl_off + (i - 2u) * 16u;
        if ((size_t)off + 16u > fw_len) break;
        nvt_fw_partition_t *p = &out->parts[i];
        memcpy(p->name, (i < 2u + extra_cnt) ? "EXT" : "OVL", 4u);
        p->sram_addr = rd_le32(fw + off + 0u);
        p->size      = rd_le32(fw + off + 4u);
        p->bin_addr  = rd_le32(fw + off + 8u);
        p->crc       = rd_le32(fw + off + 12u);
        if (p->size > 0u && ((size_t)p->bin_addr + (size_t)p->size + 1u > fw_len)) {
            return -22;
        }
    }

    return 0;
}

int nvt_boot_ready(const nvt_bus_t *bus, uint8_t hw_crc) {
    /* nt36532.ko L0xa24..0xa94: write 1 to SPI_DMA_TX_INFO (0x1FB50D), delay 5ms. */
    int rc = nvt_write_u8(bus, NVT36532_SPI_DMA_TX_INFO, 0x01u);
    if (rc != 0) return rc;
    msleep_wrap(bus, 5u);
    if (!hw_crc) {
        (void)nvt_write_u8(bus, NVT36532_SPI_DMA_TX_INFO, 0x00u);
    }
    return 0;
}

int nvt_check_fw_reset_state(const nvt_bus_t *bus, uint8_t min_state) {
    /* nt36532.ko L0x117c..0x126c: read 5 bytes at EVENT_BUF_ADDR + 0x60; state in [min_state..0xAF]. */
    int retries = 50;
    for (int i = 0; i < retries; ++i) {
        uint8_t buf[5] = {0};
        if (nvt_read_bytes(bus, NVT36532_EVENT_BUF_ADDR + NVT36532_RESET_STATE_OFF, buf, 5u) == 0) {
            if (buf[0] >= min_state && buf[0] <= 0xAFu) {
                return 0;
            }
        }
        msleep_wrap(bus, 10u);
    }
    return -110;
}

int nvt_get_fw_info(const nvt_bus_t *bus, nvt_fw_info_t *out) {
    if (!bus || !out) return -1;
    memset(out, 0, sizeof(*out));
    for (int retry = 0; retry < 3; ++retry) {
        uint8_t buf[16] = {0};
        if (nvt_read_bytes(bus, NVT36532_EVENT_BUF_ADDR + NVT36532_FW_INFO_OFF, buf, 16u) != 0) {
            continue;
        }
        /* nt36532.ko L1418..1428: fw_ver + (~fw_ver) == 0xFF check (buf[0] + buf[1] == 0xFF). */
        if ((uint8_t)(buf[0] + buf[1]) == 0xFFu) {
            out->fw_ver         = buf[0];
            out->x_num          = buf[2];
            out->y_num          = buf[3];
            out->abs_x_max      = (uint16_t)(((uint16_t)buf[4] << 8) | buf[5]);
            out->abs_y_max      = (uint16_t)(((uint16_t)buf[6] << 8) | buf[7]);
            out->max_button_num = buf[10];
            out->pid            = (uint16_t)(((uint16_t)buf[12] << 8) | buf[11]);
            return 0;
        }
        msleep_wrap(bus, 10u);
    }
    return -5;
}

static int nvt_set_bld_crc_bank(const nvt_bus_t *bus,
                                uint32_t sram_reg, uint32_t sram_addr,
                                uint32_t len_reg,  uint32_t len_val,
                                uint32_t crc_reg,  uint32_t crc_val) {
    uint8_t b3[3] = { (uint8_t)sram_addr, (uint8_t)(sram_addr >> 8), (uint8_t)(sram_addr >> 16) };
    if (nvt_write_bytes(bus, sram_reg, b3, 3u) != 0) return -1;
    uint8_t l3[3] = { (uint8_t)len_val, (uint8_t)(len_val >> 8), (uint8_t)(len_val >> 16) };
    if (nvt_write_bytes(bus, len_reg, l3, 3u) != 0) return -2;
    uint8_t c4[4] = { (uint8_t)crc_val, (uint8_t)(crc_val >> 8), (uint8_t)(crc_val >> 16), (uint8_t)(crc_val >> 24) };
    if (nvt_write_bytes(bus, crc_reg, c4, 4u) != 0) return -3;
    return 0;
}

int nvt_download_firmware(const nvt_bus_t *bus, const nvt_chip_id_t *id,
                          const uint8_t *fw, size_t fw_len, nvt_fw_info_t *info_out) {
    if (!bus || !id || !fw || fw_len < 4096u) return -22;

    nvt_fw_plan_t plan;
    int rc = nvt_fw_parse(fw, fw_len, &plan);
    if (rc != 0) return rc;

    /* 1. Reset bootloader and clear FW status register (EVENT_BUF_ADDR + 0x60 = 0x00). */
    rc = nvt_bootloader_reset(bus, id->swrst_addr, id->spi_rd_fast_addr);
    if (rc != 0) return rc;
    (void)nvt_write_u8(bus, NVT36532_EVENT_BUF_ADDR + NVT36532_RESET_STATE_OFF, 0x00u);

    /* 1a. Stock nt36532.ko (0x6dc4, 0x6de8): program NT36532 hardware CRC banks for ILM & DLM (hw_crc=2). */
    if (nvt_set_bld_crc_bank(bus, 0x001FB528u, plan.parts[0].sram_addr,
                             0x001FB518u, plan.parts[0].size,
                             0x001FB500u, plan.parts[0].crc) != 0) return -30;
    if (nvt_set_bld_crc_bank(bus, 0x001FB52Cu, plan.parts[1].sram_addr,
                             0x001FB530u, plan.parts[1].size,
                             0x001FB504u, plan.parts[1].crc) != 0) return -30;

    /* 1b. Stock nt36532.ko (0x6ed0): nvt_tx_auto_copy_mode() for NT36532 (auto_copy=2 -> write 0x56 to 0x1FC925). */
    rc = nvt_write_u8(bus, 0x001FC925u, 0x56u);
    if (rc != 0) return -31;

    /* 2. Download each non-empty partition to NT36532 SRAM (size + 1 bytes). */
    for (uint8_t p = 0; p < plan.partition_count; ++p) {
        const nvt_fw_partition_t *part = &plan.parts[p];
        if (part->size == 0u) continue;
        size_t total = (size_t)part->size + 1u;
        size_t done  = 0;
        while (done < total) {
            size_t chunk = total - done;
            if (chunk > NVT_FW_CHUNK_BYTES) chunk = NVT_FW_CHUNK_BYTES;
            rc = nvt_write_bytes(bus, part->sram_addr + (uint32_t)done,
                                 fw + part->bin_addr + done, chunk);
            if (rc != 0) return -32;
            done += chunk;
        }
    }

    /* 2a. Also write the two 256B info header partitions at 0x0f0 / 0x1f0 (if present in bin). */
    if (fw_len >= 0x200u) {
        const uint32_t info_offs[2] = { 0x0F0u, 0x1F0u };
        for (int k = 0; k < 2; ++k) {
            uint32_t sram_a = rd_le32(fw + info_offs[k] + 0u);
            uint32_t sz_m1  = rd_le32(fw + info_offs[k] + 4u);
            uint32_t bin_a  = rd_le32(fw + info_offs[k] + 8u);
            size_t   len    = (size_t)sz_m1 + 1u;
            if (sram_a >= 0x100000u && sz_m1 > 0u && len <= NVT_FW_CHUNK_BYTES &&
                (size_t)bin_a + len <= fw_len) {
                (void)nvt_write_bytes(bus, sram_a, fw + bin_a, len);
            }
        }
    }

    /* 2a2. Verify DLM (part[1]) readback in 256B blocks and auto-rewrite any mismatched block! */
    if (plan.partition_count >= 2u && plan.parts[1].size > 0u) {
        const nvt_fw_partition_t *dlm = &plan.parts[1];
        size_t total = (size_t)dlm->size + 1u;
        uint8_t rb[256];
        for (size_t off = 0; off < total; off += 256u) {
            size_t blk = total - off;
            if (blk > 256u) blk = 256u;
            if (nvt_read_bytes(bus, dlm->sram_addr + (uint32_t)off, rb, blk) == 0 &&
                memcmp(rb, fw + dlm->bin_addr + off, blk) != 0) {
                /* Retry writing this 256B block up to 2 times */
                for (int r = 0; r < 2; ++r) {
                    (void)nvt_write_bytes(bus, dlm->sram_addr + (uint32_t)off, fw + dlm->bin_addr + off, blk);
                    if (nvt_read_bytes(bus, dlm->sram_addr + (uint32_t)off, rb, blk) == 0 &&
                        memcmp(rb, fw + dlm->bin_addr + off, blk) == 0) {
                        break;
                    }
                }
            }
        }
    }

    /* 2b. Stock nt36532.ko (0x6ee4): nvt_wait_auto_copy() — poll 0x1FC925 == 0x00 (up to 200ms). */
    {
        int ac_ok = 0;
        for (int tries = 0; tries < 200; ++tries) {
            uint8_t ac_val = 0xFFu;
            if (nvt_read_bytes(bus, 0x001FC925u, &ac_val, 1u) == 0 && ac_val == 0x00u) {
                ac_ok = 1;
                break;
            }
            msleep_wrap(bus, 1u);
        }
        if (!ac_ok) return -33;
    }

    /* 2c. Stock nt36532.ko (0x6eec): nvt_fw_crc_enable() — zero 6B at 0x125860, write [0xAE, 0x00] at 0x125850. */
    {
        static const uint8_t z6[6] = {0, 0, 0, 0, 0, 0};
        static const uint8_t crc_cmd[2] = {0xAEu, 0x00u};
        (void)nvt_write_bytes(bus, NVT36532_EVENT_BUF_ADDR + NVT36532_RESET_STATE_OFF, z6, sizeof(z6));
        (void)nvt_write_bytes(bus, NVT36532_EVENT_BUF_ADDR + 0x50u, crc_cmd, sizeof(crc_cmd));
    }

    /* 3. Enable bootloader CRC & release MCU to run SRAM firmware (nvt_boot_ready). */
    rc = nvt_boot_ready(bus, plan.ilm_dlm_crc_hw);
    if (rc != 0) return -34;

    /* 4. Wait for firmware reset state >= 0xA0 (RESET_STATE_INIT). */
    rc = nvt_check_fw_reset_state(bus, 0xA0u);
    if (rc != 0) return -35;

    /* 5. Read live firmware version & PID. */
    if (info_out) {
        rc = nvt_get_fw_info(bus, info_out);
        if (rc != 0) return -36;
    }
    return 0;
}

/* ---- Coordinate Decoder (verified from nvt_ts_work_func L40cc..4514) ---- */

void nvt_transform_coords(uint16_t raw_x, uint16_t raw_y, nvt_rotation_t rot,
                          uint16_t *disp_x, uint16_t *disp_y) {
    uint32_t rx = (raw_x > TB375FC_NVT_RAW_X_MAX) ? TB375FC_NVT_RAW_X_MAX : raw_x;
    uint32_t ry = (raw_y > TB375FC_NVT_RAW_Y_MAX) ? TB375FC_NVT_RAW_Y_MAX : raw_y;
    uint32_t dx = 0, dy = 0;

    switch (rot) {
        case NVT_ROT_LANDSCAPE_CW:
            dx = ry / 10u;
            dy = (TB375FC_NVT_RAW_X_MAX - rx) / 10u;
            break;
        case NVT_ROT_LANDSCAPE_CCW:
            dx = (TB375FC_NVT_RAW_Y_MAX - ry) / 10u;
            dy = rx / 10u;
            break;
        case NVT_ROT_LANDSCAPE_XY:
            dx = ry / 10u;
            dy = rx / 10u;
            break;
        case NVT_ROT_LANDSCAPE_INV:
        default:
            dx = (TB375FC_NVT_RAW_Y_MAX - ry) / 10u;
            dy = (TB375FC_NVT_RAW_X_MAX - rx) / 10u;
            break;
    }
    if (dx >= TB375FC_DISP_WIDTH)  dx = TB375FC_DISP_WIDTH - 1u;
    if (dy >= TB375FC_DISP_HEIGHT) dy = TB375FC_DISP_HEIGHT - 1u;
    if (disp_x) *disp_x = (uint16_t)dx;
    if (disp_y) *disp_y = (uint16_t)dy;
}

int nvt_decode_packet(const uint8_t payload[NVT_COORD_PAYLOAD_LEN], nvt_rotation_t rot, nvt_frame_t *out) {
    if (!payload || !out) return -1;
    memset(out, 0, sizeof(*out));

    /* 1. ESD reset check (nt36532.ko L40cc..4154): payload[0..5] all 0xFD or 0xFE. */
    int esd = 1;
    for (int i = 0; i < 6; ++i) {
        uint8_t b = (uint8_t)(payload[i] + 3u);
        if (b > 1u) { esd = 0; break; }
    }
    if (esd) {
        out->esd_reset_req = 1u;
        return 0;
    }

    /* 2. 64-byte packet checksum (nt36532.ko L42c8..42f4): payload[64] == (uint8_t)(-sum(payload[0..63])). */
    uint8_t sum = 0;
    for (size_t i = 0; i < NVT_CHECKSUM_LEN; ++i) {
        sum = (uint8_t)(sum + payload[i]);
    }
    uint8_t expected = (uint8_t)(0u - sum);
    if (payload[NVT_CHECKSUM_LEN] != expected) {
        out->checksum_ok = 0u;
        return -74; /* EBADMSG */
    }
    out->checksum_ok = 1u;

    /* 3. Decode up to 10 finger slots (stride 6 bytes starting at payload[0]). */
    uint8_t count = 0;
    for (uint32_t i = 0; i < NVT_MAX_SLOTS; ++i) {
        const uint8_t *slot = payload + i * 6u;
        uint8_t hdr = slot[0];
        if (hdr < 8u) continue;
        uint8_t id1 = (uint8_t)(hdr >> 3);
        if (id1 == 0u || id1 > NVT_MAX_SLOTS) continue;
        uint8_t status = hdr & 0x07u;
        if (status != 1u && status != 2u) continue; /* 1=DOWN, 2=MOVE */

        uint16_t raw_x = rd_be16(slot + 1u); /* nt36532.ko w20 -> ABS_MT_POSITION_X (0..18400) */
        uint16_t raw_y = rd_be16(slot + 3u); /* nt36532.ko w19 -> ABS_MT_POSITION_Y (0..29440) */
        if (raw_x > TB375FC_NVT_RAW_X_MAX || raw_y > TB375FC_NVT_RAW_Y_MAX) continue;

        uint8_t major = slot[5];
        if (major == 0u) major = 1u;

        nvt_contact_t *c = &out->contacts[count++];
        c->slot_id = (uint8_t)(id1 - 1u);
        c->active  = 1u;
        c->major   = major;
        c->raw_x   = raw_x;
        c->raw_y   = raw_y;
        nvt_transform_coords(raw_x, raw_y, rot, &c->disp_x, &c->disp_y);
    }
    out->contact_count = count;
    return 0;
}

int nvt_read_frame(const nvt_bus_t *bus, nvt_rotation_t rot, nvt_frame_t *out) {
    uint8_t payload[NVT_COORD_PAYLOAD_LEN];
    int rc = nvt_read_bytes(bus, NVT36532_EVENT_BUF_ADDR, payload, sizeof(payload));
    if (rc != 0) return rc;
    return nvt_decode_packet(payload, rot, out);
}

/* ---- Multi-Touch to macOS 6-byte HID Absolute Pointer State Machine ---- */

void nvt_pointer_init(nvt_pointer_state_t *st) {
    if (!st) return;
    memset(st, 0, sizeof(*st));
    st->last_x = TB375FC_DISP_WIDTH / 2u;
    st->last_y = TB375FC_DISP_HEIGHT / 2u;
}

int nvt_pointer_update(nvt_pointer_state_t *st, const nvt_frame_t *frame, uint64_t now_ms,
                       nvt_hid_pointer_report_t reports_out[2]) {
    if (!st || !frame || !reports_out || !frame->checksum_ok || frame->esd_reset_req) {
        return 0;
    }
    int nrep = 0;

    if (frame->contact_count == 0u) {
        if (st->mode == 2u && !st->two_moved && (now_ms >= st->two_start_ms) && (now_ms - st->two_start_ms) <= 300u) {
            /* Quick 2-finger tap -> right-click press + release! */
            reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x02u, st->last_x, st->last_y, 0 };
            reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x00u, st->last_x, st->last_y, 0 };
        } else if (st->last_buttons != 0u) {
            /* Lift finger -> release button at last known coordinate. */
            reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x00u, st->last_x, st->last_y, 0 };
        }
        st->mode             = 0u;
        st->last_buttons     = 0u;
        st->two_moved        = 0u;
        st->two_scroll_accum = 0;
        return nrep;
    }

    if (frame->contact_count == 1u) {
        if (st->mode == 2u) {
            /* Transitioning out of 2-finger gesture: wait for full lift before left-clicking. */
            if (st->last_buttons != 0u) {
                st->last_buttons = 0u;
                reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x00u, st->last_x, st->last_y, 0 };
            }
            return nrep;
        }
        uint16_t x = frame->contacts[0].disp_x;
        uint16_t y = frame->contacts[0].disp_y;
        st->mode         = 1u;
        st->last_x       = x;
        st->last_y       = y;
        st->last_buttons = 0x01u; /* Left button down (supports tap and drag) */
        reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x01u, x, y, 0 };
        return nrep;
    }

    /* >= 2 fingers: scroll gesture or 2-finger right-click tap. */
    int32_t avg_x = ((int32_t)frame->contacts[0].disp_x + (int32_t)frame->contacts[1].disp_x) / 2;
    int32_t avg_y = ((int32_t)frame->contacts[0].disp_y + (int32_t)frame->contacts[1].disp_y) / 2;
    if (st->mode != 2u) {
        /* Release left button immediately if a second finger just landed. */
        if (st->last_buttons != 0u) {
            st->last_buttons = 0u;
            reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x00u, st->last_x, st->last_y, 0 };
        }
        st->mode             = 2u;
        st->last_x           = (uint16_t)avg_x;
        st->last_y           = (uint16_t)avg_y;
        st->two_start_y      = avg_y;
        st->two_last_y       = avg_y;
        st->two_start_ms     = now_ms;
        st->two_moved        = 0u;
        st->two_scroll_accum = 0;
        return nrep;
    }

    int32_t dy = avg_y - st->two_last_y;
    st->two_last_y = avg_y;
    int32_t total_dy = avg_y - st->two_start_y;
    if (total_dy > 24 || total_dy < -24) {
        st->two_moved = 1u;
    }
    st->two_scroll_accum += dy;
    /* Emit 1 wheel notch per 20 display pixels of 2-finger drag. */
    int32_t notches = st->two_scroll_accum / 20;
    if (notches != 0) {
        st->two_scroll_accum -= notches * 20;
        if (notches > 15)  notches = 15;
        if (notches < -15) notches = -15;
        reports_out[nrep++] = (nvt_hid_pointer_report_t){ 0x00u, st->last_x, st->last_y, (int8_t)notches };
    }
    return nrep;
}
