/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * TB375FC / MT6897 bounded UFS LUN2 persistence store for NativeRamBlock PersistOverlay (P2).
 *
 * Boot-arg gated:
 *   data-persist=0 (default): completely disabled (volatile RAM Data disk)
 *   data-persist=1          : strictly READ-ONLY probe (verifies live GPT + samples target LBA window, ZERO writes)
 *   data-persist=2          : bounded READ-WRITE PersistOverlay on LUN2 LBA >= 54,220,000
 *
 * Safety guarantees:
 *   - Hard floor LBA >= 54,220,000 (31.875 MiB above macOS Desktop RO roots end 54,211,840).
 *   - Live LUN2 GPT verification (LBA 1 "EFI PART" + sdc80 "userdata" == [2,803,712 .. 62,410,743]) required before open.
 *   - Upper bound strictly <= 62,410,744 (never touches mrdump, flashinfo, or backup GPT).
 *   - Underlying Tb375fcUfsTransport enforces write-once window lock and rejects any out-of-range WRITE(10).
 */
#include <IOKit/IOLib.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>
#include "Tb375fcUfsTransport.h"

extern "C" {
extern volatile uint64_t native_data_persist_writes;
extern volatile uint64_t native_data_persist_syncs;
}

namespace {
constexpr uint32_t kPersistHighFirstLba  = 54220000u; /* <=31.24 GiB window strictly above macOS RO roots [54220000..62410743] */
constexpr uint32_t kPersistHighMaxBlocks = 8190744u;
constexpr uint32_t kPersistLowFirstLba   = 16350000u; /* 128 GiB window strictly below macOS RO roots [16350000..49906175] */
constexpr uint32_t kPersistLowCeilExcl   = 49906176u; /* 31.0 MiB hard guard below kDesktopManifestLba (49,914,112) */
constexpr uint32_t kPersistLowMaxBlocks  = kPersistLowCeilExcl - kPersistLowFirstLba; /* 33,556,176 blocks = 128.0066 GiB */
constexpr uint32_t kUserdataFirstLba     = 2803712u;
constexpr uint32_t kUserdataLastLba      = 62410743u;
constexpr unsigned kBusyRetries          = 2000u;

static uint32_t g_mode = 0;
static uint32_t g_first_lba = 0;
static uint32_t g_win_blocks = 0;
static bool     g_opened = false;

static uint32_t rd_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t rd_le64(const uint8_t *p)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < 8; ++i) v |= (uint64_t)p[i] << (i * 8);
    return v;
}

static int ufs_read_retry(uint32_t lba, uint32_t count, void *dst)
{
    for (unsigned r = 0; ; ++r) {
        int rc = tb375fc_ufs_read_blocks(lba, count, dst);
        if (rc != -(int)TB375FC_UFS_BUSY || r >= kBusyRetries) return rc;
        IOSleep(1);
    }
}

static int ufs_write_retry(uint32_t lba, uint32_t count, const void *src, int fua)
{
    for (unsigned r = 0; ; ++r) {
        int rc = tb375fc_ufs_write_blocks(lba, count, src, fua);
        if (rc != -(int)TB375FC_UFS_BUSY || r >= kBusyRetries) return rc;
        IOSleep(1);
    }
}

static int ufs_sync_retry(void)
{
    for (unsigned r = 0; ; ++r) {
        int rc = tb375fc_ufs_sync_cache();
        if (rc != -(int)TB375FC_UFS_BUSY || r >= kBusyRetries) return rc;
        IOSleep(1);
    }
}

static uint32_t count_nonzero(const uint8_t *buf, size_t n)
{
    uint32_t nz = 0;
    for (size_t i = 0; i < n; ++i) {
        if (buf[i] != 0) ++nz;
    }
    return nz;
}
} // namespace

extern "C" {

unsigned native_ufs_store_mode(void)
{
    uint32_t mode = 0;
    if (!PE_parse_boot_argn("data-persist", &mode, sizeof(mode))) return 0;
    if (mode == 1u || mode == 2u) return mode;
    return 0;
}

/* Returns 0 on verified LUN2 GPT match, non-zero on any mismatch or I/O error. */
unsigned native_ufs_store_gpt_check(void)
{
    uint8_t *blk = static_cast<uint8_t *>(IOMalloc(4096));
    if (!blk) return 10u;

    int rc = ufs_read_retry(1u, 1u, blk);
    if (rc != 0) {
        IOFree(blk, 4096);
        return 1u;
    }
    if (memcmp(blk, "EFI PART", 8) != 0) {
        IOFree(blk, 4096);
        return 2u;
    }
    const uint64_t entry_lba  = rd_le64(blk + 72);
    const uint32_t num_entries = rd_le32(blk + 80);
    const uint32_t entry_size = rd_le32(blk + 84);
    if (entry_lba != 2ULL || entry_size != 128u || num_entries < 80u || num_entries > 256u) {
        IOFree(blk, 4096);
        return 3u;
    }

    /* Locate GPT partition named "userdata" and verify its exact LBA bounds. */
    bool found = false;
    uint64_t ud_first = 0, ud_last = 0;
    const uint32_t per_block = 4096u / entry_size;
    const uint32_t table_blocks = (num_entries + per_block - 1u) / per_block;
    for (uint32_t b = 0; b < table_blocks && !found; ++b) {
        if (ufs_read_retry((uint32_t)entry_lba + b, 1u, blk) != 0) {
            IOFree(blk, 4096);
            return 4u;
        }
        for (uint32_t e = 0; e < per_block; ++e) {
            const uint32_t idx = b * per_block + e;
            if (idx >= num_entries) break;
            const uint8_t *ent = blk + e * entry_size;
            /* UTF-16LE "userdata\0" at offset 56 */
            static const char kTarget[9] = "userdata";
            bool match = true;
            for (unsigned c = 0; c < 9; ++c) {
                uint16_t ch = (uint16_t)ent[56 + c * 2] | ((uint16_t)ent[57 + c * 2] << 8);
                if (ch != (uint16_t)kTarget[c]) { match = false; break; }
            }
            if (match) {
                ud_first = rd_le64(ent + 32);
                ud_last  = rd_le64(ent + 40);
                found = true;
                break;
            }
        }
    }
    IOFree(blk, 4096);
    if (!found) return 5u;
    if (ud_first != kUserdataFirstLba || ud_last != kUserdataLastLba) {
        IOLog("Tb375fcPersistStore: GPT userdata mismatch got=[%llu..%llu] want=[%u..%u]\n",
              ud_first, ud_last, kUserdataFirstLba, kUserdataLastLba);
        return 6u;
    }
    return 0u;
}

int native_ufs_store_open(uint64_t window_blocks, uint64_t *first_lba_out, const char **why_out)
{
    const uint32_t mode = native_ufs_store_mode();
    if (mode != 1u && mode != 2u) {
        if (why_out) *why_out = "data-persist not enabled";
        return 0;
    }
    uint32_t chosen_first = 0;
    uint64_t zone_ceil_excl = 0;
    if (window_blocks > 0 && window_blocks <= kPersistHighMaxBlocks) {
        chosen_first = kPersistHighFirstLba;
        zone_ceil_excl = (uint64_t)kUserdataLastLba + 1ULL;
    } else if (window_blocks > kPersistHighMaxBlocks && window_blocks <= kPersistLowMaxBlocks) {
        chosen_first = kPersistLowFirstLba;
        zone_ceil_excl = (uint64_t)kPersistLowCeilExcl;
    } else {
        if (why_out) *why_out = "invalid window_blocks";
        return 0;
    }
    const unsigned gpt = native_ufs_store_gpt_check();
    if (gpt != 0u) {
        if (why_out) *why_out = "live LUN2 GPT check failed";
        return 0;
    }
    const uint64_t end_lba_excl = (uint64_t)chosen_first + window_blocks;
    if (end_lba_excl > zone_ceil_excl) {
        if (why_out) *why_out = "window exceeds safe zone ceiling";
        return 0;
    }

    /* Read-only diagnostic sample of 5 positions across the target window before any write. */
    uint8_t *sample = static_cast<uint8_t *>(IOMalloc(4096));
    uint32_t sample_nz[5] = {0, 0, 0, 0, 0};
    char hdr_sig[9] = "........";
    if (sample) {
        const uint32_t offsets[5] = {
            0u,
            (uint32_t)(window_blocks / 4u),
            (uint32_t)(window_blocks / 2u),
            (uint32_t)((window_blocks * 3u) / 4u),
            (uint32_t)(window_blocks - 1u),
        };
        for (unsigned i = 0; i < 5u; ++i) {
            if (ufs_read_retry(chosen_first + offsets[i], 1u, sample) == 0) {
                sample_nz[i] = count_nonzero(sample, 4096);
                if (i == 0u) {
                    for (unsigned c = 0; c < 8u; ++c) {
                        hdr_sig[c] = (sample[c] >= 32 && sample[c] < 127) ? (char)sample[c] : '.';
                    }
                    hdr_sig[8] = '\0';
                }
            } else {
                sample_nz[i] = 0xffffffffu;
            }
        }
        IOFree(sample, 4096);
    }

    if (mode == 2u) {
        int wrc = tb375fc_ufs_set_write_window(2u, chosen_first, (uint32_t)window_blocks);
        if (wrc != 0) {
            IOLog("Tb375fcPersistStore: set_write_window failed rc=%d\n", wrc);
            if (why_out) *why_out = "set_write_window rejected";
            return 0;
        }
    }

    g_mode = mode;
    g_first_lba = chosen_first;
    g_win_blocks = (uint32_t)window_blocks;
    g_opened = true;
    if (first_lba_out) *first_lba_out = chosen_first;
    IOLog("Tb375fcPersistStore: OPEN mode=%u (%s) lba=[%u..%u] blocks=%u gpt=0 hdr='%s' sample_nz=(%u,%u,%u,%u,%u)\n",
          mode, (mode == 2u) ? "RW" : "RO-PROBE",
          chosen_first, (uint32_t)(end_lba_excl - 1ULL), g_win_blocks, hdr_sig,
          sample_nz[0], sample_nz[1], sample_nz[2], sample_nz[3], sample_nz[4]);
    return 1;
}

int native_ufs_store_read(uint64_t rel_block, uint32_t count, void *dst)
{
    if (!g_opened || !dst || !count || rel_block >= g_win_blocks || count > g_win_blocks - rel_block) {
        return 0;
    }
    uint8_t *p = static_cast<uint8_t *>(dst);
    uint32_t lba = g_first_lba + (uint32_t)rel_block;
    while (count > 0u) {
        uint32_t chunk = (count > TB375FC_UFS_MAX_BLOCKS) ? TB375FC_UFS_MAX_BLOCKS : count;
        int rc = ufs_read_retry(lba, chunk, p);
        if (rc != 0) {
            IOLog("Tb375fcPersistStore: read failed lba=%u count=%u rc=%d\n", lba, chunk, rc);
            return 0;
        }
        lba += chunk;
        p += (size_t)chunk * 4096u;
        count -= chunk;
    }
    return 1;
}

static int store_write_common(uint64_t rel_block, uint32_t count, const void *src, int fua)
{
    if (!g_opened || g_mode != 2u || !src || !count || rel_block >= g_win_blocks || count > g_win_blocks - rel_block) {
        return 0;
    }
    const uint8_t *p = static_cast<const uint8_t *>(src);
    uint32_t lba = g_first_lba + (uint32_t)rel_block;
    uint32_t rem = count;
    while (rem > 0u) {
        uint32_t chunk = (rem > TB375FC_UFS_MAX_BLOCKS) ? TB375FC_UFS_MAX_BLOCKS : rem;
        int rc = ufs_write_retry(lba, chunk, p, fua);
        if (rc != 0) {
            IOLog("Tb375fcPersistStore: write(fua=%d) failed lba=%u count=%u rc=%d\n", fua, lba, chunk, rc);
            return 0;
        }
        lba += chunk;
        p += (size_t)chunk * 4096u;
        rem -= chunk;
    }
    __atomic_fetch_add(&native_data_persist_writes, count, __ATOMIC_RELAXED);
    return 1;
}

int native_ufs_store_write(uint64_t rel_block, uint32_t count, const void *src)
{
    return store_write_common(rel_block, count, src, 1);
}

int native_ufs_store_write_cached(uint64_t rel_block, uint32_t count, const void *src)
{
    return store_write_common(rel_block, count, src, 0);
}

int native_ufs_store_sync(void)
{
    if (!g_opened || g_mode != 2u) return 0;
    int rc = ufs_sync_retry();
    if (rc != 0) {
        IOLog("Tb375fcPersistStore: sync_cache failed rc=%d\n", rc);
        return 0;
    }
    __atomic_fetch_add(&native_data_persist_syncs, 1, __ATOMIC_RELAXED);
    return 1;
}

} // extern "C"
