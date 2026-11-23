#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/upstream/drivers/RamBlock/PersistOverlay.h"

namespace {
constexpr uint32_t kHardFloor = 54220000u;
constexpr uint32_t kMaxBlocks = 16u;

struct BoundedMockUfs {
    bool locked = false;
    uint8_t winLun = 0xffu;
    uint32_t winFirst = 0, winBlocks = 0;
    std::vector<uint8_t> durable;   // committed on flash
    std::vector<uint8_t> volatile_; // in device write cache (lost on simulated power loss)
    uint64_t writes = 0, syncs = 0, policyRejects = 0;

    explicit BoundedMockUfs(uint32_t capBlocks)
        : durable((size_t)capBlocks * persist::kBlock, 0),
          volatile_((size_t)capBlocks * persist::kBlock, 0) {}

    bool setWindow(uint8_t lun, uint32_t first, uint32_t blocks) {
        if (locked || lun != 2 || !blocks || first < kHardFloor || first > 0xffffffffu - blocks) {
            ++policyRejects;
            return false;
        }
        winLun = lun; winFirst = first; winBlocks = blocks; locked = true;
        return true;
    }

    bool allowed(uint8_t lun, uint32_t lba, uint32_t count) const {
        if (!locked || lun != winLun || !count || count > kMaxBlocks || lba < kHardFloor || lba < winFirst) return false;
        uint32_t off = lba - winFirst;
        return off < winBlocks && count <= winBlocks - off;
    }

    bool readAbs(uint32_t lba, uint32_t count, void *dst) const {
        if (lba < winFirst || lba - winFirst + count > winBlocks) return false;
        size_t off = (size_t)(lba - winFirst) * persist::kBlock;
        memcpy(dst, volatile_.data() + off, (size_t)count * persist::kBlock);
        return true;
    }

    bool writeAbs(uint32_t lba, uint32_t count, const void *src, bool fua) {
        if (!allowed(2, lba, count)) { ++policyRejects; return false; }
        size_t off = (size_t)(lba - winFirst) * persist::kBlock;
        size_t bytes = (size_t)count * persist::kBlock;
        memcpy(volatile_.data() + off, src, bytes);
        if (fua) memcpy(durable.data() + off, src, bytes);
        ++writes;
        return true;
    }

    bool syncAbs() {
        if (!locked) { ++policyRejects; return false; }
        durable = volatile_;
        ++syncs;
        return true;
    }

    void simulatePowerLoss() {
        volatile_ = durable;
    }
};

struct OverlayOps {
    BoundedMockUfs &ufs;
    bool writeThrough;

    bool read(uint64_t relBlock, uint32_t count, void *dst) {
        uint8_t *p = (uint8_t *)dst;
        while (count > 0) {
            uint32_t chunk = count > kMaxBlocks ? kMaxBlocks : count;
            if (!ufs.readAbs(ufs.winFirst + (uint32_t)relBlock, chunk, p)) return false;
            relBlock += chunk; p += (size_t)chunk * persist::kBlock; count -= chunk;
        }
        return true;
    }
    bool write(uint64_t relBlock, uint32_t count, const void *src) {
        const uint8_t *p = (const uint8_t *)src;
        while (count > 0) {
            uint32_t chunk = count > kMaxBlocks ? kMaxBlocks : count;
            if (!ufs.writeAbs(ufs.winFirst + (uint32_t)relBlock, chunk, p, writeThrough)) return false;
            relBlock += chunk; p += (size_t)chunk * persist::kBlock; count -= chunk;
        }
        return true;
    }
    bool sync() { return ufs.syncAbs(); }
};
} // namespace

int main() {
    constexpr uint64_t kPages = 256;
    const uint64_t winBlocks = persist::windowBlocks(kPages);
    BoundedMockUfs ufs((uint32_t)winBlocks);
    uint8_t dummy[persist::kBlock] = {0x5a};

    // 1. Policy guard tests: writes disabled before window lock; bad windows rejected
    assert(!ufs.writeAbs(kHardFloor, 1, dummy, true));
    assert(!ufs.syncAbs());
    assert(!ufs.setWindow(0, kHardFloor, (uint32_t)winBlocks));         // wrong LUN
    assert(!ufs.setWindow(2, kHardFloor - 1, (uint32_t)winBlocks));     // below hard floor (touches RO roots!)
    assert(!ufs.setWindow(2, 0xffffffffu - 4, 16));                     // LBA wrap-around
    assert(ufs.setWindow(2, kHardFloor, (uint32_t)winBlocks));
    assert(!ufs.setWindow(2, kHardFloor + 1000, (uint32_t)winBlocks));  // write-once lock
    // Out-of-bounds write attempts after lock
    assert(!ufs.writeAbs(0, 1, dummy, true));                           // GPT LBA 0
    assert(!ufs.writeAbs(49914368u, 1, dummy, true));                   // OS root
    assert(!ufs.writeAbs(kHardFloor + (uint32_t)winBlocks, 1, dummy, true));
    assert(!ufs.writeAbs(kHardFloor, kMaxBlocks + 1, dummy, true));

    // 2. PersistOverlay lifecycle: blank -> Kept (no reset) -> Fresh (reset) -> write-back -> crash -> resume
    uint8_t seedA[32], seedB[32];
    memset(seedA, 0xA5, sizeof(seedA));
    memset(seedB, 0x5A, sizeof(seedB));
    std::vector<uint8_t> bits(persist::bitmapBlocks(kPages) * persist::kBlock, 0);
    std::vector<uint8_t> dirty(persist::bitmapBlocks(kPages), 0);

    OverlayOps wbOps{ufs, false};
    persist::Overlay ov;
    ov.dirty = dirty.data();
    assert(ov.open(wbOps, kPages, seedA, false, bits.data(), false) == persist::Open::Kept);
    assert(ov.open(wbOps, kPages, seedA, false, bits.data(), true) == persist::Open::Fresh);
    assert(ov.generation == 1);

    // Write page 7 and flush (becomes durable)
    uint8_t page7[persist::kBlock], page9[persist::kBlock];
    memset(page7, 0x77, sizeof(page7));
    memset(page9, 0x99, sizeof(page9));
    assert(ov.writePage(wbOps, 7, page7));
    uint64_t flushedBmp = 0;
    assert(ov.flush(wbOps, &flushedBmp) && flushedBmp == 1);

    // Write page 9 WITHOUT flushing, then simulate sudden power loss!
    assert(ov.writePage(wbOps, 9, page9));
    ufs.simulatePowerLoss();

    // Cold reboot with same seedA: page 7 MUST persist, page 9 MUST NOT be torn/present!
    persist::Overlay ov2;
    ov2.dirty = dirty.data();
    assert(ov2.open(wbOps, kPages, seedA, false, bits.data(), false) == persist::Open::Resumed);
    assert(ov2.present == 1 && ov2.has(7) && !ov2.has(9));

    uint8_t scratch[kMaxBlocks * persist::kBlock];
    int loaded = 0;
    auto loader = [&](uint64_t idx, const uint8_t *buf) -> bool {
        assert(idx == 7 && memcmp(buf, page7, persist::kBlock) == 0);
        ++loaded;
        return true;
    };
    assert(ov2.forEachPresent(wbOps, loader, scratch, kMaxBlocks) && loaded == 1);

    // 3. Seed mismatch protection: opening with seedB without reset MUST return Kept (never clobber seedA!)
    persist::Overlay ov3;
    assert(ov3.open(wbOps, kPages, seedB, false, bits.data(), false) == persist::Open::Kept);
    assert(ov3.header == 3);

    // 4. Write-through mode (FUA) survives immediate power loss without explicit flush()
    OverlayOps wtOps{ufs, true};
    persist::Overlay ov4;
    assert(ov4.open(wtOps, kPages, seedA, false, bits.data(), false) == persist::Open::Resumed);
    assert(ov4.writePage(wtOps, 9, page9));
    ufs.simulatePowerLoss();
    persist::Overlay ov5;
    assert(ov5.open(wtOps, kPages, seedA, false, bits.data(), false) == persist::Open::Resumed);
    assert(ov5.present == 2 && ov5.has(7) && ov5.has(9));

    printf("PASS: UFS write-window policy guard (%llu rejects) + PersistOverlay (WB crash-safe + WT FUA + seed-isolation, writes=%llu syncs=%llu)\n",
           (unsigned long long)ufs.policyRejects, (unsigned long long)ufs.writes, (unsigned long long)ufs.syncs);
    return 0;
}
