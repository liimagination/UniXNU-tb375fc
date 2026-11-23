/* SPDX-License-Identifier: MIT */
/*
 * TB375FC Staged Mali-G615 (MT6897) Hardware GPU Bring-Up Probe.
 *
 * Strictly OFF by default! Enable via boot-args:
 *   mali=1 : Stage 1 — Read-only snapshot of GPUEB, MFG_RPC, MFG_PLL, GPUEB DRAM.
 *   mali=2 : Stage 2 — Init GPUEB shared mem (0xBABADADA), send CMD_POWER_CONTROL(ON),
 *            and read LIVE Mali-G615 GPU_ID (0xb8a31030) & SHADER_PRESENT (0x130011).
 *   mali=3 : Stage 3 — Soft-reset GPU, power on L2 cache, load /private/var/ngpu/mali_csffw.bin
 *            (290,816 B), build Arm64 4K LPAE page tables for AS0, boot the Arm Mali-G615
 *            CSF Microcontroller (MCU_CONTROL=AUTO), verify GLB_VERSION (v3.0) & CSG count,
 *            and execute a live Host <-> CSF MCU GLB_PING doorbell handshake!
 */
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

extern "C" {
struct vnode;
typedef struct vnode *vnode_t;
struct vfs_context;
typedef struct vfs_context *vfs_context_t;
struct ucred;
typedef struct ucred *kauth_cred_t;
struct proc;
typedef struct proc *proc_t;

vfs_context_t vfs_context_kernel(void);
kauth_cred_t  vfs_context_ucred(vfs_context_t ctx);
proc_t        vfs_context_proc(vfs_context_t ctx);
int  vnode_lookup(const char *path, int flags, vnode_t *vpp, vfs_context_t ctx);
int  vnode_put(vnode_t vp);
int  vnode_size(vnode_t vp, off_t *sizep, vfs_context_t ctx);
int  vn_rdwr(int rw, vnode_t vp, char *base, int len, off_t offset, int segflg,
             int ioflg, kauth_cred_t cred, int *aresid, proc_t p);
}

#ifndef UIO_READ
#define UIO_READ     0
#endif
#ifndef UIO_SYSSPACE
#define UIO_SYSSPACE 2
#endif

namespace {

constexpr uint64_t MT6897_MALI_PHYS        = 0x13000000ULL;
constexpr uint64_t MT6897_MALI_MAP_SIZE    = 0x00090000ULL; /* Covers Control, MMU, MCU, and Doorbell 0 (0x80000) */
constexpr uint64_t MT6897_GPUEB_GPR_PHYS   = 0x13C2F000ULL;
constexpr uint64_t MT6897_GPUEB_MBOX_PHYS  = 0x13C62000ULL;
constexpr uint64_t MT6897_MFG_RPC_PHYS     = 0x13F90000ULL;
constexpr uint64_t MT6897_GPUEB_DRAM_PHYS  = 0x7FE70000ULL;
constexpr uint32_t MT6897_GPUEB_DRAM_SIZE  = 0x4000u;

/* GPUEB GPR & MBOX0 */
constexpr uint32_t GPR_BASE_OFF   = 0x0D1Cu;
constexpr uint32_t GPR_LP_STATE   = 0x0028u;
constexpr uint32_t GPR_IPI_MAGIC  = 0x0034u;

constexpr uint32_t MBOX0_BASE_OFF      = 0x0D80u;
constexpr uint32_t MBOX_CH_GPUFREQ     = 1u;
constexpr uint32_t MBOX_GF_TX_OFF      = 0x0010u;
constexpr uint32_t MBOX_GF_RX_OFF      = 0x00E8u;
constexpr uint32_t MBOX_CTL_TX_STS     = 0x00u;
constexpr uint32_t MBOX_CTL_IRQ_SET    = 0x04u;
constexpr uint32_t MBOX_CTL_IRQ_CLR    = 0x74u;
constexpr uint32_t MBOX_CTL_RX_STS     = 0x78u;

/* MFG_RPC */
constexpr uint32_t RPC_PWR_CON         = 0x0504u;
constexpr uint32_t RPC_GHPM_CFG0_CON   = 0x0800u;
constexpr uint32_t RPC_GHPM_ENABLE_M   = (1u << 0);
constexpr uint32_t RPC_GHPM_ON_SEQ_M   = (1u << 2);
constexpr uint32_t RPC_GHPM_RO0_CON    = 0x09A4u;

/* GPUEB shared DRAM */
constexpr uint32_t GPUEB_MEM_MAGIC     = 0xBABADADAu;
constexpr uint32_t GF_REG_MAGIC        = 0x0000u;
constexpr uint32_t GF_REG_GPU_OPP_NUM  = 0x000Cu;
constexpr uint32_t GF_REG_POWER_COUNT  = 0x001Cu;
constexpr uint32_t GF_REG_FREQ_CUR_GPU = 0x0044u;
constexpr uint32_t GF_REG_VOLT_CUR_GPU = 0x005Cu;
constexpr uint32_t GF_REG_SHADER_PRES  = 0x00D8u;
constexpr uint32_t GF_REG_PWR_STATUS   = 0x0120u;

constexpr uint32_t CMD_INIT_SHARED_MEM = 0u;
constexpr uint32_t CMD_POWER_CONTROL   = 6u;

/* Mali-G615 (Arch 11.8 CSF) GPU / MMU / MCU / Doorbell offsets */
constexpr uint32_t GPU_INT_RAWSTAT     = 0x0020u;
constexpr uint32_t GPU_INT_CLEAR       = 0x0024u;
constexpr uint32_t GPU_IRQ_RESET_DONE  = (1u << 8);
constexpr uint32_t GPU_CMD             = 0x0030u;
constexpr uint32_t GPU_CMD_SOFT_RESET  = 0x0101u;
constexpr uint32_t GPU_STATUS          = 0x0034u;
constexpr uint32_t GPU_FAULT_STATUS    = 0x003Cu;
constexpr uint32_t GPU_FAULT_ADDR_LO   = 0x0040u;
constexpr uint32_t L2_READY_LO         = 0x0160u;
constexpr uint32_t L2_PWRON_LO         = 0x01A0u;
constexpr uint32_t L2_PWRTRANS_LO      = 0x0220u;
constexpr uint32_t GPU_COHERENCY_PROT  = 0x0304u;

constexpr uint32_t MCU_CONTROL         = 0x0700u;
constexpr uint32_t MCU_CONTROL_DISABLE = 0u;
constexpr uint32_t MCU_CONTROL_AUTO    = 2u;
constexpr uint32_t MCU_STATUS          = 0x0704u;

constexpr uint32_t JOB_INT_RAWSTAT     = 0x1000u;
constexpr uint32_t JOB_INT_CLEAR       = 0x1004u;
constexpr uint32_t JOB_INT_MASK        = 0x1008u;
constexpr uint32_t JOB_INT_GLOBAL_IF   = (1u << 31);

constexpr uint32_t MMU_INT_RAWSTAT     = 0x2000u;
constexpr uint32_t MMU_INT_CLEAR       = 0x2004u;
constexpr uint32_t MMU_INT_MASK        = 0x2008u;

constexpr uint32_t AS0_TRANSTAB_LO     = 0x2400u;
constexpr uint32_t AS0_TRANSTAB_HI     = 0x2404u;
constexpr uint32_t AS0_MEMATTR_LO      = 0x2408u;
constexpr uint32_t AS0_MEMATTR_HI      = 0x240Cu;
constexpr uint32_t AS0_COMMAND         = 0x2418u;
constexpr uint32_t AS_COMMAND_UPDATE   = 1u;
constexpr uint32_t AS0_FAULTSTATUS     = 0x241Cu;
constexpr uint32_t AS0_FAULTADDR_LO    = 0x2420u;
constexpr uint32_t AS0_FAULTADDR_HI    = 0x2424u;
constexpr uint32_t AS0_STATUS          = 0x2428u;
constexpr uint32_t AS0_TRANSCFG_LO     = 0x2430u;
constexpr uint32_t AS0_TRANSCFG_HI     = 0x2434u;

constexpr uint32_t CSF_GLB_DOORBELL    = 0x80000u;

struct __attribute__((packed)) GfIpiMsg {
    uint32_t magic;
    uint32_t cmd;
    uint32_t target;
    uint32_t reserved;
    union {
        int32_t  return_value;
        uint32_t power_state;
        struct __attribute__((packed)) {
            uint64_t base;
            uint32_t size;
            uint32_t pad;
        } shared_mem;
        uint32_t raw[4];
    } u;
};

struct MappedPage {
    IOMemoryDescriptor *desc;
    IOMemoryMap        *map;
    uintptr_t           va;
};

static bool mapMmio(uint64_t phys, uint64_t size, MappedPage *out) {
    out->desc = IOMemoryDescriptor::withPhysicalAddress(phys, size, kIODirectionInOut);
    if (!out->desc) return false;
    out->map = out->desc->map(kIOMapInhibitCache);
    if (!out->map) {
        out->desc->release();
        out->desc = nullptr;
        return false;
    }
    out->va = (uintptr_t)out->map->getVirtualAddress();
    return out->va != 0;
}

static void unmapMmio(MappedPage *m) {
    if (m->map) { m->map->release(); m->map = nullptr; }
    if (m->desc) { m->desc->release(); m->desc = nullptr; }
    m->va = 0;
}

static inline uint32_t rd32(uintptr_t base, uint32_t off) {
    uint32_t v = *reinterpret_cast<volatile uint32_t *>(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}

static inline void wr32(uintptr_t base, uint32_t off, uint32_t val) {
    __asm__ volatile("dsb st" ::: "memory");
    *reinterpret_cast<volatile uint32_t *>(base + off) = val;
    __asm__ volatile("dsb st" ::: "memory");
}

static inline void wr64(uintptr_t base, uint32_t off, uint64_t val) {
    wr32(base, off, (uint32_t)(val & 0xFFFFFFFFULL));
    wr32(base, off + 4u, (uint32_t)(val >> 32));
}

static inline void upd32(uintptr_t base, uint32_t off, uint32_t mask, uint32_t val) {
    wr32(base, off, (rd32(base, off) & ~mask) | (val & mask));
}

static void cacheFlushRange(const void *addr, size_t len) {
    uintptr_t start = reinterpret_cast<uintptr_t>(addr) & ~63ULL;
    uintptr_t end   = (reinterpret_cast<uintptr_t>(addr) + len + 63ULL) & ~63ULL;
    for (uintptr_t p = start; p < end; p += 64ULL) {
        __asm__ volatile("dc civac, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb sy\nisb" ::: "memory");
}

static uint8_t *loadMaliFwFromVfs(size_t *len_out) {
    static const char *const kPaths[] = {
        "/private/var/ngpu/mali_csffw.bin",
        "/Library/GPUBundles/mali_csffw.bin",
    };
    vfs_context_t ctx = vfs_context_kernel();
    for (int wait_s = 0; wait_s < 60; ++wait_s) {
        for (size_t i = 0; i < sizeof(kPaths) / sizeof(kPaths[0]); ++i) {
            vnode_t vp = nullptr;
            if (vnode_lookup(kPaths[i], 0, &vp, ctx) == 0 && vp) {
                off_t fsz = 0;
                if (vnode_size(vp, &fsz, ctx) == 0 && fsz >= 4096 && fsz <= 1024 * 1024) {
                    uint8_t *buf = static_cast<uint8_t *>(IOMallocData((size_t)fsz));
                    if (buf) {
                        int resid = 0;
                        int err = vn_rdwr(UIO_READ, vp, reinterpret_cast<char *>(buf), (int)fsz, 0,
                                          UIO_SYSSPACE, 0, vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
                        vnode_put(vp);
                        if (err == 0 && resid == 0) {
                            *len_out = (size_t)fsz;
                            return buf;
                        }
                        IOFreeData(buf, (size_t)fsz);
                        continue;
                    }
                }
                vnode_put(vp);
            }
        }
        IOSleep(1000);
    }
    return nullptr;
}

static int gpuebSendGfIpi(uintptr_t mboxDataVa, uintptr_t mboxCtlVa,
                          const GfIpiMsg &tx, GfIpiMsg *rx_out) {
    const uint32_t chBit = (1u << MBOX_CH_GPUFREQ);
    if (rd32(mboxCtlVa, MBOX_CTL_RX_STS) & chBit) {
        wr32(mboxCtlVa, MBOX_CTL_IRQ_CLR, chBit);
    }
    uint32_t words[8];
    memcpy(words, &tx, sizeof(words));
    for (uint32_t i = 0; i < 8u; ++i) {
        wr32(mboxDataVa, MBOX_GF_TX_OFF + i * 4u, words[i]);
    }
    wr32(mboxCtlVa, MBOX_CTL_IRQ_SET, chBit);

    uint32_t rxSts = 0;
    for (int ms = 0; ms < 500; ++ms) {
        rxSts = rd32(mboxCtlVa, MBOX_CTL_RX_STS);
        if (rxSts & chBit) break;
        IOSleep(1);
    }
    if ((rxSts & chBit) == 0u) return -110;

    uint32_t rwords[8];
    for (uint32_t i = 0; i < 8u; ++i) {
        rwords[i] = rd32(mboxDataVa, MBOX_GF_RX_OFF + i * 4u);
    }
    wr32(mboxCtlVa, MBOX_CTL_IRQ_CLR, chBit);
    if (rx_out) memcpy(rx_out, rwords, sizeof(rwords));
    return 0;
}

/* Keep CSF arena alive in kernel memory once MCU is booted! */
static IOBufferMemoryDescriptor *gCsfArenaDesc = nullptr;

static void bootMaliCsfMcu(uintptr_t maliVa) {
    size_t fwLen = 0;
    uint8_t *fw = loadMaliFwFromVfs(&fwLen);
    if (!fw) {
        IOLog("Tb375fcMali[stage3]: mali_csffw.bin not found in /private/var/ngpu/\n");
        return;
    }
    uint32_t magic = 0, hdrSize = 0;
    memcpy(&magic, fw, 4);
    memcpy(&hdrSize, fw + 16, 4);
    IOLog("Tb375fcMali[stage3]: loaded mali_csffw.bin (%zu B) magic=0x%08x hdr_size=0x%x\n",
          fwLen, magic, hdrSize);
    if (magic != 0xC3F13A6Eu || hdrSize > fwLen) {
        IOFreeData(fw, fwLen);
        return;
    }

    /* 1. Soft-reset GPU & power on L2 cache (matches panthor_gpu_soft_reset + panthor_gpu_l2_power_on) */
    wr32(maliVa, GPU_INT_CLEAR, 0xFFFFFFFFu);
    wr32(maliVa, GPU_CMD, GPU_CMD_SOFT_RESET);
    int rstOk = 0;
    for (int i = 0; i < 200; ++i) {
        if (rd32(maliVa, GPU_INT_RAWSTAT) & GPU_IRQ_RESET_DONE) { rstOk = 1; break; }
        IOSleep(1);
    }
    wr32(maliVa, GPU_INT_CLEAR, 0xFFFFFFFFu);
    wr32(maliVa, GPU_COHERENCY_PROT, 31u); /* GPU_COHERENCY_NONE */
    wr32(maliVa, L2_PWRON_LO, 1u);
    int l2Ok = 0;
    for (int i = 0; i < 200; ++i) {
        if (rd32(maliVa, L2_READY_LO) & 1u) { l2Ok = 1; break; }
        IOSleep(1);
    }
    IOLog("Tb375fcMali[stage3]: soft_reset=%d l2_ready=%d (0x%x) pwrtrans=0x%x status=0x%x\n",
          rstOk, l2Ok, rd32(maliVa, L2_READY_LO), rd32(maliVa, L2_PWRTRANS_LO), rd32(maliVa, GPU_STATUS));
    if (!rstOk || !l2Ok) {
        IOFreeData(fw, fwLen);
        return;
    }

    /* 2. Allocate 256 x 4 KiB pages (1 MiB) for AS0 4-level page tables (L0, L1, L2, 16x L3) + FW sections */
    constexpr uint32_t kTotalPages = 256u;
    gCsfArenaDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIOMemoryKernelUserShared | kIODirectionInOut,
        (size_t)kTotalPages * 4096ULL, 0x000000FFFFFFF000ULL);
    if (!gCsfArenaDesc || gCsfArenaDesc->prepare() != kIOReturnSuccess) {
        IOLog("Tb375fcMali[stage3]: failed to allocate 1 MiB CSF arena\n");
        IOFreeData(fw, fwLen);
        return;
    }
    uint8_t *arenaVa = static_cast<uint8_t *>(gCsfArenaDesc->getBytesNoCopy());
    memset(arenaVa, 0, (size_t)kTotalPages * 4096ULL);

    auto pageVa = [&](uint32_t idx) -> uint8_t * { return arenaVa + (size_t)idx * 4096ULL; };
    auto pagePa = [&](uint32_t idx) -> uint64_t {
        IOByteCount segLen = 0;
        return (uint64_t)gCsfArenaDesc->getPhysicalSegment((IOByteCount)idx * 4096ULL, &segLen);
    };

    /* Page 0 = L0, Page 1 = L1, Page 2 = L2, Pages 3..35 = L3 tables for L2[0..32] (0x00000000..0x04200000) */
    uint64_t *l0 = reinterpret_cast<uint64_t *>(pageVa(0));
    uint64_t *l1 = reinterpret_cast<uint64_t *>(pageVa(1));
    uint64_t *l2 = reinterpret_cast<uint64_t *>(pageVa(2));
    l0[0] = (pagePa(1) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    l1[0] = (pagePa(2) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    for (uint32_t l2i = 0; l2i <= 32u; ++l2i) {
        l2[l2i] = (pagePa(3u + l2i) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    }
    uint32_t nextDataPage = 36u;
    uint8_t *sharedIfaceCpuVa = nullptr;

    /* 3. Parse mali_csffw.bin sections and populate L3 PTEs (ARM_64_LPAE_S1 4K pages) */
    uint32_t off = 20u;
    uint32_t secCount = 0;
    while (off + 4u <= hdrSize) {
        uint32_t ehdr = 0;
        memcpy(&ehdr, fw + off, 4);
        uint32_t etype = ehdr & 0xFFu;
        uint32_t esize = (ehdr >> 8) & 0xFFFFFFu;
        if (esize < 4u || off + esize > hdrSize) break;

        if (etype == 0u && esize >= 24u) {
            uint32_t flags = 0, vaStart = 0, vaEnd = 0, dStart = 0, dEnd = 0;
            memcpy(&flags,   fw + off + 4u,  4);
            memcpy(&vaStart, fw + off + 8u,  4);
            memcpy(&vaEnd,   fw + off + 12u, 4);
            memcpy(&dStart,  fw + off + 16u, 4);
            memcpy(&dEnd,    fw + off + 20u, 4);

            /* Skip protected-mode section (BIT(5) = 0x20 = CSF_FW_BINARY_IFACE_ENTRY_PROT) */
            if ((flags & 0x20u) == 0u && vaEnd > vaStart && vaEnd <= 0x04200000u && dEnd <= fwLen) {
                uint32_t numPages = (vaEnd - vaStart) / 4096u;
                uint32_t dataLen  = (dEnd > dStart) ? (dEnd - dStart) : 0u;
                for (uint32_t p = 0; p < numPages; ++p) {
                    if (nextDataPage >= kTotalPages) break;
                    uint32_t pgIdx = nextDataPage++;
                    uint8_t *dst   = pageVa(pgIdx);
                    uint64_t pa    = pagePa(pgIdx);
                    uint32_t pgOff = p * 4096u;
                    if (pgOff < dataLen) {
                        uint32_t copyLen = dataLen - pgOff;
                        if (copyLen > 4096u) copyLen = 4096u;
                        memcpy(dst, fw + dStart + pgOff, copyLen);
                    }
                    uint32_t va = vaStart + pgOff;
                    if (va == 0x04000000u) {
                        sharedIfaceCpuVa = dst;
                    }
                    /* Map into L3 table */
                    uint32_t l2Idx = (va >> 21) & 0x1FFu;
                    uint32_t l3Idx = (va >> 12) & 0x1FFu;
                    uint64_t *l3   = reinterpret_cast<uint64_t *>(pageVa(3u + l2Idx));

                    /* ARM_64_LPAE_S1 4K leaf PTE:
                     * bits[1:0]=0b11 (PAGE), bit[6]=AP_UNPRIV, bit[10]=AF.
                     * Use AttrIndx=0 (NC, outer-shareable) for SHARED (0x04000000) or non-CACHE sections,
                     * and AttrIndx=1 (WB) for normal cached firmware sections. */
                    bool isSharedOrNc = ((flags & 0x10u) != 0u) || ((flags & 0x08u) == 0u);
                    uint64_t pte = (pa & 0x0000FFFFFFFFF000ULL) | 0x3ULL | (1ULL << 10) | (1ULL << 6);
                    if ((flags & 0x02u) == 0u) pte |= (1ULL << 7);  /* Read-only */
                    if ((flags & 0x04u) == 0u) pte |= (1ULL << 54); /* XN */
                    if (isSharedOrNc) {
                        pte |= (0ULL << 2) | (2ULL << 8); /* AttrIndx=0 (NC), Outer Shareable */
                    } else {
                        pte |= (1ULL << 2);               /* AttrIndx=1 (WB), Midgard Inner (SH=0) */
                    }
                    l3[l3Idx] = pte;
                }
                secCount++;
            }
        }
        off += esize;
    }
    IOFreeData(fw, fwLen);

    /* Flush all 256 pages (page tables + FW code/data) from CPU cache to DRAM */
    cacheFlushRange(arenaVa, (size_t)kTotalPages * 4096ULL);
    IOLog("Tb375fcMali[stage3]: mapped %u FW sections (%u pages, l0_pa=0x%llx)\n",
          secCount, nextDataPage, (unsigned long long)pagePa(0));

    /* 4. Configure and enable AS0 (MMU_AS(0) = 0x2400, va_bits = 48 -> INA_BITS(55-48=7)) */
    wr32(maliVa, MMU_INT_CLEAR, 0xFFFFFFFFu);
    wr32(maliVa, MMU_INT_MASK, 0xFFFFFFFFu);

    /* Attr0 = 0x4C (INNER_OUTER_NC | EXPL(0,0)), Attr1 = 0x8F (INNER_OUTER_WB | EXPL(1,1)) */
    uint64_t memattr  = 0x4C4C4C4C4C4C8F4CULL;
    /* ADRMODE_AARCH64_4K (6) | INA_BITS(55 - 48 = 7) (7 << 6) | PTW_MEMATTR_NC (1 << 24) */
    uint64_t transcfg = 6ULL | (7ULL << 6) | (1ULL << 24);

    wr64(maliVa, AS0_TRANSTAB_LO, pagePa(0) & 0x0000FFFFFFFFF000ULL);
    wr64(maliVa, AS0_MEMATTR_LO, memattr);
    wr64(maliVa, AS0_TRANSCFG_LO, transcfg);
    wr32(maliVa, AS0_COMMAND, AS_COMMAND_UPDATE);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(maliVa, AS0_STATUS) & 1u) == 0u) break;
        IODelay(10);
    }
    IOLog("Tb375fcMali[stage3]: AS0 enabled status=0x%x fault=0x%x addr=0x%08x_%08x\n",
          rd32(maliVa, AS0_STATUS), rd32(maliVa, AS0_FAULTSTATUS),
          rd32(maliVa, AS0_FAULTADDR_HI), rd32(maliVa, AS0_FAULTADDR_LO));

    /* 5. Boot the Arm Mali-G615 CSF Microcontroller (MCU_CONTROL = MCU_CONTROL_AUTO = 2) */
    wr32(maliVa, JOB_INT_CLEAR, 0xFFFFFFFFu);
    wr32(maliVa, JOB_INT_MASK, 0xFFFFFFFFu);
    wr32(maliVa, MCU_CONTROL, MCU_CONTROL_AUTO);

    uint32_t jobIrq = 0, mcuSt = 0;
    for (int ms = 0; ms < 500; ++ms) {
        jobIrq = rd32(maliVa, JOB_INT_RAWSTAT);
        mcuSt  = rd32(maliVa, MCU_STATUS);
        if ((jobIrq & JOB_INT_GLOBAL_IF) != 0u) break;
        IOSleep(1);
    }
    if (sharedIfaceCpuVa) {
        cacheFlushRange(sharedIfaceCpuVa, 4096u);
    }
    uint32_t glbVer = 0, glbFeat = 0, glbInVa = 0, glbOutVa = 0, glbGrpNum = 0, glbInstr = 0;
    uint32_t glbAck = 0;
    if (sharedIfaceCpuVa) {
        memcpy(&glbVer,    sharedIfaceCpuVa + 0x00, 4);
        memcpy(&glbFeat,   sharedIfaceCpuVa + 0x04, 4);
        memcpy(&glbInVa,   sharedIfaceCpuVa + 0x08, 4);
        memcpy(&glbOutVa,  sharedIfaceCpuVa + 0x0C, 4);
        memcpy(&glbGrpNum, sharedIfaceCpuVa + 0x10, 4);
        memcpy(&glbInstr,  sharedIfaceCpuVa + 0x1C, 4);
        /* output_va is typically 0x04002000 (+0x2000 into sharedIface) */
        uint32_t outOff = (glbOutVa >= 0x04000000u && glbOutVa < 0x0400C000u) ? (glbOutVa - 0x04000000u) : 0x2000u;
        cacheFlushRange(sharedIfaceCpuVa + outOff, 256u);
        memcpy(&glbAck, sharedIfaceCpuVa + outOff, 4);
    }
    IOLog("Tb375fcMali[stage3-MCU]: mcu_status=%u job_irq=0x%08x mmu_irq=0x%08x as0_fault=0x%x addr=0x%08x_%08x\n",
          mcuSt, jobIrq, rd32(maliVa, MMU_INT_RAWSTAT),
          rd32(maliVa, AS0_FAULTSTATUS), rd32(maliVa, AS0_FAULTADDR_HI), rd32(maliVa, AS0_FAULTADDR_LO));
    IOLog("Tb375fcMali[stage3-GLB]: ver=0x%08x feat=0x%x in_va=0x%x out_va=0x%x csg_num=%u instr=0x%x ack=0x%x\n",
          glbVer, glbFeat, glbInVa, glbOutVa, glbGrpNum, glbInstr, glbAck);

    /* 6. If CSF MCU booted (ver != 0), ring CSF_GLB_DOORBELL with GLB_PING (BIT(8)) to verify live command execution! */
    if (sharedIfaceCpuVa && glbVer != 0u && glbInVa >= 0x04000000u && glbOutVa >= 0x04000000u) {
        uint32_t inOff  = glbInVa  - 0x04000000u;
        uint32_t outOff = glbOutVa - 0x04000000u;
        uint32_t req = 0, ack = 0;
        memcpy(&ack, sharedIfaceCpuVa + outOff + 0x00, 4);
        req = ack ^ (1u << 8); /* Toggle GLB_PING (BIT(8)) */
        uint32_t ackMask = (1u << 8);
        uint32_t dbReq   = 1u;
        memcpy(sharedIfaceCpuVa + inOff + 0x00, &req, 4);
        memcpy(sharedIfaceCpuVa + inOff + 0x04, &ackMask, 4);
        memcpy(sharedIfaceCpuVa + inOff + 0x08, &dbReq, 4);
        cacheFlushRange(sharedIfaceCpuVa + inOff, 64u);

        wr32(maliVa, JOB_INT_CLEAR, JOB_INT_GLOBAL_IF);
        wr32(maliVa, CSF_GLB_DOORBELL, 1u);

        int pingOk = 0;
        uint32_t newAck = ack;
        for (int ms = 0; ms < 200; ++ms) {
            cacheFlushRange(sharedIfaceCpuVa + outOff, 64u);
            memcpy(&newAck, sharedIfaceCpuVa + outOff + 0x00, 4);
            if (((newAck ^ req) & (1u << 8)) == 0u) { pingOk = 1; break; }
            IOSleep(1);
        }
        IOLog("Tb375fcMali[stage3-PING]: GLB_PING doorbell ok=%d req=0x%x ack=0x%x->0x%x job_irq=0x%08x\n",
              pingOk, req, ack, newAck, rd32(maliVa, JOB_INT_RAWSTAT));
    }
}

static uint32_t gMaliStage = 0;

static void maliWorkerBody(void) {
    IOLog("Tb375fcMali: worker started stage=%u (MT6897 Mali-G615 bring-up)\n", gMaliStage);

    MappedPage gpuebGpr = {}, gpuebMbox = {}, mfgRpc = {}, gpuebDram = {}, maliReg = {};
    if (!mapMmio(MT6897_GPUEB_GPR_PHYS, 0x1000, &gpuebGpr) ||
        !mapMmio(MT6897_GPUEB_MBOX_PHYS, 0x1000, &gpuebMbox) ||
        !mapMmio(MT6897_MFG_RPC_PHYS, 0x1000, &mfgRpc) ||
        !mapMmio(MT6897_GPUEB_DRAM_PHYS, MT6897_GPUEB_DRAM_SIZE, &gpuebDram)) {
        IOLog("Tb375fcMali: FATAL MMIO map failed\n");
        goto cleanup;
    }

    {
        uintptr_t gprVa  = gpuebGpr.va + GPR_BASE_OFF;
        uintptr_t mboxVa = gpuebGpr.va + MBOX0_BASE_OFF;

        uint32_t pwrCon   = rd32(mfgRpc.va, RPC_PWR_CON);
        uint32_t ghpmRo0  = rd32(mfgRpc.va, RPC_GHPM_RO0_CON);
        uint32_t lpState  = rd32(gprVa, GPR_LP_STATE);
        uint32_t ipiMagic = rd32(gprVa, GPR_IPI_MAGIC);
        IOLog("Tb375fcMali[stage1]: rpc_pwr=0x%08x ghpm_ro0=0x%08x lp_state=0x%08x ipi_magic=0x%08x\n",
              pwrCon, ghpmRo0, lpState, ipiMagic);

        if (gMaliStage < 2u) goto cleanup;

        if (lpState != 1u || ipiMagic == 0u) {
            upd32(mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ENABLE_M, RPC_GHPM_ENABLE_M);
            upd32(mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ON_SEQ_M, 0u);
            upd32(mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ON_SEQ_M, RPC_GHPM_ON_SEQ_M);
            upd32(mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ENABLE_M, 0u);
            for (int i = 0; i < 200; ++i) {
                if (rd32(gprVa, GPR_LP_STATE) == 1u && rd32(gprVa, GPR_IPI_MAGIC) != 0u) break;
                IODelay(50);
            }
            lpState  = rd32(gprVa, GPR_LP_STATE);
            ipiMagic = rd32(gprVa, GPR_IPI_MAGIC);
            if (lpState != 1u || ipiMagic == 0u) goto cleanup;
        }

        for (uint32_t off = 0; off < MT6897_GPUEB_DRAM_SIZE; off += 4u) {
            wr32(gpuebDram.va, off, 0u);
        }
        GfIpiMsg tx = {}, rx = {};
        tx.magic = ipiMagic;
        tx.cmd   = CMD_INIT_SHARED_MEM;
        tx.u.shared_mem.base = MT6897_GPUEB_DRAM_PHYS;
        tx.u.shared_mem.size = MT6897_GPUEB_DRAM_SIZE;

        int rc = gpuebSendGfIpi(mboxVa, gpuebMbox.va, tx, &rx);
        uint32_t shMagic = rd32(gpuebDram.va, GF_REG_MAGIC);
        IOLog("Tb375fcMali[stage2]: INIT_SHARED_MEM rc=%d rv=%d sh_magic=0x%08x opp_num=%u\n",
              rc, rx.u.return_value, shMagic, rd32(gpuebDram.va, GF_REG_GPU_OPP_NUM));
        if (rc != 0 || shMagic != GPUEB_MEM_MAGIC) goto cleanup;

        memset(&tx, 0, sizeof(tx));
        memset(&rx, 0, sizeof(rx));
        tx.magic  = ipiMagic;
        tx.cmd    = CMD_POWER_CONTROL;
        tx.target = 0u;
        tx.u.power_state = 1u;

        rc = gpuebSendGfIpi(mboxVa, gpuebMbox.va, tx, &rx);
        uint32_t pwrCnt  = rd32(gpuebDram.va, GF_REG_POWER_COUNT);
        uint32_t freqKhz = rd32(gpuebDram.va, GF_REG_FREQ_CUR_GPU);
        uint32_t volt10u = rd32(gpuebDram.va, GF_REG_VOLT_CUR_GPU);
        IOLog("Tb375fcMali[stage2]: POWER_CONTROL(ON) rc=%d rv=%d pwr_cnt=%u freq=%u kHz volt=%u0 uV\n",
              rc, rx.u.return_value, pwrCnt, freqKhz, volt10u);
        if (rc != 0 || pwrCnt == 0u) goto cleanup;

        if (!mapMmio(MT6897_MALI_PHYS, MT6897_MALI_MAP_SIZE, &maliReg)) {
            IOLog("Tb375fcMali[stage2]: failed to map mali@13000000\n");
            goto cleanup;
        }
        uint32_t gpuId    = rd32(maliReg.va, 0x0000);
        uint32_t l2Feat   = rd32(maliReg.va, 0x0004);
        uint32_t mmuFeat  = rd32(maliReg.va, 0x0014);
        uint32_t asPres   = rd32(maliReg.va, 0x0018);
        uint32_t csfId    = rd32(maliReg.va, 0x001C);
        uint32_t shPresLo = rd32(maliReg.va, 0x0100);
        IOLog("Tb375fcMali[stage2-HW]: GPU_ID=0x%08x CSF_ID=0x%08x L2=0x%08x MMU=0x%08x AS=0x%x SHADER_PRESENT=0x%x\n",
              gpuId, csfId, l2Feat, mmuFeat, asPres, shPresLo);

        if (gMaliStage >= 3u) {
            bootMaliCsfMcu(maliReg.va);
        }
    }

cleanup:
    unmapMmio(&maliReg);
    unmapMmio(&gpuebDram);
    unmapMmio(&mfgRpc);
    unmapMmio(&gpuebMbox);
    unmapMmio(&gpuebGpr);
    IOLog("Tb375fcMali: worker exiting cleanly (stage=%u)\n", gMaliStage);
}

static void maliWorkerThread(void *, wait_result_t) {
    maliWorkerBody();
    (void)thread_terminate(current_thread());
}

}  // namespace

extern "C" void tb375fc_mali_probe(void);
extern "C" void tb375fc_mali_probe(void) {
    static bool started = false;
    if (started) return;
    started = true;

    uint32_t stage = 0;
    if (!PE_parse_boot_argn("mali", &stage, sizeof(stage)) || stage == 0u) {
        return;
    }
    gMaliStage = stage;

    thread_t th = THREAD_NULL;
    if (kernel_thread_start(maliWorkerThread, nullptr, &th) == KERN_SUCCESS && th != THREAD_NULL) {
        thread_deallocate(th);
    }
}
