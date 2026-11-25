/* SPDX-License-Identifier: MIT
 * TB375FC / MT6897 Mali-G615 (Arch 11.8 Valhall CSF v10) Kernel Driver + IOUserClient.
 *
 * Boot-arg gated:
 *   mali=4 : Bring up GPUEB + Mali-G615 + L2 + AS0 + mali_csffw.bin v2.5 MCU + GLB_PING,
 *            and publish IOService "Tb375fcMali" with panthor-compatible IOUserClient
 *            for userland Mesa panvk (libvulkan_panfrost.dylib) + m2v v2!
 */
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOUserClient.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

#include "../../mali-panvk/Tb375fcMaliABI.h"

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
#define UIO_READ 0
#endif
#ifndef UIO_SYSSPACE
#define UIO_SYSSPACE 2
#endif

namespace {

constexpr uint64_t MT6897_MALI_PHYS        = 0x13000000ULL;
constexpr uint64_t MT6897_MALI_MAP_SIZE    = 0x00200000ULL; /* 2 MiB: Control + MMU + MCU + Doorbells 0..23 */
constexpr uint64_t MT6897_GPUEB_GPR_PHYS   = 0x13C2F000ULL;
constexpr uint64_t MT6897_GPUEB_MBOX_PHYS  = 0x13C62000ULL;
constexpr uint64_t MT6897_MFG_RPC_PHYS     = 0x13F90000ULL;
constexpr uint64_t MT6897_GPUEB_DRAM_PHYS  = 0x7FE70000ULL;
constexpr uint32_t MT6897_GPUEB_DRAM_SIZE  = 0x4000u;

/* GPUEB GPR & MBOX0 */
constexpr uint32_t GPR_BASE_OFF   = 0x0D1Cu;
constexpr uint32_t GPR_LP_STATE   = 0x0028u;
constexpr uint32_t GPR_IPI_MAGIC  = 0x0034u;
constexpr uint32_t MBOX0_BASE_OFF = 0x0D80u;
constexpr uint32_t MBOX_CH_GPUFREQ = 1u;
constexpr uint32_t MBOX_GF_TX_OFF = 0x0010u;
constexpr uint32_t MBOX_GF_RX_OFF = 0x00E8u;
constexpr uint32_t MBOX_CTL_TX_STS = 0x00u;
constexpr uint32_t MBOX_CTL_IRQ_SET = 0x04u;
constexpr uint32_t MBOX_CTL_IRQ_CLR = 0x74u;
constexpr uint32_t MBOX_CTL_RX_STS  = 0x78u;

/* MFG_RPC */
constexpr uint32_t RPC_GHPM_CFG0_CON = 0x0800u;
constexpr uint32_t RPC_GHPM_ENABLE_M = (1u << 0);
constexpr uint32_t RPC_GHPM_ON_SEQ_M = (1u << 2);

/* GPUEB shared DRAM */
constexpr uint32_t GPUEB_MEM_MAGIC     = 0xBABADADAu;
constexpr uint32_t GF_REG_MAGIC        = 0x0000u;
constexpr uint32_t GF_REG_GPU_OPP_NUM  = 0x000Cu;
constexpr uint32_t GF_REG_POWER_COUNT  = 0x001Cu;
constexpr uint32_t GF_REG_FREQ_CUR_GPU = 0x0044u;
constexpr uint32_t CMD_INIT_SHARED_MEM = 0u;
constexpr uint32_t CMD_POWER_CONTROL   = 6u;

/* Mali-G615 GPU control registers */
constexpr uint32_t GPU_ID_REG             = 0x0000u;
constexpr uint32_t GPU_L2_FEATURES        = 0x0004u;
constexpr uint32_t GPU_TILER_FEATURES     = 0x000Cu;
constexpr uint32_t GPU_MEM_FEATURES       = 0x0010u;
constexpr uint32_t GPU_MMU_FEATURES       = 0x0014u;
constexpr uint32_t GPU_AS_PRESENT         = 0x0018u;
constexpr uint32_t GPU_CSF_ID             = 0x001Cu;
constexpr uint32_t GPU_INT_RAWSTAT        = 0x0020u;
constexpr uint32_t GPU_INT_CLEAR          = 0x0024u;
constexpr uint32_t GPU_IRQ_RESET_DONE     = (1u << 8);
constexpr uint32_t GPU_CMD                = 0x0030u;
constexpr uint32_t GPU_CMD_SOFT_RESET     = 0x0101u;
constexpr uint32_t GPU_STATUS             = 0x0034u;
constexpr uint32_t GPU_TIMESTAMP_LO       = 0x0090u;
constexpr uint32_t GPU_TIMESTAMP_HI       = 0x0094u;
constexpr uint32_t GPU_THREAD_MAX_THREADS = 0x00A0u;
constexpr uint32_t GPU_THREAD_MAX_WG_SIZE = 0x00A4u;
constexpr uint32_t GPU_THREAD_MAX_BARRIER = 0x00A8u;
constexpr uint32_t GPU_THREAD_FEATURES    = 0x00ACu;
constexpr uint32_t GPU_TEX_FEATURES_0     = 0x00B0u;
constexpr uint32_t GPU_SHADER_PRESENT_LO  = 0x0100u;
constexpr uint32_t GPU_SHADER_PRESENT_HI  = 0x0104u;
constexpr uint32_t GPU_TILER_PRESENT_LO   = 0x0110u;
constexpr uint32_t GPU_TILER_PRESENT_HI   = 0x0114u;
constexpr uint32_t GPU_L2_PRESENT_LO      = 0x0120u;
constexpr uint32_t GPU_L2_PRESENT_HI      = 0x0124u;
constexpr uint32_t L2_READY_LO            = 0x0160u;
constexpr uint32_t L2_PWRON_LO            = 0x01A0u;
constexpr uint32_t GPU_COHERENCY_FEATURES = 0x0300u;
constexpr uint32_t GPU_COHERENCY_PROT     = 0x0304u;

constexpr uint32_t MCU_CONTROL         = 0x0700u;
constexpr uint32_t MCU_CONTROL_AUTO    = 2u;
constexpr uint32_t MCU_STATUS          = 0x0704u;

constexpr uint32_t JOB_INT_RAWSTAT     = 0x1000u;
constexpr uint32_t JOB_INT_CLEAR       = 0x1004u;
constexpr uint32_t JOB_INT_MASK        = 0x1008u;
constexpr uint32_t JOB_INT_GLOBAL_IF   = (1u << 31);

constexpr uint32_t MMU_INT_RAWSTAT     = 0x2000u;
constexpr uint32_t MMU_INT_CLEAR       = 0x2004u;
constexpr uint32_t MMU_INT_MASK        = 0x2008u;

/* MMU Address Space registers: AS(n) = 0x2400 + n * 0x40 */
static inline uint32_t AS_REG(uint32_t as, uint32_t off) { return 0x2400u + as * 0x40u + off; }
constexpr uint32_t AS_TRANSTAB_LO   = 0x00u;
constexpr uint32_t AS_MEMATTR_LO    = 0x08u;
constexpr uint32_t AS_LOCKADDR_LO   = 0x10u;
constexpr uint32_t AS_COMMAND       = 0x18u;
constexpr uint32_t AS_FAULTSTATUS   = 0x1Cu;
constexpr uint32_t AS_FAULTADDR_LO  = 0x20u;
constexpr uint32_t AS_FAULTADDR_HI  = 0x24u;
constexpr uint32_t AS_STATUS        = 0x28u;
constexpr uint32_t AS_TRANSCFG_LO   = 0x30u;

constexpr uint32_t AS_CMD_UPDATE    = 1u;
constexpr uint32_t AS_CMD_LOCK      = 2u;
constexpr uint32_t AS_CMD_FLUSH_PT  = 4u;

/* CSF Doorbells: DB 0 = Global (0x80000), DB n = 0x80000 + n * 0x10000 */
static inline uint32_t CSF_DOORBELL(uint32_t db) { return 0x80000u + db * 0x10000u; }

struct __attribute__((packed)) GfIpiMsg {
    uint32_t magic, cmd, target, reserved;
    union {
        int32_t  return_value;
        uint32_t power_state;
        struct __attribute__((packed)) { uint64_t base; uint32_t size; uint32_t pad; } shared_mem;
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
    out->map = out->desc->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapInhibitCache);
    if (!out->map) { out->desc->release(); out->desc = nullptr; return false; }
    out->va = static_cast<uintptr_t>(out->map->getVirtualAddress());
    return out->va != 0;
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
    wr32(base, off, static_cast<uint32_t>(val & 0xFFFFFFFFULL));
    wr32(base, off + 4u, static_cast<uint32_t>(val >> 32));
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
    __asm__ volatile("dsb sy" ::: "memory");
}

static uint8_t *loadMaliFwFromVfs(size_t *len_out) {
    vfs_context_t ctx = vfs_context_kernel();
    if (!ctx) return nullptr;
    vnode_t vp = nullptr;
    if (vnode_lookup("/private/var/ngpu/mali_csffw.bin", 0, &vp, ctx) != 0 || !vp) return nullptr;
    off_t sz = 0;
    if (vnode_size(vp, &sz, ctx) != 0 || sz <= 20 || sz > 2 * 1024 * 1024) {
        vnode_put(vp);
        return nullptr;
    }
    uint8_t *buf = static_cast<uint8_t *>(IOMallocData(static_cast<size_t>(sz)));
    if (!buf) { vnode_put(vp); return nullptr; }
    int resid = 0;
    int rc = vn_rdwr(UIO_READ, vp, reinterpret_cast<char *>(buf), static_cast<int>(sz), 0,
                     UIO_SYSSPACE, 0, vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
    vnode_put(vp);
    if (rc != 0 || resid != 0) { IOFreeData(buf, static_cast<size_t>(sz)); return nullptr; }
    *len_out = static_cast<size_t>(sz);
    return buf;
}

static int gpuebSendGfIpi(uintptr_t mboxDataVa, uintptr_t mboxCtlVa, const GfIpiMsg &tx, GfIpiMsg *rx) {
    const uint32_t chBit = (1u << MBOX_CH_GPUFREQ);
    if (rd32(mboxCtlVa, MBOX_CTL_RX_STS) & chBit) wr32(mboxCtlVa, MBOX_CTL_IRQ_CLR, chBit);
    const uint32_t *words = reinterpret_cast<const uint32_t *>(&tx);
    for (uint32_t i = 0; i < 8u; ++i) wr32(mboxDataVa, MBOX_GF_TX_OFF + i * 4u, words[i]);
    wr32(mboxCtlVa, MBOX_CTL_IRQ_SET, chBit);
    for (int i = 0; i < 2000; ++i) {
        if ((rd32(mboxCtlVa, MBOX_CTL_TX_STS) & chBit) == 0u) break;
        IODelay(10);
    }
    for (int i = 0; i < 5000; ++i) {
        if ((rd32(mboxCtlVa, MBOX_CTL_RX_STS) & chBit) != 0u) {
            uint32_t *out = reinterpret_cast<uint32_t *>(rx);
            for (uint32_t j = 0; j < 8u; ++j) out[j] = rd32(mboxDataVa, MBOX_GF_RX_OFF + j * 4u);
            wr32(mboxCtlVa, MBOX_CTL_IRQ_CLR, chBit);
            return 0;
        }
        IODelay(10);
    }
    return -1;
}

/* ---- Mali CSF v2.5 Firmware Shared-Memory Interfaces (aligned with panthor_fw.h) ---- */
constexpr uint32_t CSF_SHARED_BASE_VA         = 0x04000000u;
constexpr uint32_t CSF_GROUP_CONTROL_OFFSET   = 0x1000u;
constexpr uint32_t CSF_STREAM_CONTROL_OFFSET  = 0x40u;

constexpr uint32_t GLB_CFG_PROGRESS_TIMER     = (1u << 1);
constexpr uint32_t GLB_CFG_ALLOC_EN           = (1u << 2);
constexpr uint32_t GLB_CFG_POWEROFF_TIMER     = (1u << 3);
constexpr uint32_t GLB_COUNTER_EN             = (1u << 7);
constexpr uint32_t GLB_IDLE_EN                = (1u << 10);

constexpr uint32_t CSG_STATE_MASK             = 0x7u;
constexpr uint32_t CSG_STATE_TERMINATE        = 0u;
constexpr uint32_t CSG_STATE_START            = 1u;
constexpr uint32_t CSG_ENDPOINT_CONFIG        = (1u << 4);
constexpr uint32_t CSG_REQ_MASK               = CSG_STATE_MASK | CSG_ENDPOINT_CONFIG | (1u << 5);

constexpr uint32_t CS_STATE_MASK              = 0x7u;
constexpr uint32_t CS_STATE_STOP              = 0u;
constexpr uint32_t CS_STATE_START             = 1u;
constexpr uint32_t CS_IDLE_SYNC_WAIT          = (1u << 8);
constexpr uint32_t CS_IDLE_EMPTY              = (1u << 10);
constexpr uint32_t CS_REQ_MASK                = CS_STATE_MASK | (1u << 4) | CS_IDLE_SYNC_WAIT | (1u << 9) | CS_IDLE_EMPTY | (1u << 11);

struct FwRingIoInput {
    uint64_t insert;
    uint64_t extract;
};

struct FwRingIoOutput {
    uint64_t extract;
    uint32_t active;
};

struct FwGlobalControl {
    uint32_t version;
    uint32_t features;
    uint32_t input_va;
    uint32_t output_va;
    uint32_t group_num;
    uint32_t group_stride;
    uint32_t perfcnt_size;
    uint32_t instr_features;
};

struct FwGlobalInput {
    uint32_t req;
    uint32_t ack_irq_mask;
    uint32_t doorbell_req;
    uint32_t reserved1;
    uint32_t progress_timer;
    uint32_t poweroff_timer;
    uint64_t core_en_mask;
    uint32_t reserved2;
    uint32_t perfcnt_as;
    uint64_t perfcnt_base;
    uint32_t perfcnt_extract;
    uint32_t reserved3[3];
    uint32_t perfcnt_config;
    uint32_t perfcnt_csg_select;
    uint32_t perfcnt_fw_enable;
    uint32_t perfcnt_csg_enable;
    uint32_t perfcnt_csf_enable;
    uint32_t perfcnt_shader_enable;
    uint32_t perfcnt_tiler_enable;
    uint32_t perfcnt_mmu_l2_enable;
    uint32_t reserved4[8];
    uint32_t idle_timer;
};

struct FwGlobalOutput {
    uint32_t ack;
    uint32_t reserved1;
    uint32_t doorbell_ack;
    uint32_t reserved2;
    uint32_t halt_status;
    uint32_t perfcnt_status;
    uint32_t perfcnt_insert;
};

struct FwCsgControl {
    uint32_t features;
    uint32_t input_va;
    uint32_t output_va;
    uint32_t suspend_size;
    uint32_t protm_suspend_size;
    uint32_t stream_num;
    uint32_t stream_stride;
};

struct FwCsgInput {
    uint32_t req;
    uint32_t ack_irq_mask;
    uint32_t doorbell_req;
    uint32_t cs_irq_ack;
    uint32_t reserved1[4];
    uint64_t allow_compute;
    uint64_t allow_fragment;
    uint32_t allow_other;
    uint32_t endpoint_req;
    uint64_t endpoint_req2;
    uint64_t suspend_buf;
    uint64_t protm_suspend_buf;
    uint32_t config;
    uint32_t iter_trace_config;
};

struct FwCsgOutput {
    uint32_t ack;
    uint32_t reserved1;
    uint32_t doorbell_ack;
    uint32_t cs_irq_req;
    uint32_t status_endpoint_current;
    uint32_t status_endpoint_req;
    uint32_t status_state;
    uint32_t resource_dep;
};

struct FwCsControl {
    uint32_t features;
    uint32_t input_va;
    uint32_t output_va;
};

struct FwCsInput {
    uint32_t req;
    uint32_t config;
    uint32_t reserved1;
    uint32_t ack_irq_mask;
    uint64_t ringbuf_base;
    uint32_t ringbuf_size;
    uint32_t reserved2;
    uint64_t heap_start;
    uint64_t heap_end;
    uint64_t ringbuf_input;
    uint64_t ringbuf_output;
    uint32_t instr_config;
    uint32_t instrbuf_size;
    uint64_t instrbuf_base;
    uint64_t instrbuf_offset_ptr;
};

struct FwCsOutput {
    uint32_t ack;
    uint32_t reserved1[15];
    uint64_t status_cmd_ptr;
    uint32_t status_wait;
    uint32_t status_req_resource;
    uint64_t status_wait_sync_ptr;
    uint32_t status_wait_sync_value;
    uint32_t status_scoreboards;
    uint32_t status_blocked_reason;
    uint32_t status_wait_sync_value_hi;
    uint32_t reserved2[6];
    uint32_t fault;
    uint32_t fatal;
    uint64_t fault_info;
    uint64_t fatal_info;
};

/* ---- Global Mali-G615 hardware state (owned by Tb375fcMali service) ---- */
struct MaliHwState {
    IOLock *lock;
    bool    ready;
    MappedPage gpuebGpr, gpuebMbox, mfgRpc, gpuebDram, maliReg;
    IOBufferMemoryDescriptor *csfArenaDesc;
    uint8_t *csfArenaVa;
    uint8_t *sharedIfaceVa;
    uint32_t sharedIfaceSize;
    uint32_t glbInOff;
    uint32_t glbOutOff;
    uint32_t csgCount;
    uint32_t csgStride;
    uint32_t csCount;
    uint32_t csStride;
    uint32_t csgSuspendSize;
    uint32_t nextAs0FreePage;
    uint32_t nextAs0FreeVa;
    uint32_t asSlotMask;     /* bit0 reserved for AS0 (MCU) */
    uint32_t csgSlotMask;    /* bitmask of allocated CSG slots */
    Tb375fcMaliDevInfo devInfo;
};

static MaliHwState gMali = {};

static uint8_t *fwVaToCpu(uint32_t fwVa) {
    if (!gMali.sharedIfaceVa || fwVa < CSF_SHARED_BASE_VA) return nullptr;
    uint32_t off = fwVa - CSF_SHARED_BASE_VA;
    if (off >= gMali.sharedIfaceSize) return nullptr;
    return gMali.sharedIfaceVa + off;
}

static FwCsgControl *getCsgCtrl(uint32_t csg) {
    uint32_t off = CSF_GROUP_CONTROL_OFFSET + csg * gMali.csgStride;
    if (!gMali.sharedIfaceVa || off + sizeof(FwCsgControl) > gMali.sharedIfaceSize) return nullptr;
    return reinterpret_cast<FwCsgControl *>(gMali.sharedIfaceVa + off);
}

static FwCsControl *getCsCtrl(uint32_t csg, uint32_t cs) {
    FwCsgControl *c = getCsgCtrl(csg);
    if (!c) return nullptr;
    uint32_t off = CSF_GROUP_CONTROL_OFFSET + csg * gMali.csgStride +
                   CSF_STREAM_CONTROL_OFFSET + cs * c->stream_stride;
    if (off + sizeof(FwCsControl) > gMali.sharedIfaceSize) return nullptr;
    return reinterpret_cast<FwCsControl *>(gMali.sharedIfaceVa + off);
}

static uint64_t arenaPagePa(uint32_t idx) {
    IOByteCount segLen = 0;
    return static_cast<uint64_t>(gMali.csfArenaDesc->getPhysicalSegment(static_cast<IOByteCount>(idx) * 4096ULL, &segLen));
}
static uint8_t *arenaPageVa(uint32_t idx) {
    return gMali.csfArenaVa + static_cast<size_t>(idx) * 4096ULL;
}

/* Map a physical 4K page into AS0 at va (< 0x04200000) for MCU access */
static void mapAs0Page(uint32_t va, uint64_t pa, bool nc) {
    uint32_t l2Idx = (va >> 21) & 0x1FFu;
    uint32_t l3Idx = (va >> 12) & 0x1FFu;
    uint64_t *l3   = reinterpret_cast<uint64_t *>(arenaPageVa(3u + l2Idx));
    uint64_t pte   = (pa & 0x0000FFFFFFFFF000ULL) | 0x3ULL | (1ULL << 10) | (1ULL << 6) | (1ULL << 54);
    if (nc) pte |= (0ULL << 2) | (2ULL << 8);
    else    pte |= (1ULL << 2);
    l3[l3Idx] = pte;
    cacheFlushRange(&l3[l3Idx], 64);
}

static void flushAs0Mmu(void) {
    wr64(gMali.maliReg.va, AS_REG(0, AS_LOCKADDR_LO), 47ULL);
    wr32(gMali.maliReg.va, AS_REG(0, AS_COMMAND), AS_CMD_LOCK);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(0, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
    wr32(gMali.maliReg.va, AS_REG(0, AS_COMMAND), AS_CMD_FLUSH_PT);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(0, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
}

/* Allocate contiguous pages from csfArenaDesc and map them into AS0 for MCU access */
static bool allocAs0Pages(uint32_t numPages, bool nc, uint32_t *fwVaOut, uint8_t **cpuVaOut) {
    if (numPages == 0) return false;
    IOLockLock(gMali.lock);
    if (gMali.nextAs0FreePage + numPages > 512u ||
        gMali.nextAs0FreeVa + numPages * 4096u > 0x04200000u) {
        IOLockUnlock(gMali.lock);
        return false;
    }
    uint32_t startPg = gMali.nextAs0FreePage;
    uint32_t startVa = gMali.nextAs0FreeVa;
    gMali.nextAs0FreePage += numPages;
    gMali.nextAs0FreeVa   += numPages * 4096u;
    for (uint32_t i = 0; i < numPages; ++i) {
        uint8_t *dst = arenaPageVa(startPg + i);
        memset(dst, 0, 4096);
        cacheFlushRange(dst, 4096);
        mapAs0Page(startVa + i * 4096u, arenaPagePa(startPg + i), nc);
    }
    flushAs0Mmu();
    IOLockUnlock(gMali.lock);
    if (fwVaOut)  *fwVaOut  = startVa;
    if (cpuVaOut) *cpuVaOut = arenaPageVa(startPg);
    return true;
}

/* ---- 4-Level ARM_64_LPAE_S1 Page Table for User VMs (AS1..AS7) ---- */
constexpr uint32_t kMaxPtPagesPerVm = 4096u; /* Up to 16 MiB of page table pages (~8 GiB sparse mapped VA) */

struct MaliVm {
    bool     in_use;
    uint32_t id;
    uint32_t as_slot;
    IOBufferMemoryDescriptor *ptPoolDesc;
    uint8_t *ptPoolVa;
    uint64_t *ptPagePas;
    uint32_t ptPagesUsed;
    uint64_t l0Pa;
    uint64_t *l0Va;
    uint64_t nextInternalVa;
};

static uint32_t vmAllocPtPage(MaliVm *vm, uint64_t *paOut, uint64_t **vaOut) {
    if (vm->ptPagesUsed >= kMaxPtPagesPerVm) return UINT32_MAX;
    uint32_t idx = vm->ptPagesUsed++;
    uint8_t *va = vm->ptPoolVa + static_cast<size_t>(idx) * 4096ULL;
    memset(va, 0, 4096);
    IOByteCount segLen = 0;
    uint64_t pa = static_cast<uint64_t>(vm->ptPoolDesc->getPhysicalSegment(static_cast<IOByteCount>(idx) * 4096ULL, &segLen));
    if (vm->ptPagePas) vm->ptPagePas[idx] = pa & 0x0000FFFFFFFFF000ULL;
    cacheFlushRange(va, 4096);
    *paOut = pa;
    *vaOut = reinterpret_cast<uint64_t *>(va);
    return idx;
}

/* Find CPU VA of a page table page given its PA (fast cached array scan) */
static uint64_t *vmPaToVa(MaliVm *vm, uint64_t pa) {
    pa &= 0x0000FFFFFFFFF000ULL;
    if (vm->ptPagePas) {
        for (uint32_t i = 0; i < vm->ptPagesUsed; ++i) {
            if (vm->ptPagePas[i] == pa) {
                return reinterpret_cast<uint64_t *>(vm->ptPoolVa + static_cast<size_t>(i) * 4096ULL);
            }
        }
        return nullptr;
    }
    for (uint32_t i = 0; i < vm->ptPagesUsed; ++i) {
        IOByteCount segLen = 0;
        uint64_t p = static_cast<uint64_t>(vm->ptPoolDesc->getPhysicalSegment(static_cast<IOByteCount>(i) * 4096ULL, &segLen));
        if ((p & 0x0000FFFFFFFFF000ULL) == pa) {
            return reinterpret_cast<uint64_t *>(vm->ptPoolVa + static_cast<size_t>(i) * 4096ULL);
        }
    }
    return nullptr;
}

static bool vmMapPage(MaliVm *vm, uint64_t va, uint64_t pa, uint32_t flags) {
    uint32_t i0 = static_cast<uint32_t>((va >> 39) & 0x1FFu);
    uint32_t i1 = static_cast<uint32_t>((va >> 30) & 0x1FFu);
    uint32_t i2 = static_cast<uint32_t>((va >> 21) & 0x1FFu);
    uint32_t i3 = static_cast<uint32_t>((va >> 12) & 0x1FFu);

    uint64_t *tbl = vm->l0Va;
    uint32_t idxs[3] = { i0, i1, i2 };
    for (int lvl = 0; lvl < 3; ++lvl) {
        uint64_t e = tbl[idxs[lvl]];
        if ((e & 0x3ULL) != 0x3ULL) {
            uint64_t childPa = 0;
            uint64_t *childVa = nullptr;
            if (vmAllocPtPage(vm, &childPa, &childVa) == UINT32_MAX) return false;
            tbl[idxs[lvl]] = (childPa & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
            cacheFlushRange(&tbl[idxs[lvl]], 64);
            tbl = childVa;
        } else {
            tbl = vmPaToVa(vm, e);
            if (!tbl) return false;
        }
    }
    uint64_t pte = (pa & 0x0000FFFFFFFFF000ULL) | 0x3ULL | (1ULL << 10) | (1ULL << 6);
    if (flags & MALI_VM_MAP_READONLY) pte |= (1ULL << 7);
    if (flags & MALI_VM_MAP_NOEXEC)   pte |= (1ULL << 54);
    if (flags & MALI_VM_MAP_UNCACHED) pte |= (0ULL << 2) | (2ULL << 8);
    else                              pte |= (1ULL << 2) | (3ULL << 8); /* Inner Shareable: coherent across all shader cores + Tiler + L2 */
    tbl[i3] = pte;
    cacheFlushRange(&tbl[i3], 64);
    return true;
}

static void vmUnmapPage(MaliVm *vm, uint64_t va) {
    uint32_t idxs[4] = {
        static_cast<uint32_t>((va >> 39) & 0x1FFu),
        static_cast<uint32_t>((va >> 30) & 0x1FFu),
        static_cast<uint32_t>((va >> 21) & 0x1FFu),
        static_cast<uint32_t>((va >> 12) & 0x1FFu),
    };
    uint64_t *tbl = vm->l0Va;
    for (int lvl = 0; lvl < 3; ++lvl) {
        uint64_t e = tbl[idxs[lvl]];
        if ((e & 0x3ULL) != 0x3ULL) return;
        tbl = vmPaToVa(vm, e);
        if (!tbl) return;
    }
    tbl[idxs[3]] = 0;
    cacheFlushRange(&tbl[idxs[3]], 64);
}

static void vmFlushMmu(MaliVm *vm) {
    uint32_t as = vm->as_slot;
    wr64(gMali.maliReg.va, AS_REG(as, AS_LOCKADDR_LO), 47ULL); /* full 48-bit lock (log2(2^48)-1 = 47) */
    wr32(gMali.maliReg.va, AS_REG(as, AS_COMMAND), AS_CMD_LOCK);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(as, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
    wr32(gMali.maliReg.va, AS_REG(as, AS_COMMAND), AS_CMD_FLUSH_PT);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(as, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
}

static void vmProgramAs(MaliVm *vm) {
    uint32_t as = vm->as_slot;
    uint64_t memattr  = 0x4C4C4C4C4C4C8F4CULL;
    uint64_t transcfg = 6ULL | (7ULL << 6) | (1ULL << 24);
    wr64(gMali.maliReg.va, AS_REG(as, AS_TRANSTAB_LO), vm->l0Pa & 0x0000FFFFFFFFF000ULL);
    wr64(gMali.maliReg.va, AS_REG(as, AS_MEMATTR_LO), memattr);
    wr64(gMali.maliReg.va, AS_REG(as, AS_TRANSCFG_LO), transcfg);
    wr32(gMali.maliReg.va, AS_REG(as, AS_COMMAND), AS_CMD_UPDATE);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(as, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
}

/* ---- Hardware bring-up (proven Boot 62 sequence) ---- */
static bool initMaliHardware(void) {
    if (!mapMmio(MT6897_GPUEB_GPR_PHYS, 0x1000, &gMali.gpuebGpr) ||
        !mapMmio(MT6897_GPUEB_MBOX_PHYS, 0x1000, &gMali.gpuebMbox) ||
        !mapMmio(MT6897_MFG_RPC_PHYS, 0x1000, &gMali.mfgRpc) ||
        !mapMmio(MT6897_GPUEB_DRAM_PHYS, MT6897_GPUEB_DRAM_SIZE, &gMali.gpuebDram)) {
        IOLog("Tb375fcMali: FATAL GPUEB MMIO map failed\n");
        return false;
    }

    uintptr_t gprVa  = gMali.gpuebGpr.va + GPR_BASE_OFF;
    uintptr_t mboxVa = gMali.gpuebGpr.va + MBOX0_BASE_OFF;
    uint32_t lpState  = rd32(gprVa, GPR_LP_STATE);
    uint32_t ipiMagic = rd32(gprVa, GPR_IPI_MAGIC);
    if (lpState != 1u || ipiMagic == 0u) {
        upd32(gMali.mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ENABLE_M, RPC_GHPM_ENABLE_M);
        upd32(gMali.mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ON_SEQ_M, 0u);
        upd32(gMali.mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ON_SEQ_M, RPC_GHPM_ON_SEQ_M);
        upd32(gMali.mfgRpc.va, RPC_GHPM_CFG0_CON, RPC_GHPM_ENABLE_M, 0u);
        for (int i = 0; i < 200; ++i) {
            if (rd32(gprVa, GPR_LP_STATE) == 1u && rd32(gprVa, GPR_IPI_MAGIC) != 0u) break;
            IODelay(50);
        }
        ipiMagic = rd32(gprVa, GPR_IPI_MAGIC);
        if (ipiMagic == 0u) return false;
    }

    for (uint32_t off = 0; off < MT6897_GPUEB_DRAM_SIZE; off += 4u) wr32(gMali.gpuebDram.va, off, 0u);
    GfIpiMsg tx = {}, rx = {};
    tx.magic = ipiMagic;
    tx.cmd   = CMD_INIT_SHARED_MEM;
    tx.u.shared_mem.base = MT6897_GPUEB_DRAM_PHYS;
    tx.u.shared_mem.size = MT6897_GPUEB_DRAM_SIZE;
    if (gpuebSendGfIpi(mboxVa, gMali.gpuebMbox.va, tx, &rx) != 0 ||
        rd32(gMali.gpuebDram.va, GF_REG_MAGIC) != GPUEB_MEM_MAGIC) {
        IOLog("Tb375fcMali: GPUEB INIT_SHARED_MEM failed\n");
        return false;
    }

    memset(&tx, 0, sizeof(tx));
    memset(&rx, 0, sizeof(rx));
    tx.magic = ipiMagic;
    tx.cmd   = CMD_POWER_CONTROL;
    tx.u.power_state = 1u;
    if (gpuebSendGfIpi(mboxVa, gMali.gpuebMbox.va, tx, &rx) != 0 ||
        rd32(gMali.gpuebDram.va, GF_REG_POWER_COUNT) == 0u) {
        IOLog("Tb375fcMali: GPUEB POWER_CONTROL(ON) failed\n");
        return false;
    }

    if (!mapMmio(MT6897_MALI_PHYS, MT6897_MALI_MAP_SIZE, &gMali.maliReg)) {
        IOLog("Tb375fcMali: failed to map mali@13000000\n");
        return false;
    }

    uintptr_t mv = gMali.maliReg.va;
    /* Soft reset + L2 power on */
    wr32(mv, GPU_INT_CLEAR, 0xFFFFFFFFu);
    wr32(mv, GPU_CMD, GPU_CMD_SOFT_RESET);
    for (int i = 0; i < 200; ++i) {
        if (rd32(mv, GPU_INT_RAWSTAT) & GPU_IRQ_RESET_DONE) break;
        IOSleep(1);
    }
    wr32(mv, GPU_INT_CLEAR, 0xFFFFFFFFu);
    wr32(mv, GPU_COHERENCY_PROT, 31u);
    wr32(mv, L2_PWRON_LO, 1u);
    for (int i = 0; i < 200; ++i) {
        if (rd32(mv, L2_READY_LO) & 1u) break;
        IOSleep(1);
    }

    /* Populate Tb375fcMaliDevInfo from live hardware registers */
    Tb375fcMaliDevInfo &di = gMali.devInfo;
    memset(&di, 0, sizeof(di));
    di.abi_version               = TB375FC_MALI_ABI_VERSION;
    di.gpu_id                    = rd32(mv, GPU_ID_REG);
    di.gpu_rev                   = 0;
    di.csf_id                    = rd32(mv, GPU_CSF_ID);
    di.l2_features               = rd32(mv, GPU_L2_FEATURES);
    di.tiler_features            = rd32(mv, GPU_TILER_FEATURES);
    di.mem_features              = rd32(mv, GPU_MEM_FEATURES);
    di.mmu_features              = rd32(mv, GPU_MMU_FEATURES);
    di.thread_features           = rd32(mv, GPU_THREAD_FEATURES);
    di.max_threads               = rd32(mv, GPU_THREAD_MAX_THREADS);
    di.thread_max_workgroup_size = rd32(mv, GPU_THREAD_MAX_WG_SIZE);
    di.thread_max_barrier_size   = rd32(mv, GPU_THREAD_MAX_BARRIER);
    di.coherency_features        = rd32(mv, GPU_COHERENCY_FEATURES);
    for (uint32_t i = 0; i < 4; ++i) di.texture_features[i] = rd32(mv, GPU_TEX_FEATURES_0 + i * 4u);
    di.as_present                = rd32(mv, GPU_AS_PRESENT);
    di.selected_coherency        = 0u; /* ACE_LITE: expose HOST_VISIBLE|HOST_COHERENT|HOST_CACHED to panvk */
    di.shader_present            = static_cast<uint64_t>(rd32(mv, GPU_SHADER_PRESENT_LO)) |
                                   (static_cast<uint64_t>(rd32(mv, GPU_SHADER_PRESENT_HI)) << 32);
    di.tiler_present             = static_cast<uint64_t>(rd32(mv, GPU_TILER_PRESENT_LO)) |
                                   (static_cast<uint64_t>(rd32(mv, GPU_TILER_PRESENT_HI)) << 32);
    di.l2_present                = static_cast<uint64_t>(rd32(mv, GPU_L2_PRESENT_LO)) |
                                   (static_cast<uint64_t>(rd32(mv, GPU_L2_PRESENT_HI)) << 32);
    di.timestamp_frequency       = 24000000ULL;

    /* Boot CSF MCU (mali_csffw.bin) into AS0 */
    size_t fwLen = 0;
    uint8_t *fw = loadMaliFwFromVfs(&fwLen);
    if (!fw) {
        IOLog("Tb375fcMali: mali_csffw.bin missing; HW regs ready but MCU offline\n");
        gMali.asSlotMask = 1u;
        gMali.ready = true;
        return true;
    }

    constexpr uint32_t kTotalPages = 512u; /* 2 MiB CSF arena */
    gMali.csfArenaDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIOMemoryKernelUserShared | kIODirectionInOut,
        static_cast<size_t>(kTotalPages) * 4096ULL, 0x000000FFFFFFF000ULL);
    if (!gMali.csfArenaDesc || gMali.csfArenaDesc->prepare() != kIOReturnSuccess) {
        IOFreeData(fw, fwLen);
        return false;
    }
    gMali.csfArenaVa = static_cast<uint8_t *>(gMali.csfArenaDesc->getBytesNoCopy());
    memset(gMali.csfArenaVa, 0, static_cast<size_t>(kTotalPages) * 4096ULL);

    uint64_t *l0 = reinterpret_cast<uint64_t *>(arenaPageVa(0));
    uint64_t *l1 = reinterpret_cast<uint64_t *>(arenaPageVa(1));
    uint64_t *l2 = reinterpret_cast<uint64_t *>(arenaPageVa(2));
    l0[0] = (arenaPagePa(1) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    l1[0] = (arenaPagePa(2) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    for (uint32_t l2i = 0; l2i <= 32u; ++l2i) {
        l2[l2i] = (arenaPagePa(3u + l2i) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    }
    uint32_t nextPg = 36u;
    uint32_t magic = 0, hdrSize = 0;
    memcpy(&magic, fw, 4);
    memcpy(&hdrSize, fw + 16, 4);
    if (magic == 0xC3F13A6Eu && hdrSize <= fwLen) {
        uint32_t off = 20u;
        while (off + 4u <= hdrSize) {
            uint32_t ehdr = 0;
            memcpy(&ehdr, fw + off, 4);
            uint32_t etype = ehdr & 0xFFu;
            uint32_t esize = (ehdr >> 8) & 0xFFFFFFu;
            if (esize < 4u || off + esize > hdrSize) break;
            if (etype == 0u && esize >= 24u) {
                uint32_t flags = 0, vaStart = 0, vaEnd = 0, dStart = 0, dEnd = 0;
                memcpy(&flags, fw + off + 4u, 4);
                memcpy(&vaStart, fw + off + 8u, 4);
                memcpy(&vaEnd, fw + off + 12u, 4);
                memcpy(&dStart, fw + off + 16u, 4);
                memcpy(&dEnd, fw + off + 20u, 4);
                if ((flags & 0x20u) == 0u && vaEnd > vaStart && vaEnd <= 0x04200000u && dEnd <= fwLen) {
                    uint32_t numPages = (vaEnd - vaStart) / 4096u;
                    uint32_t dataLen  = (dEnd > dStart) ? (dEnd - dStart) : 0u;
                    for (uint32_t p = 0; p < numPages && nextPg < kTotalPages; ++p) {
                        uint32_t pgIdx = nextPg++;
                        uint8_t *dst = arenaPageVa(pgIdx);
                        uint64_t pa  = arenaPagePa(pgIdx);
                        uint32_t pgOff = p * 4096u;
                        if (pgOff < dataLen) {
                            uint32_t cpy = dataLen - pgOff;
                            if (cpy > 4096u) cpy = 4096u;
                            memcpy(dst, fw + dStart + pgOff, cpy);
                        }
                        uint32_t va = vaStart + pgOff;
                        if (va == 0x04000000u) {
                            gMali.sharedIfaceVa = dst;
                            gMali.sharedIfaceSize = numPages * 4096u;
                        }
                        bool isNc = ((flags & 0x10u) != 0u) || ((flags & 0x08u) == 0u);
                        uint32_t l2Idx = (va >> 21) & 0x1FFu;
                        uint32_t l3Idx = (va >> 12) & 0x1FFu;
                        uint64_t *l3   = reinterpret_cast<uint64_t *>(arenaPageVa(3u + l2Idx));
                        uint64_t pte   = (pa & 0x0000FFFFFFFFF000ULL) | 0x3ULL | (1ULL << 10) | (1ULL << 6);
                        if ((flags & 0x02u) == 0u) pte |= (1ULL << 7);
                        if ((flags & 0x04u) == 0u) pte |= (1ULL << 54);
                        if (isNc) pte |= (0ULL << 2) | (2ULL << 8);
                        else      pte |= (1ULL << 2);
                        l3[l3Idx] = pte;
                    }
                }
            }
            off += esize;
        }
    }
    IOFreeData(fw, fwLen);
    gMali.nextAs0FreePage = nextPg;
    gMali.nextAs0FreeVa   = 0x04080000u; /* Free MCU VA region above shared iface (inside l2[32]) */
    cacheFlushRange(gMali.csfArenaVa, static_cast<size_t>(kTotalPages) * 4096ULL);

    /* Enable AS0 and boot MCU */
    wr32(mv, MMU_INT_CLEAR, 0xFFFFFFFFu);
    wr32(mv, MMU_INT_MASK, 0xFFFFFFFFu);
    wr64(mv, AS_REG(0, AS_TRANSTAB_LO), arenaPagePa(0) & 0x0000FFFFFFFFF000ULL);
    wr64(mv, AS_REG(0, AS_MEMATTR_LO), 0x4C4C4C4C4C4C8F4CULL);
    wr64(mv, AS_REG(0, AS_TRANSCFG_LO), 6ULL | (7ULL << 6) | (1ULL << 24));
    wr32(mv, AS_REG(0, AS_COMMAND), AS_CMD_UPDATE);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(mv, AS_REG(0, AS_STATUS)) & 1u) == 0u) break;
        IODelay(10);
    }

    wr32(mv, JOB_INT_CLEAR, 0xFFFFFFFFu);
    wr32(mv, JOB_INT_MASK, 0xFFFFFFFFu);
    wr32(mv, MCU_CONTROL, MCU_CONTROL_AUTO);
    for (int ms = 0; ms < 500; ++ms) {
        if ((rd32(mv, JOB_INT_RAWSTAT) & JOB_INT_GLOBAL_IF) != 0u) break;
        IOSleep(1);
    }

    if (gMali.sharedIfaceVa) {
        cacheFlushRange(gMali.sharedIfaceVa, gMali.sharedIfaceSize);
        FwGlobalControl *glbCtrl = reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa);
        gMali.glbInOff  = (glbCtrl->input_va  >= CSF_SHARED_BASE_VA) ? (glbCtrl->input_va  - CSF_SHARED_BASE_VA) : 0x1000u;
        gMali.glbOutOff = (glbCtrl->output_va >= CSF_SHARED_BASE_VA) ? (glbCtrl->output_va - CSF_SHARED_BASE_VA) : 0x2000u;
        gMali.csgCount  = glbCtrl->group_num ? glbCtrl->group_num : 8u;
        gMali.csgStride = glbCtrl->group_stride ? glbCtrl->group_stride : 0x100u;
        di.fw_iface_ver = glbCtrl->version;
        di.mcu_alive    = (glbCtrl->version != 0u) ? 1u : 0u;
        di.csg_slot_count          = gMali.csgCount;
        di.cs_slot_count           = 8u;
        di.cs_reg_count            = 96u;
        di.scoreboard_slot_count   = 8u;
        di.unpreserved_cs_reg_count = 4u;

        FwCsgControl *csg0 = getCsgCtrl(0);
        if (csg0) {
            gMali.csCount        = csg0->stream_num ? csg0->stream_num : 8u;
            gMali.csStride       = csg0->stream_stride;
            gMali.csgSuspendSize = csg0->suspend_size;
            di.cs_slot_count     = gMali.csCount;
            FwCsControl *cs0 = getCsCtrl(0, 0);
            if (cs0 && cs0->features) {
                di.cs_reg_count          = (cs0->features & 0xFFu) + 1u;
                di.scoreboard_slot_count = (cs0->features >> 8) & 0xFFu;
            }
        }

        /* Configure global MCU interface: enable shader cores & timers (GLB_CFG_ALLOC_EN) */
        FwGlobalInput  *glbIn  = reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va));
        FwGlobalOutput *glbOut = reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va));
        if (glbIn && glbOut && di.mcu_alive) {
            cacheFlushRange(glbOut, sizeof(*glbOut));
            uint32_t ack0 = glbOut->ack;
            glbIn->core_en_mask   = di.shader_present;
            glbIn->poweroff_timer = 0x00100000u;
            glbIn->progress_timer = 0x00040000u;
            glbIn->idle_timer     = 0x00100000u;
            glbIn->ack_irq_mask   = 0xFFFFFFFFu;
            uint32_t toggle = GLB_CFG_ALLOC_EN | GLB_CFG_POWEROFF_TIMER | GLB_CFG_PROGRESS_TIMER;
            uint32_t newReq = (ack0 ^ toggle) | GLB_COUNTER_EN;
            glbIn->req = newReq;
            cacheFlushRange(glbIn, sizeof(*glbIn));
            wr32(mv, CSF_DOORBELL(0), 1u);
            bool cfgAcked = false;
            for (int ms = 0; ms < 500; ++ms) {
                cacheFlushRange(glbOut, sizeof(*glbOut));
                if (((glbOut->ack ^ newReq) & toggle) == 0u) { cfgAcked = true; break; }
                IOSleep(1);
            }
            IOLog("Tb375fcMali: GLB_CFG_ALLOC_EN %s (req=0x%x ack=0x%x suspend=%u cs=%u regs=%u sb=%u)\n",
                  cfgAcked ? "ACK" : "TIMEOUT", newReq, glbOut->ack,
                  gMali.csgSuspendSize, di.cs_slot_count, di.cs_reg_count, di.scoreboard_slot_count);
        }
    }

    gMali.asSlotMask = 1u; /* AS0 reserved for MCU */
    gMali.ready = true;
    IOLog("Tb375fcMali: READY gpu_id=0x%08x shader=0x%llx fw=0x%08x csg=%u mcu=%u\n",
          di.gpu_id, (unsigned long long)di.shader_present, di.fw_iface_ver, di.csg_slot_count, di.mcu_alive);
    return true;
}

} // namespace

/* ---- IOService & IOUserClient classes ---- */

class Tb375fcMali : public IOService {
    OSDeclareDefaultStructors(Tb375fcMali)
public:
    bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
    IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type,
                           OSDictionary *properties, IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};

constexpr uint32_t kMaxClientBos    = 16384u;
constexpr uint32_t kMaxClientVms    = 8u;
constexpr uint32_t kMaxClientGroups = 8u;

struct MaliBoEntry {
    bool                in_use;
    IOMemoryDescriptor *desc;
    uint64_t            size;
};

struct MaliGroupEntry {
    bool     in_use;
    bool     hw_started;
    uint32_t vm_id;
    uint32_t csg_slot;
    uint32_t queue_count;
    uint32_t suspendFwVa;
    uint64_t last_seq;
    IOBufferMemoryDescriptor *ringDesc[MALI_MAX_QUEUES_PER_GROUP];
    uint64_t ringGpuVa[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t ringSize[MALI_MAX_QUEUES_PER_GROUP];
    uint64_t ringInsert[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t dbId[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t ringIoInFwVa[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t ringIoOutFwVa[MALI_MAX_QUEUES_PER_GROUP];
    FwRingIoInput  *ringIoInCpu[MALI_MAX_QUEUES_PER_GROUP];
    FwRingIoOutput *ringIoOutCpu[MALI_MAX_QUEUES_PER_GROUP];
};

class Tb375fcMaliUserClient : public IOUserClient {
    OSDeclareDefaultStructors(Tb375fcMaliUserClient)
private:
    task_t          fTask;
    IOLock         *fLock;
    MaliBoEntry    *fBos;
    MaliVm         *fVms;
    MaliGroupEntry *fGroups;
public:
    bool initWithTask(task_t owningTask, void *securityToken, UInt32 type,
                      OSDictionary *properties) APPLE_KEXT_OVERRIDE;
    IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
    void free(void) APPLE_KEXT_OVERRIDE;
    IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options,
                                 IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
    IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
                            IOExternalMethodDispatch *dispatch, OSObject *target,
                            void *reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(Tb375fcMali, IOService)
OSDefineMetaClassAndStructors(Tb375fcMaliUserClient, IOUserClient)

bool Tb375fcMali::start(IOService *provider) {
    if (!IOService::start(provider)) return false;
    setName(TB375FC_MALI_SERVICE_NAME);
    registerService();
    return true;
}

IOReturn Tb375fcMali::newUserClient(task_t owningTask, void *securityID, UInt32 type,
                                    OSDictionary *properties, IOUserClient **handler) {
    IOLog("Tb375fcMali: newUserClient type=%u\n", (unsigned)type);
    Tb375fcMaliUserClient *uc = new Tb375fcMaliUserClient;
    if (!uc) return kIOReturnNoMemory;
    if (!uc->initWithTask(owningTask, securityID, type, properties) ||
        !uc->attach(this) || !uc->start(this)) {
        uc->release();
        return kIOReturnError;
    }
    *handler = uc;
    return kIOReturnSuccess;
}

bool Tb375fcMaliUserClient::initWithTask(task_t owningTask, void *securityToken,
                                         UInt32 type, OSDictionary *properties) {
    if (!IOUserClient::initWithTask(owningTask, securityToken, type, properties)) return false;
    fTask   = owningTask;
    fLock   = IOLockAlloc();
    fBos    = static_cast<MaliBoEntry *>(IOMallocZeroData(sizeof(MaliBoEntry) * kMaxClientBos));
    fVms    = static_cast<MaliVm *>(IOMallocZeroData(sizeof(MaliVm) * kMaxClientVms));
    fGroups = static_cast<MaliGroupEntry *>(IOMallocZeroData(sizeof(MaliGroupEntry) * kMaxClientGroups));
    return fLock && fBos && fVms && fGroups;
}

IOReturn Tb375fcMaliUserClient::clientClose(void) {
    terminate();
    return kIOReturnSuccess;
}

void Tb375fcMaliUserClient::free(void) {
    if (fBos) {
        for (uint32_t i = 0; i < kMaxClientBos; ++i) {
            if (fBos[i].in_use && fBos[i].desc) {
                fBos[i].desc->complete();
                fBos[i].desc->release();
            }
        }
        IOFreeData(fBos, sizeof(MaliBoEntry) * kMaxClientBos);
        fBos = nullptr;
    }
    if (fVms) {
        for (uint32_t i = 0; i < kMaxClientVms; ++i) {
            if (fVms[i].in_use) {
                if (fVms[i].ptPagePas) { IOFreeData(fVms[i].ptPagePas, sizeof(uint64_t) * kMaxPtPagesPerVm); fVms[i].ptPagePas = nullptr; }
                if (fVms[i].ptPoolDesc) { fVms[i].ptPoolDesc->complete(); fVms[i].ptPoolDesc->release(); }
                if (gMali.lock) {
                    IOLockLock(gMali.lock);
                    gMali.asSlotMask &= ~(1u << fVms[i].as_slot);
                    IOLockUnlock(gMali.lock);
                }
            }
        }
        IOFreeData(fVms, sizeof(MaliVm) * kMaxClientVms);
        fVms = nullptr;
    }
    if (fGroups) {
        for (uint32_t i = 0; i < kMaxClientGroups; ++i) {
            if (fGroups[i].in_use) {
                for (uint32_t q = 0; q < fGroups[i].queue_count; ++q) {
                    if (fGroups[i].ringDesc[q]) {
                        fGroups[i].ringDesc[q]->complete();
                        fGroups[i].ringDesc[q]->release();
                    }
                }
                if (gMali.lock) {
                    IOLockLock(gMali.lock);
                    gMali.csgSlotMask &= ~(1u << fGroups[i].csg_slot);
                    IOLockUnlock(gMali.lock);
                }
            }
        }
        IOFreeData(fGroups, sizeof(MaliGroupEntry) * kMaxClientGroups);
        fGroups = nullptr;
    }
    if (fLock) { IOLockFree(fLock); fLock = nullptr; }
    IOUserClient::free();
}

IOReturn Tb375fcMaliUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options,
                                                    IOMemoryDescriptor **memory) {
    if (!memory || type == 0 || type >= kMaxClientBos) return kIOReturnBadArgument;
    IOLockLock(fLock);
    MaliBoEntry &b = fBos[type];
    if (!b.in_use || !b.desc) {
        IOLockUnlock(fLock);
        return kIOReturnNotFound;
    }
    b.desc->retain();
    *memory = b.desc;
    if (options) *options = kIOMapWriteCombineCache;
    IOLockUnlock(fLock);
    return kIOReturnSuccess;
}

IOReturn Tb375fcMaliUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
                                               IOExternalMethodDispatch *, OSObject *, void *) {
    if (!gMali.ready || !args) return kIOReturnNotReady;

    switch (selector) {
    case MALI_SEL_DEV_QUERY: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliDevInfo))
            return kIOReturnBadArgument;
        Tb375fcMaliDevInfo info = gMali.devInfo;
        uintptr_t mv = gMali.maliReg.va;
        info.current_timestamp = static_cast<uint64_t>(rd32(mv, GPU_TIMESTAMP_LO)) |
                                 (static_cast<uint64_t>(rd32(mv, GPU_TIMESTAMP_HI)) << 32);
        memcpy(args->structureOutput, &info, sizeof(info));
        args->structureOutputSize = sizeof(info);
        return kIOReturnSuccess;
    }

    case MALI_SEL_BO_CREATE: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliBoCreateIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliBoCreateOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliBoCreateIn *>(args->structureInput);
        uint64_t sz = (in->size + 4095ULL) & ~4095ULL;
        if (sz == 0 || sz > 512ULL * 1024ULL * 1024ULL) return kIOReturnBadArgument;

        IOBufferMemoryDescriptor *desc = nullptr;
        /* Only allocate NO_MMAP or >= 8 MiB BOs (e.g. 84 MiB TLS scratch) in fTask pageable VM;
         * keep all CPU-mmapped CS/desc/staging BOs (< 8 MiB) in KernelUserShared + WriteCombine! */
        bool isLargeOrNoMmap = ((in->flags & MALI_BO_NO_MMAP) != 0) || (sz >= 8ULL * 1024ULL * 1024ULL);
        if (isLargeOrNoMmap) {
            /* Allocate large / GPU-only BOs (e.g. 84 MiB TLS scratch) in fTask user VM map
             * so we never exhaust kernel_map VA space! XNU zero-fills pageable VM pages automatically. */
            IOOptionBits opts = kIODirectionInOut | kIOMemoryPageable;
            if ((in->flags & MALI_BO_NO_MMAP) == 0) opts |= kIOMapWriteCombineCache;
            desc = IOBufferMemoryDescriptor::inTaskWithOptions(fTask, opts, sz, 4096);
            if (!desc || desc->prepare() != kIOReturnSuccess) {
                if (desc) desc->release();
                return kIOReturnNoMemory;
            }
            if (sz >= 16ULL * 1024ULL * 1024ULL) {
                IOByteCount segLen = 0;
                uint64_t pa0 = static_cast<uint64_t>(desc->getPhysicalSegment(0, &segLen));
                IOLog("Tb375fcMali: large BO %llu MiB OK in fTask (pa0=0x%llx)\n",
                      (unsigned long long)(sz >> 20), (unsigned long long)pa0);
            }
        } else {
            desc = IOBufferMemoryDescriptor::withOptions(
                kIOMemoryKernelUserShared | kIODirectionInOut | kIOMapWriteCombineCache, sz, 4096);
            if (!desc || desc->prepare() != kIOReturnSuccess) {
                if (desc) desc->release();
                return kIOReturnNoMemory;
            }
            memset(desc->getBytesNoCopy(), 0, sz);
            cacheFlushRange(desc->getBytesNoCopy(), sz);
        }

        IOLockLock(fLock);
        uint32_t handle = 0;
        for (uint32_t i = 1; i < kMaxClientBos; ++i) {
            if (!fBos[i].in_use) {
                fBos[i].in_use = true;
                fBos[i].desc = desc;
                fBos[i].size = sz;
                handle = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!handle) { desc->complete(); desc->release(); return kIOReturnNoResources; }

        auto *out = static_cast<Tb375fcMaliBoCreateOut *>(args->structureOutput);
        out->bo_handle = handle;
        out->pad = 0;
        out->size = sz;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_BO_DESTROY: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliHandleIn))
            return kIOReturnBadArgument;
        uint32_t h = static_cast<const Tb375fcMaliHandleIn *>(args->structureInput)->handle;
        if (h == 0 || h >= kMaxClientBos) return kIOReturnBadArgument;
        IOLockLock(fLock);
        IOMemoryDescriptor *desc = nullptr;
        if (fBos[h].in_use) {
            desc = fBos[h].desc;
            fBos[h].in_use = false;
            fBos[h].desc = nullptr;
            fBos[h].size = 0;
        }
        IOLockUnlock(fLock);
        if (desc) { desc->complete(); desc->release(); }
        return kIOReturnSuccess;
    }

    case MALI_SEL_BO_IMPORT_HOST: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliBoImportHostIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliBoCreateOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliBoImportHostIn *>(args->structureInput);
        if ((in->host_ptr & 4095ULL) != 0 || in->size == 0) return kIOReturnBadArgument;
        uint64_t sz = (in->size + 4095ULL) & ~4095ULL;
        IOMemoryDescriptor *desc = IOMemoryDescriptor::withAddressRange(
            static_cast<mach_vm_address_t>(in->host_ptr), static_cast<mach_vm_size_t>(sz),
            kIODirectionInOut, fTask);
        if (!desc || desc->prepare() != kIOReturnSuccess) {
            if (desc) desc->release();
            return kIOReturnNoMemory;
        }
        IOLockLock(fLock);
        uint32_t handle = 0;
        for (uint32_t i = 1; i < kMaxClientBos; ++i) {
            if (!fBos[i].in_use) {
                fBos[i].in_use = true;
                fBos[i].desc = desc;
                fBos[i].size = sz;
                handle = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!handle) { desc->complete(); desc->release(); return kIOReturnNoResources; }
        auto *out = static_cast<Tb375fcMaliBoCreateOut *>(args->structureOutput);
        out->bo_handle = handle;
        out->pad = 0;
        out->size = sz;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_VM_CREATE: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliVmCreateOut))
            return kIOReturnBadArgument;
        IOLockLock(gMali.lock);
        uint32_t asSlot = 0;
        for (uint32_t a = 1; a < 8u; ++a) {
            if ((gMali.devInfo.as_present & (1u << a)) && !(gMali.asSlotMask & (1u << a))) {
                gMali.asSlotMask |= (1u << a);
                asSlot = a;
                break;
            }
        }
        IOLockUnlock(gMali.lock);
        if (!asSlot) return kIOReturnNoResources;

        IOBufferMemoryDescriptor *ptDesc = IOBufferMemoryDescriptor::withOptions(
            kIOMemoryKernelUserShared | kIODirectionInOut,
            static_cast<size_t>(kMaxPtPagesPerVm) * 4096ULL, 4096);
        if (!ptDesc || ptDesc->prepare() != kIOReturnSuccess) {
            if (ptDesc) ptDesc->release();
            IOLockLock(gMali.lock); gMali.asSlotMask &= ~(1u << asSlot); IOLockUnlock(gMali.lock);
            return kIOReturnNoMemory;
        }

        IOLockLock(fLock);
        uint32_t vmId = 0;
        for (uint32_t i = 1; i < kMaxClientVms; ++i) {
            if (!fVms[i].in_use) {
                MaliVm &v = fVms[i];
                memset(&v, 0, sizeof(v));
                v.in_use = true;
                v.id = i;
                v.as_slot = asSlot;
                v.ptPoolDesc = ptDesc;
                v.ptPoolVa = static_cast<uint8_t *>(ptDesc->getBytesNoCopy());
                v.ptPagePas = static_cast<uint64_t *>(IOMallocZeroData(sizeof(uint64_t) * kMaxPtPagesPerVm));
                v.nextInternalVa = 0x000000F000000000ULL;
                vmAllocPtPage(&v, &v.l0Pa, &v.l0Va);
                /* Map a 4K scratch guard page at VA 0x0 so any null-descriptor/optional write
                 * (e.g. @0x50/@0x70) never permanently stalls AS1's MMU! */
                uint64_t nullPgPa = 0;
                uint64_t *nullPgVa = nullptr;
                if (vmAllocPtPage(&v, &nullPgPa, &nullPgVa) != UINT32_MAX) {
                    vmMapPage(&v, 0ULL, nullPgPa, MALI_VM_MAP_NOEXEC | MALI_VM_MAP_UNCACHED);
                }
                vmProgramAs(&v);
                vmId = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!vmId) {
            ptDesc->complete(); ptDesc->release();
            IOLockLock(gMali.lock); gMali.asSlotMask &= ~(1u << asSlot); IOLockUnlock(gMali.lock);
            return kIOReturnNoResources;
        }
        auto *out = static_cast<Tb375fcMaliVmCreateOut *>(args->structureOutput);
        out->vm_id = vmId;
        out->as_slot = asSlot;
        out->va_start = 0x0000000004200000ULL;
        out->va_size  = 0x000000E000000000ULL;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_VM_DESTROY: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliHandleIn))
            return kIOReturnBadArgument;
        uint32_t id = static_cast<const Tb375fcMaliHandleIn *>(args->structureInput)->handle;
        if (id == 0 || id >= kMaxClientVms) return kIOReturnBadArgument;
        IOLockLock(fLock);
        if (fVms[id].in_use) {
            uint32_t asSlot = fVms[id].as_slot;
            if (fVms[id].ptPagePas) { IOFreeData(fVms[id].ptPagePas, sizeof(uint64_t) * kMaxPtPagesPerVm); fVms[id].ptPagePas = nullptr; }
            if (fVms[id].ptPoolDesc) { fVms[id].ptPoolDesc->complete(); fVms[id].ptPoolDesc->release(); }
            fVms[id].in_use = false;
            IOLockLock(gMali.lock); gMali.asSlotMask &= ~(1u << asSlot); IOLockUnlock(gMali.lock);
        }
        IOLockUnlock(fLock);
        return kIOReturnSuccess;
    }

    case MALI_SEL_VM_BIND: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliVmBindIn))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliVmBindIn *>(args->structureInput);
        if (in->vm_id == 0 || in->vm_id >= kMaxClientVms) return kIOReturnBadArgument;

        IOLockLock(fLock);
        MaliVm &vm = fVms[in->vm_id];
        if (!vm.in_use) { IOLockUnlock(fLock); return kIOReturnNotFound; }

        uint64_t pages = (in->size + 4095ULL) / 4096ULL;
        if (in->op == MALI_VM_BIND_UNMAP) {
            for (uint64_t p = 0; p < pages; ++p) vmUnmapPage(&vm, in->va + p * 4096ULL);
            vmFlushMmu(&vm);
            IOLockUnlock(fLock);
            return kIOReturnSuccess;
        }
        if (in->bo_handle == 0 || in->bo_handle >= kMaxClientBos || !fBos[in->bo_handle].in_use) {
            IOLockUnlock(fLock);
            return kIOReturnNotFound;
        }
        IOMemoryDescriptor *desc = fBos[in->bo_handle].desc;
        for (uint64_t p = 0; p < pages; ++p) {
            IOByteCount segLen = 0;
            uint64_t pa = static_cast<uint64_t>(
                desc->getPhysicalSegment(static_cast<IOByteCount>(in->bo_offset + p * 4096ULL), &segLen));
            if (!pa || !vmMapPage(&vm, in->va + p * 4096ULL, pa, in->flags)) {
                IOLockUnlock(fLock);
                return kIOReturnNoMemory;
            }
        }
        vmFlushMmu(&vm);
        IOLockUnlock(fLock);
        return kIOReturnSuccess;
    }

    case MALI_SEL_TILER_HEAP_CREATE: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliTilerHeapCreateIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliTilerHeapCreateOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliTilerHeapCreateIn *>(args->structureInput);
        if (in->vm_id == 0 || in->vm_id >= kMaxClientVms) return kIOReturnBadArgument;
        uint32_t chunkSz = (in->chunk_size + 4095u) & ~4095u;
        if (chunkSz < 128u * 1024u) chunkSz = 256u * 1024u;
        uint32_t cnt = in->initial_chunk_count ? in->initial_chunk_count : 4u;
        if (cnt < 8u) cnt = 8u;
        if (cnt > 16u) cnt = 16u;
        uint64_t totalSz = 4096ULL + static_cast<uint64_t>(cnt) * chunkSz;

        IOBufferMemoryDescriptor *desc = IOBufferMemoryDescriptor::withOptions(
            kIOMemoryKernelUserShared | kIODirectionInOut, totalSz, 4096);
        if (!desc || desc->prepare() != kIOReturnSuccess) {
            if (desc) desc->release();
            return kIOReturnNoMemory;
        }
        memset(desc->getBytesNoCopy(), 0, totalSz);

        IOLockLock(fLock);
        MaliVm &vm = fVms[in->vm_id];
        if (!vm.in_use) { IOLockUnlock(fLock); desc->complete(); desc->release(); return kIOReturnNotFound; }
        uint64_t baseVa = vm.nextInternalVa;
        vm.nextInternalVa += (totalSz + 0xFFFFFULL) & ~0xFFFFFULL;
        for (uint64_t off = 0; off < totalSz; off += 4096ULL) {
            IOByteCount segLen = 0;
            uint64_t pa = static_cast<uint64_t>(desc->getPhysicalSegment(static_cast<IOByteCount>(off), &segLen));
            vmMapPage(&vm, baseVa + off, pa, MALI_VM_MAP_NOEXEC);
        }
        /* Link chunks in a singly-linked list (panthor_heap.c:178: next_gpuva[63:12] | (chunk_size >> 12)) */
        uint8_t *cpuBase = static_cast<uint8_t *>(desc->getBytesNoCopy());
        for (uint32_t c = 0; c < cnt; ++c) {
            uint64_t *hdr = reinterpret_cast<uint64_t *>(cpuBase + 4096ULL + static_cast<uint64_t>(c) * chunkSz);
            uint64_t nextVa = (c + 1 < cnt) ? (baseVa + 4096ULL + static_cast<uint64_t>(c + 1) * chunkSz) : 0ULL;
            hdr[0] = nextVa ? ((nextVa & ~4095ULL) | (static_cast<uint64_t>(chunkSz) >> 12)) : 0ULL;
        }
        cacheFlushRange(cpuBase, totalSz);
        vmFlushMmu(&vm);

        uint32_t handle = 0;
        for (uint32_t i = 1; i < kMaxClientBos; ++i) {
            if (!fBos[i].in_use) { fBos[i].in_use = true; fBos[i].desc = desc; fBos[i].size = totalSz; handle = i; break; }
        }
        IOLockUnlock(fLock);
        if (!handle) { desc->complete(); desc->release(); return kIOReturnNoResources; }

        auto *out = static_cast<Tb375fcMaliTilerHeapCreateOut *>(args->structureOutput);
        out->handle = handle;
        out->pad = 0;
        out->tiler_heap_ctx_gpu_va   = baseVa;
        out->first_heap_chunk_gpu_va = baseVa + 4096ULL;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_TILER_HEAP_DESTROY: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliHandleIn))
            return kIOReturnBadArgument;
        uint32_t h = static_cast<const Tb375fcMaliHandleIn *>(args->structureInput)->handle;
        if (h == 0 || h >= kMaxClientBos) return kIOReturnBadArgument;
        IOLockLock(fLock);
        IOMemoryDescriptor *desc = nullptr;
        if (fBos[h].in_use) { desc = fBos[h].desc; fBos[h].in_use = false; fBos[h].desc = nullptr; }
        IOLockUnlock(fLock);
        if (desc) { desc->complete(); desc->release(); }
        return kIOReturnSuccess;
    }

    case MALI_SEL_GROUP_CREATE: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliGroupCreateIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliGroupCreateOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliGroupCreateIn *>(args->structureInput);
        if (in->vm_id == 0 || in->vm_id >= kMaxClientVms) return kIOReturnBadArgument;

        IOLockLock(gMali.lock);
        uint32_t csgSlot = UINT32_MAX;
        for (uint32_t s = 0; s < gMali.csgCount && s < 8u; ++s) {
            if (!(gMali.csgSlotMask & (1u << s))) { gMali.csgSlotMask |= (1u << s); csgSlot = s; break; }
        }
        IOLockUnlock(gMali.lock);
        if (csgSlot == UINT32_MAX) return kIOReturnNoResources;

        IOLockLock(fLock);
        MaliVm &vm = fVms[in->vm_id];
        if (!vm.in_use) {
            IOLockUnlock(fLock);
            IOLockLock(gMali.lock); gMali.csgSlotMask &= ~(1u << csgSlot); IOLockUnlock(gMali.lock);
            return kIOReturnNotFound;
        }
        uint32_t grpHandle = 0;
        for (uint32_t i = 1; i < kMaxClientGroups; ++i) {
            if (!fGroups[i].in_use) {
                MaliGroupEntry &g = fGroups[i];
                memset(&g, 0, sizeof(g));
                g.in_use = true;
                g.vm_id = in->vm_id;
                g.csg_slot = csgSlot;
                g.queue_count = in->queue_count ? in->queue_count : 1u;
                if (g.queue_count > MALI_MAX_QUEUES_PER_GROUP) g.queue_count = MALI_MAX_QUEUES_PER_GROUP;

                /* Allocate optional CSG suspend buffer in AS0 */
                if (gMali.csgSuspendSize > 0) {
                    uint32_t suspPages = (gMali.csgSuspendSize + 4095u) / 4096u;
                    uint8_t *suspCpu = nullptr;
                    allocAs0Pages(suspPages, false, &g.suspendFwVa, &suspCpu);
                }

                uint32_t queueMask = 0;
                for (uint32_t q = 0; q < g.queue_count; ++q) {
                    uint32_t rsz = in->queues[q].ringbuf_size;
                    if (rsz < 65536u) rsz = 65536u;
                    rsz = (rsz + 4095u) & ~4095u;
                    g.ringDesc[q] = IOBufferMemoryDescriptor::withOptions(
                        kIOMemoryKernelUserShared | kIODirectionInOut, rsz, 4096);
                    if (g.ringDesc[q] && g.ringDesc[q]->prepare() == kIOReturnSuccess) {
                        memset(g.ringDesc[q]->getBytesNoCopy(), 0, rsz);
                        cacheFlushRange(g.ringDesc[q]->getBytesNoCopy(), rsz);
                        uint64_t rva = vm.nextInternalVa;
                        vm.nextInternalVa += (rsz + 0xFFFFFULL) & ~0xFFFFFULL;
                        for (uint32_t off = 0; off < rsz; off += 4096u) {
                            IOByteCount segLen = 0;
                            uint64_t pa = static_cast<uint64_t>(g.ringDesc[q]->getPhysicalSegment(off, &segLen));
                            vmMapPage(&vm, rva + off, pa, MALI_VM_MAP_UNCACHED);
                        }
                        g.ringGpuVa[q] = rva;
                        g.ringSize[q] = rsz;
                        g.dbId[q] = csgSlot + 1u; /* Per-CSG doorbell 1..8 (exact panthor_sched.c convention; DB0 is global) */
                        uint8_t *ioInCpu = nullptr, *ioOutCpu = nullptr;
                        if (allocAs0Pages(1u, true, &g.ringIoInFwVa[q], &ioInCpu) &&
                            allocAs0Pages(1u, true, &g.ringIoOutFwVa[q], &ioOutCpu)) {
                            g.ringIoInCpu[q]  = reinterpret_cast<FwRingIoInput *>(ioInCpu);
                            g.ringIoOutCpu[q] = reinterpret_cast<FwRingIoOutput *>(ioOutCpu);
                            queueMask |= (1u << q);
                        }
                    }
                }
                vmFlushMmu(&vm);

                /* Program CS & CSG shared interfaces and start CSG on hardware MCU */
                FwGlobalControl *glbCtrl = gMali.sharedIfaceVa
                    ? reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa) : nullptr;
                FwGlobalInput  *glbIn  = glbCtrl ? reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va))  : nullptr;
                FwGlobalOutput *glbOut = glbCtrl ? reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va)) : nullptr;
                FwCsgControl   *csgCtrl = getCsgCtrl(csgSlot);
                FwCsgInput     *csgIn   = csgCtrl ? reinterpret_cast<FwCsgInput  *>(fwVaToCpu(csgCtrl->input_va))  : nullptr;
                FwCsgOutput    *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
                if (glbIn && glbOut && csgIn && csgOut && gMali.devInfo.mcu_alive && queueMask) {
                    for (uint32_t q = 0; q < g.queue_count; ++q) {
                        if (!(queueMask & (1u << q))) continue;
                        FwCsControl *csCtrl = getCsCtrl(csgSlot, q);
                        FwCsInput   *csIn   = csCtrl ? reinterpret_cast<FwCsInput  *>(fwVaToCpu(csCtrl->input_va))  : nullptr;
                        FwCsOutput  *csOut  = csCtrl ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(csCtrl->output_va)) : nullptr;
                        if (!csIn || !csOut) continue;
                        cacheFlushRange(csOut, sizeof(*csOut));
                        csIn->ringbuf_base   = g.ringGpuVa[q];
                        csIn->ringbuf_size   = g.ringSize[q];
                        csIn->ringbuf_input  = g.ringIoInFwVa[q];
                        csIn->ringbuf_output = g.ringIoOutFwVa[q];
                        uint32_t qPrio = in->queues[q].priority & 0xFu;
                        csIn->config         = qPrio | ((g.dbId[q] & 0xFFu) << 8);
                        csIn->ack_irq_mask   = 0xFFFFFFFFu;
                        csIn->req            = (csOut->ack & ~CS_REQ_MASK) |
                                               CS_IDLE_SYNC_WAIT | CS_IDLE_EMPTY | CS_STATE_START;
                        cacheFlushRange(csIn, sizeof(*csIn));
                    }

                    cacheFlushRange(csgOut, sizeof(*csgOut));
                    csgIn->allow_compute  = in->compute_core_mask  ? in->compute_core_mask  : gMali.devInfo.shader_present;
                    csgIn->allow_fragment = in->fragment_core_mask ? in->fragment_core_mask : gMali.devInfo.shader_present;
                    csgIn->allow_other    = in->tiler_core_mask    ? static_cast<uint32_t>(in->tiler_core_mask)
                                                                   : static_cast<uint32_t>(gMali.devInfo.tiler_present);
                    uint32_t maxC = in->max_compute_cores  ? in->max_compute_cores  : 5u;
                    uint32_t maxF = in->max_fragment_cores ? in->max_fragment_cores : 5u;
                    uint32_t maxT = in->max_tiler_cores    ? in->max_tiler_cores    : 1u;
                    uint32_t prio = (in->priority & 0xFu);
                    csgIn->endpoint_req   = (maxC & 0xFFu) | ((maxF & 0xFFu) << 8) |
                                            ((maxT & 0xFu) << 16) | (prio << 28);
                    csgIn->config         = vm.as_slot;
                    csgIn->suspend_buf    = g.suspendFwVa;
                    csgIn->protm_suspend_buf = 0;
                    csgIn->ack_irq_mask   = 0xFFFFFFFFu;
                    csgIn->doorbell_req   = csgOut->doorbell_ack ^ queueMask;
                    uint32_t epBit        = (csgOut->ack ^ CSG_ENDPOINT_CONFIG) & CSG_ENDPOINT_CONFIG;
                    csgIn->req            = (csgOut->ack & ~CSG_REQ_MASK) | CSG_STATE_START | epBit;
                    cacheFlushRange(csgIn, sizeof(*csgIn));

                    IOLockLock(gMali.lock);
                    cacheFlushRange(glbOut, sizeof(*glbOut));
                    glbIn->doorbell_req = glbOut->doorbell_ack ^ (1u << csgSlot);
                    cacheFlushRange(glbIn, sizeof(*glbIn));
                    wr32(gMali.maliReg.va, CSF_DOORBELL(0), 1u);
                    IOLockUnlock(gMali.lock);

                    for (int ms = 0; ms < 500; ++ms) {
                        cacheFlushRange(csgOut, sizeof(*csgOut));
                        if ((csgOut->ack & CSG_STATE_MASK) == CSG_STATE_START) {
                            g.hw_started = true;
                            break;
                        }
                        IOSleep(1);
                    }
                    FwCsControl *cs0Ctrl = getCsCtrl(csgSlot, 0);
                    FwCsOutput  *cs0Out  = cs0Ctrl ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(cs0Ctrl->output_va)) : nullptr;
                    uint32_t cs0Ack = 0;
                    if (cs0Out) { cacheFlushRange(cs0Out, sizeof(*cs0Out)); cs0Ack = cs0Out->ack; }
                    IOLog("Tb375fcMali: CSG[%u] %s as=%u q=%u db=%u ack=0x%x cs0_ack=0x%x ep=0x%x\n",
                          csgSlot, g.hw_started ? "STARTED" : "START_TIMEOUT",
                          vm.as_slot, g.queue_count, g.dbId[0],
                          csgOut->ack, cs0Ack, csgOut->status_endpoint_current);
                }
                grpHandle = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!grpHandle) {
            IOLockLock(gMali.lock); gMali.csgSlotMask &= ~(1u << csgSlot); IOLockUnlock(gMali.lock);
            return kIOReturnNoResources;
        }
        auto *out = static_cast<Tb375fcMaliGroupCreateOut *>(args->structureOutput);
        out->group_handle = grpHandle;
        out->csg_slot = csgSlot;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_GROUP_DESTROY: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliHandleIn))
            return kIOReturnBadArgument;
        uint32_t h = static_cast<const Tb375fcMaliHandleIn *>(args->structureInput)->handle;
        if (h == 0 || h >= kMaxClientGroups) return kIOReturnBadArgument;
        IOLockLock(fLock);
        if (fGroups[h].in_use) {
            uint32_t csg = fGroups[h].csg_slot;
            if (fGroups[h].hw_started && gMali.sharedIfaceVa) {
                FwGlobalControl *glbCtrl = reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa);
                FwGlobalInput   *glbIn   = reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va));
                FwGlobalOutput  *glbOut  = reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va));
                FwCsgControl    *csgCtrl = getCsgCtrl(csg);
                FwCsgInput      *csgIn   = csgCtrl ? reinterpret_cast<FwCsgInput  *>(fwVaToCpu(csgCtrl->input_va))  : nullptr;
                FwCsgOutput     *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
                if (glbIn && glbOut && csgIn && csgOut) {
                    cacheFlushRange(csgOut, sizeof(*csgOut));
                    csgIn->req = (csgOut->ack & ~CSG_REQ_MASK) | CSG_STATE_TERMINATE;
                    cacheFlushRange(csgIn, sizeof(*csgIn));
                    IOLockLock(gMali.lock);
                    cacheFlushRange(glbOut, sizeof(*glbOut));
                    glbIn->doorbell_req = glbOut->doorbell_ack ^ (1u << csg);
                    cacheFlushRange(glbIn, sizeof(*glbIn));
                    wr32(gMali.maliReg.va, CSF_DOORBELL(0), 1u);
                    IOLockUnlock(gMali.lock);
                    for (int ms = 0; ms < 200; ++ms) {
                        cacheFlushRange(csgOut, sizeof(*csgOut));
                        if ((csgOut->ack & CSG_STATE_MASK) == CSG_STATE_TERMINATE) break;
                        IOSleep(1);
                    }
                }
            }
            for (uint32_t q = 0; q < fGroups[h].queue_count; ++q) {
                if (fGroups[h].ringDesc[q]) { fGroups[h].ringDesc[q]->complete(); fGroups[h].ringDesc[q]->release(); }
            }
            fGroups[h].in_use = false;
            IOLockLock(gMali.lock); gMali.csgSlotMask &= ~(1u << csg); IOLockUnlock(gMali.lock);
        }
        IOLockUnlock(fLock);
        return kIOReturnSuccess;
    }

    case MALI_SEL_GROUP_SUBMIT: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliGroupSubmitIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliGroupSubmitOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliGroupSubmitIn *>(args->structureInput);
        if (in->group_handle == 0 || in->group_handle >= kMaxClientGroups) return kIOReturnBadArgument;
        IOLockLock(fLock);
        MaliGroupEntry &g = fGroups[in->group_handle];
        if (!g.in_use) { IOLockUnlock(fLock); return kIOReturnNotFound; }

        uint32_t scnt = in->submit_count;
        if (scnt > MALI_MAX_SUBMITS_PER_CALL) scnt = MALI_MAX_SUBMITS_PER_CALL;

        uint64_t seq = 0;
        if (g.hw_started && scnt > 0) {
            /* Emit 16-instruction (128-byte, 2-cacheline) CSF ring packet per submit:
             * FLUSH_CACHE2 -> CALL(stream_addr, stream_size) -> WAIT(all) -> FLUSH_CACHE2 -> ERROR_BARRIER */
            uint64_t addrReg = (gMali.devInfo.cs_reg_count >= 4u) ? (gMali.devInfo.cs_reg_count - 4u) : 92u;
            uint64_t valReg  = addrReg + 2u;
            uint64_t waitAll = (gMali.devInfo.scoreboard_slot_count > 0 && gMali.devInfo.scoreboard_slot_count <= 16u)
                               ? ((1ULL << gMali.devInfo.scoreboard_slot_count) - 1ULL) : 0xFFULL;
            for (uint32_t s = 0; s < scnt; ++s) {
                const Tb375fcMaliQueueSubmitDesc &sub = in->submits[s];
                uint32_t q = sub.queue_index;
                if (q >= g.queue_count) q = 0;
                if (!g.ringDesc[q] || !g.ringIoInCpu[q]) continue;

                uint64_t pkt[16];
                memset(pkt, 0, sizeof(pkt));
                uint32_t n = 0;
                if (sub.stream_size > 0 && sub.stream_addr != 0) {
                    /* 0: MOV32 valReg, latest_flush */
                    pkt[n++] = (2ULL << 56) | (valReg << 48) | static_cast<uint64_t>(sub.latest_flush);
                    /* 1: FLUSH_CACHE2.clean_inv_all.no_wait.signal(0) valReg */
                    pkt[n++] = (36ULL << 56) | (0ULL << 48) | (valReg << 40) | (0ULL << 16) | 0x233ULL;
                    /* 2: MOV48 addrReg:addrReg+1, stream_addr */
                    pkt[n++] = (1ULL << 56) | (addrReg << 48) | (sub.stream_addr & 0x0000FFFFFFFFFFFFULL);
                    /* 3: MOV32 valReg, stream_size */
                    pkt[n++] = (2ULL << 56) | (valReg << 48) | static_cast<uint64_t>(sub.stream_size);
                    /* 4: WAIT(0) */
                    pkt[n++] = (3ULL << 56) | (1ULL << 16);
                    /* 5: CALL addrReg:addrReg+1, valReg */
                    pkt[n++] = (32ULL << 56) | (addrReg << 40) | (valReg << 32);
                    /* 6: WAIT(all scoreboards) */
                    pkt[n++] = (3ULL << 56) | (waitAll << 16);
                    /* 7: MOV32 valReg, 0 */
                    pkt[n++] = (2ULL << 56) | (valReg << 48);
                    /* 8: FLUSH_CACHE2.clean_inv_all.no_wait.signal(0) valReg (flush GPU writes to DRAM) */
                    pkt[n++] = (36ULL << 56) | (0ULL << 48) | (valReg << 40) | (0ULL << 16) | 0x233ULL;
                    /* 9: WAIT(0) */
                    pkt[n++] = (3ULL << 56) | (1ULL << 16);
                    /* 10: ERROR_BARRIER */
                    pkt[n++] = (47ULL << 56);
                } else {
                    pkt[n++] = (3ULL << 56) | (waitAll << 16);
                    pkt[n++] = (47ULL << 56);
                }
                uint32_t ringOff = static_cast<uint32_t>(g.ringInsert[q] % g.ringSize[q]);
                uint8_t *ringCpu = static_cast<uint8_t *>(g.ringDesc[q]->getBytesNoCopy());
                memcpy(ringCpu + ringOff, pkt, sizeof(pkt));
                cacheFlushRange(ringCpu + ringOff, sizeof(pkt));

                g.ringInsert[q] += sizeof(pkt);
                g.ringIoInCpu[q]->insert = g.ringInsert[q];
                cacheFlushRange(g.ringIoInCpu[q], 64);

                /* Ring per-queue hardware doorbell! */
                wr32(gMali.maliReg.va, CSF_DOORBELL(g.dbId[q]), 1u);
                seq = (static_cast<uint64_t>(q) << 56) | (g.ringInsert[q] & 0x00FFFFFFFFFFFFFFULL);
            }
        } else {
            g.last_seq++;
            seq = g.last_seq;
        }
        IOLockUnlock(fLock);
        auto *out = static_cast<Tb375fcMaliGroupSubmitOut *>(args->structureOutput);
        out->fence_seq = seq;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_FENCE_WAIT: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliFenceWaitIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliFenceWaitOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliFenceWaitIn *>(args->structureInput);
        auto *out = static_cast<Tb375fcMaliFenceWaitOut *>(args->structureOutput);
        out->pad = 0;

        if (in->group_handle == 0 || in->group_handle >= kMaxClientGroups) {
            out->signaled = 1u;
            out->current_seq = in->fence_seq;
            args->structureOutputSize = sizeof(*out);
            return kIOReturnSuccess;
        }

        IOLockLock(fLock);
        MaliGroupEntry &g = fGroups[in->group_handle];
        if (!g.in_use || !g.hw_started) {
            IOLockUnlock(fLock);
            out->signaled = 1u;
            out->current_seq = in->fence_seq;
            args->structureOutputSize = sizeof(*out);
            return kIOReturnSuccess;
        }
        uint32_t q = static_cast<uint32_t>(in->fence_seq >> 56);
        if (q >= g.queue_count) q = 0;
        uint32_t qCount = g.queue_count;
        uint64_t targets[MALI_MAX_QUEUES_PER_GROUP] = {};
        FwRingIoOutput *ioOuts[MALI_MAX_QUEUES_PER_GROUP] = {};
        for (uint32_t qi = 0; qi < qCount; ++qi) {
            targets[qi] = g.ringInsert[qi];
            ioOuts[qi]  = g.ringIoOutCpu[qi];
        }
        uint64_t targetExtract = in->fence_seq & 0x00FFFFFFFFFFFFFFULL;
        if (targetExtract > targets[q]) targets[q] = targetExtract;
        uint32_t csgSlot = g.csg_slot;
        uint32_t vmId = g.vm_id;
        uint32_t asSlot = (vmId < kMaxClientVms && fVms[vmId].in_use) ? fVms[vmId].as_slot : 0;
        IOLockUnlock(fLock);

        uint64_t curExtract = 0;
        bool done = false;
        uint32_t maxMs = static_cast<uint32_t>(in->timeout_ns / 1000000ULL);
        if (maxMs == 0) maxMs = 1u;
        if (maxMs > 2000u) maxMs = 2000u;

        auto allQueuesDone = [&]() -> bool {
            bool ok = true;
            for (uint32_t qi = 0; qi < qCount; ++qi) {
                if (!ioOuts[qi] || targets[qi] == 0) continue;
                cacheFlushRange(ioOuts[qi], 64);
                uint64_t ext = ioOuts[qi]->extract;
                if (qi == q) curExtract = ext;
                if (ext < targets[qi]) ok = false;
            }
            return ok;
        };

        /* Fast spin for first 2ms (50us steps), then 1ms sleep */
        for (int spin = 0; spin < 40; ++spin) {
            if (allQueuesDone()) { done = true; break; }
            IODelay(50);
        }
        for (uint32_t ms = 2; !done && ms < maxMs; ++ms) {
            if (allQueuesDone()) { done = true; break; }
            IOSleep(1);
        }

        if (done) {
            static uint32_t sOkLogCount = 0;
            if (sOkLogCount < 16u) {
                sOkLogCount++;
                uint32_t flt[3] = {}, ftl[3] = {}, csgEp = 0;
                uint64_t fltI[3] = {}, ftlI[3] = {};
                FwCsgControl *csgCtrl = getCsgCtrl(csgSlot);
                FwCsgOutput  *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
                if (csgOut) { cacheFlushRange(csgOut, sizeof(*csgOut)); csgEp = csgOut->status_endpoint_current; }
                for (uint32_t qi = 0; qi < qCount && qi < 3u; ++qi) {
                    FwCsControl *cc = getCsCtrl(csgSlot, qi);
                    FwCsOutput  *co = cc ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(cc->output_va)) : nullptr;
                    if (co) {
                        cacheFlushRange(co, sizeof(*co));
                        flt[qi] = co->fault; ftl[qi] = co->fatal;
                        fltI[qi] = co->fault_info; ftlI[qi] = co->fatal_info;
                    }
                }
                uint32_t gpuFlt = rd32(gMali.maliReg.va, 0x003Cu); /* GPU_FAULTSTATUS */
                uint32_t asFlt  = asSlot ? rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTSTATUS)) : 0u;
                IOLog("Tb375fcMali: HW FENCE OK #%u csg=%u q=%u ext=%llu/%llu ep=0x%x gpuFlt=0x%x asFlt=0x%x "
                      "q0(flt=0x%x/0x%llx ftl=0x%x) q1(flt=0x%x ftl=0x%x) q2(flt=0x%x ftl=0x%x)\n",
                      sOkLogCount, csgSlot, q,
                      (unsigned long long)curExtract, (unsigned long long)targetExtract,
                      csgEp, gpuFlt, asFlt,
                      flt[0], (unsigned long long)fltI[0], ftl[0],
                      flt[1], ftl[1], flt[2], ftl[2]);
            }
        } else {
            /* Request CSG_STATUS_UPDATE so status_cmd_ptr / status_blocked_reason / status_wait are fresh! */
            FwGlobalControl *glbCtrl = gMali.sharedIfaceVa ? reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa) : nullptr;
            FwGlobalInput   *glbIn   = glbCtrl ? reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va))  : nullptr;
            FwGlobalOutput  *glbOut  = glbCtrl ? reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va)) : nullptr;
            FwCsgControl    *csgCtrl = getCsgCtrl(csgSlot);
            FwCsgInput      *csgIn   = csgCtrl ? reinterpret_cast<FwCsgInput  *>(fwVaToCpu(csgCtrl->input_va))  : nullptr;
            FwCsgOutput     *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
            uint32_t csgEpCur = 0, csgEpReq = 0, csgState = 0;
            if (glbIn && glbOut && csgIn && csgOut) {
                cacheFlushRange(csgOut, sizeof(*csgOut));
                uint32_t updBit = (csgOut->ack ^ (1u << 5)) & (1u << 5);
                csgIn->req = (csgIn->req & ~(1u << 5)) | updBit;
                cacheFlushRange(csgIn, sizeof(*csgIn));
                IOLockLock(gMali.lock);
                cacheFlushRange(glbOut, sizeof(*glbOut));
                glbIn->doorbell_req = glbOut->doorbell_ack ^ (1u << csgSlot);
                cacheFlushRange(glbIn, sizeof(*glbIn));
                wr32(gMali.maliReg.va, CSF_DOORBELL(0), 1u);
                IOLockUnlock(gMali.lock);
                IOSleep(20);
                cacheFlushRange(csgOut, sizeof(*csgOut));
                csgEpCur = csgOut->status_endpoint_current;
                csgEpReq = csgOut->status_endpoint_req;
                csgState = csgOut->status_state;
            }
            uint32_t shRdy = rd32(gMali.maliReg.va, 0x0140u);
            uint32_t tlRdy = rd32(gMali.maliReg.va, 0x0150u);
            uint32_t l2Rdy = rd32(gMali.maliReg.va, L2_READY_LO);
            uint32_t mmuIrq = rd32(gMali.maliReg.va, MMU_INT_RAWSTAT);
            uint32_t asFlt  = asSlot ? rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTSTATUS)) : 0;
            uint64_t asAddr = asSlot ? (static_cast<uint64_t>(rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTADDR_LO))) |
                                       (static_cast<uint64_t>(rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTADDR_HI))) << 32)) : 0;
            IOLog("Tb375fcMali: FENCE TIMEOUT csg=%u q=%u shRdy=0x%x tlRdy=0x%x l2Rdy=0x%x ep=0x%x/0x%x st=0x%x mmu=0x%x as%u_flt=0x%x@0x%llx\n",
                  csgSlot, q, shRdy, tlRdy, l2Rdy, csgEpCur, csgEpReq, csgState,
                  mmuIrq, asSlot, asFlt, (unsigned long long)asAddr);
            for (uint32_t qi = 0; qi < qCount; ++qi) {
                FwCsControl *cc = getCsCtrl(csgSlot, qi);
                FwCsOutput  *co = cc ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(cc->output_va)) : nullptr;
                uint64_t qe = 0;
                if (ioOuts[qi]) { cacheFlushRange(ioOuts[qi], 64); qe = ioOuts[qi]->extract; }
                if (co) {
                    cacheFlushRange(co, sizeof(*co));
                    IOLog("Tb375fcMali:   q[%u] ext=%llu/%llu ack=0x%x cmd=0x%llx blk=%u wait=0x%x sb=0x%x flt=0x%x(0x%llx) ftl=0x%x(0x%llx)\n",
                          qi, (unsigned long long)qe, (unsigned long long)targets[qi],
                          co->ack, (unsigned long long)co->status_cmd_ptr, co->status_blocked_reason,
                          co->status_wait, co->status_scoreboards,
                          co->fault, (unsigned long long)co->fault_info,
                          co->fatal, (unsigned long long)co->fatal_info);
                    if ((co->fatal != 0 || co->fault != 0) && co->status_cmd_ptr >= 16ULL && vmId < kMaxClientVms) {
                        IOLockLock(fLock);
                        MaliVm &dvm = fVms[vmId];
                        uint64_t baseVa = co->status_cmd_ptr & ~4095ULL;
                        uint64_t pgPa = 0;
                        if (dvm.in_use && dvm.l0Va) {
                            uint64_t *tbl = dvm.l0Va;
                            uint32_t idxs[4] = {
                                static_cast<uint32_t>((baseVa >> 39) & 0x1FFu),
                                static_cast<uint32_t>((baseVa >> 30) & 0x1FFu),
                                static_cast<uint32_t>((baseVa >> 21) & 0x1FFu),
                                static_cast<uint32_t>((baseVa >> 12) & 0x1FFu),
                            };
                            for (int lvl = 0; lvl < 3 && tbl; ++lvl) {
                                uint64_t e = tbl[idxs[lvl]];
                                tbl = ((e & 0x3ULL) == 0x3ULL) ? vmPaToVa(&dvm, e) : nullptr;
                            }
                            if (tbl && (tbl[idxs[3]] & 0x3ULL) == 0x3ULL) {
                                pgPa = tbl[idxs[3]] & 0x0000FFFFFFFFF000ULL;
                            }
                        }
                        IOLockUnlock(fLock);
                        if (pgPa) {
                            IOMemoryDescriptor *pd = IOMemoryDescriptor::withPhysicalAddress(pgPa, 4096, kIODirectionInOut);
                            IOMemoryMap *pm = pd ? pd->createMappingInTask(kernel_task, 0, kIOMapAnywhere) : nullptr;
                            if (pm) {
                                const uint8_t *pva = reinterpret_cast<const uint8_t *>(pm->getVirtualAddress());
                                cacheFlushRange(pva, 4096);
                                uint32_t off = static_cast<uint32_t>(co->status_cmd_ptr & 4095ULL) & ~7u;
                                uint32_t sOff = (off >= 16u) ? (off - 16u) : 0u;
                                uint64_t w[6] = {};
                                for (uint32_t wi = 0; wi < 6u && (sOff + wi * 8u) < 4096u; ++wi) {
                                    memcpy(&w[wi], pva + sOff + wi * 8u, 8);
                                }
                                IOLog("Tb375fcMali:   q[%u] insns@0x%llx: %016llx %016llx [%016llx] %016llx %016llx %016llx\n",
                                      qi, (unsigned long long)(baseVa + sOff),
                                      (unsigned long long)w[0], (unsigned long long)w[1],
                                      (unsigned long long)w[2], (unsigned long long)w[3],
                                      (unsigned long long)w[4], (unsigned long long)w[5]);
                                uint64_t p0[8] = {};
                                for (uint32_t wi = 0; wi < 8u; ++wi) memcpy(&p0[wi], pva + wi * 8u, 8);
                                IOLog("Tb375fcMali:   q[%u] page0@0x%llx: %016llx %016llx %016llx %016llx %016llx %016llx %016llx %016llx\n",
                                      qi, (unsigned long long)baseVa,
                                      (unsigned long long)p0[0], (unsigned long long)p0[1],
                                      (unsigned long long)p0[2], (unsigned long long)p0[3],
                                      (unsigned long long)p0[4], (unsigned long long)p0[5],
                                      (unsigned long long)p0[6], (unsigned long long)p0[7]);
                                uint64_t p1[8] = {};
                                for (uint32_t wi = 0; wi < 8u; ++wi) memcpy(&p1[wi], pva + 64u + wi * 8u, 8);
                                IOLog("Tb375fcMali:   q[%u] page1@0x%llx: %016llx %016llx %016llx %016llx %016llx %016llx %016llx %016llx\n",
                                      qi, (unsigned long long)(baseVa + 64u),
                                      (unsigned long long)p1[0], (unsigned long long)p1[1],
                                      (unsigned long long)p1[2], (unsigned long long)p1[3],
                                      (unsigned long long)p1[4], (unsigned long long)p1[5],
                                      (unsigned long long)p1[6], (unsigned long long)p1[7]);
                                pm->release();
                            }
                            if (pd) pd->release();
                        }
                    }
                }
            }
            if (asSlot && (mmuIrq & (1u << asSlot))) {
                wr32(gMali.maliReg.va, MMU_INT_CLEAR, (1u << asSlot));
                wr32(gMali.maliReg.va, AS_REG(asSlot, AS_COMMAND), 3u); /* AS_CMD_UNLOCK */
            }
        }

        out->signaled = done ? 1u : 0u;
        out->current_seq = (static_cast<uint64_t>(q) << 56) | curExtract;
        args->structureOutputSize = sizeof(*out);
        return done ? kIOReturnSuccess : kIOReturnTimeout;
    }

    case MALI_SEL_GROUP_GET_STATE: {
        if (!args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliGroupStateOut))
            return kIOReturnBadArgument;
        auto *out = static_cast<Tb375fcMaliGroupStateOut *>(args->structureOutput);
        out->state = 0;
        out->fatal_queues = 0;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    default:
        return kIOReturnUnsupported;
    }
}

static void maliDriverWorker(void *, wait_result_t) {
    IOLog("Tb375fcMali: driver worker starting (mali=4 IOUserClient bring-up)\n");
    gMali.lock = IOLockAlloc();
    if (initMaliHardware()) {
        Tb375fcMali *svc = new Tb375fcMali;
        if (svc && svc->init()) {
            svc->attach(IOService::getPlatform());
            svc->start(IOService::getPlatform());
            IOLog("Tb375fcMali: IOService published for Mesa panvk!\n");
        }
    }
    (void)thread_terminate(current_thread());
}

extern "C" void tb375fc_mali_probe(void);
extern "C" void tb375fc_mali_probe(void) {
    static bool started = false;
    if (started) return;
    started = true;
    uint32_t stage = 0;
    if (!PE_parse_boot_argn("mali", &stage, sizeof(stage)) || stage < 4u) return;
    thread_t th = THREAD_NULL;
    if (kernel_thread_start(maliDriverWorker, nullptr, &th) == KERN_SUCCESS && th != THREAD_NULL) {
        thread_deallocate(th);
    }
}
