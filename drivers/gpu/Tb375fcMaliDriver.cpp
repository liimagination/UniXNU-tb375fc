/* SPDX-License-Identifier: MIT
 * TB375FC / MT6897 Mali-G615 (Arch 11.8 Valhall CSF v10) Kernel Driver + IOUserClient.
 *
 * Boot-arg gated:
 *   mali=4 : Bring up GPUEB + Mali-G615 + L2 + AS0 + mali_csffw.bin v2.5 MCU + GLB_PING,
 *            and publish IOService "Tb375fcMali" with panthor-compatible IOUserClient
 *            for multi-process in-process Mesa panvk (libvulkan_panfrost.dylib) + m2v!
 *
 * Multi-process scaling features:
 *   - Sets "NativeSandboxGrantAll"=true so all sandboxed macOS Metal clients can open Tb375fcMali.
 *   - On-demand 32-page (128 KiB) page-table chunks per MaliVm (instead of 16 MiB upfront).
 *   - Reserves kmem_io WriteCombine budget (24 MiB) for WindowServer; other clients + tiler heaps
 *     use zero-kmem_io pageable fTask descriptors.
 *   - Decouples software VMs/Groups from the 7 user hardware AS/CSG slots (AS1..AS7, CSG0..CSG6):
 *     WindowServer pins Slot 0 (AS1+CSG0) permanently; other processes share Slots 1..6 via clean
 *     drain+terminate LRU multiplexing, with monotonic software fence sequences.
 *   - Supports MALI_SEL_DEBUG_LOG (selector 14) for sandbox-safe userspace telemetry over CDC.
 */
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOUserClient.h>
#include <kern/clock.h>
#include <kern/sched_prim.h>
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
void proc_selfname(char *buf, int size);
}

#ifndef UIO_READ
#define UIO_READ 0
#endif
#ifndef UIO_SYSSPACE
#define UIO_SYSSPACE 2
#endif

class Tb375fcMaliUserClient;

namespace {

static int sMaliShortSleepEvent = 0;

/* Non-coalesced microsecond wait: avoids XNU IOSleep(1) 1.5-4 ms timer coalescing slack
 * while yielding the CPU core cleanly instead of spinning 1.5 ms in IODelay(). */
static inline void maliShortSleepUs(uint32_t us) {
    assert_wait_timeout_with_leeway(reinterpret_cast<event_t>(&sMaliShortSleepEvent),
                                    THREAD_UNINT, TIMEOUT_URGENCY_SYS_CRITICAL,
                                    us, 0, kMicrosecondScale);
    thread_block(THREAD_CONTINUE_NULL);
}

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
constexpr uint32_t CMD_INIT_SHARED_MEM      = 0u;
constexpr uint32_t CMD_GET_FREQ_BY_IDX      = 1u;
constexpr uint32_t CMD_POWER_CONTROL        = 6u;
constexpr uint32_t CMD_ACTIVE_SLEEP_CONTROL = 7u;
constexpr uint32_t CMD_COMMIT               = 8u;

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
constexpr uint32_t AS_CMD_UNLOCK    = 3u;
constexpr uint32_t AS_CMD_FLUSH_PT  = 4u;
constexpr uint32_t AS_CMD_FLUSH_MEM = 5u;

/* CSF Doorbells: DB 0 = Global (0x80000), DB n = 0x80000 + n * 0x10000 */
static inline uint32_t CSF_DOORBELL(uint32_t db) { return 0x80000u + db * 0x10000u; }
/* Valhall CSF User MMIO page: monotonic hardware flush ID register */
constexpr uint32_t CSF_GPU_LATEST_FLUSH_ID = 0x10000u;

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
static void unmapMmio(MappedPage *p) {
    if (p->map)  { p->map->release();  p->map  = nullptr; }
    if (p->desc) { p->desc->release(); p->desc = nullptr; }
    p->va = 0;
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
    if (!addr || !len) return;
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
constexpr uint32_t CSG_STATE_SUSPEND          = 2u;
constexpr uint32_t CSG_STATE_RESUME           = 3u;
constexpr uint32_t CSG_ENDPOINT_CONFIG        = (1u << 4);
constexpr uint32_t CSG_REQ_MASK               = CSG_STATE_MASK | CSG_ENDPOINT_CONFIG | (1u << 5);

constexpr uint32_t CS_STATE_MASK              = 0x7u;
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

/* ---- 7 User Hardware Slots: Slot k -> AS(k+1) + CSG(k) + DB(k+1) ---- */
constexpr uint32_t kMaxHwSlots      = 7u; /* Slot 0 pinned to WindowServer; Slots 1..6 shared LRU */
constexpr uint32_t kAs0TotalPages   = 1024u; /* 4 MiB CSF arena */
constexpr uint32_t kAs0MaxDynPages  = 896u;  /* 0x04080000..0x04400000 = 3.5 MiB */

struct MaliHwSlot {
    Tb375fcMaliUserClient *ownerClient;
    uint32_t ownerGroupIdx;
    uint32_t ownerVmIdx;
    bool     pinnedWs;
    uint64_t lastUseTick;
    uint32_t fallbackSuspFwVa;
};

/* ---- Global Mali-G615 hardware state (owned by Tb375fcMali service) ---- */
struct MaliHwState {
    IOLock *lock;      /* Protects AS0 arena & global doorbell 0 */
    IOLock *schedLock; /* Protects hw slots[1..6], AS2..7, CSG1..6 (ALWAYS acquired BEFORE client->fLock) */
    IOLock *wsLock;    /* Dedicated lightweight lock for pinned Slot 0 (WindowServer) so non-WS evictions NEVER stall WS/display */
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
    uint32_t as0DynBasePage;
    uint32_t as0DynBaseVa;
    uint32_t as0DynCount;
    uint8_t  as0DynUsed[kAs0MaxDynPages];
    uint64_t useClock;
    uint32_t evictCount;
    uint32_t resumeCount;
    uint64_t sharedKmemBytes; /* Global kmem_io WC budget across all clients (capped at 512 MiB of 1985 MiB DATA_SHARED) */
    IOBufferMemoryDescriptor *faultScratchDesc;
    uint64_t faultScratchPa;
    uint32_t mmuFaultCount;
    Tb375fcMaliUserClient *wsClient;
    uint32_t wsGroupIdx;
    MaliHwSlot slots[kMaxHwSlots];
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

static void mapAs0Page(uint32_t va, uint64_t pa, bool nc) {
    uint32_t l2Idx = (va >> 21) & 0x1FFu;
    uint32_t l3Idx = (va >> 12) & 0x1FFu;
    uint64_t *l3   = reinterpret_cast<uint64_t *>(arenaPageVa(3u + l2Idx));
    uint64_t pte   = (pa & 0x0000FFFFFFFFF000ULL) | 0x3ULL | (1ULL << 10) | (1ULL << 6) | (1ULL << 54);
    if (nc) pte |= (0ULL << 2) | (2ULL << 8);
    else    pte |= (1ULL << 2);
    l3[l3Idx] = pte;
    cacheFlushRange(&l3[l3Idx], sizeof(uint64_t));
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

static void unmapAs0Page(uint32_t va) {
    uint32_t l2Idx = (va >> 21) & 0x1FFu;
    uint32_t l3Idx = (va >> 12) & 0x1FFu;
    if (l2Idx > 33u) return;
    uint64_t *l3   = reinterpret_cast<uint64_t *>(arenaPageVa(3u + l2Idx));
    l3[l3Idx] = 0ULL;
    cacheFlushRange(&l3[l3Idx], sizeof(uint64_t));
}

static bool allocAs0Pages(uint32_t numPages, bool nc, uint32_t *fwVaOut, uint8_t **cpuVaOut,
                          uint32_t *dynIdxOut = nullptr) {
    if (numPages == 0 || numPages > gMali.as0DynCount) return false;
    IOLockLock(gMali.lock);
    uint32_t found = UINT32_MAX;
    uint32_t run = 0;
    for (uint32_t i = 0; i < gMali.as0DynCount; ++i) {
        if (!gMali.as0DynUsed[i]) {
            if (++run == numPages) {
                found = i + 1u - numPages;
                break;
            }
        } else {
            run = 0;
        }
    }
    if (found == UINT32_MAX) {
        IOLockUnlock(gMali.lock);
        return false;
    }
    uint32_t startPg = gMali.as0DynBasePage + found;
    uint32_t startVa = gMali.as0DynBaseVa + found * 4096u;
    for (uint32_t i = 0; i < numPages; ++i) {
        gMali.as0DynUsed[found + i] = 1u;
        uint8_t *dst = arenaPageVa(startPg + i);
        memset(dst, 0, 4096);
        cacheFlushRange(dst, 4096);
        mapAs0Page(startVa + i * 4096u, arenaPagePa(startPg + i), nc);
    }
    flushAs0Mmu();
    IOLockUnlock(gMali.lock);
    if (fwVaOut)   *fwVaOut   = startVa;
    if (cpuVaOut)  *cpuVaOut  = arenaPageVa(startPg);
    if (dynIdxOut) *dynIdxOut = found;
    return true;
}

static void freeAs0Pages(uint32_t dynIdx, uint32_t numPages) {
    if (numPages == 0 || dynIdx >= gMali.as0DynCount || dynIdx + numPages > gMali.as0DynCount) return;
    IOLockLock(gMali.lock);
    for (uint32_t i = 0; i < numPages; ++i) {
        unmapAs0Page(gMali.as0DynBaseVa + (dynIdx + i) * 4096u);
        gMali.as0DynUsed[dynIdx + i] = 0u;
    }
    flushAs0Mmu();
    IOLockUnlock(gMali.lock);
}

static bool ensureSlotAs0Buffers(uint32_t slotIdx, uint32_t /*queueCount*/) {
    MaliHwSlot &slot = gMali.slots[slotIdx];
    if (gMali.csgSuspendSize > 0 && slot.fallbackSuspFwVa == 0) {
        uint32_t suspPages = (gMali.csgSuspendSize + 4095u) / 4096u;
        uint8_t *suspCpu = nullptr;
        if (!allocAs0Pages(suspPages, false, &slot.fallbackSuspFwVa, &suspCpu)) return false;
    }
    return true;
}

/* ---- 4-Level ARM_64_LPAE_S1 Page Table for User VMs (On-Demand 128 KiB Chunks) ---- */
constexpr uint32_t kPtPagesPerChunk = 32u;  /* 128 KiB per chunk */
constexpr uint32_t kMaxPtChunks     = 192u; /* up to 6144 PT pages = 24 MiB max per VM */
constexpr uint32_t kMaxPtPagesPerVm = kPtPagesPerChunk * kMaxPtChunks;

struct MaliVm {
    bool     in_use;
    uint32_t id;
    uint32_t as_slot_mask; /* bit (as) set while programmed into hardware AS(as) */
    IOBufferMemoryDescriptor *ptChunks[kMaxPtChunks];
    uint8_t *ptChunkVas[kMaxPtChunks];
    uint32_t ptChunksAlloc;
    uint64_t *ptPagePas;
    uint32_t ptPagesUsed;
    uint32_t faultRecoveries;
    uint64_t l0Pa;
    uint64_t *l0Va;
    uint64_t nextInternalVa;
};

static inline uint8_t *vmPageCpuVa(MaliVm *vm, uint32_t pageIdx) {
    uint32_t c = pageIdx / kPtPagesPerChunk;
    uint32_t p = pageIdx % kPtPagesPerChunk;
    if (c >= kMaxPtChunks || !vm->ptChunkVas[c]) return nullptr;
    return vm->ptChunkVas[c] + static_cast<size_t>(p) * 4096ULL;
}

static uint32_t vmAllocPtPage(MaliVm *vm, uint64_t *paOut, uint64_t **vaOut) {
    if (vm->ptPagesUsed >= kMaxPtPagesPerVm) return UINT32_MAX;
    uint32_t idx = vm->ptPagesUsed;
    uint32_t c = idx / kPtPagesPerChunk;
    uint32_t p = idx % kPtPagesPerChunk;
    if (!vm->ptChunks[c]) {
        /* Page tables are kernel+GPU only (never userspace-mapped): omit kIOMemoryKernelUserShared
         * so PT chunks do not consume XNU's small 128 MiB KMEM_RANGE_ID_DATA_SHARED pool! */
        IOBufferMemoryDescriptor *desc = IOBufferMemoryDescriptor::withOptions(
            kIODirectionInOut,
            static_cast<size_t>(kPtPagesPerChunk) * 4096ULL, 4096);
        if (!desc || desc->prepare() != kIOReturnSuccess) {
            if (desc) desc->release();
            return UINT32_MAX;
        }
        vm->ptChunks[c] = desc;
        vm->ptChunkVas[c] = static_cast<uint8_t *>(desc->getBytesNoCopy());
        if (c + 1u > vm->ptChunksAlloc) vm->ptChunksAlloc = c + 1u;
    }
    vm->ptPagesUsed++;
    uint8_t *va = vm->ptChunkVas[c] + static_cast<size_t>(p) * 4096ULL;
    memset(va, 0, 4096);
    IOByteCount segLen = 0;
    uint64_t pa = static_cast<uint64_t>(
        vm->ptChunks[c]->getPhysicalSegment(static_cast<IOByteCount>(p) * 4096ULL, &segLen));
    if (vm->ptPagePas) vm->ptPagePas[idx] = pa & 0x0000FFFFFFFFF000ULL;
    cacheFlushRange(va, 4096);
    *paOut = pa;
    *vaOut = reinterpret_cast<uint64_t *>(va);
    return idx;
}

static void vmFreeStorage(MaliVm *vm) {
    if (vm->ptPagePas) {
        IOFreeData(vm->ptPagePas, sizeof(uint64_t) * kMaxPtPagesPerVm);
        vm->ptPagePas = nullptr;
    }
    for (uint32_t c = 0; c < kMaxPtChunks; ++c) {
        if (vm->ptChunks[c]) {
            vm->ptChunks[c]->complete();
            vm->ptChunks[c]->release();
            vm->ptChunks[c] = nullptr;
            vm->ptChunkVas[c] = nullptr;
        }
    }
    vm->ptChunksAlloc = 0;
    vm->ptPagesUsed = 0;
    vm->l0Pa = 0;
    vm->l0Va = nullptr;
}

static uint64_t *vmPaToVa(MaliVm *vm, uint64_t pa) {
    pa &= 0x0000FFFFFFFFF000ULL;
    if (vm->ptPagePas) {
        for (uint32_t i = 0; i < vm->ptPagesUsed; ++i) {
            if (vm->ptPagePas[i] == pa) {
                return reinterpret_cast<uint64_t *>(vmPageCpuVa(vm, i));
            }
        }
        return nullptr;
    }
    for (uint32_t i = 0; i < vm->ptPagesUsed; ++i) {
        uint32_t c = i / kPtPagesPerChunk;
        uint32_t p = i % kPtPagesPerChunk;
        IOByteCount segLen = 0;
        uint64_t pgPa = static_cast<uint64_t>(
            vm->ptChunks[c]->getPhysicalSegment(static_cast<IOByteCount>(p) * 4096ULL, &segLen));
        if ((pgPa & 0x0000FFFFFFFFF000ULL) == pa) {
            return reinterpret_cast<uint64_t *>(vmPageCpuVa(vm, i));
        }
    }
    return nullptr;
}

static uint64_t *vmEnsureL3Table(MaliVm *vm, uint64_t va, bool create) {
    uint32_t idxs[3] = {
        static_cast<uint32_t>((va >> 39) & 0x1FFu),
        static_cast<uint32_t>((va >> 30) & 0x1FFu),
        static_cast<uint32_t>((va >> 21) & 0x1FFu),
    };
    uint64_t *tbl = vm->l0Va;
    for (int lvl = 0; lvl < 3; ++lvl) {
        uint64_t e = tbl[idxs[lvl]];
        if ((e & 0x3ULL) != 0x3ULL) {
            if (!create) return nullptr;
            uint64_t childPa = 0;
            uint64_t *childVa = nullptr;
            if (vmAllocPtPage(vm, &childPa, &childVa) == UINT32_MAX) return nullptr;
            tbl[idxs[lvl]] = (childPa & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
            cacheFlushRange(&tbl[idxs[lvl]], sizeof(uint64_t));
            tbl = childVa;
        } else {
            tbl = vmPaToVa(vm, e);
            if (!tbl) return nullptr;
        }
    }
    return tbl;
}

static inline uint64_t vmMakeLeafPte(uint64_t pa, uint32_t flags) {
    uint64_t pte = (pa & 0x0000FFFFFFFFF000ULL) | 0x3ULL | (1ULL << 10) | (1ULL << 6);
    if (flags & MALI_VM_MAP_READONLY) pte |= (1ULL << 7);
    if (flags & MALI_VM_MAP_NOEXEC)   pte |= (1ULL << 54);
    if (flags & MALI_VM_MAP_UNCACHED) pte |= (0ULL << 2) | (2ULL << 8);
    else                              pte |= (1ULL << 2) | (3ULL << 8);
    return pte;
}

static bool vmMapPage(MaliVm *vm, uint64_t va, uint64_t pa, uint32_t flags) {
    uint64_t *l3 = vmEnsureL3Table(vm, va, true);
    if (!l3) return false;
    uint32_t i3 = static_cast<uint32_t>((va >> 12) & 0x1FFu);
    l3[i3] = vmMakeLeafPte(pa, flags);
    cacheFlushRange(&l3[i3], sizeof(uint64_t));
    return true;
}

/* Fast batched range mapper: walks L0->L1->L2->L3 once per 2 MiB L3 chunk, consumes full
 * contiguous getPhysicalSegment runs, and issues ONE cacheFlushRange per L3 span (100x+ faster). */
static bool vmMapRange(MaliVm *vm, uint64_t va, IOMemoryDescriptor *desc,
                       uint64_t boOff, uint64_t totalPages, uint32_t flags,
                       uint64_t *failPageOut = nullptr, uint64_t *failPaOut = nullptr) {
    uint64_t p = 0;
    uint64_t segPa = 0;
    uint64_t segRemPages = 0;
    while (p < totalPages) {
        uint64_t curVa = va + p * 4096ULL;
        uint64_t *l3 = vmEnsureL3Table(vm, curVa, true);
        if (!l3) {
            if (failPageOut) *failPageOut = p;
            if (failPaOut)   *failPaOut   = 0;
            return false;
        }
        uint32_t i3Start = static_cast<uint32_t>((curVa >> 12) & 0x1FFu);
        uint64_t chunkPages = 512ULL - i3Start;
        if (chunkPages > totalPages - p) chunkPages = totalPages - p;
        for (uint64_t k = 0; k < chunkPages; ++k) {
            if (segRemPages == 0) {
                IOByteCount segLen = 0;
                segPa = static_cast<uint64_t>(
                    desc->getPhysicalSegment(static_cast<IOByteCount>(boOff + (p + k) * 4096ULL), &segLen));
                if (!segPa) {
                    if (k > 0) cacheFlushRange(&l3[i3Start], static_cast<size_t>(k) * sizeof(uint64_t));
                    if (failPageOut) *failPageOut = p + k;
                    if (failPaOut)   *failPaOut   = 0;
                    return false;
                }
                segRemPages = (segLen >= 4096ULL) ? (segLen / 4096ULL) : 1ULL;
            }
            l3[i3Start + k] = vmMakeLeafPte(segPa, flags);
            segPa += 4096ULL;
            segRemPages--;
        }
        cacheFlushRange(&l3[i3Start], static_cast<size_t>(chunkPages) * sizeof(uint64_t));
        p += chunkPages;
    }
    return true;
}

static void vmUnmapRange(MaliVm *vm, uint64_t va, uint64_t totalPages) {
    uint64_t p = 0;
    while (p < totalPages) {
        uint64_t curVa = va + p * 4096ULL;
        uint32_t i3Start = static_cast<uint32_t>((curVa >> 12) & 0x1FFu);
        uint64_t chunkPages = 512ULL - i3Start;
        if (chunkPages > totalPages - p) chunkPages = totalPages - p;
        uint64_t *l3 = vmEnsureL3Table(vm, curVa, false);
        if (l3) {
            memset(&l3[i3Start], 0, static_cast<size_t>(chunkPages) * sizeof(uint64_t));
            cacheFlushRange(&l3[i3Start], static_cast<size_t>(chunkPages) * sizeof(uint64_t));
        }
        p += chunkPages;
    }
}

static void hwFlushAs(uint32_t as, bool flushMem = false) {
    if (as == 0 || as >= 8u) return;
    wr64(gMali.maliReg.va, AS_REG(as, AS_LOCKADDR_LO), 47ULL);
    wr32(gMali.maliReg.va, AS_REG(as, AS_COMMAND), AS_CMD_LOCK);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(as, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
    wr32(gMali.maliReg.va, AS_REG(as, AS_COMMAND), flushMem ? AS_CMD_FLUSH_MEM : AS_CMD_FLUSH_PT);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(as, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
}

static void hwProgramAs(uint32_t as, uint64_t l0Pa) {
    if (as == 0 || as >= 8u) return;
    uint64_t memattr  = 0x4C4C4C4C4C4C8F4CULL;
    uint64_t transcfg = 6ULL | (7ULL << 6) | (1ULL << 24);
    wr64(gMali.maliReg.va, AS_REG(as, AS_TRANSTAB_LO), l0Pa & 0x0000FFFFFFFFF000ULL);
    wr64(gMali.maliReg.va, AS_REG(as, AS_MEMATTR_LO), memattr);
    wr64(gMali.maliReg.va, AS_REG(as, AS_TRANSCFG_LO), transcfg);
    wr32(gMali.maliReg.va, AS_REG(as, AS_COMMAND), AS_CMD_UPDATE);
    for (int i = 0; i < 1000; ++i) {
        if ((rd32(gMali.maliReg.va, AS_REG(as, AS_STATUS)) & 1u) == 0u) break;
        IODelay(5);
    }
    hwFlushAs(as);
}

static void hwDisableAs(uint32_t as) {
    if (as == 0 || as >= 8u) return;
    wr64(gMali.maliReg.va, AS_REG(as, AS_TRANSTAB_LO), 0ULL);
    wr64(gMali.maliReg.va, AS_REG(as, AS_TRANSCFG_LO), 0ULL);
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

    /* GPU & CPU clock OPP boost (MT6897) */
    {
        uint32_t gpuFreq = rd32(gMali.gpuebDram.va, GF_REG_FREQ_CUR_GPU);
        uint32_t gpuOppN = rd32(gMali.gpuebDram.va, GF_REG_GPU_OPP_NUM);
        IOLog("Tb375fcMali: GPUEB initial cur_gpu_freq=%u kHz opp_num=%u\n", gpuFreq, gpuOppN);

        uint32_t boostEn = 1u;
        PE_parse_boot_argn("boost", &boostEn, sizeof(boostEn));
        uint32_t gpuMhzArg = 1400u; /* Stock Dimensity 8300 OPP0 (1400 MHz, +16% fill/shader vs 1206; safe with noafbc + MMU healer); override via gpumhz=1206 */
        PE_parse_boot_argn("gpumhz", &gpuMhzArg, sizeof(gpuMhzArg));
        if (gpuMhzArg < 300u) gpuMhzArg = 300u;
        if (gpuMhzArg > 1400u) gpuMhzArg = 1400u;
        const uint32_t targetGpuKhz = gpuMhzArg * 1000u;

        if (boostEn && gpuOppN > 0 && gpuOppN <= 128) {
            uint32_t bestIdx = gpuOppN / 2u;
            uint32_t bestDiff = 0xFFFFFFFFu;
            uint32_t bestFreq = 0u;
            uint32_t f0 = 0u, fMid = 0u, fEnd = 0u;
            for (uint32_t idx = 0; idx < gpuOppN; ++idx) {
                memset(&tx, 0, sizeof(tx));
                memset(&rx, 0, sizeof(rx));
                tx.magic = ipiMagic;
                tx.cmd   = CMD_GET_FREQ_BY_IDX;
                tx.target = 0u;
                tx.u.raw[0] = idx;
                if (gpuebSendGfIpi(mboxVa, gMali.gpuebMbox.va, tx, &rx) == 0) {
                    uint32_t fk = static_cast<uint32_t>(rx.u.return_value);
                    if (idx == 0) f0 = fk;
                    if (idx == gpuOppN / 2u) fMid = fk;
                    if (idx == gpuOppN - 1u) fEnd = fk;
                    if (fk >= 200000u && fk <= 1500000u) {
                        uint32_t diff = (fk > targetGpuKhz) ? (fk - targetGpuKhz) : (targetGpuKhz - fk);
                        if (diff < bestDiff) { bestDiff = diff; bestIdx = idx; bestFreq = fk; }
                    }
                }
            }
            IOLog("Tb375fcMali: GPUEB OPP table[0]=%u mid=%u end=%u -> picking idx=%u (%u kHz)\n",
                  f0, fMid, fEnd, bestIdx, bestFreq);
            if (bestFreq > 0u) {
                memset(&tx, 0, sizeof(tx));
                memset(&rx, 0, sizeof(rx));
                tx.magic = ipiMagic;
                tx.cmd   = CMD_ACTIVE_SLEEP_CONTROL;
                tx.u.power_state = 1u;
                int arc = gpuebSendGfIpi(mboxVa, gMali.gpuebMbox.va, tx, &rx);
                IOLog("Tb375fcMali: GPUEB ACTIVE_SLEEP(1) rc=%d ret=%d\n", arc, rx.u.return_value);

                for (uint32_t tgt = 0u; tgt <= 1u; ++tgt) {
                    memset(&tx, 0, sizeof(tx));
                    memset(&rx, 0, sizeof(rx));
                    tx.magic = ipiMagic;
                    tx.cmd   = CMD_COMMIT;
                    tx.target = tgt;
                    tx.u.raw[0] = bestIdx;
                    int crc = gpuebSendGfIpi(mboxVa, gMali.gpuebMbox.va, tx, &rx);
                    IOSleep(2);
                    uint32_t newGpuFreq = rd32(gMali.gpuebDram.va, GF_REG_FREQ_CUR_GPU);
                    IOLog("Tb375fcMali: GPUEB CMD_COMMIT(tgt=%u idx=%u) rc=%d ret=%d -> cur_gpu_freq=%u kHz opp=%u\n",
                          tgt, bestIdx, crc, rx.u.return_value, newGpuFreq, rd32(gMali.gpuebDram.va, 0x04u));
                    if (newGpuFreq >= 200000u) break;
                }
            }
        }

        MappedPage pllMap = {}, csramMap = {}, mcuCfgMap = {};
        auto logPll = [&](const char *tag) {
            if (!pllMap.va) return;
            static const uint32_t offs[4] = {0x00cu, 0x40cu, 0x80cu, 0xc0cu};
            uint32_t raw[4] = {}, mhz[4] = {};
            for (int i = 0; i < 4; ++i) {
                raw[i] = rd32(pllMap.va, offs[i]);
                uint32_t pcw = raw[i] & 0x3fffffu;
                uint32_t posdiv = (raw[i] >> 24) & 0x7u;
                mhz[i] = (uint32_t)((52ULL * pcw) >> (14u + posdiv)); /* MT6897 ARMPLL 52 MHz effective fin */
            }
            uint32_t div[4] = {};
            if (mcuCfgMap.va) {
                div[0] = rd32(mcuCfgMap.va, 0x100u);
                div[1] = rd32(mcuCfgMap.va, 0x10cu);
                div[2] = rd32(mcuCfgMap.va, 0x118u);
                div[3] = rd32(mcuCfgMap.va, 0x124u);
            }
            IOLog("Tb375fcMali: CPU PLL (%s) MHz: cci=%u cl0=%u cl1=%u cl2=%u | raw=0x%x/0x%x/0x%x/0x%x div=0x%x/0x%x/0x%x/0x%x\n",
                  tag, mhz[0], mhz[1], mhz[2], mhz[3],
                  raw[0], raw[1], raw[2], raw[3],
                  div[0], div[1], div[2], div[3]);
        };
        mapMmio(0x0c030000ULL, 0x1000u, &pllMap);
        mapMmio(0x0c000000ULL, 0x1000u, &mcuCfgMap);
        logPll("before");

        if (mapMmio(0x0011b000ULL, 0x2000u, &csramMap)) {
            uint32_t freqScaleFlag = rd32(csramMap.va, 0xc00u + 0x4ccu); /* REG_FREQ_SCALING @ csram+0x4cc */
            uint32_t cciModeBefore = rd32(csramMap.va, 0xc00u + 0xf9cu); /* OFFS_CCI_TBL_MODE @ csram+0xf9c */
            uint32_t cpuBoostArg = 1u;
            PE_parse_boot_argn("cpuboost", &cpuBoostArg, sizeof(cpuBoostArg));
            static const uint32_t domOff[3] = {0xc10u, 0xd30u, 0xe50u};
            /* Full MT6897 Dimensity 8300 max OPPs (L=2200, M=3200, B=3350 MHz) when cpuboost=1 */
            const uint32_t targetMhz[3] = {
                cpuBoostArg ? 2200u : 2000u,
                cpuBoostArg ? 3200u : 2800u,
                cpuBoostArg ? 3350u : 3000u,
            };
            if (boostEn && cpuBoostArg) {
                /* Set CCI/DSU performance table mode = 1 (70% ratio instead of Normal 50%) */
                wr32(csramMap.va, 0xc00u + 0xf9cu, 1u);
            }
            IOLog("Tb375fcMali: CPUHW freqScale=0x%x cciMode=%u->%u\n",
                  freqScaleFlag, cciModeBefore, rd32(csramMap.va, 0xc00u + 0xf9cu));
            for (int d = 0; d < 3; ++d) {
                uintptr_t base = csramMap.va + domOff[d];
                uint32_t en = rd32(base, 0x84u);
                uint32_t hwSt = rd32(base, 0x8cu);
                uint32_t l0 = rd32(base, 0x00u), l1 = rd32(base, 0x04u), l2 = rd32(base, 0x08u);
                if (boostEn) {
                    uint32_t bestIdx = 0xFFFFFFFFu, bestDiff = 0xFFFFFFFFu, bestVal = 0u, prev = 0xFFFFFFFFu;
                    for (uint32_t i = 0; i < 32u; ++i) {
                        uint32_t v = rd32(base, i * 4u);
                        if (v == 0u || v == prev) continue;
                        prev = v;
                        uint32_t vmhz = (v & 0xFFFu);
                        if (v >= 400000u && v <= 4000000u && (v % 1000u) == 0u) vmhz = v / 1000u;
                        if (vmhz >= 400u && vmhz <= 3600u) {
                            uint32_t diff = (vmhz > targetMhz[d]) ? (vmhz - targetMhz[d]) : (targetMhz[d] - vmhz);
                            if (diff < bestDiff) { bestDiff = diff; bestIdx = i; bestVal = vmhz; }
                        }
                    }
                    if (bestIdx != 0xFFFFFFFFu) {
                        if (!en) wr32(base, 0x84u, 1u);
                        uint32_t writeVal = freqScaleFlag ? (bestVal * 1000u) : bestIdx;
                        wr32(base, 0x88u, writeVal);
                        IOLog("Tb375fcMali: CPUHW dom%d -> set %s=%u (%u MHz) lut0..2=%u/%u/%u hwSt=0x%x\n",
                              d, freqScaleFlag ? "kHz" : "idx", writeVal, bestVal,
                              l0 & 0xFFFu, l1 & 0xFFFu, l2 & 0xFFFu, hwSt);
                    }
                }
            }
            IOSleep(10);
            unmapMmio(&csramMap);
        }
        logPll("after");
        if (mcuCfgMap.va) unmapMmio(&mcuCfgMap);
        unmapMmio(&pllMap);

        /* MT6897 DVFSRC (0x1C00F000): Preloader/LK boots LPDDR5X at 3094 Mbps with DVFSRC disabled (bctl=0x8).
         * Call official MT6897 ATF SMC MTK_SIP_VCOREFS_CONTROL(0xC2000506) / MTK_SIP_DVFSRC_INIT(0)
         * to initialize DVFSRC and unlock full LPDDR5X bandwidth (up to 8533 Mbps / 40.8 GB/s)! */
        if (boostEn) {
            auto readDramcMbps = [&]() -> uint32_t {
                uint32_t vco = 0;
                MappedPage phyAo = {}, phyNao = {};
                if (mapMmio(0x10238000ULL, 0x2000u, &phyAo) && mapMmio(0x10236000ULL, 0x2000u, &phyNao)) {
                    uint32_t rE98   = rd32(phyAo.va, 0xE98u);
                    uint32_t shuLv  = (rE98 & 0xC000u) >> 14;
                    uint32_t pllId  = (rE98 & 0x02000000u) ? 1u : 0u;
                    uint32_t shuOff = 0x700u * (shuLv & 1u);
                    uint32_t sdmOff = (pllId ? 0x928u : 0x908u) + shuOff;
                    uint32_t divOff = (pllId ? 0x92Cu : 0x90Cu) + shuOff;
                    uint32_t ck4Off = 0x153Cu + shuOff;
                    uint32_t fbkOff = 0x910u + shuOff;
                    uint32_t dqsOff = 0xD94u + shuOff;
                    uint32_t caOff  = 0x148Cu + shuOff;
                    if (ck4Off < 0x2000u && caOff < 0x2000u && sdmOff < 0x2000u) {
                        uint32_t sdmpcw = (rd32(phyNao.va, sdmOff) & 0x7FFF8u) >> 3;
                        uint32_t posdiv = (rd32(phyNao.va, divOff) & 0x3800u) >> 11;
                        uint32_t ckdiv4 = (rd32(phyAo.va,  ck4Off) & 0x4u) >> 2;
                        uint32_t fbksel = (rd32(phyNao.va, fbkOff) & 0x40u) >> 6;
                        uint32_t dqsop  = (rd32(phyNao.va, dqsOff) & 0x4000000u) >> 26;
                        uint32_t caDiv4 = (rd32(phyAo.va,  caOff)  & 0x4u) >> 2;
                        vco = ((26u * (sdmpcw >> 7)) >> posdiv >> 1 >> ckdiv4) << fbksel;
                        if (dqsop == 1u) vco >>= (caDiv4 ? 2u : 1u);
                    }
                }
                if (phyAo.va)  unmapMmio(&phyAo);
                if (phyNao.va) unmapMmio(&phyNao);
                return vco;
            };
            auto mtkSmc4 = [](uint64_t a0, uint64_t a1, uint64_t a2, uint64_t a3,
                              uint64_t *o0, uint64_t *o1) {
                register uint64_t x0 __asm__("x0") = a0;
                register uint64_t x1 __asm__("x1") = a1;
                register uint64_t x2 __asm__("x2") = a2;
                register uint64_t x3 __asm__("x3") = a3;
                register uint64_t x4 __asm__("x4") = 0;
                register uint64_t x5 __asm__("x5") = 0;
                register uint64_t x6 __asm__("x6") = 0;
                register uint64_t x7 __asm__("x7") = 0;
                __asm__ volatile("smc #0"
                    : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3), "+r"(x4), "+r"(x5), "+r"(x6), "+r"(x7)
                    : : "memory");
                if (o0) *o0 = x0;
                if (o1) *o1 = x1;
            };

            uint32_t dramBefore = readDramcMbps();
            uint32_t noDramBoost = 0;
            PE_parse_boot_argn("nodram", &noDramBoost, sizeof(noDramBoost));
            MappedPage dvfsrcMap = {};
            if (mapMmio(0x1C00F000ULL, 0x1000u, &dvfsrcMap)) {
                uint32_t bctl0 = rd32(dvfsrcMap.va, 0x000u);
                uint32_t sw0   = rd32(dvfsrcMap.va, 0x018u);
                uint32_t lvl0  = rd32(dvfsrcMap.va, 0x5F0u);
                uint64_t smcRc = 0xFFFFULL, dramType = 0;
                if (!noDramBoost && !(bctl0 & 1u)) {
                    /* Official MT6897 mtk_dvfsrc_probe() ATF init: locks high OPP! */
                    mtkSmc4(0xC2000506ULL, 0ULL /* MTK_SIP_DVFSRC_INIT */, 0ULL, 0ULL, &smcRc, &dramType);
                    IOSleep(3);
                }
                uint32_t bctl1 = rd32(dvfsrcMap.va, 0x000u);
                if (!noDramBoost && (bctl1 & 1u)) {
                    /* Vote max DRAM OPP9 (8533 Mbps) + max VCORE OPP5 + peak BW */
                    uint32_t swCur = rd32(dvfsrcMap.va, 0x018u);
                    uint32_t swNew = (swCur & ~0x0000F070u) | (9u << 12) | (5u << 4);
                    wr32(dvfsrcMap.va, 0x018u, swNew);
                    wr32(dvfsrcMap.va, 0x1E8u, 0x3FFu); /* DVFSRC_SW_BW */
                    wr32(dvfsrcMap.va, 0x1F4u, 0x3FFu); /* DVFSRC_SW_PEAK_BW */
                    IOSleep(3);
                }
                uint32_t dramAfterInit = readDramcMbps();
                /* If INIT kept a lower boot lock, release lock via MTK_SIP_DVFSRC_START(1) with SW_REQ=OPP9 already voted! */
                uint64_t startRc = 0xFFFFULL;
                if (!noDramBoost && (bctl1 & 1u) && dramAfterInit > 0u && dramAfterInit < 5500u) {
                    mtkSmc4(0xC2000506ULL, 1ULL /* MTK_SIP_DVFSRC_START */, 0ULL, 0ULL, &startRc, nullptr);
                    IOSleep(3);
                }
                uint32_t dramFinal = readDramcMbps();
                uint32_t lvl1 = rd32(dvfsrcMap.va, 0x5F0u);
                IOLog("Tb375fcMali: DVFSRC initRc=0x%llx type=%llu startRc=0x%llx bctl=0x%x->0x%x swReq=0x%x->0x%x lvl=0x%x->0x%x | DRAMC %u -> %u -> %u Mbps\n",
                      (unsigned long long)smcRc, (unsigned long long)dramType, (unsigned long long)startRc,
                      bctl0, bctl1, sw0, rd32(dvfsrcMap.va, 0x018u), lvl0, lvl1,
                      dramBefore, dramAfterInit, dramFinal);
                unmapMmio(&dvfsrcMap);
            }
            MappedPage mfgTopMap = {};
            if (mapMmio(0x13FBF000ULL, 0x1000u, &mfgTopMap)) {
                uint32_t c1 = rd32(mfgTopMap.va, 0x704u);
                uint32_t c2 = rd32(mfgTopMap.va, 0x708u);
                uint32_t c3 = rd32(mfgTopMap.va, 0x70Cu);
                if (c1 != 0x14u || c2 != 0x4u || c3 != 0x00400400u) {
                    wr32(mfgTopMap.va, 0x704u, 0x00000014u);
                    wr32(mfgTopMap.va, 0x708u, 0x00000004u);
                    wr32(mfgTopMap.va, 0x70Cu, 0x00400400u);
                }
                IOLog("Tb375fcMali: MFG_TOP AXUSER cfg=0x%x/0x%x/0x%x -> 0x%x/0x%x/0x%x\n",
                      c1, c2, c3, rd32(mfgTopMap.va, 0x704u), rd32(mfgTopMap.va, 0x708u), rd32(mfgTopMap.va, 0x70Cu));
                unmapMmio(&mfgTopMap);
            }
        }
    }

    if (!mapMmio(MT6897_MALI_PHYS, MT6897_MALI_MAP_SIZE, &gMali.maliReg)) {
        IOLog("Tb375fcMali: failed to map mali@13000000\n");
        return false;
    }

    uintptr_t mv = gMali.maliReg.va;
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
    di.selected_coherency        = 0u;
    di.shader_present            = static_cast<uint64_t>(rd32(mv, GPU_SHADER_PRESENT_LO)) |
                                   (static_cast<uint64_t>(rd32(mv, GPU_SHADER_PRESENT_HI)) << 32);
    di.tiler_present             = static_cast<uint64_t>(rd32(mv, GPU_TILER_PRESENT_LO)) |
                                   (static_cast<uint64_t>(rd32(mv, GPU_TILER_PRESENT_HI)) << 32);
    di.l2_present                = static_cast<uint64_t>(rd32(mv, GPU_L2_PRESENT_LO)) |
                                   (static_cast<uint64_t>(rd32(mv, GPU_L2_PRESENT_HI)) << 32);
    di.timestamp_frequency       = 24000000ULL;

    size_t fwLen = 0;
    uint8_t *fw = loadMaliFwFromVfs(&fwLen);
    if (!fw) {
        IOLog("Tb375fcMali: mali_csffw.bin missing; HW regs ready but MCU offline\n");
        gMali.ready = true;
        return true;
    }

    constexpr uint32_t kTotalPages = kAs0TotalPages; /* 4 MiB CSF arena */
    /* CSF firmware arena is kernel+MCU only (never userspace-mapped): keep out of 128 MiB DATA_SHARED */
    gMali.csfArenaDesc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIODirectionInOut,
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
    for (uint32_t l2i = 0; l2i <= 33u; ++l2i) {
        l2[l2i] = (arenaPagePa(3u + l2i) & 0x0000FFFFFFFFF000ULL) | 0x3ULL;
    }
    uint32_t nextPg = 37u;
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
    gMali.as0DynBasePage = nextPg;
    gMali.as0DynBaseVa   = 0x04080000u;
    uint32_t availPages  = (kTotalPages > nextPg) ? (kTotalPages - nextPg) : 0u;
    gMali.as0DynCount    = (availPages < kAs0MaxDynPages) ? availPages : kAs0MaxDynPages;
    memset(gMali.as0DynUsed, 0, sizeof(gMali.as0DynUsed));
    cacheFlushRange(gMali.csfArenaVa, static_cast<size_t>(kTotalPages) * 4096ULL);

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

        FwGlobalInput  *glbIn  = reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va));
        FwGlobalOutput *glbOut = reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va));
        if (glbIn && glbOut && di.mcu_alive) {
            cacheFlushRange(glbOut, sizeof(*glbOut));
            uint32_t ack0 = glbOut->ack;
            glbIn->core_en_mask   = di.shader_present;
            glbIn->poweroff_timer = 0x00100000u;
            glbIn->progress_timer = 0x00040000u;
            glbIn->idle_timer     = 0x00010000u; /* short idle timer so idle CSGs yield endpoints */
            glbIn->ack_irq_mask   = 0xFFFFFFFFu;
            uint32_t toggle = GLB_CFG_ALLOC_EN | GLB_CFG_POWEROFF_TIMER | GLB_CFG_PROGRESS_TIMER;
            uint32_t newReq = (ack0 ^ toggle) | GLB_COUNTER_EN | GLB_IDLE_EN;
            glbIn->req = newReq;
            cacheFlushRange(glbIn, sizeof(*glbIn));
            wr32(mv, CSF_DOORBELL(0), 1u);
            bool cfgAcked = false;
            for (int ms = 0; ms < 500; ++ms) {
                cacheFlushRange(glbOut, sizeof(*glbOut));
                if (((glbOut->ack ^ newReq) & toggle) == 0u) { cfgAcked = true; break; }
                IOSleep(1);
            }
            IOLog("Tb375fcMali: GLB_CFG_ALLOC_EN %s (req=0x%x ack=0x%x suspend=%u cs=%u regs=%u sb=%u as0Dyn=%u)\n",
                  cfgAcked ? "ACK" : "TIMEOUT", newReq, glbOut->ack,
                  gMali.csgSuspendSize, di.cs_slot_count, di.cs_reg_count, di.scoreboard_slot_count,
                  gMali.as0DynCount);
        }
    }

    /* Allocate a single zeroed 4 KiB physical scratch page for MMU page-fault self-healing */
    gMali.faultScratchDesc = IOBufferMemoryDescriptor::withOptions(kIODirectionInOut, 4096, 4096);
    if (gMali.faultScratchDesc && gMali.faultScratchDesc->prepare() == kIOReturnSuccess) {
        memset(gMali.faultScratchDesc->getBytesNoCopy(), 0, 4096);
        cacheFlushRange(gMali.faultScratchDesc->getBytesNoCopy(), 4096);
        IOByteCount segLen = 0;
        gMali.faultScratchPa = static_cast<uint64_t>(gMali.faultScratchDesc->getPhysicalSegment(0, &segLen));
    }

    gMali.ready = true;
    IOLog("Tb375fcMali: READY (multi-proc) gpu_id=0x%08x shader=0x%llx coh=0x%x fw=0x%08x csg=%u mcu=%u\n",
          di.gpu_id, (unsigned long long)di.shader_present, di.coherency_features,
          di.fw_iface_ver, di.csg_slot_count, di.mcu_alive);
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
constexpr uint32_t kMaxClientVms    = 16u;
constexpr uint32_t kMaxClientGroups = 16u;
constexpr uint32_t kSeqHistSize     = 64u;

struct MaliBoEntry {
    bool                in_use;
    bool                is_shared;
    IOMemoryDescriptor *desc;
    uint64_t            size;
};

struct MaliSeqTarget {
    uint64_t seq;
    uint64_t insert[MALI_MAX_QUEUES_PER_GROUP];
};

struct MaliGroupEntry {
    bool     in_use;
    bool     hw_started;
    bool     hw_suspended; /* true after CSG_STATE_SUSPEND; next bind uses CSG_STATE_RESUME */
    int32_t  hw_slot;      /* -1 when not resident, 0..6 when resident on AS(slot+1)+CSG(slot) */
    uint32_t vm_id;
    uint32_t queue_count;
    uint32_t priority;
    uint32_t max_compute_cores;
    uint32_t max_fragment_cores;
    uint32_t max_tiler_cores;
    uint64_t compute_core_mask;
    uint64_t fragment_core_mask;
    uint64_t tiler_core_mask;
    uint8_t  queuePrio[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t suspendFwVa;
    uint32_t suspendDynIdx;
    uint32_t suspendPages;
    uint32_t ringIoDynIdx;
    uint32_t ringIoPages;
    uint32_t ringIoInFwVa[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t ringIoOutFwVa[MALI_MAX_QUEUES_PER_GROUP];
    FwRingIoInput  *ringIoInCpu[MALI_MAX_QUEUES_PER_GROUP];
    FwRingIoOutput *ringIoOutCpu[MALI_MAX_QUEUES_PER_GROUP];
    uint64_t submitted_seq;
    uint64_t completed_seq; /* accessed atomically for lock-free eviction check in FENCE_WAIT */
    MaliSeqTarget seqHist[kSeqHistSize];
    IOBufferMemoryDescriptor *ringDesc[MALI_MAX_QUEUES_PER_GROUP];
    uint64_t ringGpuVa[MALI_MAX_QUEUES_PER_GROUP];
    uint32_t ringSize[MALI_MAX_QUEUES_PER_GROUP];
    uint64_t ringInsert[MALI_MAX_QUEUES_PER_GROUP];
    uint64_t ringExtract[MALI_MAX_QUEUES_PER_GROUP];
};

class Tb375fcMaliUserClient : public IOUserClient {
    OSDeclareDefaultStructors(Tb375fcMaliUserClient)
public:
    task_t          fTask;
    IOLock         *fLock;
    MaliBoEntry    *fBos;
    MaliVm         *fVms;
    MaliGroupEntry *fGroups;
    uint64_t        fSharedBytes;
    char            fProcName[17];
    bool            fIsWindowServer;

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

namespace {

/* Must be called with gMali.schedLock HELD and NO client->fLock held!
 * If permanent == false (LRU eviction), uses CSG_STATE_SUSPEND into the group's private AS0
 * suspend_buf so all 96 CSF registers (including PanVK's cs_subqueue_ctx_reg) and ring pointers
 * are preserved across CSG_STATE_RESUME!
 * If permanent == true (group destroy / client free / hardware fault), uses CSG_STATE_TERMINATE. */
static void evictHwSlotLocked(uint32_t slotIdx, bool permanent) {
    if (slotIdx >= kMaxHwSlots) return;
    MaliHwSlot &slot = gMali.slots[slotIdx];
    Tb375fcMaliUserClient *owner = slot.ownerClient;
    if (!owner) return;

    uint32_t grpIdx = slot.ownerGroupIdx;
    uint32_t vmIdx  = slot.ownerVmIdx;
    uint32_t csgSlot = slotIdx;
    uint32_t asSlot  = slotIdx + 1u;

    uint32_t qCount = 0;
    uint32_t grpSuspFwVa = 0;
    uint64_t targets[MALI_MAX_QUEUES_PER_GROUP] = {};
    FwRingIoOutput *ioOuts[MALI_MAX_QUEUES_PER_GROUP] = {};
    IOLockLock(owner->fLock);
    if (owner->fGroups && grpIdx > 0 && grpIdx < kMaxClientGroups && owner->fGroups[grpIdx].in_use) {
        MaliGroupEntry &g = owner->fGroups[grpIdx];
        qCount = g.queue_count;
        grpSuspFwVa = g.suspendFwVa;
        for (uint32_t q = 0; q < qCount && q < MALI_MAX_QUEUES_PER_GROUP; ++q) {
            targets[q] = g.ringInsert[q];
            ioOuts[q]  = g.ringIoOutCpu[q];
        }
    }
    IOLockUnlock(owner->fLock);

    /* 1. Brief drain on victim queues before SUSPEND/TERMINATE:
     * Mali CSF natively suspends in-flight CSGs via CSG_STATE_SUSPEND and resumes them cleanly on
     * CSG_STATE_RESUME, so LRU eviction (permanent==false) only waits 20ms (not 1500ms!) to avoid
     * stalling other apps waiting on gMali.schedLock. */
    bool allDone = true;
    if (qCount > 0 && gMali.devInfo.mcu_alive) {
        allDone = false;
        const int drainMaxMs = permanent ? 200 : 20;
        for (int ms = 0; ms < drainMaxMs; ++ms) {
            bool done = true;
            for (uint32_t q = 0; q < qCount; ++q) {
                if (!ioOuts[q] || targets[q] == 0) continue;
                cacheFlushRange(ioOuts[q], 64);
                if (ioOuts[q]->extract < targets[q]) {
                    done = false;
                    break;
                }
            }
            if (done) { allDone = true; break; }
            if (ms < 5) IODelay(100);
            else        IOSleep(1);
        }
    }

    /* Check if any queue on this slot hit a hardware CS fault/fatal; if so, force TERMINATE */
    if (!permanent && gMali.sharedIfaceVa && gMali.devInfo.mcu_alive) {
        for (uint32_t q = 0; q < qCount; ++q) {
            FwCsControl *cc = getCsCtrl(csgSlot, q);
            FwCsOutput  *co = cc ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(cc->output_va)) : nullptr;
            if (co) {
                cacheFlushRange(co, sizeof(*co));
                if (co->fault != 0u || co->fatal != 0u) {
                    permanent = true;
                    break;
                }
            }
        }
    }

    /* 2. Suspend (LRU eviction) or Terminate (permanent / faulted) CSG on hardware MCU */
    bool didSuspend = false;
    uint64_t savedExtract[MALI_MAX_QUEUES_PER_GROUP] = {};
    if (gMali.sharedIfaceVa && gMali.devInfo.mcu_alive) {
        FwGlobalControl *glbCtrl = reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa);
        FwGlobalInput   *glbIn   = reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va));
        FwGlobalOutput  *glbOut  = reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va));
        FwCsgControl    *csgCtrl = getCsgCtrl(csgSlot);
        FwCsgInput      *csgIn   = csgCtrl ? reinterpret_cast<FwCsgInput  *>(fwVaToCpu(csgCtrl->input_va))  : nullptr;
        FwCsgOutput     *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
        if (glbIn && glbOut && csgIn && csgOut) {
            if (!permanent && grpSuspFwVa != 0u) {
                cacheFlushRange(csgOut, sizeof(*csgOut));
                csgIn->suspend_buf = grpSuspFwVa;
                csgIn->cs_irq_ack  = csgOut->cs_irq_req;
                csgIn->req = (csgOut->ack & ~CSG_REQ_MASK) | CSG_STATE_SUSPEND;
                cacheFlushRange(csgIn, sizeof(*csgIn));
                IOLockLock(gMali.lock);
                cacheFlushRange(glbOut, sizeof(*glbOut));
                glbIn->doorbell_req = glbOut->doorbell_ack ^ (1u << csgSlot);
                cacheFlushRange(glbIn, sizeof(*glbIn));
                wr32(gMali.maliReg.va, CSF_DOORBELL(0), 1u);
                IOLockUnlock(gMali.lock);
                for (int ms = 0; ms < 250; ++ms) {
                    cacheFlushRange(csgOut, sizeof(*csgOut));
                    if ((csgOut->ack & CSG_STATE_MASK) == CSG_STATE_SUSPEND) {
                        didSuspend = true;
                        break;
                    }
                    if (ms < 5) IODelay(100);
                    else        IOSleep(1);
                }
                if (didSuspend) {
                    for (uint32_t q = 0; q < qCount && q < MALI_MAX_QUEUES_PER_GROUP; ++q) {
                        if (ioOuts[q]) {
                            cacheFlushRange(ioOuts[q], 64);
                            savedExtract[q] = ioOuts[q]->extract;
                        }
                    }
                }
            }
            if (!didSuspend) {
                cacheFlushRange(csgOut, sizeof(*csgOut));
                csgIn->cs_irq_ack = csgOut->cs_irq_req;
                csgIn->req = (csgOut->ack & ~CSG_REQ_MASK) | CSG_STATE_TERMINATE;
                cacheFlushRange(csgIn, sizeof(*csgIn));
                IOLockLock(gMali.lock);
                cacheFlushRange(glbOut, sizeof(*glbOut));
                glbIn->doorbell_req = glbOut->doorbell_ack ^ (1u << csgSlot);
                cacheFlushRange(glbIn, sizeof(*glbIn));
                wr32(gMali.maliReg.va, CSF_DOORBELL(0), 1u);
                IOLockUnlock(gMali.lock);
                for (int ms = 0; ms < 200; ++ms) {
                    cacheFlushRange(csgOut, sizeof(*csgOut));
                    if ((csgOut->ack & CSG_STATE_MASK) == CSG_STATE_TERMINATE) break;
                    if (ms < 5) IODelay(100);
                    else        IOSleep(1);
                }
            }
        }
    }

    /* 3. Disable hardware AS so MMU holds no stale page-table reference */
    hwDisableAs(asSlot);

    /* 4. Update victim software state */
    IOLockLock(owner->fLock);
    if (owner->fGroups && grpIdx > 0 && grpIdx < kMaxClientGroups && owner->fGroups[grpIdx].in_use) {
        MaliGroupEntry &g = owner->fGroups[grpIdx];
        if (allDone || permanent || !didSuspend) {
            __atomic_store_n(&g.completed_seq, g.submitted_seq, __ATOMIC_RELEASE);
        }
        g.hw_started   = false;
        g.hw_suspended = didSuspend;
        __atomic_store_n(&g.hw_slot, -1, __ATOMIC_RELEASE);
        if (didSuspend) {
            for (uint32_t q = 0; q < qCount && q < MALI_MAX_QUEUES_PER_GROUP; ++q) {
                g.ringExtract[q] = savedExtract[q];
            }
        } else {
            for (uint32_t q = 0; q < MALI_MAX_QUEUES_PER_GROUP; ++q) {
                g.ringInsert[q]  = 0;
                g.ringExtract[q] = 0;
            }
        }
    }
    if (owner->fVms && vmIdx > 0 && vmIdx < kMaxClientVms && owner->fVms[vmIdx].in_use) {
        owner->fVms[vmIdx].as_slot_mask &= ~(1u << asSlot);
    }
    IOLockUnlock(owner->fLock);

    if (slotIdx == 0u && gMali.wsLock) {
        IOLockLock(gMali.wsLock);
        if (gMali.wsClient == owner && gMali.wsGroupIdx == grpIdx) {
            gMali.wsClient   = nullptr;
            gMali.wsGroupIdx = 0u;
        }
        IOLockUnlock(gMali.wsLock);
    }
    slot.ownerClient   = nullptr;
    slot.ownerGroupIdx = 0;
    slot.ownerVmIdx    = 0;
    slot.pinnedWs      = false;
    gMali.evictCount++;
}

/* Helper to check if a slot's queues have all drained (called with schedLock held, no fLock held). */
static bool isHwSlotIdleLocked(uint32_t slotIdx) {
    MaliHwSlot &slot = gMali.slots[slotIdx];
    Tb375fcMaliUserClient *owner = slot.ownerClient;
    if (!owner) return true;
    uint32_t grpIdx = slot.ownerGroupIdx;
    uint32_t qCount = 0;
    uint64_t ins[MALI_MAX_QUEUES_PER_GROUP] = {};
    FwRingIoOutput *ioOuts[MALI_MAX_QUEUES_PER_GROUP] = {};
    IOLockLock(owner->fLock);
    if (owner->fGroups && grpIdx > 0 && grpIdx < kMaxClientGroups && owner->fGroups[grpIdx].in_use) {
        qCount = owner->fGroups[grpIdx].queue_count;
        for (uint32_t q = 0; q < qCount && q < MALI_MAX_QUEUES_PER_GROUP; ++q) {
            ins[q]    = owner->fGroups[grpIdx].ringInsert[q];
            ioOuts[q] = owner->fGroups[grpIdx].ringIoOutCpu[q];
        }
    }
    IOLockUnlock(owner->fLock);
    for (uint32_t q = 0; q < qCount; ++q) {
        if (!ioOuts[q] || ins[q] == 0) continue;
        cacheFlushRange(ioOuts[q], 64);
        if (ioOuts[q]->extract < ins[q]) return false;
    }
    return true;
}

/* Bind & start/resume (client, grpIdx) on a hardware AS+CSG slot.
 * Must be called with gMali.schedLock HELD and NO client->fLock held!
 * Returns slot index 0..6 on success, or -1 on failure. */
static int32_t ensureGroupResidentLocked(Tb375fcMaliUserClient *client, uint32_t grpIdx) {
    IOLockLock(client->fLock);
    if (!client->fGroups || grpIdx == 0 || grpIdx >= kMaxClientGroups || !client->fGroups[grpIdx].in_use) {
        IOLockUnlock(client->fLock);
        return -1;
    }
    int32_t curSlot = client->fGroups[grpIdx].hw_slot;
    uint32_t vmId   = client->fGroups[grpIdx].vm_id;
    uint32_t qCount = client->fGroups[grpIdx].queue_count;
    if (curSlot >= 0 && curSlot < (int32_t)kMaxHwSlots &&
        gMali.slots[curSlot].ownerClient == client &&
        gMali.slots[curSlot].ownerGroupIdx == grpIdx) {
        gMali.slots[curSlot].lastUseTick = ++gMali.useClock;
        IOLockUnlock(client->fLock);
        return curSlot;
    }
    if (!client->fVms || vmId == 0 || vmId >= kMaxClientVms || !client->fVms[vmId].in_use) {
        IOLockUnlock(client->fLock);
        return -1;
    }
    IOLockUnlock(client->fLock);

    /* Choose target slot: WindowServer pins Slot 0; all other groups share Slots 1..6 */
    uint32_t chosen = UINT32_MAX;
    uint32_t maxSlots = kMaxHwSlots;
    if (gMali.csgCount > 0 && gMali.csgCount < maxSlots) maxSlots = gMali.csgCount;

    if (client->fIsWindowServer && gMali.slots[0].ownerClient == nullptr) {
        chosen = 0;
    } else {
        uint32_t startSlot = client->fIsWindowServer ? 0u : 1u;
        for (uint32_t s = startSlot; s < maxSlots; ++s) {
            if ((gMali.devInfo.as_present & (1u << (s + 1u))) && gMali.slots[s].ownerClient == nullptr) {
                chosen = s;
                break;
            }
        }
        if (chosen == UINT32_MAX) {
            /* Prefer oldest ALREADY-IDLE non-WS slot; fall back to oldest busy slot */
            uint64_t bestIdleTick = UINT64_MAX;
            uint32_t bestIdleSlot = UINT32_MAX;
            uint64_t bestAnyTick  = UINT64_MAX;
            uint32_t bestAnySlot  = UINT32_MAX;
            for (uint32_t s = 1u; s < maxSlots; ++s) {
                if (!(gMali.devInfo.as_present & (1u << (s + 1u)))) continue;
                if (gMali.slots[s].pinnedWs) continue;
                uint64_t t = gMali.slots[s].lastUseTick;
                if (t < bestAnyTick) { bestAnyTick = t; bestAnySlot = s; }
                if (isHwSlotIdleLocked(s) && t < bestIdleTick) { bestIdleTick = t; bestIdleSlot = s; }
            }
            chosen = (bestIdleSlot != UINT32_MAX) ? bestIdleSlot : bestAnySlot;
            if (chosen != UINT32_MAX) {
                evictHwSlotLocked(chosen, false);
            }
        }
    }
    if (chosen == UINT32_MAX) return -1;

    if (!ensureSlotAs0Buffers(chosen, qCount)) {
        IOLog("Tb375fcMali: AS0 buffer alloc failed for slot %u (%s)\n", chosen, client->fProcName);
        return -1;
    }

    MaliHwSlot &slot = gMali.slots[chosen];
    uint32_t csgSlot = chosen;
    uint32_t asSlot  = chosen + 1u;

    /* Snapshot group & VM fields under client->fLock; preserve ring/pointers & per-group AS0 I/O pages on RESUME */
    uint64_t l0Pa = 0;
    uint32_t queueMask = 0;
    uint64_t ringGpuVa[MALI_MAX_QUEUES_PER_GROUP] = {};
    uint32_t ringSize[MALI_MAX_QUEUES_PER_GROUP]  = {};
    uint32_t ringIoInFwVa[MALI_MAX_QUEUES_PER_GROUP]  = {};
    uint32_t ringIoOutFwVa[MALI_MAX_QUEUES_PER_GROUP] = {};
    uint8_t  qPrio[MALI_MAX_QUEUES_PER_GROUP]     = {};
    uint64_t allowC = 0, allowF = 0, allowT = 0;
    uint32_t maxC = 5u, maxF = 5u, maxT = 1u, grpPrio = 1u;
    uint32_t grpSuspFwVa = 0;
    bool     resuming = false;
    bool     hasPendingOnResume = false;

    IOLockLock(client->fLock);
    MaliGroupEntry &g = client->fGroups[grpIdx];
    MaliVm &vm = client->fVms[vmId];
    if (!g.in_use || !vm.in_use) {
        IOLockUnlock(client->fLock);
        return -1;
    }
    l0Pa        = vm.l0Pa;
    allowC      = g.compute_core_mask  ? g.compute_core_mask  : gMali.devInfo.shader_present;
    allowF      = g.fragment_core_mask ? g.fragment_core_mask : gMali.devInfo.shader_present;
    allowT      = g.tiler_core_mask    ? g.tiler_core_mask    : gMali.devInfo.tiler_present;
    maxC        = g.max_compute_cores  ? g.max_compute_cores  : 5u;
    maxF        = g.max_fragment_cores ? g.max_fragment_cores : 5u;
    if (maxC > 5u) maxC = 5u;
    if (maxF > 5u) maxF = 5u;
    maxT        = 1u; /* Mali-G615 has exactly 1 tiler; never ask MCU for 5 tilers */
    /* Give WindowServer CSF priority 2 (overrideable via wsprio=1..3) and cap non-WS Fragment
     * cores to 4/5 (overrideable via bgmaxf=1..5) so dynamic wallpaper / background apps never
     * starve WindowServer compositing or delay ws-display-sync! */
    static uint32_t sWsPrio = 0u, sBgMaxF = 0u;
    if (sWsPrio == 0u) {
        uint32_t p = 2u;
        if (PE_parse_boot_argn("wsprio", &p, sizeof(p)) && p >= 1u && p <= 3u) sWsPrio = p;
        else sWsPrio = 2u;
        uint32_t bf = 4u;
        if (PE_parse_boot_argn("bgmaxf", &bf, sizeof(bf)) && bf >= 1u && bf <= 5u) sBgMaxF = bf;
        else sBgMaxF = 4u;
    }
    if (!client->fIsWindowServer && maxF > sBgMaxF) maxF = sBgMaxF;
    grpPrio     = client->fIsWindowServer ? sWsPrio : 1u;
    grpSuspFwVa = g.suspendFwVa ? g.suspendFwVa : slot.fallbackSuspFwVa;
    resuming    = g.hw_suspended && (g.suspendFwVa != 0u);

    for (uint32_t q = 0; q < g.queue_count; ++q) {
        if (g.ringDesc[q] && g.ringIoInCpu[q] && g.ringIoOutCpu[q]) {
            if (!resuming) {
                g.ringInsert[q]  = 0;
                g.ringExtract[q] = 0;
                memset(g.ringDesc[q]->getBytesNoCopy(), 0, g.ringSize[q]);
                cacheFlushRange(g.ringDesc[q]->getBytesNoCopy(), g.ringSize[q]);
                memset(g.ringIoInCpu[q], 0, 64);
                cacheFlushRange(g.ringIoInCpu[q], 64);
                memset(g.ringIoOutCpu[q], 0, 64);
                cacheFlushRange(g.ringIoOutCpu[q], 64);
            } else {
                /* Keep group's persistent ringIoIn/Out intact across CSG_STATE_RESUME */
                g.ringIoInCpu[q]->insert = g.ringInsert[q];
                cacheFlushRange(g.ringIoInCpu[q], 64);
                cacheFlushRange(g.ringIoOutCpu[q], 64);
                if (g.ringInsert[q] > g.ringIoOutCpu[q]->extract) hasPendingOnResume = true;
            }
            ringGpuVa[q]     = g.ringGpuVa[q];
            ringSize[q]      = g.ringSize[q];
            ringIoInFwVa[q]  = g.ringIoInFwVa[q];
            ringIoOutFwVa[q] = g.ringIoOutFwVa[q];
            qPrio[q]         = g.queuePrio[q] & 0xFu;
            queueMask |= (1u << q);
        }
    }
    vm.as_slot_mask |= (1u << asSlot);
    IOLockUnlock(client->fLock);

    /* Program hardware AS(asSlot) */
    hwProgramAs(asSlot, l0Pa);

    /* Program CS & CSG shared interfaces and START or RESUME CSG on hardware MCU */
    bool started = false;
    uint32_t targetState = resuming ? CSG_STATE_RESUME : CSG_STATE_START;
    FwGlobalControl *glbCtrl = gMali.sharedIfaceVa
        ? reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa) : nullptr;
    FwGlobalInput  *glbIn  = glbCtrl ? reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va))  : nullptr;
    FwGlobalOutput *glbOut = glbCtrl ? reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va)) : nullptr;
    FwCsgControl   *csgCtrl = getCsgCtrl(csgSlot);
    FwCsgInput     *csgIn   = csgCtrl ? reinterpret_cast<FwCsgInput  *>(fwVaToCpu(csgCtrl->input_va))  : nullptr;
    FwCsgOutput    *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
    uint32_t dbId = csgSlot + 1u;

    if (glbIn && glbOut && csgIn && csgOut && gMali.devInfo.mcu_alive && queueMask) {
        for (uint32_t q = 0; q < qCount; ++q) {
            if (!(queueMask & (1u << q))) continue;
            FwCsControl *csCtrl = getCsCtrl(csgSlot, q);
            FwCsInput   *csIn   = csCtrl ? reinterpret_cast<FwCsInput  *>(fwVaToCpu(csCtrl->input_va))  : nullptr;
            FwCsOutput  *csOut  = csCtrl ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(csCtrl->output_va)) : nullptr;
            if (!csIn || !csOut) continue;
            cacheFlushRange(csOut, sizeof(*csOut));
            csIn->ringbuf_base   = ringGpuVa[q];
            csIn->ringbuf_size   = ringSize[q];
            csIn->ringbuf_input  = ringIoInFwVa[q];
            csIn->ringbuf_output = ringIoOutFwVa[q];
            csIn->config         = qPrio[q] | ((dbId & 0xFFu) << 8);
            csIn->ack_irq_mask   = 0xFFFFFFFFu;
            csIn->req            = (csOut->ack & ~CS_REQ_MASK) |
                                   CS_IDLE_SYNC_WAIT | CS_IDLE_EMPTY | CS_STATE_START;
            cacheFlushRange(csIn, sizeof(*csIn));
        }

        cacheFlushRange(csgOut, sizeof(*csgOut));
        csgIn->allow_compute  = allowC;
        csgIn->allow_fragment = allowF;
        csgIn->allow_other    = static_cast<uint32_t>(allowT);
        csgIn->endpoint_req   = (maxC & 0xFFu) | ((maxF & 0xFFu) << 8) |
                                ((maxT & 0xFu) << 16) | (grpPrio << 28);
        csgIn->config         = asSlot;
        csgIn->suspend_buf    = grpSuspFwVa;
        csgIn->protm_suspend_buf = 0;
        csgIn->ack_irq_mask   = 0xFFFFFFFFu;
        csgIn->cs_irq_ack     = csgOut->cs_irq_req;
        csgIn->doorbell_req   = csgOut->doorbell_ack ^ queueMask;
        uint32_t epBit        = (csgOut->ack ^ CSG_ENDPOINT_CONFIG) & CSG_ENDPOINT_CONFIG;
        csgIn->req            = (csgOut->ack & ~CSG_REQ_MASK) | targetState | epBit;
        cacheFlushRange(csgIn, sizeof(*csgIn));

        IOLockLock(gMali.lock);
        cacheFlushRange(glbOut, sizeof(*glbOut));
        glbIn->doorbell_req = glbOut->doorbell_ack ^ (1u << csgSlot);
        cacheFlushRange(glbIn, sizeof(*glbIn));
        wr32(gMali.maliReg.va, CSF_DOORBELL(0), 1u);
        IOLockUnlock(gMali.lock);

        for (int ms = 0; ms < 500; ++ms) {
            cacheFlushRange(csgOut, sizeof(*csgOut));
            uint32_t st = csgOut->ack & CSG_STATE_MASK;
            if (st == targetState || st == CSG_STATE_START) {
                started = true;
                break;
            }
            if (ms < 10) IODelay(100);
            else         IOSleep(1);
        }

        if (resuming && started) {
            gMali.resumeCount++;
            if (hasPendingOnResume) {
                wr32(gMali.maliReg.va, CSF_DOORBELL(dbId), 1u);
            }
        }
        static uint32_t sStartLogs = 0;
        if (!started || sStartLogs++ < 32u) {
            IOLog("Tb375fcMali: CSG[%u] %s proc=%s as=%u q=%u db=%u ep=0x%x evicts=%u resumes=%u\n",
                  csgSlot, started ? (resuming ? "RESUMED" : "STARTED") : "START_TIMEOUT",
                  client->fProcName, asSlot, qCount, dbId,
                  csgOut->status_endpoint_current, gMali.evictCount, gMali.resumeCount);
        }
    }

    slot.ownerClient   = client;
    slot.ownerGroupIdx = grpIdx;
    slot.ownerVmIdx    = vmId;
    slot.pinnedWs      = (client->fIsWindowServer && chosen == 0);
    slot.lastUseTick   = ++gMali.useClock;

    IOLockLock(client->fLock);
    client->fGroups[grpIdx].hw_started   = started;
    client->fGroups[grpIdx].hw_suspended = false;
    __atomic_store_n(&client->fGroups[grpIdx].hw_slot, static_cast<int32_t>(chosen), __ATOMIC_RELEASE);
    IOLockUnlock(client->fLock);

    if (slot.pinnedWs && gMali.wsLock) {
        IOLockLock(gMali.wsLock);
        gMali.wsClient   = client;
        gMali.wsGroupIdx = grpIdx;
        IOLockUnlock(gMali.wsLock);
    }
    return static_cast<int32_t>(chosen);
}

/* Detect, log, and self-heal any live Mali-G615 MMU page fault on AS1..AS7 (bits 1..7 of MMU_INT_RAWSTAT).
 * On Mali CSF, an unhandled AS page fault stalls that AS forever and locks up all Fragment cores held by
 * its CSG (causing the entire desktop to freeze). Mapping a zeroed scratch page on translation faults +
 * issuing MMU_INT_CLEAR & AS_CMD_UNLOCK lets the faulting draw finish harmlessly while logging exact VA! */
static bool checkAndRecoverMmuFaults(const char *tag) {
    if (!gMali.ready || !gMali.maliReg.va || !gMali.schedLock) return false;
    uintptr_t mv = gMali.maliReg.va;
    uint32_t mmuIrq = rd32(mv, MMU_INT_RAWSTAT);
    if ((mmuIrq & 0xFEu) == 0u) return false;

    IOLockLock(gMali.schedLock);
    mmuIrq = rd32(mv, MMU_INT_RAWSTAT);
    uint32_t faultMask = mmuIrq & 0xFEu;
    if (faultMask == 0u) {
        IOLockUnlock(gMali.schedLock);
        return false;
    }

    uint32_t shRdy  = rd32(mv, 0x0140u);
    uint32_t tlRdy  = rd32(mv, 0x0150u);
    uint32_t gpuFlt = rd32(mv, 0x003Cu);

    for (uint32_t asSlot = 1u; asSlot <= kMaxHwSlots; ++asSlot) {
        if ((faultMask & (1u << asSlot)) == 0u) continue;
        uint32_t csgSlot = asSlot - 1u;
        uint32_t asFlt   = rd32(mv, AS_REG(asSlot, AS_FAULTSTATUS));
        uint64_t asAddr  = static_cast<uint64_t>(rd32(mv, AS_REG(asSlot, AS_FAULTADDR_LO))) |
                           (static_cast<uint64_t>(rd32(mv, AS_REG(asSlot, AS_FAULTADDR_HI))) << 32);

        MaliHwSlot &slot = gMali.slots[csgSlot];
        Tb375fcMaliUserClient *owner = slot.ownerClient;
        const char *proc = owner ? owner->fProcName : "<none>";
        uint32_t vmIdx   = slot.ownerVmIdx;
        uint32_t grpIdx  = slot.ownerGroupIdx;

        uint32_t fnum = ++gMali.mmuFaultCount;
        bool mappedScratch = false;
        uint32_t vmRecs = 0;

        if (owner && owner->fLock && owner->fVms && vmIdx > 0u && vmIdx < kMaxClientVms) {
            IOLockLock(owner->fLock);
            MaliVm &vm = owner->fVms[vmIdx];
            if (vm.in_use && gMali.faultScratchPa != 0ULL &&
                (asAddr & ~4095ULL) != 0ULL && asAddr < 0x0000FFFFFFFF0000ULL &&
                vm.faultRecoveries < 64u) {
                /* Map faulting page to zeroed scratch page so MMU replay completes harmlessly */
                if (vmMapPage(&vm, asAddr & ~4095ULL, gMali.faultScratchPa,
                              MALI_VM_MAP_NOEXEC | MALI_VM_MAP_UNCACHED)) {
                    vm.faultRecoveries++;
                    mappedScratch = true;
                }
            }
            vmRecs = vm.faultRecoveries;
            IOLockUnlock(owner->fLock);
        }

        if (fnum <= 32u || (fnum % 64u) == 0u) {
            IOLog("Tb375fcMali: MMU FAULT #%u [%s] as=%u csg=%u proc=%s flt=0x%x@0x%llx gpuFlt=0x%x mmu=0x%x shRdy=0x%x tlRdy=0x%x scratch=%u(%u)\n",
                  fnum, tag ? tag : "?", asSlot, csgSlot, proc, asFlt, (unsigned long long)asAddr,
                  gpuFlt, mmuIrq, shRdy, tlRdy, mappedScratch ? 1u : 0u, vmRecs);
            if (owner && owner->fGroups && grpIdx > 0u && grpIdx < kMaxClientGroups) {
                uint32_t qc = owner->fGroups[grpIdx].queue_count;
                if (qc > MALI_MAX_QUEUES_PER_GROUP) qc = MALI_MAX_QUEUES_PER_GROUP;
                for (uint32_t qi = 0u; qi < qc; ++qi) {
                    FwCsControl *cc = getCsCtrl(csgSlot, qi);
                    FwCsOutput  *co = cc ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(cc->output_va)) : nullptr;
                    if (co) {
                        cacheFlushRange(co, sizeof(*co));
                        IOLog("Tb375fcMali:   fault-csg%u q[%u] cmd=0x%llx blk=%u wait=0x%x sb=0x%x flt=0x%x(0x%llx) ftl=0x%x(0x%llx)\n",
                              csgSlot, qi, (unsigned long long)co->status_cmd_ptr, co->status_blocked_reason,
                              co->status_wait, co->status_scoreboards,
                              co->fault, (unsigned long long)co->fault_info,
                              co->fatal, (unsigned long long)co->fatal_info);
                    }
                }
            }
        }

        if (asSlot == 1u && gMali.wsLock) IOLockLock(gMali.wsLock);
        if (mappedScratch) {
            hwFlushAs(asSlot, true);
        }
        wr32(mv, MMU_INT_CLEAR, (1u << asSlot));
        wr32(mv, AS_REG(asSlot, AS_COMMAND), AS_CMD_UNLOCK);
        for (int i = 0; i < 1000; ++i) {
            if ((rd32(mv, AS_REG(asSlot, AS_STATUS)) & 1u) == 0u) break;
            IODelay(5);
        }
        if (asSlot == 1u && gMali.wsLock) IOLockUnlock(gMali.wsLock);

        /* Terminate runaway non-WindowServer CSGs that keep faulting */
        if (owner && !owner->fIsWindowServer && vmRecs >= 8u && slot.ownerClient == owner) {
            IOLog("Tb375fcMali: terminating runaway faulting proc=%s on csg=%u (faults=%u)\n",
                  proc, csgSlot, vmRecs);
            evictHwSlotLocked(csgSlot, true);
        }
    }

    IOLockUnlock(gMali.schedLock);
    return true;
}

} // namespace

OSDefineMetaClassAndStructors(Tb375fcMali, IOService)
OSDefineMetaClassAndStructors(Tb375fcMaliUserClient, IOUserClient)

bool Tb375fcMali::start(IOService *provider) {
    if (!IOService::start(provider)) return false;
    setName(TB375FC_MALI_SERVICE_NAME);
    setProperty("NativeSandboxGrantAll", kOSBooleanTrue);
    registerService();
    return true;
}

IOReturn Tb375fcMali::newUserClient(task_t owningTask, void *securityID, UInt32 type,
                                    OSDictionary *properties, IOUserClient **handler) {
    Tb375fcMaliUserClient *uc = new Tb375fcMaliUserClient;
    if (!uc) return kIOReturnNoMemory;
    if (!uc->initWithTask(owningTask, securityID, type, properties) ||
        !uc->attach(this) || !uc->start(this)) {
        uc->release();
        return kIOReturnError;
    }
    static uint32_t sOpenLogs = 0;
    if (sOpenLogs++ < 32u || uc->fIsWindowServer) {
        IOLog("Tb375fcMali: newUserClient #%u proc=%s ws=%d\n", sOpenLogs, uc->fProcName, uc->fIsWindowServer ? 1 : 0);
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
    fSharedBytes = 0;
    memset(fProcName, 0, sizeof(fProcName));
    proc_selfname(fProcName, sizeof(fProcName));
    fIsWindowServer = (strcmp(fProcName, "WindowServer") == 0);
    if (fGroups) {
        for (uint32_t i = 0; i < kMaxClientGroups; ++i) fGroups[i].hw_slot = -1;
    }
    return fLock && fBos && fVms && fGroups;
}

IOReturn Tb375fcMaliUserClient::clientClose(void) {
    terminate();
    return kIOReturnSuccess;
}

void Tb375fcMaliUserClient::free(void) {
    /* CRITICAL: Evict and terminate any resident hardware CSG/AS slots owned by this client
     * BEFORE releasing ring buffers, page tables, or BOs! */
    if (gMali.schedLock) {
        IOLockLock(gMali.schedLock);
        for (uint32_t s = 0; s < kMaxHwSlots; ++s) {
            if (gMali.slots[s].ownerClient == this) {
                evictHwSlotLocked(s, true);
            }
        }
        IOLockUnlock(gMali.schedLock);
    }

    if (fGroups) {
        for (uint32_t i = 0; i < kMaxClientGroups; ++i) {
            if (fGroups[i].in_use) {
                if (fGroups[i].suspendPages > 0) {
                    freeAs0Pages(fGroups[i].suspendDynIdx, fGroups[i].suspendPages);
                    fGroups[i].suspendPages = 0;
                    fGroups[i].suspendFwVa = 0;
                }
                if (fGroups[i].ringIoPages > 0) {
                    freeAs0Pages(fGroups[i].ringIoDynIdx, fGroups[i].ringIoPages);
                    fGroups[i].ringIoPages = 0;
                }
                for (uint32_t q = 0; q < fGroups[i].queue_count; ++q) {
                    if (fGroups[i].ringDesc[q]) {
                        fGroups[i].ringDesc[q]->complete();
                        fGroups[i].ringDesc[q]->release();
                    }
                }
            }
        }
        IOFreeData(fGroups, sizeof(MaliGroupEntry) * kMaxClientGroups);
        fGroups = nullptr;
    }
    if (fVms) {
        for (uint32_t i = 0; i < kMaxClientVms; ++i) {
            if (fVms[i].in_use) vmFreeStorage(&fVms[i]);
        }
        IOFreeData(fVms, sizeof(MaliVm) * kMaxClientVms);
        fVms = nullptr;
    }
    if (fBos) {
        for (uint32_t i = 0; i < kMaxClientBos; ++i) {
            if (fBos[i].in_use && fBos[i].desc) {
                if (fBos[i].is_shared && gMali.lock) {
                    IOLockLock(gMali.lock);
                    if (gMali.sharedKmemBytes >= fBos[i].size) gMali.sharedKmemBytes -= fBos[i].size;
                    IOLockUnlock(gMali.lock);
                }
                fBos[i].desc->complete();
                fBos[i].desc->release();
            }
        }
        IOFreeData(fBos, sizeof(MaliBoEntry) * kMaxClientBos);
        fBos = nullptr;
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
    case MALI_SEL_DEBUG_LOG: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliDebugLogIn))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliDebugLogIn *>(args->structureInput);
        char buf[sizeof(in->msg) + 1];
        memcpy(buf, in->msg, sizeof(in->msg));
        buf[sizeof(in->msg)] = '\0';
        size_t len = strlen(buf);
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) buf[--len] = '\0';
        if (len > 0) IOLog("%s\n", buf);
        return kIOReturnSuccess;
    }

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
        /* Live Boot 155 verified kmem_data_shared_range = 2,113 MiB!
         * On Boot 155, iconservicesagent exceeded the 32 MiB non-WS cap (m=81 MB -> 17.7 ms dc cvac)
         * and WindowServer reached 684 MB RSS. Raise WC caps to 32 MiB/BO, 256 MiB WS, 128 MiB non-WS,
         * 1024 MiB global (still leaving >1,080 MiB of DATA_SHARED free) so zero apps fall back to dc cvac! */
        bool allowShared = false;
        uint64_t maxSingleShared = 32ULL * 1024ULL * 1024ULL;
        if ((in->flags & MALI_BO_NO_MMAP) == 0 && sz <= maxSingleShared) {
            IOLockLock(fLock);
            bool clientOk = fIsWindowServer
                ? (fSharedBytes + sz <= 256ULL * 1024ULL * 1024ULL)
                : (fSharedBytes + sz <= 128ULL * 1024ULL * 1024ULL);
            IOLockUnlock(fLock);
            if (clientOk) {
                IOLockLock(gMali.lock);
                if (gMali.sharedKmemBytes + sz <= 1024ULL * 1024ULL * 1024ULL) {
                    gMali.sharedKmemBytes += sz;
                    allowShared = true;
                }
                IOLockUnlock(gMali.lock);
            }
        }
        bool usedShared = false;
        if (allowShared) {
            desc = IOBufferMemoryDescriptor::withOptions(
                kIOMemoryKernelUserShared | kIODirectionInOut | kIOMapWriteCombineCache, sz, 4096);
            if (desc && desc->prepare() == kIOReturnSuccess) {
                memset(desc->getBytesNoCopy(), 0, sz);
                cacheFlushRange(desc->getBytesNoCopy(), sz);
                usedShared = true;
            } else {
                if (desc) { desc->release(); desc = nullptr; }
                IOLockLock(gMali.lock);
                if (gMali.sharedKmemBytes >= sz) gMali.sharedKmemBytes -= sz;
                IOLockUnlock(gMali.lock);
            }
        }
        if (!usedShared) {
            if ((in->flags & MALI_BO_NO_MMAP) == 0) {
                /* Force userspace to allocate clean Normal-WB anon mmap + BO_IMPORT_HOST */
                return kIOReturnNoMemory;
            }
            desc = IOBufferMemoryDescriptor::inTaskWithOptions(
                fTask, kIODirectionInOut | kIOMemoryPageable, sz, 4096);
            if (!desc || desc->prepare() != kIOReturnSuccess) {
                if (desc) desc->release();
                return kIOReturnNoMemory;
            }
            /* Zero & clean NO_MMAP BOs to DRAM so GPU never reads stale uninitialized lines */
            if (sz <= 64ULL * 1024ULL * 1024ULL) {
                IOMemoryMap *kmap = desc->createMappingInTask(kernel_task, 0, kIOMapAnywhere);
                if (kmap) {
                    void *va = reinterpret_cast<void *>(kmap->getVirtualAddress());
                    if (va) {
                        memset(va, 0, sz);
                        cacheFlushRange(va, sz);
                    }
                    kmap->release();
                }
            }
        }

        IOLockLock(fLock);
        uint32_t handle = 0;
        for (uint32_t i = 1; i < kMaxClientBos; ++i) {
            if (!fBos[i].in_use) {
                fBos[i].in_use = true;
                fBos[i].is_shared = usedShared;
                fBos[i].desc = desc;
                fBos[i].size = sz;
                if (usedShared) fSharedBytes += sz;
                handle = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!handle) {
            if (usedShared) {
                IOLockLock(gMali.lock);
                if (gMali.sharedKmemBytes >= sz) gMali.sharedKmemBytes -= sz;
                IOLockUnlock(gMali.lock);
            }
            desc->complete(); desc->release(); return kIOReturnNoResources;
        }

        auto *out = static_cast<Tb375fcMaliBoCreateOut *>(args->structureOutput);
        out->bo_handle = handle;
        out->pad = usedShared ? 0u : 1u; /* 0 = WC (no dc cvac), 1 = cached fTask VM */
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
        bool wasShared = false;
        uint64_t freedSz = 0;
        if (fBos[h].in_use) {
            desc = fBos[h].desc;
            wasShared = fBos[h].is_shared;
            freedSz = fBos[h].size;
            if (wasShared && fSharedBytes >= freedSz) fSharedBytes -= freedSz;
            fBos[h].in_use = false;
            fBos[h].is_shared = false;
            fBos[h].desc = nullptr;
            fBos[h].size = 0;
        }
        IOLockUnlock(fLock);
        if (wasShared) {
            IOLockLock(gMali.lock);
            if (gMali.sharedKmemBytes >= freedSz) gMali.sharedKmemBytes -= freedSz;
            IOLockUnlock(gMali.lock);
        }
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
            desc = IOMemoryDescriptor::withAddressRange(
                static_cast<mach_vm_address_t>(in->host_ptr), static_cast<mach_vm_size_t>(sz),
                kIODirectionOut, fTask);
            if (!desc || desc->prepare() != kIOReturnSuccess) {
                if (desc) desc->release();
                return kIOReturnNoMemory;
            }
            static uint32_t sRoLog = 0;
            if (sRoLog++ < 8u) {
                IOLog("Tb375fcMali: host import (%s) 0x%llx+%llu wired read-only\n",
                      fProcName, (unsigned long long)in->host_ptr, (unsigned long long)sz);
            }
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

        IOLockLock(fLock);
        uint32_t vmId = 0;
        for (uint32_t i = 1; i < kMaxClientVms; ++i) {
            if (!fVms[i].in_use) {
                MaliVm &v = fVms[i];
                memset(&v, 0, sizeof(v));
                v.in_use = true;
                v.id = i;
                v.as_slot_mask = 0;
                v.ptPagePas = static_cast<uint64_t *>(IOMallocZeroData(sizeof(uint64_t) * kMaxPtPagesPerVm));
                v.nextInternalVa = 0x000000F000000000ULL;
                if (!v.ptPagePas || vmAllocPtPage(&v, &v.l0Pa, &v.l0Va) == UINT32_MAX) {
                    vmFreeStorage(&v);
                    v.in_use = false;
                    IOLockUnlock(fLock);
                    return kIOReturnNoMemory;
                }
                uint64_t nullPgPa = 0;
                uint64_t *nullPgVa = nullptr;
                if (vmAllocPtPage(&v, &nullPgPa, &nullPgVa) != UINT32_MAX) {
                    vmMapPage(&v, 0ULL, nullPgPa, MALI_VM_MAP_NOEXEC | MALI_VM_MAP_UNCACHED);
                }
                vmId = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!vmId) return kIOReturnNoResources;

        auto *out = static_cast<Tb375fcMaliVmCreateOut *>(args->structureOutput);
        out->vm_id = vmId;
        out->as_slot = fIsWindowServer ? 1u : 2u; /* informational; actual AS assigned on submit */
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

        IOLockLock(gMali.schedLock);
        for (uint32_t s = 0; s < kMaxHwSlots; ++s) {
            if (gMali.slots[s].ownerClient == this && gMali.slots[s].ownerVmIdx == id) {
                evictHwSlotLocked(s, true);
            }
        }
        IOLockUnlock(gMali.schedLock);

        IOLockLock(fLock);
        if (fVms[id].in_use) {
            vmFreeStorage(&fVms[id]);
            fVms[id].in_use = false;
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
            vmUnmapRange(&vm, in->va, pages);
        } else {
            if (in->bo_handle == 0 || in->bo_handle >= kMaxClientBos || !fBos[in->bo_handle].in_use) {
                IOLockUnlock(fLock);
                return kIOReturnNotFound;
            }
            IOMemoryDescriptor *desc = fBos[in->bo_handle].desc;
            uint64_t failP = 0, failPa = 0;
            if (!vmMapRange(&vm, in->va, desc, in->bo_offset, pages, in->flags, &failP, &failPa)) {
                uint32_t usedPt = vm.ptPagesUsed;
                IOLockUnlock(fLock);
                IOLog("Tb375fcMali: VM_BIND FAIL proc=%s vm=%u bo=%u va=0x%llx off=0x%llx sz=0x%llx p=%llu pa=0x%llx pt=%u/%u\n",
                      fProcName, in->vm_id, in->bo_handle, (unsigned long long)in->va,
                      (unsigned long long)in->bo_offset, (unsigned long long)in->size,
                      (unsigned long long)failP, (unsigned long long)failPa, usedPt, kMaxPtPagesPerVm);
                return kIOReturnNoMemory;
            }
        }
        uint32_t asMask = vm.as_slot_mask;
        IOLockUnlock(fLock);

        if (asMask) {
            bool isUnmap = (in->op == MALI_VM_BIND_UNMAP);
            if (fIsWindowServer && (asMask == (1u << 1u)) && gMali.wsLock) {
                /* Fast path for WindowServer's pinned AS1: serialize on wsLock, never block on schedLock */
                IOLockLock(gMali.wsLock);
                hwFlushAs(1u, isUnmap);
                IOLockUnlock(gMali.wsLock);
            } else {
                IOLockLock(gMali.schedLock);
                for (uint32_t s = 0; s < kMaxHwSlots; ++s) {
                    if ((asMask & (1u << (s + 1u))) &&
                        gMali.slots[s].ownerClient == this &&
                        gMali.slots[s].ownerVmIdx == in->vm_id) {
                        hwFlushAs(s + 1u, isUnmap);
                    }
                }
                IOLockUnlock(gMali.schedLock);
            }
        }
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
        if (fIsWindowServer) {
            if (cnt < 16u) cnt = 16u;
            if (cnt > 24u) cnt = 24u;
        } else {
            if (cnt < 4u) cnt = 4u;
            if (cnt > 8u) cnt = 8u;
        }
        uint64_t totalSz = 4096ULL + static_cast<uint64_t>(cnt) * chunkSz;

        /* Zero-kmem_io tiler heap: wire pages in fTask, map transiently in kernel_task to link chunks */
        IOBufferMemoryDescriptor *desc = IOBufferMemoryDescriptor::inTaskWithOptions(
            fTask, kIODirectionInOut | kIOMemoryPageable, totalSz, 4096);
        if (!desc || desc->prepare() != kIOReturnSuccess) {
            if (desc) desc->release();
            return kIOReturnNoMemory;
        }
        IOMemoryMap *kmap = desc->createMappingInTask(kernel_task, 0, kIOMapAnywhere);
        if (!kmap) {
            desc->complete(); desc->release();
            return kIOReturnNoMemory;
        }
        uint8_t *cpuBase = reinterpret_cast<uint8_t *>(kmap->getVirtualAddress());
        memset(cpuBase, 0, totalSz);

        IOLockLock(fLock);
        MaliVm &vm = fVms[in->vm_id];
        if (!vm.in_use) {
            IOLockUnlock(fLock);
            kmap->release(); desc->complete(); desc->release();
            return kIOReturnNotFound;
        }
        uint64_t baseVa = vm.nextInternalVa;
        vm.nextInternalVa += (totalSz + 0xFFFFFULL) & ~0xFFFFFULL;
        if (!vmMapRange(&vm, baseVa, desc, 0ULL, totalSz / 4096ULL, MALI_VM_MAP_NOEXEC)) {
            IOLockUnlock(fLock);
            kmap->release(); desc->complete(); desc->release();
            return kIOReturnNoMemory;
        }
        for (uint32_t c = 0; c < cnt; ++c) {
            uint64_t *hdr = reinterpret_cast<uint64_t *>(cpuBase + 4096ULL + static_cast<uint64_t>(c) * chunkSz);
            uint64_t nextVa = (c + 1 < cnt) ? (baseVa + 4096ULL + static_cast<uint64_t>(c + 1) * chunkSz) : 0ULL;
            hdr[0] = nextVa ? ((nextVa & ~4095ULL) | (static_cast<uint64_t>(chunkSz) >> 12)) : 0ULL;
        }
        cacheFlushRange(cpuBase, totalSz);
        kmap->release();

        uint32_t asMask = vm.as_slot_mask;
        uint32_t handle = 0;
        for (uint32_t i = 1; i < kMaxClientBos; ++i) {
            if (!fBos[i].in_use) {
                fBos[i].in_use = true;
                fBos[i].is_shared = false;
                fBos[i].desc = desc;
                fBos[i].size = totalSz;
                handle = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!handle) { desc->complete(); desc->release(); return kIOReturnNoResources; }

        if (asMask) {
            IOLockLock(gMali.schedLock);
            for (uint32_t s = 0; s < kMaxHwSlots; ++s) {
                if ((asMask & (1u << (s + 1u))) &&
                    gMali.slots[s].ownerClient == this &&
                    gMali.slots[s].ownerVmIdx == in->vm_id) {
                    hwFlushAs(s + 1u);
                }
            }
            IOLockUnlock(gMali.schedLock);
        }

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

        /* Pre-allocate per-group private AS0 suspend_buf + ringIoIn/Out pages OUTSIDE fLock */
        uint32_t reqQueues = in->queue_count ? in->queue_count : 1u;
        if (reqQueues > MALI_MAX_QUEUES_PER_GROUP) reqQueues = MALI_MAX_QUEUES_PER_GROUP;
        uint32_t suspFwVa = 0, suspDynIdx = 0, suspPages = 0;
        if (gMali.csgSuspendSize > 0u) {
            uint32_t sp = (gMali.csgSuspendSize + 4095u) / 4096u;
            if (allocAs0Pages(sp, false, &suspFwVa, nullptr, &suspDynIdx)) {
                suspPages = sp;
            }
        }
        uint32_t ioBaseFwVa = 0, ioDynIdx = 0, ioPages = 0;
        uint8_t *ioBaseCpu = nullptr;
        if (allocAs0Pages(reqQueues * 2u, true, &ioBaseFwVa, &ioBaseCpu, &ioDynIdx)) {
            ioPages = reqQueues * 2u;
        } else {
            if (suspPages > 0) freeAs0Pages(suspDynIdx, suspPages);
            return kIOReturnNoMemory;
        }

        IOLockLock(fLock);
        MaliVm &vm = fVms[in->vm_id];
        if (!vm.in_use) {
            IOLockUnlock(fLock);
            if (suspPages > 0) freeAs0Pages(suspDynIdx, suspPages);
            if (ioPages > 0)   freeAs0Pages(ioDynIdx, ioPages);
            return kIOReturnNotFound;
        }

        uint32_t grpHandle = 0;
        for (uint32_t i = 1; i < kMaxClientGroups; ++i) {
            if (!fGroups[i].in_use) {
                MaliGroupEntry &g = fGroups[i];
                memset(&g, 0, sizeof(g));
                g.in_use = true;
                g.hw_slot = -1;
                g.vm_id = in->vm_id;
                g.suspendFwVa   = suspFwVa;
                g.suspendDynIdx = suspDynIdx;
                g.suspendPages  = suspPages;
                g.ringIoDynIdx  = ioDynIdx;
                g.ringIoPages   = ioPages;
                g.queue_count   = reqQueues;
                g.priority           = in->priority;
                g.max_compute_cores  = in->max_compute_cores;
                g.max_fragment_cores = in->max_fragment_cores;
                g.max_tiler_cores    = in->max_tiler_cores;
                g.compute_core_mask  = in->compute_core_mask;
                g.fragment_core_mask = in->fragment_core_mask;
                g.tiler_core_mask    = in->tiler_core_mask;

                for (uint32_t q = 0; q < g.queue_count; ++q) {
                    g.ringIoInFwVa[q]  = ioBaseFwVa + (q * 2u) * 4096u;
                    g.ringIoInCpu[q]   = reinterpret_cast<FwRingIoInput *>(ioBaseCpu + (q * 2u) * 4096u);
                    g.ringIoOutFwVa[q] = ioBaseFwVa + (q * 2u + 1u) * 4096u;
                    g.ringIoOutCpu[q]  = reinterpret_cast<FwRingIoOutput *>(ioBaseCpu + (q * 2u + 1u) * 4096u);
                    g.queuePrio[q] = in->queues[q].priority;
                    uint32_t rsz = in->queues[q].ringbuf_size;
                    if (fIsWindowServer && rsz < 262144u) rsz = 262144u; /* 256 KiB = 2048 packets for WindowServer burst headroom */
                    if (rsz < 65536u) rsz = 65536u;
                    if (!fIsWindowServer && rsz > 65536u) rsz = 65536u;
                    if (rsz > 262144u) rsz = 262144u;
                    rsz = (rsz + 4095u) & ~4095u;
                    /* Ring buffers are kernel+GPU only (never userspace-mapped): omit kIOMemoryKernelUserShared */
                    g.ringDesc[q] = IOBufferMemoryDescriptor::withOptions(
                        kIODirectionInOut, rsz, 4096);
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
                        g.ringSize[q]  = rsz;
                    }
                }
                grpHandle = i;
                break;
            }
        }
        IOLockUnlock(fLock);
        if (!grpHandle) {
            if (suspPages > 0) freeAs0Pages(suspDynIdx, suspPages);
            if (ioPages > 0)   freeAs0Pages(ioDynIdx, ioPages);
            return kIOReturnNoResources;
        }

        /* Pin WindowServer's group onto hardware Slot 0 immediately so its first submit has 0ms setup */
        uint32_t reportedCsg = 0;
        if (fIsWindowServer) {
            IOLockLock(gMali.schedLock);
            int32_t s = ensureGroupResidentLocked(this, grpHandle);
            if (s >= 0) reportedCsg = static_cast<uint32_t>(s);
            IOLockUnlock(gMali.schedLock);
        }

        auto *out = static_cast<Tb375fcMaliGroupCreateOut *>(args->structureOutput);
        out->group_handle = grpHandle;
        out->csg_slot = reportedCsg;
        args->structureOutputSize = sizeof(*out);
        return kIOReturnSuccess;
    }

    case MALI_SEL_GROUP_DESTROY: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliHandleIn))
            return kIOReturnBadArgument;
        uint32_t h = static_cast<const Tb375fcMaliHandleIn *>(args->structureInput)->handle;
        if (h == 0 || h >= kMaxClientGroups) return kIOReturnBadArgument;

        IOLockLock(gMali.schedLock);
        for (uint32_t s = 0; s < kMaxHwSlots; ++s) {
            if (gMali.slots[s].ownerClient == this && gMali.slots[s].ownerGroupIdx == h) {
                evictHwSlotLocked(s, true);
            }
        }
        IOLockUnlock(gMali.schedLock);

        uint32_t freeDynIdx = 0, freePages = 0, freeIoIdx = 0, freeIoPages = 0;
        IOLockLock(fLock);
        if (fGroups[h].in_use) {
            freeDynIdx  = fGroups[h].suspendDynIdx;
            freePages   = fGroups[h].suspendPages;
            freeIoIdx   = fGroups[h].ringIoDynIdx;
            freeIoPages = fGroups[h].ringIoPages;
            fGroups[h].suspendPages = 0;
            fGroups[h].suspendFwVa  = 0;
            fGroups[h].ringIoPages  = 0;
            for (uint32_t q = 0; q < fGroups[h].queue_count; ++q) {
                if (fGroups[h].ringDesc[q]) {
                    fGroups[h].ringDesc[q]->complete();
                    fGroups[h].ringDesc[q]->release();
                    fGroups[h].ringDesc[q] = nullptr;
                }
            }
            fGroups[h].in_use = false;
            fGroups[h].hw_started = false;
            fGroups[h].hw_suspended = false;
            fGroups[h].hw_slot = -1;
        }
        IOLockUnlock(fLock);
        if (freePages > 0)   freeAs0Pages(freeDynIdx, freePages);
        if (freeIoPages > 0) freeAs0Pages(freeIoIdx, freeIoPages);
        return kIOReturnSuccess;
    }

    case MALI_SEL_GROUP_SUBMIT: {
        if (!args->structureInput || args->structureInputSize < sizeof(Tb375fcMaliGroupSubmitIn) ||
            !args->structureOutput || args->structureOutputSize < sizeof(Tb375fcMaliGroupSubmitOut))
            return kIOReturnBadArgument;
        const auto *in = static_cast<const Tb375fcMaliGroupSubmitIn *>(args->structureInput);
        if (in->group_handle == 0 || in->group_handle >= kMaxClientGroups) return kIOReturnBadArgument;

        uint32_t scnt = in->submit_count;
        if (scnt > MALI_MAX_SUBMITS_PER_CALL) scnt = MALI_MAX_SUBMITS_PER_CALL;

        /* Fast path for WindowServer's permanently pinned Slot 0 (CSG0/AS1/DB1):
         * skip global gMali.schedLock completely so non-WS LRU evictions NEVER stall compositor! */
        bool lockedSched = false;
        int32_t slotIdx = -1;
        if (fIsWindowServer && __atomic_load_n(&fGroups[in->group_handle].hw_slot, __ATOMIC_ACQUIRE) == 0) {
            slotIdx = 0;
        } else {
            IOLockLock(gMali.schedLock);
            lockedSched = true;
            slotIdx = ensureGroupResidentLocked(this, in->group_handle);
            if (slotIdx < 0) {
                IOLockUnlock(gMali.schedLock);
                return kIOReturnNoResources;
            }
        }
        uint32_t dbId = static_cast<uint32_t>(slotIdx) + 1u;

        IOLockLock(fLock);
        MaliGroupEntry &g = fGroups[in->group_handle];
        if (!g.in_use) {
            IOLockUnlock(fLock);
            if (lockedSched) IOLockUnlock(gMali.schedLock);
            return kIOReturnNotFound;
        }

        if (g.hw_started && scnt > 0) {
            uint64_t addrReg = (gMali.devInfo.cs_reg_count >= 4u) ? (gMali.devInfo.cs_reg_count - 4u) : 92u;
            uint64_t valReg  = addrReg + 2u;
            uint64_t waitAll = (gMali.devInfo.scoreboard_slot_count > 0 && gMali.devInfo.scoreboard_slot_count <= 16u)
                               ? ((1ULL << gMali.devInfo.scoreboard_slot_count) - 1ULL) : 0xFFULL;
            /* Sample hardware CSF_GPU_LATEST_FLUSH_ID ONCE for this submit batch:
             * The first active subqueue that executes FLUSH_CACHE2(0x233, batchFlushId)
             * invalidates L2+LSC+OTHER and hardware increments CSF_GPU_LATEST_FLUSH_ID,
             * so all other subqueues (q=1 FRAG, q=2 COMPUTE) in the same submit batch
             * skip the duplicate full-L2 invalidate in hardware at zero cost! */
            uint32_t batchFlushId = rd32(gMali.maliReg.va, CSF_GPU_LATEST_FLUSH_ID);
            uint32_t qFirstFlushed = 0u;
            uint32_t qDirtyMask = 0u;
            uint32_t lastIdxForQ[MALI_MAX_QUEUES_PER_GROUP];
            for (uint32_t qi = 0; qi < MALI_MAX_QUEUES_PER_GROUP; ++qi) lastIdxForQ[qi] = UINT32_MAX;
            for (uint32_t s = 0; s < scnt; ++s) {
                if (in->submits[s].stream_size > 0 && in->submits[s].stream_addr != 0) {
                    uint32_t qq = in->submits[s].queue_index;
                    if (qq >= g.queue_count) qq = 0;
                    lastIdxForQ[qq] = s;
                }
            }
            for (uint32_t s = 0; s < scnt; ++s) {
                const Tb375fcMaliQueueSubmitDesc &sub = in->submits[s];
                /* Mirror mainline Linux panthor_sched.c queue_run_job(): zero-size wait/signal
                 * sync markers emit NO ringbuffer packet and NO pipeline-draining WAIT(waitAll). */
                if (sub.stream_size == 0 || sub.stream_addr == 0) continue;
                uint32_t q = sub.queue_index;
                if (q >= g.queue_count) q = 0;
                if (!g.ringDesc[q] || !g.ringIoInCpu[q]) continue;

                /* 64-byte (8 x u64, 1 cacheline) ringbuffer packet:
                 * Up to 3 entry-flush + 3 CALL + 2 end-of-batch WAIT(waitAll)+ERROR_BARRIER = 8.
                 * Intermediate calls (e.g. 8B req_resource or multi-cmdbuf chunks) omit redundant
                 * WAIT(waitAll) and end-of-batch 0x011 clean because panvk's finish_cs() already
                 * emits cs_wait_slots(all) + cs_flush_caches(CLEAN, CLEAN, NONE, 0) per cmdbuf! */
                uint64_t pkt[8];
                memset(pkt, 0, sizeof(pkt));
                uint32_t n = 0;
                if ((qFirstFlushed & (1u << q)) == 0u) {
                    qFirstFlushed |= (1u << q);
                    pkt[n++] = (2ULL << 56) | (valReg << 48) | static_cast<uint64_t>(batchFlushId);
                    pkt[n++] = (36ULL << 56) | (0ULL << 48) | (valReg << 40) | (0ULL << 16) | 0x233ULL;
                    pkt[n++] = (3ULL << 56) | (1ULL << 16);
                }
                pkt[n++] = (1ULL << 56) | (addrReg << 48) | (sub.stream_addr & 0x0000FFFFFFFFFFFFULL);
                pkt[n++] = (2ULL << 56) | (valReg << 48) | static_cast<uint64_t>(sub.stream_size);
                pkt[n++] = (32ULL << 56) | (addrReg << 40) | (valReg << 32);
                if (s == lastIdxForQ[q]) {
                    pkt[n++] = (3ULL << 56) | (waitAll << 16);
                    pkt[n++] = (47ULL << 56);
                }
                /* Guard against CS ringbuffer wraparound overwrite if queue is backed up */
                if (g.ringIoOutCpu[q] && g.ringSize[q] > sizeof(pkt) * 2u) {
                    for (uint32_t w = 0u; w < 200u; ++w) {
                        cacheFlushRange(g.ringIoOutCpu[q], 64);
                        uint64_t ext = g.ringIoOutCpu[q]->extract;
                        if (g.ringInsert[q] < ext || (g.ringInsert[q] - ext) <= (g.ringSize[q] - sizeof(pkt) * 2u)) break;
                        IODelay(50);
                    }
                }
                uint32_t ringOff = static_cast<uint32_t>(g.ringInsert[q] % g.ringSize[q]);
                uint8_t *ringCpu = static_cast<uint8_t *>(g.ringDesc[q]->getBytesNoCopy());
                memcpy(ringCpu + ringOff, pkt, sizeof(pkt));
                cacheFlushRange(ringCpu + ringOff, sizeof(pkt));

                g.ringInsert[q] += sizeof(pkt);
                qDirtyMask |= (1u << q);
            }
            if (qDirtyMask) {
                /* Publish updated ringInsert ONCE per active queue, then ring CSG doorbell ONCE */
                for (uint32_t q = 0; q < g.queue_count && q < MALI_MAX_QUEUES_PER_GROUP; ++q) {
                    if ((qDirtyMask & (1u << q)) && g.ringIoInCpu[q]) {
                        g.ringIoInCpu[q]->insert = g.ringInsert[q];
                        cacheFlushRange(g.ringIoInCpu[q], 64);
                    }
                }
                wr32(gMali.maliReg.va, CSF_DOORBELL(dbId), 1u);
            }
        }
        uint64_t seq = ++g.submitted_seq;
        MaliSeqTarget &st = g.seqHist[seq % kSeqHistSize];
        st.seq = seq;
        for (uint32_t qi = 0; qi < MALI_MAX_QUEUES_PER_GROUP; ++qi) {
            st.insert[qi] = g.ringInsert[qi];
        }
        if (!g.hw_started || scnt == 0) {
            __atomic_store_n(&g.completed_seq, seq, __ATOMIC_RELEASE);
        }
        IOLockUnlock(fLock);
        if (lockedSched) IOLockUnlock(gMali.schedLock);

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

        uint64_t wantSeq = in->fence_seq;
        if (in->group_handle == 0 || in->group_handle >= kMaxClientGroups || wantSeq == 0) {
            out->signaled = 1u;
            out->current_seq = wantSeq;
            args->structureOutputSize = sizeof(*out);
            return kIOReturnSuccess;
        }

        const bool wsFastFence = fIsWindowServer && gMali.wsLock &&
                                 (__atomic_load_n(&fGroups[in->group_handle].hw_slot, __ATOMIC_ACQUIRE) == 0);
        IOLock *slotLock = wsFastFence ? gMali.wsLock : gMali.schedLock;
        IOLockLock(slotLock);
        IOLockLock(fLock);
        MaliGroupEntry &g = fGroups[in->group_handle];
        uint64_t curComp = __atomic_load_n(&g.completed_seq, __ATOMIC_ACQUIRE);
        if (!g.in_use || curComp >= wantSeq || !g.hw_started || g.hw_slot < 0) {
            IOLockUnlock(fLock);
            IOLockUnlock(slotLock);
            out->signaled = 1u;
            out->current_seq = (curComp >= wantSeq) ? curComp : wantSeq;
            args->structureOutputSize = sizeof(*out);
            return kIOReturnSuccess;
        }
        int32_t slotIdx = g.hw_slot;
        uint32_t qCount = g.queue_count;
        uint32_t vmId   = g.vm_id;
        uint64_t targets[MALI_MAX_QUEUES_PER_GROUP] = {};
        FwRingIoOutput *ioOuts[MALI_MAX_QUEUES_PER_GROUP] = {};
        const MaliSeqTarget &st = g.seqHist[wantSeq % kSeqHistSize];
        for (uint32_t qi = 0; qi < qCount; ++qi) {
            targets[qi] = (st.seq == wantSeq) ? st.insert[qi] : g.ringInsert[qi];
            ioOuts[qi]  = g.ringIoOutCpu[qi];
        }
        IOLockUnlock(fLock);
        IOLockUnlock(slotLock);

        bool done = false;

        auto isDone = [&]() -> bool {
            if (__atomic_load_n(&g.completed_seq, __ATOMIC_ACQUIRE) >= wantSeq) return true;
            int32_t liveSlot = __atomic_load_n(&g.hw_slot, __ATOMIC_ACQUIRE);
            if (liveSlot != slotIdx) {
                if (__atomic_load_n(&g.completed_seq, __ATOMIC_ACQUIRE) >= wantSeq) return true;
                if (in->timeout_ns == 0) return false;
                /* Group was suspended mid-flight by LRU eviction; resume it to finish wantSeq */
                IOLockLock(gMali.schedLock);
                int32_t newSlot = ensureGroupResidentLocked(this, in->group_handle);
                if (newSlot >= 0) {
                    slotIdx = newSlot;
                    wr32(gMali.maliReg.va, CSF_DOORBELL(static_cast<uint32_t>(slotIdx) + 1u), 1u);
                }
                IOLockUnlock(gMali.schedLock);
                return __atomic_load_n(&g.completed_seq, __ATOMIC_ACQUIRE) >= wantSeq;
            }
            for (uint32_t qi = 0; qi < qCount; ++qi) {
                if (!ioOuts[qi] || targets[qi] == 0) continue;
                cacheFlushRange(ioOuts[qi], 64);
                if (ioOuts[qi]->extract < targets[qi]) return false;
            }
            return true;
        };

        if (in->timeout_ns == 0) {
            done = isDone();
        } else {
            uint64_t t0 = 0, tNow = 0, ns0 = 0, nsNow = 0;
            clock_get_uptime(&t0);
            absolutetime_to_nanoseconds(t0, &ns0);
            uint64_t capNs = (in->timeout_ns > 5000000000ULL) ? 5000000000ULL : in->timeout_ns;
            const uint64_t deadlineNs = ns0 + capNs;

            /* Brief 150us spin for tiny submits, then non-coalesced 200us timer waits
             * (0ms leeway, 0% CPU burn while GPU runs, <=0.2ms wakeup jitter). */
            uint32_t spinMax = (capNs < 150000ULL)
                ? static_cast<uint32_t>((capNs + 14999ULL) / 15000ULL) : 10u;
            for (uint32_t spin = 0; spin < spinMax; ++spin) {
                if (isDone()) { done = true; break; }
                IODelay(15);
            }
            uint32_t sleeps = 0;
            while (!done) {
                if (isDone()) { done = true; break; }
                clock_get_uptime(&tNow);
                absolutetime_to_nanoseconds(tNow, &nsNow);
                if (nsNow >= deadlineNs) break;
                if ((++sleeps % 100u) == 0u &&
                    (rd32(gMali.maliReg.va, MMU_INT_RAWSTAT) & 0xFEu) != 0u) {
                    checkAndRecoverMmuFaults("fence-wait");
                    if (isDone()) { done = true; break; }
                }
                uint64_t remUs = (deadlineNs - nsNow) / 1000ULL;
                uint32_t stepUs = (sleeps <= 50u) ? 200u : 500u;
                if (remUs < stepUs) stepUs = (remUs > 50u) ? static_cast<uint32_t>(remUs) : 50u;
                maliShortSleepUs(stepUs);
            }
        }

        if (!done && (rd32(gMali.maliReg.va, MMU_INT_RAWSTAT) & 0xFEu) != 0u) {
            checkAndRecoverMmuFaults("fence-to");
            if (isDone()) done = true;
        }

        if (done) {
            IOLockLock(fLock);
            if (g.in_use && wantSeq > __atomic_load_n(&g.completed_seq, __ATOMIC_RELAXED)) {
                __atomic_store_n(&g.completed_seq, wantSeq, __ATOMIC_RELEASE);
            }
            IOLockUnlock(fLock);

            static uint32_t sOkLogCount = 0;
            if (in->timeout_ns != 0 && sOkLogCount < 16u) {
                sOkLogCount++;
                uint32_t asSlot = static_cast<uint32_t>(slotIdx) + 1u;
                uint32_t gpuFlt = rd32(gMali.maliReg.va, 0x003Cu);
                uint32_t asFlt  = rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTSTATUS));
                IOLog("Tb375fcMali: HW FENCE OK #%u proc=%s csg=%d seq=%llu gpuFlt=0x%x asFlt=0x%x\n",
                      sOkLogCount, fProcName, slotIdx, (unsigned long long)wantSeq, gpuFlt, asFlt);
            }
        } else if (in->timeout_ns >= 1000000000ULL) {
            /* Only emit heavy diagnostic dump + 15ms status update on genuine >= 1s hangs,
             * never on normal short userspace poll slices (e.g. 8ms WAIT_ANY or 16ms frame checks). */
            uint32_t csgSlot = static_cast<uint32_t>(slotIdx);
            uint32_t asSlot  = csgSlot + 1u;
            uint32_t csgEpCur = 0, csgEpReq = 0, csgState = 0;
            FwGlobalControl *glbCtrl = gMali.sharedIfaceVa ? reinterpret_cast<FwGlobalControl *>(gMali.sharedIfaceVa) : nullptr;
            FwGlobalInput   *glbIn   = glbCtrl ? reinterpret_cast<FwGlobalInput  *>(fwVaToCpu(glbCtrl->input_va))  : nullptr;
            FwGlobalOutput  *glbOut  = glbCtrl ? reinterpret_cast<FwGlobalOutput *>(fwVaToCpu(glbCtrl->output_va)) : nullptr;
            FwCsgControl    *csgCtrl = getCsgCtrl(csgSlot);
            FwCsgInput      *csgIn   = csgCtrl ? reinterpret_cast<FwCsgInput  *>(fwVaToCpu(csgCtrl->input_va))  : nullptr;
            FwCsgOutput     *csgOut  = csgCtrl ? reinterpret_cast<FwCsgOutput *>(fwVaToCpu(csgCtrl->output_va)) : nullptr;
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
                IOSleep(15);
                cacheFlushRange(csgOut, sizeof(*csgOut));
                csgEpCur = csgOut->status_endpoint_current;
                csgEpReq = csgOut->status_endpoint_req;
                csgState = csgOut->status_state;
            }
            uint32_t shRdy = rd32(gMali.maliReg.va, 0x0140u);
            uint32_t tlRdy = rd32(gMali.maliReg.va, 0x0150u);
            uint32_t mmuIrq = rd32(gMali.maliReg.va, MMU_INT_RAWSTAT);
            uint32_t asFlt  = rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTSTATUS));
            uint64_t asAddr = static_cast<uint64_t>(rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTADDR_LO))) |
                              (static_cast<uint64_t>(rd32(gMali.maliReg.va, AS_REG(asSlot, AS_FAULTADDR_HI))) << 32);
            IOLog("Tb375fcMali: FENCE TIMEOUT proc=%s csg=%u seq=%llu ep=0x%x/0x%x st=0x%x shRdy=0x%x tlRdy=0x%x mmu=0x%x as%u_flt=0x%x@0x%llx vm=%u kmem=%lluM\n",
                  fProcName, csgSlot, (unsigned long long)wantSeq, csgEpCur, csgEpReq, csgState,
                  shRdy, tlRdy, mmuIrq, asSlot, asFlt, (unsigned long long)asAddr, vmId,
                  (unsigned long long)(gMali.sharedKmemBytes >> 20));
            bool anyFault = (asFlt != 0u);
            bool allSyncWaitIdle = (qCount > 0u);
            for (uint32_t qi = 0; qi < qCount; ++qi) {
                FwCsControl *cc = getCsCtrl(csgSlot, qi);
                FwCsOutput  *co = cc ? reinterpret_cast<FwCsOutput *>(fwVaToCpu(cc->output_va)) : nullptr;
                uint64_t qe = 0;
                if (ioOuts[qi]) { cacheFlushRange(ioOuts[qi], 64); qe = ioOuts[qi]->extract; }
                if (co) {
                    cacheFlushRange(co, sizeof(*co));
                    if (co->fault != 0u || co->fatal != 0u) anyFault = true;
                    if (co->status_blocked_reason != 3u || co->status_scoreboards != 0u) allSyncWaitIdle = false;
                    IOLog("Tb375fcMali:   q[%u] ext=%llu/%llu ack=0x%x cmd=0x%llx blk=%u wait=0x%x sync=0x%llx:%u sb=0x%x flt=0x%x(0x%llx) ftl=0x%x(0x%llx)\n",
                          qi, (unsigned long long)qe, (unsigned long long)targets[qi],
                          co->ack, (unsigned long long)co->status_cmd_ptr, co->status_blocked_reason,
                          co->status_wait, (unsigned long long)co->status_wait_sync_ptr, co->status_wait_sync_value,
                          co->status_scoreboards,
                          co->fault, (unsigned long long)co->fault_info,
                          co->fatal, (unsigned long long)co->fatal_info);
                } else {
                    allSyncWaitIdle = false;
                }
            }
            if (mmuIrq & (1u << asSlot)) {
                wr32(gMali.maliReg.va, MMU_INT_CLEAR, (1u << asSlot));
                wr32(gMali.maliReg.va, AS_REG(asSlot, AS_COMMAND), 3u);
            }
            /* Never leave a faulted or permanently SYNC_WAIT-deadlocked non-WS CSG wedged on hardware */
            if ((anyFault || allSyncWaitIdle) && !fIsWindowServer) {
                IOLockLock(gMali.schedLock);
                if (gMali.slots[csgSlot].ownerClient == this &&
                    gMali.slots[csgSlot].ownerGroupIdx == in->group_handle) {
                    evictHwSlotLocked(csgSlot, true);
                }
                IOLockUnlock(gMali.schedLock);
            }
        }

        out->signaled = done ? 1u : 0u;
        out->current_seq = done ? wantSeq : __atomic_load_n(&g.completed_seq, __ATOMIC_ACQUIRE);
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
    IOLog("Tb375fcMali: driver worker starting (mali=4 multi-process IOUserClient)\n");
    gMali.lock      = IOLockAlloc();
    gMali.schedLock = IOLockAlloc();
    gMali.wsLock    = IOLockAlloc();
    if (initMaliHardware()) {
        Tb375fcMali *svc = new Tb375fcMali;
        if (svc && svc->init()) {
            svc->attach(IOService::getPlatform());
            svc->start(IOService::getPlatform());
            IOLog("Tb375fcMali: IOService published (NativeSandboxGrantAll=true)!\n");
        }
    }
    (void)thread_terminate(current_thread());
}

/* Snapshot WindowServer CSG0's current submitted_seq at validateSwap() time so kickOnce()
 * waits ONLY for Frame N (never stalling on Frame N+1 submitted early under triple-buffering).
 * Uses dedicated gMali.wsLock (never blocked by non-WS LRU evictions on gMali.schedLock). */
extern "C" uint64_t tb375fc_mali_ws_submitted_seq(void);
extern "C" uint64_t tb375fc_mali_ws_submitted_seq(void) {
    if (!gMali.wsLock || !gMali.devInfo.mcu_alive) return 0ULL;
    uint64_t seq = 0ULL;
    IOLockLock(gMali.wsLock);
    Tb375fcMaliUserClient *owner = gMali.wsClient;
    uint32_t grpIdx = gMali.wsGroupIdx;
    if (owner && owner->fIsWindowServer && owner->fLock && owner->fGroups) {
        IOLockLock(owner->fLock);
        if (grpIdx > 0u && grpIdx < kMaxClientGroups &&
            owner->fGroups[grpIdx].in_use && owner->fGroups[grpIdx].hw_started) {
            seq = owner->fGroups[grpIdx].submitted_seq;
        }
        IOLockUnlock(owner->fLock);
    }
    IOLockUnlock(gMali.wsLock);
    return seq;
}

/* Synchronize NativeLegacyDisplay CPU blit with WindowServer's pinned Mali-G615 CSG0 queue.
 * Uses dedicated gMali.wsLock (never blocked by non-WS LRU evictions on gMali.schedLock). */
extern "C" uint32_t tb375fc_mali_wait_ws_seq(uint64_t wantSeq, uint32_t max_us);
extern "C" uint32_t tb375fc_mali_wait_ws_seq(uint64_t wantSeq, uint32_t max_us) {
    if (!gMali.wsLock || !gMali.devInfo.mcu_alive) return 0u;

    Tb375fcMaliUserClient *owner = nullptr;
    uint32_t qCount = 0u;
    uint32_t grpIdx = 0u;
    uint64_t targetSeq = wantSeq;
    uint64_t targets[MALI_MAX_QUEUES_PER_GROUP] = {};
    FwRingIoOutput *ioOuts[MALI_MAX_QUEUES_PER_GROUP] = {};

    IOLockLock(gMali.wsLock);
    if (gMali.wsClient && gMali.wsClient->fIsWindowServer &&
        gMali.wsClient->fLock && gMali.wsClient->fGroups) {
        owner = gMali.wsClient;
        grpIdx = gMali.wsGroupIdx;
        owner->retain();
        IOLockLock(owner->fLock);
        if (grpIdx > 0u && grpIdx < kMaxClientGroups &&
            owner->fGroups[grpIdx].in_use && owner->fGroups[grpIdx].hw_started) {
            MaliGroupEntry &g = owner->fGroups[grpIdx];
            if (targetSeq == 0ULL || targetSeq > g.submitted_seq) {
                targetSeq = g.submitted_seq;
            }
            if (targetSeq > 0ULL && __atomic_load_n(&g.completed_seq, __ATOMIC_ACQUIRE) < targetSeq) {
                qCount = g.queue_count;
                if (qCount > MALI_MAX_QUEUES_PER_GROUP) qCount = MALI_MAX_QUEUES_PER_GROUP;
                const MaliSeqTarget &st = g.seqHist[targetSeq % kSeqHistSize];
                const bool hasHist = (st.seq == targetSeq);
                for (uint32_t qi = 0u; qi < qCount; ++qi) {
                    targets[qi] = hasHist ? st.insert[qi] : g.ringInsert[qi];
                    ioOuts[qi]  = g.ringIoOutCpu[qi];
                }
            }
        }
        IOLockUnlock(owner->fLock);
    }
    IOLockUnlock(gMali.wsLock);

    if (!owner) return 0u;
    if (qCount == 0u || targetSeq == 0ULL) {
        owner->release();
        return 0u;
    }

    auto allDone = [&]() -> bool {
        if (__atomic_load_n(&owner->fGroups[grpIdx].completed_seq, __ATOMIC_ACQUIRE) >= targetSeq) {
            return true;
        }
        for (uint32_t qi = 0u; qi < qCount; ++qi) {
            if (!ioOuts[qi] || targets[qi] == 0u) continue;
            cacheFlushRange(ioOuts[qi], 64);
            if (ioOuts[qi]->extract < targets[qi]) return false;
        }
        return true;
    };

    if (allDone()) {
        uint64_t prev = __atomic_load_n(&owner->fGroups[grpIdx].completed_seq, __ATOMIC_RELAXED);
        while (targetSeq > prev && !__atomic_compare_exchange_n(&owner->fGroups[grpIdx].completed_seq,
                                                                 &prev, targetSeq, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {}
        owner->release();
        return 0u;
    }

    uint64_t t0 = 0, tNow = 0, ns0 = 0, nsNow = 0;
    clock_get_uptime(&t0);
    absolutetime_to_nanoseconds(t0, &ns0);
    const uint64_t deadlineNs = ns0 + static_cast<uint64_t>(max_us) * 1000ULL;

    bool done = false;
    /* Brief 150us spin then non-coalesced 200us sleep: avoids 1.5ms P-core spin AND 1.5-4ms IOSleep slack */
    uint32_t spinCap = (max_us < 150u) ? ((max_us + 14u) / 15u) : 10u;
    for (uint32_t s = 0u; s < spinCap; ++s) {
        IODelay(15);
        if (allDone()) { done = true; break; }
    }
    uint32_t sleeps = 0u;
    while (!done) {
        clock_get_uptime(&tNow);
        absolutetime_to_nanoseconds(tNow, &nsNow);
        if (nsNow >= deadlineNs) break;
        uint64_t remUs = (deadlineNs - nsNow) / 1000ULL;
        uint32_t stepUs = (++sleeps <= 50u) ? 200u : 500u;
        if (remUs < stepUs) stepUs = (remUs > 50u) ? static_cast<uint32_t>(remUs) : 50u;
        maliShortSleepUs(stepUs);
        if (allDone()) { done = true; break; }
    }

    if (!done) {
        if (checkAndRecoverMmuFaults("ws-sync") && allDone()) {
            done = true;
        }
    }

    if (done) {
        uint64_t prev = __atomic_load_n(&owner->fGroups[grpIdx].completed_seq, __ATOMIC_RELAXED);
        while (targetSeq > prev && !__atomic_compare_exchange_n(&owner->fGroups[grpIdx].completed_seq,
                                                                 &prev, targetSeq, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {}
    }

    clock_get_uptime(&tNow);
    absolutetime_to_nanoseconds(tNow, &nsNow);
    uint32_t waitedUs = (nsNow > ns0) ? static_cast<uint32_t>((nsNow - ns0) / 1000ULL) : 1u;

    owner->release();

    static uint32_t sSyncWaits = 0u;
    static uint32_t sSyncTimeouts = 0u;
    uint32_t n = ++sSyncWaits;
    if (!done) ++sSyncTimeouts;
    if (n <= 20u || (n % 500u) == 0u || (!done && (sSyncTimeouts <= 32u || (sSyncTimeouts % 32u) == 0u))) {
        if (!done && gMali.maliReg.va) {
            uint32_t mmuRaw = rd32(gMali.maliReg.va, MMU_INT_RAWSTAT);
            uint32_t as1Flt = rd32(gMali.maliReg.va, AS_REG(1u, AS_FAULTSTATUS));
            uint64_t as1Adr = static_cast<uint64_t>(rd32(gMali.maliReg.va, AS_REG(1u, AS_FAULTADDR_LO))) |
                              (static_cast<uint64_t>(rd32(gMali.maliReg.va, AS_REG(1u, AS_FAULTADDR_HI))) << 32);
            uint32_t shRdy  = rd32(gMali.maliReg.va, 0x0140u);
            IOLog("Tb375fcMali: ws-display-sync #%u waited=%uus done=0 seq=%llu timeouts=%u mmu=0x%x as1=0x%x@0x%llx shRdy=0x%x\n",
                  n, waitedUs, (unsigned long long)targetSeq, sSyncTimeouts, mmuRaw, as1Flt, (unsigned long long)as1Adr, shRdy);
        } else {
            IOLog("Tb375fcMali: ws-display-sync #%u waited=%uus done=%u seq=%llu timeouts=%u\n",
                  n, waitedUs, (unsigned)done, (unsigned long long)targetSeq, sSyncTimeouts);
        }
    }
    return waitedUs;
}

extern "C" uint32_t tb375fc_mali_wait_ws_idle(uint32_t max_us);
extern "C" uint32_t tb375fc_mali_wait_ws_idle(uint32_t max_us) {
    return tb375fc_mali_wait_ws_seq(0ULL, max_us);
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
