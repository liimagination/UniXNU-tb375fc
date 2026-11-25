/* SPDX-License-Identifier: MIT
 * Shared Kernel <-> User ABI for TB375FC / MT6897 Mali-G615 (Valhall CSF v10)
 * IOService: "Tb375fcMali"
 *
 * Used by:
 *   1. Kernel driver: ram-expansion/kernel/mali-bringup/Tb375fcMaliDriver.cpp
 *   2. Userland Mesa panvk Darwin shim: ram-expansion/mali-panvk/panthor_darwin_shim.c
 */
#ifndef TB375FC_MALI_ABI_H
#define TB375FC_MALI_ABI_H

#include <stdint.h>

#define TB375FC_MALI_SERVICE_NAME "Tb375fcMali"
#define TB375FC_MALI_ABI_VERSION  1u

enum Tb375fcMaliSelector {
    MALI_SEL_DEV_QUERY          = 0,  /* out: Tb375fcMaliDevInfo */
    MALI_SEL_VM_CREATE          = 1,  /* in:  Tb375fcMaliVmCreateIn, out: Tb375fcMaliVmCreateOut */
    MALI_SEL_VM_DESTROY         = 2,  /* in:  Tb375fcMaliHandleIn */
    MALI_SEL_VM_BIND            = 3,  /* in:  Tb375fcMaliVmBindIn */
    MALI_SEL_BO_CREATE          = 4,  /* in:  Tb375fcMaliBoCreateIn, out: Tb375fcMaliBoCreateOut */
    MALI_SEL_BO_DESTROY         = 5,  /* in:  Tb375fcMaliHandleIn */
    MALI_SEL_BO_IMPORT_HOST     = 6,  /* in:  Tb375fcMaliBoImportHostIn, out: Tb375fcMaliBoCreateOut */
    MALI_SEL_TILER_HEAP_CREATE  = 7,  /* in:  Tb375fcMaliTilerHeapCreateIn, out: Tb375fcMaliTilerHeapCreateOut */
    MALI_SEL_TILER_HEAP_DESTROY = 8,  /* in:  Tb375fcMaliHandleIn */
    MALI_SEL_GROUP_CREATE       = 9,  /* in:  Tb375fcMaliGroupCreateIn, out: Tb375fcMaliGroupCreateOut */
    MALI_SEL_GROUP_DESTROY      = 10, /* in:  Tb375fcMaliHandleIn */
    MALI_SEL_GROUP_SUBMIT       = 11, /* in:  Tb375fcMaliGroupSubmitIn, out: Tb375fcMaliGroupSubmitOut */
    MALI_SEL_FENCE_WAIT         = 12, /* in:  Tb375fcMaliFenceWaitIn, out: Tb375fcMaliFenceWaitOut */
    MALI_SEL_GROUP_GET_STATE    = 13, /* in:  Tb375fcMaliHandleIn, out: Tb375fcMaliGroupStateOut */
    MALI_SEL_COUNT              = 14
};

/* Mirrors Linux drm_panthor_gpu_info + drm_panthor_csif_info + drm_panthor_timestamp_info */
struct Tb375fcMaliDevInfo {
    uint32_t abi_version;
    uint32_t pad0;
    /* GPU info */
    uint32_t gpu_id;
    uint32_t gpu_rev;
    uint32_t csf_id;
    uint32_t l2_features;
    uint32_t tiler_features;
    uint32_t mem_features;
    uint32_t mmu_features;
    uint32_t thread_features;
    uint32_t max_threads;
    uint32_t thread_max_workgroup_size;
    uint32_t thread_max_barrier_size;
    uint32_t coherency_features;
    uint32_t texture_features[4];
    uint32_t as_present;
    uint32_t selected_coherency;
    uint64_t shader_present;
    uint64_t tiler_present;
    uint64_t l2_present;
    /* CSIF info */
    uint32_t csg_slot_count;
    uint32_t cs_slot_count;
    uint32_t cs_reg_count;
    uint32_t scoreboard_slot_count;
    uint32_t unpreserved_cs_reg_count;
    uint32_t csif_pad;
    /* Timestamp info */
    uint64_t timestamp_frequency;
    uint64_t current_timestamp;
    uint64_t timestamp_offset;
    /* Firmware info */
    uint32_t fw_iface_ver;
    uint32_t mcu_alive;
};

struct Tb375fcMaliHandleIn {
    uint32_t handle;
    uint32_t pad;
};

struct Tb375fcMaliVmCreateIn {
    uint32_t flags;
    uint32_t pad;
    uint64_t user_va_range;
};

struct Tb375fcMaliVmCreateOut {
    uint32_t vm_id;
    uint32_t as_slot;
    uint64_t va_start;
    uint64_t va_size;
};

enum Tb375fcMaliVmBindOp {
    MALI_VM_BIND_MAP   = 0,
    MALI_VM_BIND_UNMAP = 1,
};

#define MALI_VM_MAP_READONLY  (1u << 0)
#define MALI_VM_MAP_NOEXEC    (1u << 1)
#define MALI_VM_MAP_UNCACHED  (1u << 2)

struct Tb375fcMaliVmBindIn {
    uint32_t vm_id;
    uint32_t op;         /* Tb375fcMaliVmBindOp */
    uint32_t bo_handle;  /* ignored for UNMAP */
    uint32_t flags;      /* MALI_VM_MAP_* */
    uint64_t bo_offset;
    uint64_t va;
    uint64_t size;
};

#define MALI_BO_NO_MMAP (1u << 0)

struct Tb375fcMaliBoCreateIn {
    uint64_t size;
    uint32_t flags;
    uint32_t exclusive_vm_id;
};

struct Tb375fcMaliBoCreateOut {
    uint32_t bo_handle;  /* also memoryType for IOConnectMapMemory64 */
    uint32_t pad;
    uint64_t size;
};

struct Tb375fcMaliBoImportHostIn {
    uint64_t host_ptr;
    uint64_t size;
    uint32_t flags;
    uint32_t exclusive_vm_id;
};

struct Tb375fcMaliTilerHeapCreateIn {
    uint32_t vm_id;
    uint32_t initial_chunk_count;
    uint32_t chunk_size;
    uint32_t max_chunks;
    uint32_t target_in_flight;
    uint32_t pad;
};

struct Tb375fcMaliTilerHeapCreateOut {
    uint32_t handle;
    uint32_t pad;
    uint64_t tiler_heap_ctx_gpu_va;
    uint64_t first_heap_chunk_gpu_va;
};

#define MALI_MAX_QUEUES_PER_GROUP 8u

struct Tb375fcMaliQueueCreateDesc {
    uint8_t  priority;
    uint8_t  pad[3];
    uint32_t ringbuf_size;
};

struct Tb375fcMaliGroupCreateIn {
    uint32_t vm_id;
    uint32_t max_compute_cores;
    uint32_t max_fragment_cores;
    uint32_t max_tiler_cores;
    uint32_t priority;
    uint32_t queue_count;
    uint64_t compute_core_mask;
    uint64_t fragment_core_mask;
    uint64_t tiler_core_mask;
    struct Tb375fcMaliQueueCreateDesc queues[MALI_MAX_QUEUES_PER_GROUP];
};

struct Tb375fcMaliGroupCreateOut {
    uint32_t group_handle;
    uint32_t csg_slot;
};

struct Tb375fcMaliQueueSubmitDesc {
    uint32_t queue_index;
    uint32_t stream_size;
    uint64_t stream_addr;  /* GPU VA of command stream to CALL */
    uint32_t latest_flush;
    uint32_t pad;
};

#define MALI_MAX_SUBMITS_PER_CALL 8u

struct Tb375fcMaliGroupSubmitIn {
    uint32_t group_handle;
    uint32_t submit_count;
    struct Tb375fcMaliQueueSubmitDesc submits[MALI_MAX_SUBMITS_PER_CALL];
};

struct Tb375fcMaliGroupSubmitOut {
    uint64_t fence_seq;
};

struct Tb375fcMaliFenceWaitIn {
    uint32_t group_handle;
    uint32_t pad;
    uint64_t fence_seq;
    uint64_t timeout_ns;
};

struct Tb375fcMaliFenceWaitOut {
    uint32_t signaled;
    uint32_t pad;
    uint64_t current_seq;
};

struct Tb375fcMaliGroupStateOut {
    uint32_t state;       /* 0 = healthy, bit0 = timedout, bit1 = fatal */
    uint32_t fatal_queues;
};

#endif /* TB375FC_MALI_ABI_H */
