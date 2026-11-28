/* SPDX-License-Identifier: MIT
 * Darwin libdrm + panthor IOKit shim for Mesa panvk on TB375FC (MT6897 Mali-G615).
 * Translates Mesa panvk's DRM_IOCTL_PANTHOR_* ioctls, BO mmaps, and timeline syncobjs
 * into IOKit IOUserClient calls against the kernel "Tb375fcMali" service.
 */
#include "xf86drm.h"
#include "drm-uapi/panthor_drm.h"
#include "Tb375fcMaliABI.h"

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <libkern/OSCacheControl.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define MAX_DARWIN_FDS             32
#define MAX_DARWIN_SYNCOBJ         4096
#define MAX_DARWIN_SYNCOBJ_PENDING 128
#define MAX_DARWIN_HOST_BOS        16384

#ifndef DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL        (1u << 0)
#endif
#ifndef DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT (1u << 1)
#endif
#ifndef DRM_SYNCOBJ_WAIT_FLAGS_WAIT_AVAILABLE
#define DRM_SYNCOBJ_WAIT_FLAGS_WAIT_AVAILABLE  (1u << 2)
#endif
#ifndef DRM_SYNCOBJ_QUERY_FLAGS_LAST_SUBMITTED
#define DRM_SYNCOBJ_QUERY_FLAGS_LAST_SUBMITTED (1u << 0)
#endif

struct darwin_host_bo {
    void  *ptr;
    size_t size;
};

static struct darwin_host_bo g_host_bos[MAX_DARWIN_HOST_BOS];
static struct darwin_host_bo g_mmap_bos[MAX_DARWIN_HOST_BOS];
static struct darwin_host_bo g_anon_bos[MAX_DARWIN_HOST_BOS];
static struct darwin_host_bo g_wc_zero_bos[MAX_DARWIN_HOST_BOS];
static bool                  g_mmap_cached[MAX_DARWIN_HOST_BOS];
static uint16_t              g_anon_map_cnt[MAX_DARWIN_HOST_BOS];
static uint32_t              g_max_bo_handle = 0;

/* Compact O(N_active) active-handle lists so flush_all_host_bos() never scans thousands
 * of empty/WriteCombine BO slots on every vkQueueSubmit! (pos == 0 means not in list.) */
static uint32_t              g_active_host_list[MAX_DARWIN_HOST_BOS];
static uint32_t              g_active_host_pos[MAX_DARWIN_HOST_BOS];
static uint32_t              g_active_host_cnt = 0;

static uint32_t              g_active_cached_list[MAX_DARWIN_HOST_BOS];
static uint32_t              g_active_cached_pos[MAX_DARWIN_HOST_BOS];
static uint32_t              g_active_cached_cnt = 0;

static inline void active_list_add(uint32_t *list, uint32_t *pos, uint32_t *cnt, uint32_t h) {
    if (h == 0 || h >= MAX_DARWIN_HOST_BOS || pos[h] != 0) return;
    uint32_t idx = (*cnt)++;
    list[idx] = h;
    pos[h] = idx + 1u;
}

static inline void active_list_remove(uint32_t *list, uint32_t *pos, uint32_t *cnt, uint32_t h) {
    if (h == 0 || h >= MAX_DARWIN_HOST_BOS || pos[h] == 0) return;
    uint32_t idx = pos[h] - 1u;
    uint32_t last = --(*cnt);
    if (idx != last) {
        uint32_t moved = list[last];
        list[idx] = moved;
        pos[moved] = idx + 1u;
    }
    pos[h] = 0;
}

static inline int64_t now_mono_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

static inline void note_bo_handle_locked(uint32_t h) {
    if (h > g_max_bo_handle && h < MAX_DARWIN_HOST_BOS) g_max_bo_handle = h;
}

static inline void arm64_dcache_clean_nodsb(const void *addr, size_t len) {
    if (!addr || !len) return;
    uintptr_t p = (uintptr_t)addr & ~63ULL;
    uintptr_t end = ((uintptr_t)addr + len + 63ULL) & ~63ULL;
    for (; p < end; p += 64ULL) {
        __asm__ volatile("dc cvac, %0" :: "r"(p) : "memory");
    }
}

static inline void arm64_dcache_clean_inval_nodsb(const void *addr, size_t len) {
    if (!addr || !len) return;
    uintptr_t p = (uintptr_t)addr & ~63ULL;
    uintptr_t end = ((uintptr_t)addr + len + 63ULL) & ~63ULL;
    for (; p < end; p += 64ULL) {
        __asm__ volatile("dc civac, %0" :: "r"(p) : "memory");
    }
}

static inline void arm64_dcache_clean(const void *addr, size_t len) {
    if (!addr || !len) return;
    arm64_dcache_clean_nodsb(addr, len);
    __asm__ volatile("dsb sy" ::: "memory");
}

static inline void arm64_dcache_clean_inval(const void *addr, size_t len) {
    if (!addr || !len) return;
    arm64_dcache_clean_inval_nodsb(addr, len);
    __asm__ volatile("dsb sy" ::: "memory");
}

static void flush_all_host_bos(bool invalidate, size_t *out_hkb, size_t *out_mkb) {
    size_t hbytes = 0, mbytes = 0;
    if (g_active_host_cnt == 0 && g_active_cached_cnt == 0) {
        if (out_hkb) *out_hkb = 0;
        if (out_mkb) *out_mkb = 0;
        return;
    }
    for (uint32_t k = 0; k < g_active_host_cnt; ++k) {
        uint32_t i = g_active_host_list[k];
        void *p = g_host_bos[i].ptr;
        size_t sz = g_host_bos[i].size;
        if (p && sz > 0 && sz < (2ULL << 20)) {
            hbytes += sz;
            if (invalidate) arm64_dcache_clean_inval_nodsb(p, sz);
            else            arm64_dcache_clean_nodsb(p, sz);
        }
    }
    for (uint32_t k = 0; k < g_active_cached_cnt; ++k) {
        uint32_t i = g_active_cached_list[k];
        void *p = g_mmap_bos[i].ptr;
        size_t sz = g_mmap_bos[i].size;
        if (p && g_mmap_cached[i] && sz > 0 && sz <= (128ULL << 20)) {
            mbytes += sz;
            if (invalidate) arm64_dcache_clean_inval_nodsb(p, sz);
            else            arm64_dcache_clean_nodsb(p, sz);
        }
    }
    if (hbytes > 0 || mbytes > 0) {
        __asm__ volatile("dsb sy" ::: "memory");
    }
    if (out_hkb) *out_hkb = hbytes >> 10;
    if (out_mkb) *out_mkb = mbytes >> 10;
}

struct darwin_mali_fd {
    int           fd;
    io_connect_t  conn;
    bool          in_use;
    struct Tb375fcMaliDevInfo cached_info;
    bool          info_valid;
};

struct darwin_sync_pending {
    uint64_t point;
    uint32_t group;
    uint64_t fence_seq;
    bool     done;
};

struct darwin_syncobj {
    bool     in_use;
    uint64_t signaled_point;
    uint32_t pending_count;
    struct darwin_sync_pending pending[MAX_DARWIN_SYNCOBJ_PENDING];
};

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_bo_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_sync_cond = PTHREAD_COND_INITIALIZER;
static struct darwin_mali_fd g_fds[MAX_DARWIN_FDS];
static struct darwin_syncobj g_syncs[MAX_DARWIN_SYNCOBJ];

static struct darwin_mali_fd *find_fd_locked(int fd) {
    for (int i = 0; i < MAX_DARWIN_FDS; ++i) {
        if (g_fds[i].in_use && g_fds[i].fd == fd) return &g_fds[i];
    }
    /* Fallback for dup()'d fds */
    for (int i = 0; i < MAX_DARWIN_FDS; ++i) {
        if (g_fds[i].in_use) return &g_fds[i];
    }
    return NULL;
}

static io_connect_t fd_to_conn(int fd) {
    pthread_mutex_lock(&g_lock);
    struct darwin_mali_fd *e = find_fd_locked(fd);
    io_connect_t c = e ? e->conn : MACH_PORT_NULL;
    pthread_mutex_unlock(&g_lock);
    return c;
}

int panthor_darwin_open_render_node(void) {
    CFMutableDictionaryRef match = IOServiceMatching(TB375FC_MALI_SERVICE_NAME);
    if (!match) { errno = ENODEV; return -1; }
    io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, match);
    if (svc == IO_OBJECT_NULL) {
        errno = ENODEV;
        return -1;
    }
    io_connect_t conn = MACH_PORT_NULL;
    kern_return_t kr = IOServiceOpen(svc, mach_task_self(), 0, &conn);
    IOObjectRelease(svc);
    if (kr != KERN_SUCCESS || conn == MACH_PORT_NULL) {
        errno = EACCES;
        return -1;
    }

    int dummy_fd = open("/dev/null", O_RDWR | O_CLOEXEC);
    if (dummy_fd < 0) dummy_fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
    if (dummy_fd < 0) {
        int pfd[2] = {-1, -1};
        if (pipe(pfd) == 0) {
            close(pfd[1]);
            dummy_fd = pfd[0];
            fcntl(dummy_fd, F_SETFD, FD_CLOEXEC);
        }
    }
    if (dummy_fd < 0) {
        IOServiceClose(conn);
        return -1;
    }

    pthread_mutex_lock(&g_lock);
    int slot = -1;
    for (int i = 0; i < MAX_DARWIN_FDS; ++i) {
        if (!g_fds[i].in_use) { slot = i; break; }
    }
    if (slot < 0) {
        pthread_mutex_unlock(&g_lock);
        close(dummy_fd);
        IOServiceClose(conn);
        errno = EMFILE;
        return -1;
    }
    memset(&g_fds[slot], 0, sizeof(g_fds[slot]));
    g_fds[slot].fd = dummy_fd;
    g_fds[slot].conn = conn;
    g_fds[slot].in_use = true;
    pthread_mutex_unlock(&g_lock);
    return dummy_fd;
}

void panthor_darwin_close_render_node(int fd) {
    pthread_mutex_lock(&g_lock);
    struct darwin_mali_fd *e = find_fd_locked(fd);
    io_connect_t conn = MACH_PORT_NULL;
    if (e) {
        conn = e->conn;
        e->in_use = false;
        e->conn = MACH_PORT_NULL;
        e->fd = -1;
    }
    pthread_mutex_unlock(&g_lock);
    if (conn != MACH_PORT_NULL) IOServiceClose(conn);
    if (fd >= 0) close(fd);
}

void *panthor_darwin_mmap_bo(int fd, uint64_t mmap_offset, size_t size) {
    io_connect_t conn = fd_to_conn(fd);
    if (conn == MACH_PORT_NULL) { errno = EBADF; return (void *)-1; }
    uint32_t bo_handle = (uint32_t)(mmap_offset >> 12);
    /* For anon-mmap BOs (imported via BO_IMPORT_HOST), return the single Normal-WB mapping
     * directly: zero IOConnectMapMemory64 double-mapping, zero ARM64 WB/NC cache aliasing!
     * Only register in g_mmap_bos[] once CPU actually maps it, so unmapped Private GPU
     * textures never pay CPU dc civac overhead on submit. */
    if (bo_handle > 0 && bo_handle < MAX_DARWIN_HOST_BOS) {
        pthread_mutex_lock(&g_bo_lock);
        void *ap = g_anon_bos[bo_handle].ptr;
        size_t asz = g_anon_bos[bo_handle].size;
        if (ap) {
            g_mmap_bos[bo_handle].ptr = ap;
            g_mmap_bos[bo_handle].size = asz;
            g_mmap_cached[bo_handle] = true;
            g_anon_map_cnt[bo_handle]++;
            active_list_add(g_active_cached_list, g_active_cached_pos, &g_active_cached_cnt, bo_handle);
            note_bo_handle_locked(bo_handle);
        }
        pthread_mutex_unlock(&g_bo_lock);
        if (ap) return ap;
    }
    mach_vm_address_t addr = 0;
    mach_vm_size_t map_size = size;
    kern_return_t kr = IOConnectMapMemory64(conn, bo_handle, mach_task_self(),
                                            &addr, &map_size,
                                            kIOMapAnywhere | kIOMapWriteCombineCache);
    if (kr != KERN_SUCCESS || !addr) {
        errno = ENOMEM;
        return (void *)-1;
    }
    if (bo_handle > 0 && bo_handle < MAX_DARWIN_HOST_BOS) {
        pthread_mutex_lock(&g_bo_lock);
        g_mmap_bos[bo_handle].ptr = (void *)(uintptr_t)addr;
        g_mmap_bos[bo_handle].size = (size_t)map_size;
        if (g_mmap_cached[bo_handle]) {
            active_list_add(g_active_cached_list, g_active_cached_pos, &g_active_cached_cnt, bo_handle);
        }
        note_bo_handle_locked(bo_handle);
        pthread_mutex_unlock(&g_bo_lock);
    }
    return (void *)(uintptr_t)addr;
}

int panthor_darwin_munmap_bo(int fd, void *addr, size_t size) {
    io_connect_t conn = fd_to_conn(fd);
    if (conn == MACH_PORT_NULL || !addr) return 0;
    uint32_t found_handle = 0;
    bool is_anon = false;
    pthread_mutex_lock(&g_bo_lock);
    uint32_t max_h = g_max_bo_handle;
    if (max_h >= MAX_DARWIN_HOST_BOS) max_h = MAX_DARWIN_HOST_BOS - 1;
    for (uint32_t i = 1; i <= max_h; ++i) {
        if (g_anon_bos[i].ptr == addr) {
            is_anon = true;
            found_handle = i;
            if (g_anon_map_cnt[i] > 0) g_anon_map_cnt[i]--;
            if (g_anon_map_cnt[i] == 0) {
                g_mmap_bos[i].ptr = NULL;
                g_mmap_bos[i].size = 0;
                g_mmap_cached[i] = false;
                active_list_remove(g_active_cached_list, g_active_cached_pos, &g_active_cached_cnt, i);
            }
            break;
        }
        if (g_mmap_bos[i].ptr == addr) {
            found_handle = i;
            g_mmap_bos[i].ptr = NULL;
            g_mmap_bos[i].size = 0;
            active_list_remove(g_active_cached_list, g_active_cached_pos, &g_active_cached_cnt, i);
            break;
        }
    }
    pthread_mutex_unlock(&g_bo_lock);
    if (is_anon) {
        /* Keep anon mmap alive until DRM_IOCTL_GEM_CLOSE unwires it via MALI_SEL_BO_DESTROY */
        if (size > 0 && size <= (128ULL << 20)) arm64_dcache_clean_inval(addr, size);
        return 0;
    }
    if (size > 0 && size <= (128ULL << 20)) {
        arm64_dcache_clean(addr, size);
    }
    (void)IOConnectUnmapMemory64(conn, found_handle, mach_task_self(), (mach_vm_address_t)(uintptr_t)addr);
    return 0;
}

void panthor_darwin_zero_bo_range(int fd, uint32_t bo_handle, uint64_t offset, uint64_t size) {
    if (size == 0 || bo_handle == 0 || bo_handle >= MAX_DARWIN_HOST_BOS) return;
    io_connect_t conn = fd_to_conn(fd);
    if (conn == MACH_PORT_NULL) return;

    void *ptr = NULL;
    size_t bo_sz = 0;
    bool need_dcache_clean = false;

    pthread_mutex_lock(&g_bo_lock);
    if (g_anon_bos[bo_handle].ptr) {
        ptr = g_anon_bos[bo_handle].ptr;
        bo_sz = g_anon_bos[bo_handle].size;
        need_dcache_clean = true;
    } else if (g_host_bos[bo_handle].ptr) {
        ptr = g_host_bos[bo_handle].ptr;
        bo_sz = g_host_bos[bo_handle].size;
        need_dcache_clean = true;
    } else if (g_mmap_bos[bo_handle].ptr) {
        ptr = g_mmap_bos[bo_handle].ptr;
        bo_sz = g_mmap_bos[bo_handle].size;
        need_dcache_clean = g_mmap_cached[bo_handle];
    } else if (g_wc_zero_bos[bo_handle].ptr) {
        ptr = g_wc_zero_bos[bo_handle].ptr;
        bo_sz = g_wc_zero_bos[bo_handle].size;
        need_dcache_clean = g_mmap_cached[bo_handle];
    }
    pthread_mutex_unlock(&g_bo_lock);

    if (!ptr) {
        /* Lazily map WriteCombine BO once for fast zeroing of recycled AFBC superblock headers
         * without adding unmapped private GPU texture slabs to g_active_cached_list. */
        mach_vm_address_t addr = 0;
        mach_vm_size_t map_size = 0;
        kern_return_t kr = IOConnectMapMemory64(conn, bo_handle, mach_task_self(),
                                                &addr, &map_size,
                                                kIOMapAnywhere | kIOMapWriteCombineCache);
        if (kr != KERN_SUCCESS || !addr) return;
        bool drop_dup = false;
        pthread_mutex_lock(&g_bo_lock);
        if (!g_wc_zero_bos[bo_handle].ptr) {
            g_wc_zero_bos[bo_handle].ptr = (void *)(uintptr_t)addr;
            g_wc_zero_bos[bo_handle].size = (size_t)map_size;
            note_bo_handle_locked(bo_handle);
        } else {
            drop_dup = true;
        }
        ptr = g_wc_zero_bos[bo_handle].ptr;
        bo_sz = g_wc_zero_bos[bo_handle].size;
        need_dcache_clean = g_mmap_cached[bo_handle];
        pthread_mutex_unlock(&g_bo_lock);
        if (drop_dup) {
            (void)IOConnectUnmapMemory64(conn, bo_handle, mach_task_self(), addr);
        }
    }

    if (!ptr || offset >= bo_sz) return;
    uint64_t avail = bo_sz - offset;
    size_t len = (size > avail) ? (size_t)avail : (size_t)size;
    void *dst = (uint8_t *)ptr + offset;
    memset(dst, 0, len);
    if (need_dcache_clean) {
        arm64_dcache_clean(dst, len);
    } else {
        __asm__ volatile("dsb sy" ::: "memory");
    }
}

static int call_mali(io_connect_t conn, uint32_t sel,
                     const void *in_buf, size_t in_sz,
                     void *out_buf, size_t *out_sz) {
    size_t osz = out_sz ? *out_sz : 0;
    kern_return_t kr = IOConnectCallStructMethod(conn, sel, in_buf, in_sz, out_buf, out_sz ? &osz : NULL);
    if (out_sz) *out_sz = osz;
    if (kr == KERN_SUCCESS) return 0;
    if (kr == kIOReturnNoMemory) errno = ENOMEM;
    else if (kr == kIOReturnTimeout) errno = ETIMEDOUT;
    else if (kr == kIOReturnBadArgument) errno = EINVAL;
    else errno = EIO;
    return -1;
}

int panthor_darwin_import_host_handle(int fd, void *host_ptr, uint64_t size,
                                      uint32_t *out_handle, uint64_t *out_size);
int panthor_darwin_import_host_handle(int fd, void *host_ptr, uint64_t size,
                                      uint32_t *out_handle, uint64_t *out_size) {
    io_connect_t conn = fd_to_conn(fd);
    if (conn == MACH_PORT_NULL) { errno = EBADF; return -1; }
    if (host_ptr && size > 0) arm64_dcache_clean(host_ptr, (size_t)size);
    struct Tb375fcMaliBoImportHostIn in;
    memset(&in, 0, sizeof(in));
    in.host_ptr = (uint64_t)(uintptr_t)host_ptr;
    in.size = size;
    struct Tb375fcMaliBoCreateOut out = {0};
    size_t osz = sizeof(out);
    if (call_mali(conn, MALI_SEL_BO_IMPORT_HOST, &in, sizeof(in), &out, &osz) != 0)
        return -1;
    if (out.bo_handle > 0 && out.bo_handle < MAX_DARWIN_HOST_BOS) {
        pthread_mutex_lock(&g_bo_lock);
        g_host_bos[out.bo_handle].ptr = host_ptr;
        g_host_bos[out.bo_handle].size = (size_t)out.size;
        if (host_ptr && out.size > 0 && out.size < (2ULL << 20)) {
            active_list_add(g_active_host_list, g_active_host_pos, &g_active_host_cnt, out.bo_handle);
        }
        note_bo_handle_locked(out.bo_handle);
        pthread_mutex_unlock(&g_bo_lock);
    }
    if (out_handle) *out_handle = out.bo_handle;
    if (out_size) *out_size = out.size;
    return 0;
}

static int ensure_dev_info(int fd, struct Tb375fcMaliDevInfo *out) {
    pthread_mutex_lock(&g_lock);
    struct darwin_mali_fd *e = find_fd_locked(fd);
    if (!e) { pthread_mutex_unlock(&g_lock); errno = EBADF; return -1; }
    if (e->info_valid) {
        *out = e->cached_info;
        pthread_mutex_unlock(&g_lock);
        return 0;
    }
    io_connect_t conn = e->conn;
    pthread_mutex_unlock(&g_lock);

    struct Tb375fcMaliDevInfo info;
    memset(&info, 0, sizeof(info));
    size_t osz = sizeof(info);
    if (call_mali(conn, MALI_SEL_DEV_QUERY, NULL, 0, &info, &osz) != 0)
        return -1;

    pthread_mutex_lock(&g_lock);
    e = find_fd_locked(fd);
    if (e) { e->cached_info = info; e->info_valid = true; }
    pthread_mutex_unlock(&g_lock);
    *out = info;
    return 0;
}

/* ---- Timeline Syncobj engine ---- */

static void syncobj_prune_locked(struct darwin_syncobj *s) {
    uint32_t w = 0;
    for (uint32_t i = 0; i < s->pending_count; ++i) {
        if (s->pending[i].point > s->signaled_point) {
            if (w != i) s->pending[w] = s->pending[i];
            w++;
        }
    }
    s->pending_count = w;
}

static void syncobj_add_pending_locked(struct darwin_syncobj *s, uint64_t point,
                                       uint32_t group, uint64_t fence_seq) {
    if (point <= s->signaled_point) return;
    for (uint32_t i = 0; i < s->pending_count; ++i) {
        if (s->pending[i].point == point && s->pending[i].group == group) {
            if (fence_seq > s->pending[i].fence_seq) s->pending[i].fence_seq = fence_seq;
            s->pending[i].done = false;
            return;
        }
    }
    if (s->pending_count < MAX_DARWIN_SYNCOBJ_PENDING) {
        struct darwin_sync_pending *e = &s->pending[s->pending_count++];
        e->point = point;
        e->group = group;
        e->fence_seq = fence_seq;
        e->done = false;
        return;
    }
    /* Overflow fallback: replace earliest-point entry on same group (or slot 0) */
    uint32_t victim = 0;
    for (uint32_t i = 1; i < s->pending_count; ++i) {
        if (s->pending[i].point < s->pending[victim].point) victim = i;
    }
    s->pending[victim].point = point;
    s->pending[victim].group = group;
    s->pending[victim].fence_seq = fence_seq;
    s->pending[victim].done = false;
}

static void syncobj_mark_group_seq_done_locked(uint32_t group, uint64_t completed_seq) {
    for (uint32_t h = 1; h < MAX_DARWIN_SYNCOBJ; ++h) {
        struct darwin_syncobj *s = &g_syncs[h];
        if (!s->in_use || s->pending_count == 0) continue;
        bool touched = false;
        for (uint32_t i = 0; i < s->pending_count; ++i) {
            if (!s->pending[i].done && s->pending[i].group == group &&
                s->pending[i].fence_seq <= completed_seq) {
                s->pending[i].done = true;
                touched = true;
            }
        }
        if (!touched) continue;
        for (uint32_t i = 0; i < s->pending_count; ++i) {
            uint64_t pt = s->pending[i].point;
            if (pt <= s->signaled_point) continue;
            bool all_done = true;
            for (uint32_t j = 0; j < s->pending_count; ++j) {
                if (s->pending[j].point == pt && !s->pending[j].done) {
                    all_done = false;
                    break;
                }
            }
            if (all_done && pt > s->signaled_point) s->signaled_point = pt;
        }
        syncobj_prune_locked(s);
    }
}

static bool syncobj_has_point_available_locked(const struct darwin_syncobj *s, uint64_t target) {
    if (s->signaled_point >= target) return true;
    for (uint32_t i = 0; i < s->pending_count; ++i) {
        if (s->pending[i].point >= target) return true;
    }
    return false;
}

static uint64_t syncobj_max_submitted_locked(const struct darwin_syncobj *s) {
    uint64_t m = s->signaled_point;
    for (uint32_t i = 0; i < s->pending_count; ++i) {
        if (s->pending[i].point > m) m = s->pending[i].point;
    }
    return m;
}

static bool syncobj_find_wait_fence_locked(const struct darwin_syncobj *s, uint64_t target,
                                           uint32_t *out_group, uint64_t *out_seq) {
    bool found = false;
    uint64_t best_pt = UINT64_MAX;
    uint32_t best_grp = 0;
    uint64_t best_seq = 0;
    for (uint32_t i = 0; i < s->pending_count; ++i) {
        const struct darwin_sync_pending *e = &s->pending[i];
        if (!e->done && e->point >= target && (!found || e->point < best_pt)) {
            found = true;
            best_pt = e->point;
            best_grp = e->group;
            best_seq = e->fence_seq;
        }
    }
    if (found) {
        if (out_group) *out_group = best_grp;
        if (out_seq) *out_seq = best_seq;
    }
    return found;
}

static int wait_group_fence(io_connect_t conn, uint32_t group, uint64_t fence_seq, int64_t rel_timeout_ns) {
    struct Tb375fcMaliFenceWaitIn in = {
        .group_handle = group,
        .pad = 0,
        .fence_seq = fence_seq,
        .timeout_ns = (rel_timeout_ns < 0) ? 10000000000ULL : (uint64_t)rel_timeout_ns,
    };
    struct Tb375fcMaliFenceWaitOut out = {0};
    size_t osz = sizeof(out);
    struct timespec fw0, fw1;
    clock_gettime(CLOCK_MONOTONIC, &fw0);
    int rc = call_mali(conn, MALI_SEL_FENCE_WAIT, &in, sizeof(in), &out, &osz);
    clock_gettime(CLOCK_MONOTONIC, &fw1);
    static uint32_t sFwLog = 0;
    if (rel_timeout_ns != 0 && ++sFwLog <= 12) {
        long wUs = (fw1.tv_sec - fw0.tv_sec) * 1000000L + (fw1.tv_nsec - fw0.tv_nsec) / 1000L;
        struct Tb375fcMaliDebugLogIn dlog;
        memset(&dlog, 0, sizeof(dlog));
        snprintf(dlog.msg, sizeof(dlog.msg) - 1, "[panvk:%s] fence #%u wait=%ldus rc=%d sig=%u",
                 getprogname(), sFwLog, wUs, rc, out.signaled);
        (void)call_mali(conn, MALI_SEL_DEBUG_LOG, &dlog, sizeof(dlog), NULL, NULL);
    }
    if (rc == 0 && out.signaled) {
        return 0;
    }
    errno = ETIME;
    return -1;
}

int drmSyncobjCreate(int fd, uint32_t flags, uint32_t *handle) {
    (void)fd;
    if (!handle) { errno = EINVAL; return -1; }
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 1; i < MAX_DARWIN_SYNCOBJ; ++i) {
        if (!g_syncs[i].in_use) {
            memset(&g_syncs[i], 0, sizeof(g_syncs[i]));
            g_syncs[i].in_use = true;
            if (flags & DRM_SYNCOBJ_CREATE_SIGNALED)
                g_syncs[i].signaled_point = 1;
            *handle = i;
            pthread_mutex_unlock(&g_lock);
            return 0;
        }
    }
    pthread_mutex_unlock(&g_lock);
    errno = ENOMEM;
    return -1;
}

int drmSyncobjDestroy(int fd, uint32_t handle) {
    (void)fd;
    pthread_mutex_lock(&g_lock);
    if (handle > 0 && handle < MAX_DARWIN_SYNCOBJ) {
        g_syncs[handle].in_use = false;
        g_syncs[handle].pending_count = 0;
        g_syncs[handle].signaled_point = 0;
        pthread_cond_broadcast(&g_sync_cond);
    }
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjReset(int fd, const uint32_t *handles, uint32_t handle_count) {
    (void)fd;
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 0; i < handle_count; ++i) {
        uint32_t h = handles[i];
        if (h > 0 && h < MAX_DARWIN_SYNCOBJ && g_syncs[h].in_use) {
            g_syncs[h].signaled_point = 0;
            g_syncs[h].pending_count = 0;
        }
    }
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjSignal(int fd, const uint32_t *handles, uint32_t handle_count) {
    (void)fd;
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 0; i < handle_count; ++i) {
        uint32_t h = handles[i];
        if (h > 0 && h < MAX_DARWIN_SYNCOBJ && g_syncs[h].in_use) {
            if (g_syncs[h].signaled_point == 0) g_syncs[h].signaled_point = 1;
            g_syncs[h].pending_count = 0;
        }
    }
    pthread_cond_broadcast(&g_sync_cond);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjTimelineSignal(int fd, const uint32_t *handles, uint64_t *points, uint32_t handle_count) {
    (void)fd;
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 0; i < handle_count; ++i) {
        uint32_t h = handles[i];
        if (h > 0 && h < MAX_DARWIN_SYNCOBJ && g_syncs[h].in_use) {
            if (points[i] > g_syncs[h].signaled_point)
                g_syncs[h].signaled_point = points[i];
            syncobj_prune_locked(&g_syncs[h]);
        }
    }
    pthread_cond_broadcast(&g_sync_cond);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjQuery2(int fd, uint32_t *handles, uint64_t *points, uint32_t handle_count, uint32_t flags) {
    io_connect_t conn = fd_to_conn(fd);
    bool last_submitted = (flags & DRM_SYNCOBJ_QUERY_FLAGS_LAST_SUBMITTED) != 0;
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 0; i < handle_count; ++i) {
        uint32_t h = handles[i];
        if (h == 0 || h >= MAX_DARWIN_SYNCOBJ || !g_syncs[h].in_use) {
            points[i] = 0;
            continue;
        }
        if (last_submitted) {
            points[i] = syncobj_max_submitted_locked(&g_syncs[h]);
            continue;
        }
        if (g_syncs[h].pending_count > 0 && conn != MACH_PORT_NULL) {
            /* Fast non-blocking poll: check highest pending fence first (resolves whole pipeline in 1 call),
             * then lowest pending point if pipeline still has in-flight tail. */
            uint32_t hi_grp = 0, lo_grp = 0;
            uint64_t hi_seq = 0, lo_seq = 0, hi_pt = 0, lo_pt = UINT64_MAX;
            bool has_hi = false, has_lo = false;
            for (uint32_t k = 0; k < g_syncs[h].pending_count; ++k) {
                const struct darwin_sync_pending *e = &g_syncs[h].pending[k];
                if (e->done) continue;
                if (!has_hi || e->point > hi_pt) {
                    has_hi = true; hi_pt = e->point; hi_grp = e->group; hi_seq = e->fence_seq;
                }
                if (!has_lo || e->point < lo_pt) {
                    has_lo = true; lo_pt = e->point; lo_grp = e->group; lo_seq = e->fence_seq;
                }
            }
            if (has_hi) {
                pthread_mutex_unlock(&g_lock);
                int rc = wait_group_fence(conn, hi_grp, hi_seq, 0);
                pthread_mutex_lock(&g_lock);
                if (rc == 0) {
                    syncobj_mark_group_seq_done_locked(hi_grp, hi_seq);
                    pthread_cond_broadcast(&g_sync_cond);
                } else if (has_lo && (lo_grp != hi_grp || lo_seq != hi_seq)) {
                    pthread_mutex_unlock(&g_lock);
                    int rc2 = wait_group_fence(conn, lo_grp, lo_seq, 0);
                    pthread_mutex_lock(&g_lock);
                    if (rc2 == 0) {
                        syncobj_mark_group_seq_done_locked(lo_grp, lo_seq);
                        pthread_cond_broadcast(&g_sync_cond);
                    }
                }
            }
        }
        points[i] = g_syncs[h].in_use ? g_syncs[h].signaled_point : 0;
    }
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjQuery(int fd, uint32_t *handles, uint64_t *points, uint32_t handle_count) {
    return drmSyncobjQuery2(fd, handles, points, handle_count, 0);
}

int drmSyncobjTimelineWait(int fd, uint32_t *handles, uint64_t *points,
                           unsigned num_handles, int64_t timeout_nsec,
                           unsigned flags, uint32_t *first_signaled) {
    if (num_handles == 0) return 0;
    io_connect_t conn = fd_to_conn(fd);
    bool wait_all        = (flags & DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL) != 0;
    bool wait_for_submit = (flags & DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT) != 0;
    bool wait_available  = (flags & DRM_SYNCOBJ_WAIT_FLAGS_WAIT_AVAILABLE) != 0;
    bool poll_only       = (timeout_nsec <= 0);
    bool infinite        = (timeout_nsec >= INT64_MAX - 1000000000LL);

    if (wait_all || num_handles == 1) {
        for (unsigned i = 0; i < num_handles; ++i) {
            uint32_t h = handles[i];
            uint64_t target = (points && points[i] != 0) ? points[i] : 1;
            pthread_mutex_lock(&g_lock);
            for (;;) {
                if (h == 0 || h >= MAX_DARWIN_SYNCOBJ || !g_syncs[h].in_use) {
                    pthread_mutex_unlock(&g_lock);
                    errno = ENOENT;
                    return -1;
                }
                if (g_syncs[h].signaled_point >= target) break;
                if (wait_available && syncobj_has_point_available_locked(&g_syncs[h], target)) break;

                uint32_t grp = 0;
                uint64_t seq = 0;
                bool has_fence = syncobj_find_wait_fence_locked(&g_syncs[h], target, &grp, &seq);
                if (has_fence && conn != MACH_PORT_NULL) {
                    int64_t rel_ns = 0;
                    if (poll_only) {
                        rel_ns = 0;
                    } else if (infinite) {
                        rel_ns = 10000000000LL;
                    } else {
                        int64_t rem = timeout_nsec - now_mono_ns();
                        if (rem <= 0) {
                            pthread_mutex_unlock(&g_lock);
                            errno = ETIME;
                            return -1;
                        }
                        rel_ns = (rem > 10000000000LL) ? 10000000000LL : rem;
                    }
                    pthread_mutex_unlock(&g_lock);
                    int rc = wait_group_fence(conn, grp, seq, rel_ns);
                    pthread_mutex_lock(&g_lock);
                    if (rc == 0) {
                        syncobj_mark_group_seq_done_locked(grp, seq);
                        pthread_cond_broadcast(&g_sync_cond);
                        continue;
                    }
                    pthread_mutex_unlock(&g_lock);
                    errno = ETIME;
                    return -1;
                }

                /* Target timeline point has not been submitted yet (wait-before-signal) */
                if (!wait_for_submit || poll_only) {
                    pthread_mutex_unlock(&g_lock);
                    errno = ETIME;
                    return -1;
                }
                if (infinite) {
                    pthread_cond_wait(&g_sync_cond, &g_lock);
                } else {
                    int64_t rem = timeout_nsec - now_mono_ns();
                    if (rem <= 0) {
                        pthread_mutex_unlock(&g_lock);
                        errno = ETIME;
                        return -1;
                    }
                    struct timespec rel = {
                        .tv_sec = (time_t)(rem / 1000000000LL),
                        .tv_nsec = (long)(rem % 1000000000LL),
                    };
                    (void)pthread_cond_timedwait_relative_np(&g_sync_cond, &g_lock, &rel);
                }
            }
            pthread_mutex_unlock(&g_lock);
        }
        if (first_signaled) *first_signaled = 0;
        return 0;
    }

    /* WAIT_ANY across multiple handles */
    pthread_mutex_lock(&g_lock);
    for (;;) {
        bool any_fence = false;
        uint32_t best_grp = 0;
        uint64_t best_seq = UINT64_MAX;
        for (unsigned i = 0; i < num_handles; ++i) {
            uint32_t h = handles[i];
            uint64_t target = (points && points[i] != 0) ? points[i] : 1;
            if (h == 0 || h >= MAX_DARWIN_SYNCOBJ || !g_syncs[h].in_use) {
                pthread_mutex_unlock(&g_lock);
                errno = ENOENT;
                return -1;
            }
            if (g_syncs[h].signaled_point >= target ||
                (wait_available && syncobj_has_point_available_locked(&g_syncs[h], target))) {
                if (first_signaled) *first_signaled = i;
                pthread_mutex_unlock(&g_lock);
                return 0;
            }
            uint32_t grp = 0;
            uint64_t seq = 0;
            if (!wait_available && syncobj_find_wait_fence_locked(&g_syncs[h], target, &grp, &seq)) {
                if (!any_fence || seq < best_seq) {
                    any_fence = true;
                    best_grp = grp;
                    best_seq = seq;
                }
            }
        }

        if (any_fence && conn != MACH_PORT_NULL) {
            /* Non-blocking poll of all pending handles first */
            bool progressed = false;
            for (unsigned i = 0; i < num_handles; ++i) {
                uint32_t h = handles[i];
                uint64_t target = (points && points[i] != 0) ? points[i] : 1;
                uint32_t grp = 0;
                uint64_t seq = 0;
                if (h > 0 && h < MAX_DARWIN_SYNCOBJ && g_syncs[h].in_use &&
                    syncobj_find_wait_fence_locked(&g_syncs[h], target, &grp, &seq)) {
                    pthread_mutex_unlock(&g_lock);
                    int rc = wait_group_fence(conn, grp, seq, 0);
                    pthread_mutex_lock(&g_lock);
                    if (rc == 0) {
                        syncobj_mark_group_seq_done_locked(grp, seq);
                        pthread_cond_broadcast(&g_sync_cond);
                        progressed = true;
                        break;
                    }
                }
            }
            if (progressed) continue;
        }

        if (poll_only) {
            pthread_mutex_unlock(&g_lock);
            errno = ETIME;
            return -1;
        }
        int64_t rem = infinite ? 10000000000LL : (timeout_nsec - now_mono_ns());
        if (!infinite && rem <= 0) {
            pthread_mutex_unlock(&g_lock);
            errno = ETIME;
            return -1;
        }

        if (any_fence && conn != MACH_PORT_NULL) {
            int64_t slice = (rem > 8000000LL) ? 8000000LL : rem;
            pthread_mutex_unlock(&g_lock);
            int rc = wait_group_fence(conn, best_grp, best_seq, slice);
            pthread_mutex_lock(&g_lock);
            if (rc == 0) {
                syncobj_mark_group_seq_done_locked(best_grp, best_seq);
                pthread_cond_broadcast(&g_sync_cond);
            }
            continue;
        }

        if (!wait_for_submit) {
            pthread_mutex_unlock(&g_lock);
            errno = ETIME;
            return -1;
        }
        if (infinite) {
            pthread_cond_wait(&g_sync_cond, &g_lock);
        } else {
            struct timespec rel = {
                .tv_sec = (time_t)(rem / 1000000000LL),
                .tv_nsec = (long)(rem % 1000000000LL),
            };
            (void)pthread_cond_timedwait_relative_np(&g_sync_cond, &g_lock, &rel);
        }
    }
}

int drmSyncobjWait(int fd, uint32_t *handles, unsigned num_handles,
                   int64_t timeout_nsec, unsigned flags, uint32_t *first_signaled) {
    return drmSyncobjTimelineWait(fd, handles, NULL, num_handles, timeout_nsec, flags, first_signaled);
}

int drmSyncobjTransfer(int fd, uint32_t dst_handle, uint64_t dst_point,
                       uint32_t src_handle, uint64_t src_point, uint32_t flags) {
    (void)fd; (void)flags;
    pthread_mutex_lock(&g_lock);
    if (dst_handle > 0 && dst_handle < MAX_DARWIN_SYNCOBJ &&
        src_handle > 0 && src_handle < MAX_DARWIN_SYNCOBJ &&
        g_syncs[dst_handle].in_use && g_syncs[src_handle].in_use) {
        struct darwin_syncobj *src = &g_syncs[src_handle];
        struct darwin_syncobj *dst = &g_syncs[dst_handle];
        uint64_t dst_pt = dst_point ? dst_point : 1;

        /* Snapshot matching pending entries first so dst == src self-transfer is safe */
        struct darwin_sync_pending copied[MAX_DARWIN_SYNCOBJ_PENDING];
        uint32_t ncopied = 0;
        bool src_signaled = false;

        if (src_point == 0) {
            for (uint32_t i = 0; i < src->pending_count; ++i) {
                if (!src->pending[i].done) {
                    copied[ncopied++] = src->pending[i];
                }
            }
            if (ncopied == 0 && src->signaled_point > 0) src_signaled = true;
        } else if (src->signaled_point >= src_point) {
            src_signaled = true;
        } else {
            uint64_t best_pt = UINT64_MAX;
            for (uint32_t i = 0; i < src->pending_count; ++i) {
                if (!src->pending[i].done && src->pending[i].point >= src_point &&
                    src->pending[i].point < best_pt) {
                    best_pt = src->pending[i].point;
                }
            }
            if (best_pt != UINT64_MAX) {
                for (uint32_t i = 0; i < src->pending_count; ++i) {
                    if (!src->pending[i].done && src->pending[i].point == best_pt) {
                        copied[ncopied++] = src->pending[i];
                    }
                }
            }
        }

        if (dst_point == 0 && dst != src) {
            /* Binary destination replacement */
            dst->signaled_point = 0;
            dst->pending_count = 0;
        }
        if (ncopied > 0) {
            for (uint32_t i = 0; i < ncopied; ++i) {
                syncobj_add_pending_locked(dst, dst_pt, copied[i].group, copied[i].fence_seq);
            }
        } else if (src_signaled) {
            if (dst_pt > dst->signaled_point) dst->signaled_point = dst_pt;
            syncobj_prune_locked(dst);
        }
        pthread_cond_broadcast(&g_sync_cond);
    }
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjHandleToFD(int fd, uint32_t handle, int *obj_fd) { (void)fd; (void)handle; (void)obj_fd; errno = ENOSYS; return -1; }
int drmSyncobjFDToHandle(int fd, int obj_fd, uint32_t *handle) { (void)fd; (void)obj_fd; (void)handle; errno = ENOSYS; return -1; }
int drmSyncobjImportSyncFile(int fd, uint32_t handle, int sync_file_fd) { (void)fd; (void)handle; (void)sync_file_fd; errno = ENOSYS; return -1; }
int drmSyncobjExportSyncFile(int fd, uint32_t handle, int *sync_file_fd) { (void)fd; (void)handle; (void)sync_file_fd; errno = ENOSYS; return -1; }
int drmPrimeHandleToFD(int fd, uint32_t handle, uint32_t flags, int *prime_fd) { (void)fd; (void)handle; (void)flags; (void)prime_fd; errno = ENOSYS; return -1; }
int drmPrimeFDToHandle(int fd, int prime_fd, uint32_t *handle) { (void)fd; (void)prime_fd; (void)handle; errno = ENOSYS; return -1; }

/* ---- DRM Device enumeration & version ---- */

drmVersionPtr drmGetVersion(int fd) {
    (void)fd;
    drmVersionPtr v = (drmVersionPtr)calloc(1, sizeof(drmVersion));
    if (!v) return NULL;
    v->version_major = 1;
    v->version_minor = 7;
    v->version_patchlevel = 0;
    v->name = strdup("panthor");
    v->name_len = (int)strlen(v->name);
    v->date = strdup("20261124");
    v->date_len = (int)strlen(v->date);
    v->desc = strdup("TB375FC Mali-G615 Darwin IOKit");
    v->desc_len = (int)strlen(v->desc);
    return v;
}

void drmFreeVersion(drmVersionPtr v) {
    if (!v) return;
    free(v->name);
    free(v->date);
    free(v->desc);
    free(v);
}

int drmGetCap(int fd, uint64_t capability, uint64_t *value) {
    (void)fd;
    if (!value) { errno = EINVAL; return -1; }
    switch (capability) {
    case DRM_CAP_SYNCOBJ:
    case DRM_CAP_SYNCOBJ_TIMELINE:
        *value = 1;
        return 0;
    default:
        *value = 0;
        return 0;
    }
}

int drmCloseBufferHandle(int fd, uint32_t handle) {
    struct drm_gem_close c = { .handle = handle, .pad = 0 };
    return drmIoctl(fd, DRM_IOCTL_GEM_CLOSE, &c);
}

static drmDevicePtr make_mali_drm_device(void) {
    drmDevicePtr d = (drmDevicePtr)calloc(1, sizeof(drmDevice));
    if (!d) return NULL;
    d->nodes = (char **)calloc(DRM_NODE_MAX, sizeof(char *));
    d->nodes[DRM_NODE_RENDER] = strdup("/dev/dri/renderD128");
    d->available_nodes = (1 << DRM_NODE_RENDER);
    d->bustype = DRM_BUS_PLATFORM;
    d->businfo.platform = (drmPlatformBusInfoPtr)calloc(1, sizeof(drmPlatformBusInfo));
    strncpy(d->businfo.platform->fullname, "/soc/mali@13000000", sizeof(d->businfo.platform->fullname) - 1);
    d->deviceinfo.platform = (drmPlatformDeviceInfoPtr)calloc(1, sizeof(drmPlatformDeviceInfo));
    d->deviceinfo.platform->compatible = (char **)calloc(2, sizeof(char *));
    d->deviceinfo.platform->compatible[0] = strdup("arm,mali-valhall-csf");
    return d;
}

int drmGetDevices2(uint32_t flags, drmDevicePtr devices[], int max_devices) {
    (void)flags;
    if (!devices) return 1;
    if (max_devices <= 0) return 0;
    devices[0] = make_mali_drm_device();
    return devices[0] ? 1 : 0;
}

int drmGetDevice2(int fd, uint32_t flags, drmDevicePtr *device) {
    (void)fd; (void)flags;
    if (!device) { errno = EINVAL; return -1; }
    *device = make_mali_drm_device();
    return *device ? 0 : -1;
}

void drmFreeDevice(drmDevicePtr *device) {
    if (!device || !*device) return;
    drmDevicePtr d = *device;
    if (d->nodes) {
        for (int i = 0; i < DRM_NODE_MAX; ++i) free(d->nodes[i]);
        free(d->nodes);
    }
    if (d->bustype == DRM_BUS_PLATFORM) {
        free(d->businfo.platform);
        if (d->deviceinfo.platform) {
            if (d->deviceinfo.platform->compatible) {
                for (int i = 0; d->deviceinfo.platform->compatible[i]; ++i)
                    free(d->deviceinfo.platform->compatible[i]);
                free(d->deviceinfo.platform->compatible);
            }
            free(d->deviceinfo.platform);
        }
    }
    free(d);
    *device = NULL;
}

void drmFreeDevices(drmDevicePtr devices[], int count) {
    for (int i = 0; i < count; ++i) drmFreeDevice(&devices[i]);
}

int drmGetNodeTypeFromFd(int fd) { (void)fd; return DRM_NODE_RENDER; }
char *drmGetRenderDeviceNameFromFd(int fd) { (void)fd; return strdup("/dev/dri/renderD128"); }
char *drmGetPrimaryDeviceNameFromFd(int fd) { (void)fd; return NULL; }
int drmDevicesEqual(drmDevicePtr a, drmDevicePtr b) { return (a && b) ? 1 : 0; }

/* ---- Main DRM_IOCTL_PANTHOR_* dispatcher ---- */

int drmIoctl(int fd, unsigned long request, void *arg) {
    io_connect_t conn = fd_to_conn(fd);
    if (conn == MACH_PORT_NULL) { errno = EBADF; return -1; }

    switch (request) {
    case DRM_IOCTL_PANTHOR_DEV_QUERY: {
        struct drm_panthor_dev_query *q = (struct drm_panthor_dev_query *)arg;
        struct Tb375fcMaliDevInfo info;
        if (ensure_dev_info(fd, &info) != 0) return -1;

        if (q->type == DRM_PANTHOR_DEV_QUERY_GPU_INFO) {
            struct drm_panthor_gpu_info g;
            memset(&g, 0, sizeof(g));
            g.gpu_id = info.gpu_id;
            g.gpu_rev = info.gpu_rev;
            g.csf_id = info.csf_id;
            g.l2_features = info.l2_features;
            g.tiler_features = info.tiler_features;
            g.mem_features = info.mem_features;
            g.mmu_features = info.mmu_features;
            g.thread_features = info.thread_features;
            g.max_threads = info.max_threads;
            g.thread_max_workgroup_size = info.thread_max_workgroup_size;
            g.thread_max_barrier_size = info.thread_max_barrier_size;
            g.coherency_features = info.coherency_features;
            memcpy(g.texture_features, info.texture_features, sizeof(g.texture_features));
            g.as_present = info.as_present;
            g.selected_coherency = info.selected_coherency;
            g.shader_present = info.shader_present;
            g.tiler_present = info.tiler_present;
            g.l2_present = info.l2_present;
            if (!q->pointer) { q->size = sizeof(g); return 0; }
            size_t n = q->size < sizeof(g) ? q->size : sizeof(g);
            memcpy((void *)(uintptr_t)q->pointer, &g, n);
            q->size = (uint32_t)n;
            return 0;
        }
        if (q->type == DRM_PANTHOR_DEV_QUERY_CSIF_INFO) {
            struct drm_panthor_csif_info c;
            memset(&c, 0, sizeof(c));
            c.csg_slot_count = info.csg_slot_count;
            c.cs_slot_count = info.cs_slot_count;
            c.cs_reg_count = info.cs_reg_count;
            c.scoreboard_slot_count = info.scoreboard_slot_count;
            c.unpreserved_cs_reg_count = info.unpreserved_cs_reg_count;
            if (!q->pointer) { q->size = sizeof(c); return 0; }
            size_t n = q->size < sizeof(c) ? q->size : sizeof(c);
            memcpy((void *)(uintptr_t)q->pointer, &c, n);
            q->size = (uint32_t)n;
            return 0;
        }
        if (q->type == DRM_PANTHOR_DEV_QUERY_TIMESTAMP_INFO) {
            /* Always fetch live timestamp */
            size_t osz = sizeof(info);
            (void)call_mali(conn, MALI_SEL_DEV_QUERY, NULL, 0, &info, &osz);
            struct drm_panthor_timestamp_info t;
            memset(&t, 0, sizeof(t));
            t.timestamp_frequency = info.timestamp_frequency;
            t.current_timestamp = info.current_timestamp;
            t.timestamp_offset = info.timestamp_offset;
            if (!q->pointer) { q->size = sizeof(t); return 0; }
            size_t n = q->size < sizeof(t) ? q->size : sizeof(t);
            memcpy((void *)(uintptr_t)q->pointer, &t, n);
            q->size = (uint32_t)n;
            return 0;
        }
        if (q->type == DRM_PANTHOR_DEV_QUERY_GROUP_PRIORITIES_INFO) {
            struct drm_panthor_group_priorities_info p;
            memset(&p, 0, sizeof(p));
            p.allowed_mask = 0x7u; /* LOW | MEDIUM | HIGH */
            if (!q->pointer) { q->size = sizeof(p); return 0; }
            size_t n = q->size < sizeof(p) ? q->size : sizeof(p);
            memcpy((void *)(uintptr_t)q->pointer, &p, n);
            q->size = (uint32_t)n;
            return 0;
        }
        if (q->type == DRM_PANTHOR_DEV_QUERY_MMU_INFO) {
            struct drm_panthor_mmu_info m;
            memset(&m, 0, sizeof(m));
            m.page_size_bitmap = 4096ULL;
            if (!q->pointer) { q->size = sizeof(m); return 0; }
            size_t n = q->size < sizeof(m) ? q->size : sizeof(m);
            memcpy((void *)(uintptr_t)q->pointer, &m, n);
            q->size = (uint32_t)n;
            return 0;
        }
        errno = EINVAL;
        return -1;
    }

    case DRM_IOCTL_PANTHOR_VM_CREATE: {
        struct drm_panthor_vm_create *req = (struct drm_panthor_vm_create *)arg;
        struct Tb375fcMaliVmCreateIn in = { .flags = req->flags, .pad = 0, .user_va_range = req->user_va_range };
        struct Tb375fcMaliVmCreateOut out = {0};
        size_t osz = sizeof(out);
        if (call_mali(conn, MALI_SEL_VM_CREATE, &in, sizeof(in), &out, &osz) != 0) return -1;
        req->id = out.vm_id;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_VM_DESTROY: {
        struct drm_panthor_vm_destroy *req = (struct drm_panthor_vm_destroy *)arg;
        struct Tb375fcMaliHandleIn in = { .handle = req->id, .pad = 0 };
        return call_mali(conn, MALI_SEL_VM_DESTROY, &in, sizeof(in), NULL, NULL);
    }

    case DRM_IOCTL_PANTHOR_VM_GET_STATE: {
        struct drm_panthor_vm_get_state *req = (struct drm_panthor_vm_get_state *)arg;
        req->state = DRM_PANTHOR_VM_STATE_USABLE;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_VM_BIND: {
        struct drm_panthor_vm_bind *req = (struct drm_panthor_vm_bind *)arg;
        const struct drm_panthor_vm_bind_op *ops =
            (const struct drm_panthor_vm_bind_op *)(uintptr_t)req->ops.array;
        for (uint32_t i = 0; i < req->ops.count; ++i) {
            /* Wait on any input syncobjs BEFORE mutating GPU page tables (prevents premature UNMAP faults!) */
            const struct drm_panthor_sync_op *sops =
                (const struct drm_panthor_sync_op *)(uintptr_t)ops[i].syncs.array;
            for (uint32_t s = 0; s < ops[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) == 0) {
                    bool is_tl = (sops[s].flags & DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_MASK) ==
                                 DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_TIMELINE_SYNCOBJ;
                    uint64_t pt = is_tl ? sops[s].timeline_value : 1;
                    if (pt > 0) {
                        uint32_t sh = sops[s].handle;
                        bool already_signaled = false;
                        if (sh > 0 && sh < MAX_DARWIN_SYNCOBJ) {
                            pthread_mutex_lock(&g_lock);
                            already_signaled = (g_syncs[sh].in_use && g_syncs[sh].signaled_point >= pt);
                            pthread_mutex_unlock(&g_lock);
                        }
                        if (!already_signaled) {
                            int64_t dl = now_mono_ns() + 10000000000LL;
                            drmSyncobjTimelineWait(fd, &sh, &pt, 1, dl,
                                DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL | DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT, NULL);
                        }
                    }
                }
            }
            uint32_t op_type = ops[i].flags & DRM_PANTHOR_VM_BIND_OP_TYPE_MASK;
            if (op_type == DRM_PANTHOR_VM_BIND_OP_TYPE_SYNC_ONLY) {
                /* Signal any out syncs immediately */
            } else {
                struct Tb375fcMaliVmBindIn in;
                memset(&in, 0, sizeof(in));
                in.vm_id = req->vm_id;
                in.op = (op_type == DRM_PANTHOR_VM_BIND_OP_TYPE_UNMAP) ? MALI_VM_BIND_UNMAP : MALI_VM_BIND_MAP;
                in.bo_handle = ops[i].bo_handle;
                in.bo_offset = ops[i].bo_offset;
                in.va = ops[i].va;
                in.size = ops[i].size;
                if (ops[i].flags & DRM_PANTHOR_VM_BIND_OP_MAP_READONLY) in.flags |= MALI_VM_MAP_READONLY;
                if (ops[i].flags & DRM_PANTHOR_VM_BIND_OP_MAP_NOEXEC)   in.flags |= MALI_VM_MAP_NOEXEC;
                if (ops[i].flags & DRM_PANTHOR_VM_BIND_OP_MAP_UNCACHED) in.flags |= MALI_VM_MAP_UNCACHED;
                if (call_mali(conn, MALI_SEL_VM_BIND, &in, sizeof(in), NULL, NULL) != 0)
                    return -1;
            }
            /* Process per-op output syncs (signal immediately since VM_BIND is synchronous) */
            for (uint32_t s = 0; s < ops[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) != 0) {
                    bool is_tl = (sops[s].flags & DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_MASK) ==
                                 DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_TIMELINE_SYNCOBJ;
                    uint32_t sh = sops[s].handle;
                    if (is_tl) {
                        uint64_t pt = sops[s].timeline_value;
                        if (pt > 0) drmSyncobjTimelineSignal(fd, &sh, &pt, 1);
                    } else {
                        drmSyncobjSignal(fd, &sh, 1);
                    }
                }
            }
        }
        return 0;
    }

    case DRM_IOCTL_PANTHOR_BO_CREATE: {
        struct drm_panthor_bo_create *req = (struct drm_panthor_bo_create *)arg;
        bool no_mmap = (req->flags & DRM_PANTHOR_BO_NO_MMAP) != 0;
        uint64_t aligned_sz = (req->size + 4095ULL) & ~4095ULL;
        if (aligned_sz == 0) { errno = EINVAL; return -1; }

        /* With Boot 156+ kernel WriteCombine caps (32 MiB/BO, 256 MiB WS, 128 MiB non-WS,
         * 1024 MiB global), allow mappable BOs up to 32 MiB to take the zero-flush hardware
         * WriteCombine path so m2v 16-32 MiB slabs never fall back to cached anon-mmap flush! */
        if (no_mmap || aligned_sz <= (32ULL << 20)) {
            struct Tb375fcMaliBoCreateIn in = {
                .size = aligned_sz,
                .flags = no_mmap ? MALI_BO_NO_MMAP : 0,
                .exclusive_vm_id = req->exclusive_vm_id,
            };
            struct Tb375fcMaliBoCreateOut out = {0};
            size_t osz = sizeof(out);
            if (call_mali(conn, MALI_SEL_BO_CREATE, &in, sizeof(in), &out, &osz) == 0) {
                if (out.bo_handle > 0 && out.bo_handle < MAX_DARWIN_HOST_BOS) {
                    pthread_mutex_lock(&g_bo_lock);
                    g_mmap_cached[out.bo_handle] = (out.pad != 0);
                    note_bo_handle_locked(out.bo_handle);
                    pthread_mutex_unlock(&g_bo_lock);
                }
                req->handle = out.bo_handle;
                req->size = out.size;
                return 0;
            }
            if (no_mmap) return -1;
            /* Mappable BO exceeded kernel WC budget -> fall through to anon mmap + BO_IMPORT_HOST */
        }

        void *ap = mmap(NULL, (size_t)aligned_sz, PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);
        if (ap == MAP_FAILED) { errno = ENOMEM; return -1; }
        struct Tb375fcMaliBoImportHostIn hin;
        memset(&hin, 0, sizeof(hin));
        hin.host_ptr = (uint64_t)(uintptr_t)ap;
        hin.size = aligned_sz;
        hin.exclusive_vm_id = req->exclusive_vm_id;
        struct Tb375fcMaliBoCreateOut hout = {0};
        size_t hosz = sizeof(hout);
        if (call_mali(conn, MALI_SEL_BO_IMPORT_HOST, &hin, sizeof(hin), &hout, &hosz) != 0) {
            munmap(ap, (size_t)aligned_sz);
            return -1;
        }
        /* Flush initial wired zero-fill pages to DRAM immediately so GPU never sees stale lines.
         * Do NOT register in g_mmap_bos[] until panthor_darwin_mmap_bo() is actually called,
         * so unmapped Private GPU textures never pay per-submit CPU dc civac overhead! */
        arm64_dcache_clean_inval(ap, (size_t)aligned_sz);
        if (hout.bo_handle > 0 && hout.bo_handle < MAX_DARWIN_HOST_BOS) {
            pthread_mutex_lock(&g_bo_lock);
            g_anon_bos[hout.bo_handle].ptr = ap;
            g_anon_bos[hout.bo_handle].size = (size_t)aligned_sz;
            g_anon_map_cnt[hout.bo_handle] = 0;
            note_bo_handle_locked(hout.bo_handle);
            pthread_mutex_unlock(&g_bo_lock);
        }
        req->handle = hout.bo_handle;
        req->size = hout.size;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_BO_MMAP_OFFSET: {
        struct drm_panthor_bo_mmap_offset *req = (struct drm_panthor_bo_mmap_offset *)arg;
        req->offset = ((uint64_t)req->handle) << 12;
        return 0;
    }

    case DRM_IOCTL_GEM_CLOSE: {
        struct drm_gem_close *req = (struct drm_gem_close *)arg;
        void *anon_ptr = NULL;
        size_t anon_sz = 0;
        void *wc_zero_ptr = NULL;
        if (req->handle > 0 && req->handle < MAX_DARWIN_HOST_BOS) {
            pthread_mutex_lock(&g_bo_lock);
            g_host_bos[req->handle].ptr = NULL;
            g_host_bos[req->handle].size = 0;
            g_mmap_bos[req->handle].ptr = NULL;
            g_mmap_bos[req->handle].size = 0;
            wc_zero_ptr = g_wc_zero_bos[req->handle].ptr;
            g_wc_zero_bos[req->handle].ptr = NULL;
            g_wc_zero_bos[req->handle].size = 0;
            anon_ptr = g_anon_bos[req->handle].ptr;
            anon_sz = g_anon_bos[req->handle].size;
            g_anon_bos[req->handle].ptr = NULL;
            g_anon_bos[req->handle].size = 0;
            g_anon_map_cnt[req->handle] = 0;
            g_mmap_cached[req->handle] = false;
            active_list_remove(g_active_host_list, g_active_host_pos, &g_active_host_cnt, req->handle);
            active_list_remove(g_active_cached_list, g_active_cached_pos, &g_active_cached_cnt, req->handle);
            pthread_mutex_unlock(&g_bo_lock);
        }
        if (wc_zero_ptr) {
            (void)IOConnectUnmapMemory64(conn, req->handle, mach_task_self(),
                                         (mach_vm_address_t)(uintptr_t)wc_zero_ptr);
        }
        struct Tb375fcMaliHandleIn in = { .handle = req->handle, .pad = 0 };
        int rc = call_mali(conn, MALI_SEL_BO_DESTROY, &in, sizeof(in), NULL, NULL);
        if (anon_ptr && anon_sz > 0) {
            munmap(anon_ptr, anon_sz);
        }
        return rc;
    }

    case DRM_IOCTL_PANTHOR_BO_QUERY_INFO: {
        struct drm_panthor_bo_query_info *req = (struct drm_panthor_bo_query_info *)arg;
        req->extra_flags = 0;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_BO_SET_LABEL:
    case DRM_IOCTL_PANTHOR_SET_USER_MMIO_OFFSET:
    case DRM_IOCTL_PANTHOR_BO_SYNC:
        return 0;

    case DRM_IOCTL_PANTHOR_TILER_HEAP_CREATE: {
        struct drm_panthor_tiler_heap_create *req = (struct drm_panthor_tiler_heap_create *)arg;
        struct Tb375fcMaliTilerHeapCreateIn in = {
            .vm_id = req->vm_id,
            .initial_chunk_count = req->initial_chunk_count,
            .chunk_size = req->chunk_size,
            .max_chunks = req->max_chunks,
            .target_in_flight = req->target_in_flight,
            .pad = 0,
        };
        struct Tb375fcMaliTilerHeapCreateOut out = {0};
        size_t osz = sizeof(out);
        if (call_mali(conn, MALI_SEL_TILER_HEAP_CREATE, &in, sizeof(in), &out, &osz) != 0) return -1;
        req->handle = out.handle;
        req->tiler_heap_ctx_gpu_va = out.tiler_heap_ctx_gpu_va;
        req->first_heap_chunk_gpu_va = out.first_heap_chunk_gpu_va;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_TILER_HEAP_DESTROY: {
        struct drm_panthor_tiler_heap_destroy *req = (struct drm_panthor_tiler_heap_destroy *)arg;
        struct Tb375fcMaliHandleIn in = { .handle = req->handle, .pad = 0 };
        return call_mali(conn, MALI_SEL_TILER_HEAP_DESTROY, &in, sizeof(in), NULL, NULL);
    }

    case DRM_IOCTL_PANTHOR_GROUP_CREATE: {
        struct drm_panthor_group_create *req = (struct drm_panthor_group_create *)arg;
        struct Tb375fcMaliGroupCreateIn in;
        memset(&in, 0, sizeof(in));
        in.vm_id = req->vm_id;
        in.max_compute_cores = req->max_compute_cores;
        in.max_fragment_cores = req->max_fragment_cores;
        in.max_tiler_cores = req->max_tiler_cores;
        in.priority = req->priority;
        in.compute_core_mask = req->compute_core_mask;
        in.fragment_core_mask = req->fragment_core_mask;
        in.tiler_core_mask = req->tiler_core_mask;
        uint32_t qcnt = req->queues.count;
        if (qcnt > MALI_MAX_QUEUES_PER_GROUP) qcnt = MALI_MAX_QUEUES_PER_GROUP;
        in.queue_count = qcnt;
        const struct drm_panthor_queue_create *qs =
            (const struct drm_panthor_queue_create *)(uintptr_t)req->queues.array;
        for (uint32_t i = 0; i < qcnt; ++i) {
            in.queues[i].priority = qs[i].priority;
            in.queues[i].ringbuf_size = qs[i].ringbuf_size;
        }
        struct Tb375fcMaliGroupCreateOut out = {0};
        size_t osz = sizeof(out);
        if (call_mali(conn, MALI_SEL_GROUP_CREATE, &in, sizeof(in), &out, &osz) != 0) return -1;
        req->group_handle = out.group_handle;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_GROUP_DESTROY: {
        struct drm_panthor_group_destroy *req = (struct drm_panthor_group_destroy *)arg;
        struct Tb375fcMaliHandleIn in = { .handle = req->group_handle, .pad = 0 };
        return call_mali(conn, MALI_SEL_GROUP_DESTROY, &in, sizeof(in), NULL, NULL);
    }

    case DRM_IOCTL_PANTHOR_GROUP_SUBMIT: {
        struct drm_panthor_group_submit *req = (struct drm_panthor_group_submit *)arg;
        const struct drm_panthor_queue_submit *qsubs =
            (const struct drm_panthor_queue_submit *)(uintptr_t)req->queue_submits.array;

        /* Wait for any input syncobjs first (using absolute CLOCK_MONOTONIC deadline) */
        for (uint32_t i = 0; i < req->queue_submits.count; ++i) {
            const struct drm_panthor_sync_op *sops =
                (const struct drm_panthor_sync_op *)(uintptr_t)qsubs[i].syncs.array;
            for (uint32_t s = 0; s < qsubs[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) == 0) {
                    bool is_tl = (sops[s].flags & DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_MASK) ==
                                 DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_TIMELINE_SYNCOBJ;
                    uint64_t pt = is_tl ? sops[s].timeline_value : 1;
                    if (pt > 0) {
                        uint32_t sh = sops[s].handle;
                        bool already_signaled = false;
                        if (sh > 0 && sh < MAX_DARWIN_SYNCOBJ) {
                            pthread_mutex_lock(&g_lock);
                            already_signaled = (g_syncs[sh].in_use && g_syncs[sh].signaled_point >= pt);
                            pthread_mutex_unlock(&g_lock);
                        }
                        if (!already_signaled) {
                            int64_t dl = now_mono_ns() + 10000000000LL;
                            drmSyncobjTimelineWait(fd, &sh, &pt, 1, dl,
                                DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL | DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT, NULL);
                        }
                    }
                }
            }
        }

        struct timespec ts0, ts1, ts2;
        clock_gettime(CLOCK_MONOTONIC, &ts0);
        size_t hkb = 0, mkb = 0;
        pthread_mutex_lock(&g_bo_lock);
        flush_all_host_bos(false, &hkb, &mkb);
        pthread_mutex_unlock(&g_bo_lock);
        clock_gettime(CLOCK_MONOTONIC, &ts1);

        struct Tb375fcMaliGroupSubmitOut out = {0};
        uint32_t total = req->queue_submits.count;
        if (total == 0) {
            struct Tb375fcMaliGroupSubmitIn in;
            memset(&in, 0, sizeof(in));
            in.group_handle = req->group_handle;
            size_t osz = sizeof(out);
            if (call_mali(conn, MALI_SEL_GROUP_SUBMIT, &in, sizeof(in), &out, &osz) != 0) return -1;
        } else {
            for (uint32_t off = 0; off < total; off += MALI_MAX_SUBMITS_PER_CALL) {
                struct Tb375fcMaliGroupSubmitIn in;
                memset(&in, 0, sizeof(in));
                in.group_handle = req->group_handle;
                uint32_t cnt = total - off;
                if (cnt > MALI_MAX_SUBMITS_PER_CALL) cnt = MALI_MAX_SUBMITS_PER_CALL;
                in.submit_count = cnt;
                for (uint32_t i = 0; i < cnt; ++i) {
                    in.submits[i].queue_index = qsubs[off + i].queue_index;
                    in.submits[i].stream_size = qsubs[off + i].stream_size;
                    in.submits[i].stream_addr = qsubs[off + i].stream_addr;
                    in.submits[i].latest_flush = qsubs[off + i].latest_flush;
                }
                size_t osz = sizeof(out);
                if (call_mali(conn, MALI_SEL_GROUP_SUBMIT, &in, sizeof(in), &out, &osz) != 0) return -1;
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        static uint32_t sSubLog = 0;
        static uint32_t sSlowLog = 0;
        long fUs = (ts1.tv_sec - ts0.tv_sec) * 1000000L + (ts1.tv_nsec - ts0.tv_nsec) / 1000L;
        long iUs = (ts2.tv_sec - ts1.tv_sec) * 1000000L + (ts2.tv_nsec - ts1.tv_nsec) / 1000L;
        uint32_t subNo = ++sSubLog;
        if (subNo <= 12 || (fUs > 15000L && ++sSlowLog <= 16)) {
            struct Tb375fcMaliDebugLogIn dlog;
            memset(&dlog, 0, sizeof(dlog));
            snprintf(dlog.msg, sizeof(dlog.msg) - 1, "[panvk:%s] submit #%u flush=%ldus (h=%zuKB m=%zuKB) ioctl=%ldus q=%u",
                     getprogname(), subNo, fUs, hkb, mkb, iUs, total);
            (void)call_mali(conn, MALI_SEL_DEBUG_LOG, &dlog, sizeof(dlog), NULL, NULL);
        }

        /* Attach fence_seq to all output syncobjs and wake any wait-before-signal waiters */
        pthread_mutex_lock(&g_lock);
        bool signaled_any = false;
        for (uint32_t i = 0; i < req->queue_submits.count; ++i) {
            const struct drm_panthor_sync_op *sops =
                (const struct drm_panthor_sync_op *)(uintptr_t)qsubs[i].syncs.array;
            for (uint32_t s = 0; s < qsubs[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) != 0) {
                    uint32_t sh = sops[s].handle;
                    bool is_tl = (sops[s].flags & DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_MASK) ==
                                 DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_TIMELINE_SYNCOBJ;
                    if (sh > 0 && sh < MAX_DARWIN_SYNCOBJ && g_syncs[sh].in_use) {
                        if (is_tl) {
                            uint64_t pt = sops[s].timeline_value;
                            if (pt > 0) {
                                syncobj_add_pending_locked(&g_syncs[sh], pt, req->group_handle, out.fence_seq);
                                signaled_any = true;
                            }
                        } else {
                            /* Binary syncobj: replace any previous fence with the new pending GPU fence */
                            g_syncs[sh].signaled_point = 0;
                            g_syncs[sh].pending_count = 0;
                            syncobj_add_pending_locked(&g_syncs[sh], 1, req->group_handle, out.fence_seq);
                            signaled_any = true;
                        }
                    }
                }
            }
        }
        if (signaled_any) pthread_cond_broadcast(&g_sync_cond);
        pthread_mutex_unlock(&g_lock);
        return 0;
    }

    case DRM_IOCTL_PANTHOR_GROUP_GET_STATE: {
        struct drm_panthor_group_get_state *req = (struct drm_panthor_group_get_state *)arg;
        struct Tb375fcMaliHandleIn in = { .handle = req->group_handle, .pad = 0 };
        struct Tb375fcMaliGroupStateOut out = {0};
        size_t osz = sizeof(out);
        if (call_mali(conn, MALI_SEL_GROUP_GET_STATE, &in, sizeof(in), &out, &osz) != 0) return -1;
        req->state = out.state;
        req->fatal_queues = out.fatal_queues;
        return 0;
    }

    default:
        errno = ENOTTY;
        return -1;
    }
}
