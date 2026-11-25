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
#include <time.h>
#include <unistd.h>

#define MAX_DARWIN_FDS      32
#define MAX_DARWIN_SYNCOBJ  4096
#define MAX_DARWIN_HOST_BOS 16384

struct darwin_host_bo {
    void  *ptr;
    size_t size;
};

static struct darwin_host_bo g_host_bos[MAX_DARWIN_HOST_BOS];
static struct darwin_host_bo g_mmap_bos[MAX_DARWIN_HOST_BOS];

static inline void arm64_dcache_clean(const void *addr, size_t len) {
    if (!addr || !len) return;
    uintptr_t p = (uintptr_t)addr & ~63ULL;
    uintptr_t end = ((uintptr_t)addr + len + 63ULL) & ~63ULL;
    for (; p < end; p += 64ULL) {
        __asm__ volatile("dc cvac, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static inline void arm64_dcache_clean_inval(const void *addr, size_t len) {
    if (!addr || !len) return;
    uintptr_t p = (uintptr_t)addr & ~63ULL;
    uintptr_t end = ((uintptr_t)addr + len + 63ULL) & ~63ULL;
    for (; p < end; p += 64ULL) {
        __asm__ volatile("dc civac, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void flush_all_host_bos(bool invalidate) {
    for (uint32_t i = 1; i < MAX_DARWIN_HOST_BOS; ++i) {
        void *p = g_host_bos[i].ptr;
        size_t sz = g_host_bos[i].size;
        if (p && sz > 0 && sz <= (128ULL << 20)) {
            if (invalidate) arm64_dcache_clean_inval(p, sz);
            else            arm64_dcache_clean(p, sz);
        }
        p = g_mmap_bos[i].ptr;
        sz = g_mmap_bos[i].size;
        /* BOs < 4 MiB use KernelUserShared + WriteCombine (uncached on CPU); >= 4 MiB live in fTask VM */
        if (p && sz >= (4ULL << 20) && sz <= (128ULL << 20)) {
            if (invalidate) arm64_dcache_clean_inval(p, sz);
            else            arm64_dcache_clean(p, sz);
        }
    }
}

struct darwin_mali_fd {
    int           fd;
    io_connect_t  conn;
    bool          in_use;
    struct Tb375fcMaliDevInfo cached_info;
    bool          info_valid;
};

struct darwin_syncobj {
    bool     in_use;
    uint64_t signaled_point;
    uint32_t pending_group;
    uint64_t pending_fence_seq;
    uint64_t pending_point;
    bool     has_pending;
};

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
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
        pthread_mutex_lock(&g_lock);
        g_mmap_bos[bo_handle].ptr = (void *)(uintptr_t)addr;
        g_mmap_bos[bo_handle].size = (size_t)map_size;
        pthread_mutex_unlock(&g_lock);
    }
    return (void *)(uintptr_t)addr;
}

int panthor_darwin_munmap_bo(int fd, void *addr, size_t size) {
    io_connect_t conn = fd_to_conn(fd);
    if (conn == MACH_PORT_NULL || !addr) return 0;
    if (size > 0 && size <= (128ULL << 20)) {
        arm64_dcache_clean(addr, size);
    }
    uint32_t found_handle = 0;
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 1; i < MAX_DARWIN_HOST_BOS; ++i) {
        if (g_mmap_bos[i].ptr == addr) {
            found_handle = i;
            g_mmap_bos[i].ptr = NULL;
            g_mmap_bos[i].size = 0;
            break;
        }
    }
    pthread_mutex_unlock(&g_lock);
    (void)IOConnectUnmapMemory64(conn, found_handle, mach_task_self(), (mach_vm_address_t)(uintptr_t)addr);
    return 0;
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
        pthread_mutex_lock(&g_lock);
        g_host_bos[out.bo_handle].ptr = host_ptr;
        g_host_bos[out.bo_handle].size = (size_t)out.size;
        pthread_mutex_unlock(&g_lock);
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

/* ---- Syncobj helpers ---- */

static int wait_syncobj_fence(io_connect_t conn, struct darwin_syncobj *s, int64_t timeout_ns) {
    if (!s->has_pending) return 0;
    struct Tb375fcMaliFenceWaitIn in = {
        .group_handle = s->pending_group,
        .pad = 0,
        .fence_seq = s->pending_fence_seq,
        .timeout_ns = (timeout_ns < 0) ? 10000000000ULL : (uint64_t)timeout_ns,
    };
    struct Tb375fcMaliFenceWaitOut out = {0};
    size_t osz = sizeof(out);
    int rc = call_mali(conn, MALI_SEL_FENCE_WAIT, &in, sizeof(in), &out, &osz);
    if (rc == 0 && out.signaled) {
        /* Scanout IOSurfaces are invalidated by NativeLegacyDisplay (flush_dcache64)
         * and fbshot (arm64_dcache_clean_inval); m2v BOs < 24 MiB are WriteCombine.
         * Skipping redundant ~50 MiB dc civac here cuts commit latency in half. */
        if (s->pending_point > s->signaled_point)
            s->signaled_point = s->pending_point;
        s->has_pending = false;
        return 0;
    }
    errno = ETIMEDOUT;
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
    if (handle > 0 && handle < MAX_DARWIN_SYNCOBJ)
        g_syncs[handle].in_use = false;
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
            g_syncs[h].has_pending = false;
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
            g_syncs[h].has_pending = false;
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
            g_syncs[h].has_pending = false;
        }
    }
    pthread_cond_broadcast(&g_sync_cond);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

int drmSyncobjQuery2(int fd, uint32_t *handles, uint64_t *points, uint32_t handle_count, uint32_t flags) {
    (void)flags;
    io_connect_t conn = fd_to_conn(fd);
    pthread_mutex_lock(&g_lock);
    for (uint32_t i = 0; i < handle_count; ++i) {
        uint32_t h = handles[i];
        if (h > 0 && h < MAX_DARWIN_SYNCOBJ && g_syncs[h].in_use) {
            if (g_syncs[h].has_pending && conn != MACH_PORT_NULL) {
                struct darwin_syncobj copy = g_syncs[h];
                pthread_mutex_unlock(&g_lock);
                if (wait_syncobj_fence(conn, &copy, 0) == 0) {
                    pthread_mutex_lock(&g_lock);
                    if (copy.signaled_point > g_syncs[h].signaled_point)
                        g_syncs[h].signaled_point = copy.signaled_point;
                    g_syncs[h].has_pending = false;
                } else {
                    pthread_mutex_lock(&g_lock);
                }
            }
            points[i] = g_syncs[h].signaled_point;
        } else {
            points[i] = 0;
        }
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
    io_connect_t conn = fd_to_conn(fd);
    bool wait_all = (flags & DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL) != 0;
    for (unsigned i = 0; i < num_handles; ++i) {
        uint32_t h = handles[i];
        uint64_t target = points ? points[i] : 1;
        pthread_mutex_lock(&g_lock);
        if (h == 0 || h >= MAX_DARWIN_SYNCOBJ || !g_syncs[h].in_use) {
            pthread_mutex_unlock(&g_lock);
            errno = ENOENT;
            return -1;
        }
        if (g_syncs[h].signaled_point >= target) {
            pthread_mutex_unlock(&g_lock);
            if (!wait_all) {
                if (first_signaled) *first_signaled = i;
                return 0;
            }
            continue;
        }
        if (g_syncs[h].has_pending && conn != MACH_PORT_NULL) {
            struct darwin_syncobj copy = g_syncs[h];
            pthread_mutex_unlock(&g_lock);
            int rc = wait_syncobj_fence(conn, &copy, timeout_nsec);
            pthread_mutex_lock(&g_lock);
            if (rc == 0) {
                if (copy.signaled_point > g_syncs[h].signaled_point)
                    g_syncs[h].signaled_point = copy.signaled_point;
                g_syncs[h].has_pending = false;
                pthread_mutex_unlock(&g_lock);
                if (!wait_all) {
                    if (first_signaled) *first_signaled = i;
                    return 0;
                }
                continue;
            }
            pthread_mutex_unlock(&g_lock);
            return -1;
        }
        pthread_mutex_unlock(&g_lock);
        if (timeout_nsec == 0) { errno = ETIME; return -1; }
    }
    if (first_signaled) *first_signaled = 0;
    return 0;
}

int drmSyncobjWait(int fd, uint32_t *handles, unsigned num_handles,
                   int64_t timeout_nsec, unsigned flags, uint32_t *first_signaled) {
    return drmSyncobjTimelineWait(fd, handles, NULL, num_handles, timeout_nsec, flags, first_signaled);
}

int drmSyncobjTransfer(int fd, uint32_t dst_handle, uint64_t dst_point,
                       uint32_t src_handle, uint64_t src_point, uint32_t flags) {
    (void)fd; (void)flags; (void)src_point;
    pthread_mutex_lock(&g_lock);
    if (dst_handle > 0 && dst_handle < MAX_DARWIN_SYNCOBJ &&
        src_handle > 0 && src_handle < MAX_DARWIN_SYNCOBJ &&
        g_syncs[dst_handle].in_use && g_syncs[src_handle].in_use) {
        uint64_t pt = dst_point ? dst_point : 1;
        if (g_syncs[src_handle].has_pending) {
            g_syncs[dst_handle].has_pending = true;
            g_syncs[dst_handle].pending_group = g_syncs[src_handle].pending_group;
            g_syncs[dst_handle].pending_fence_seq = g_syncs[src_handle].pending_fence_seq;
            g_syncs[dst_handle].pending_point = pt;
        } else if (g_syncs[src_handle].signaled_point > 0) {
            if (pt > g_syncs[dst_handle].signaled_point)
                g_syncs[dst_handle].signaled_point = pt;
        }
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
            /* Process per-op syncs (signal out syncobjs immediately since VM_BIND is synchronous) */
            const struct drm_panthor_sync_op *sops =
                (const struct drm_panthor_sync_op *)(uintptr_t)ops[i].syncs.array;
            for (uint32_t s = 0; s < ops[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) != 0) {
                    uint64_t pt = sops[s].timeline_value ? sops[s].timeline_value : 1;
                    uint32_t sh = sops[s].handle;
                    drmSyncobjTimelineSignal(fd, &sh, &pt, 1);
                }
            }
        }
        return 0;
    }

    case DRM_IOCTL_PANTHOR_BO_CREATE: {
        struct drm_panthor_bo_create *req = (struct drm_panthor_bo_create *)arg;
        struct Tb375fcMaliBoCreateIn in = {
            .size = req->size,
            .flags = (req->flags & DRM_PANTHOR_BO_NO_MMAP) ? MALI_BO_NO_MMAP : 0,
            .exclusive_vm_id = req->exclusive_vm_id,
        };
        struct Tb375fcMaliBoCreateOut out = {0};
        size_t osz = sizeof(out);
        if (call_mali(conn, MALI_SEL_BO_CREATE, &in, sizeof(in), &out, &osz) != 0) return -1;
        req->handle = out.bo_handle;
        req->size = out.size;
        return 0;
    }

    case DRM_IOCTL_PANTHOR_BO_MMAP_OFFSET: {
        struct drm_panthor_bo_mmap_offset *req = (struct drm_panthor_bo_mmap_offset *)arg;
        req->offset = ((uint64_t)req->handle) << 12;
        return 0;
    }

    case DRM_IOCTL_GEM_CLOSE: {
        struct drm_gem_close *req = (struct drm_gem_close *)arg;
        if (req->handle > 0 && req->handle < MAX_DARWIN_HOST_BOS) {
            pthread_mutex_lock(&g_lock);
            g_host_bos[req->handle].ptr = NULL;
            g_host_bos[req->handle].size = 0;
            g_mmap_bos[req->handle].ptr = NULL;
            g_mmap_bos[req->handle].size = 0;
            pthread_mutex_unlock(&g_lock);
        }
        struct Tb375fcMaliHandleIn in = { .handle = req->handle, .pad = 0 };
        return call_mali(conn, MALI_SEL_BO_DESTROY, &in, sizeof(in), NULL, NULL);
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

        /* Wait for any input syncobjs first */
        for (uint32_t i = 0; i < req->queue_submits.count; ++i) {
            const struct drm_panthor_sync_op *sops =
                (const struct drm_panthor_sync_op *)(uintptr_t)qsubs[i].syncs.array;
            for (uint32_t s = 0; s < qsubs[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) == 0) {
                    uint32_t sh = sops[s].handle;
                    uint64_t pt = sops[s].timeline_value ? sops[s].timeline_value : 1;
                    drmSyncobjTimelineWait(fd, &sh, &pt, 1, 10000000000LL, DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL, NULL);
                }
            }
        }

        pthread_mutex_lock(&g_lock);
        flush_all_host_bos(false);
        pthread_mutex_unlock(&g_lock);

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

        /* Attach fence_seq to all output syncobjs */
        pthread_mutex_lock(&g_lock);
        for (uint32_t i = 0; i < req->queue_submits.count; ++i) {
            const struct drm_panthor_sync_op *sops =
                (const struct drm_panthor_sync_op *)(uintptr_t)qsubs[i].syncs.array;
            for (uint32_t s = 0; s < qsubs[i].syncs.count; ++s) {
                if ((sops[s].flags & DRM_PANTHOR_SYNC_OP_SIGNAL) != 0) {
                    uint32_t sh = sops[s].handle;
                    uint64_t pt = sops[s].timeline_value ? sops[s].timeline_value : 1;
                    if (sh > 0 && sh < MAX_DARWIN_SYNCOBJ && g_syncs[sh].in_use) {
                        g_syncs[sh].has_pending = true;
                        g_syncs[sh].pending_group = req->group_handle;
                        g_syncs[sh].pending_fence_seq = out.fence_seq;
                        g_syncs[sh].pending_point = pt;
                    }
                }
            }
        }
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
