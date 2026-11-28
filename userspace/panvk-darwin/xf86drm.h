/* SPDX-License-Identifier: MIT
 * Minimal <xf86drm.h> compatibility header for Mesa panvk on Darwin (TB375FC Mali-G615).
 */
#ifndef _XF86DRM_H_
#define _XF86DRM_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include "drm-uapi/drm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRM_NODE_PRIMARY 0
#define DRM_NODE_CONTROL 1
#define DRM_NODE_RENDER  2
#define DRM_NODE_MAX     3

#define DRM_BUS_PCI      0
#define DRM_BUS_USB      1
#define DRM_BUS_PLATFORM 2
#define DRM_BUS_HOST1X   3

#define DRM_DEVICE_GET_PCI_REVISION (1 << 0)

#ifndef DRM_CLOEXEC
#define DRM_CLOEXEC O_CLOEXEC
#endif
#ifndef DRM_RDWR
#define DRM_RDWR O_RDWR
#endif

typedef struct _drmVersion {
    int   version_major;
    int   version_minor;
    int   version_patchlevel;
    int   name_len;
    char *name;
    int   date_len;
    char *date;
    int   desc_len;
    char *desc;
} drmVersion, *drmVersionPtr;

typedef struct _drmPciBusInfo {
    uint16_t domain;
    uint8_t  bus;
    uint8_t  dev;
    uint8_t  func;
} drmPciBusInfo, *drmPciBusInfoPtr;

typedef struct _drmPciDeviceInfo {
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t subvendor_id;
    uint16_t subdevice_id;
    uint8_t  revision_id;
} drmPciDeviceInfo, *drmPciDeviceInfoPtr;

typedef struct _drmPlatformBusInfo {
    char fullname[256];
} drmPlatformBusInfo, *drmPlatformBusInfoPtr;

typedef struct _drmPlatformDeviceInfo {
    char **compatible;
} drmPlatformDeviceInfo, *drmPlatformDeviceInfoPtr;

typedef struct _drmDevice {
    char **nodes;
    int    available_nodes;
    int    bustype;
    union {
        drmPciBusInfoPtr      pci;
        drmPlatformBusInfoPtr platform;
    } businfo;
    union {
        drmPciDeviceInfoPtr      pci;
        drmPlatformDeviceInfoPtr platform;
    } deviceinfo;
} drmDevice, *drmDevicePtr;

int           drmIoctl(int fd, unsigned long request, void *arg);
drmVersionPtr drmGetVersion(int fd);
void          drmFreeVersion(drmVersionPtr v);
int           drmGetCap(int fd, uint64_t capability, uint64_t *value);
int           drmCloseBufferHandle(int fd, uint32_t handle);

int  drmGetDevices2(uint32_t flags, drmDevicePtr devices[], int max_devices);
int  drmGetDevice2(int fd, uint32_t flags, drmDevicePtr *device);
void drmFreeDevice(drmDevicePtr *device);
void drmFreeDevices(drmDevicePtr devices[], int count);
int  drmGetNodeTypeFromFd(int fd);
char *drmGetRenderDeviceNameFromFd(int fd);
char *drmGetPrimaryDeviceNameFromFd(int fd);
int  drmDevicesEqual(drmDevicePtr a, drmDevicePtr b);

int drmPrimeHandleToFD(int fd, uint32_t handle, uint32_t flags, int *prime_fd);
int drmPrimeFDToHandle(int fd, int prime_fd, uint32_t *handle);

int drmSyncobjCreate(int fd, uint32_t flags, uint32_t *handle);
int drmSyncobjDestroy(int fd, uint32_t handle);
int drmSyncobjHandleToFD(int fd, uint32_t handle, int *obj_fd);
int drmSyncobjFDToHandle(int fd, int obj_fd, uint32_t *handle);
int drmSyncobjImportSyncFile(int fd, uint32_t handle, int sync_file_fd);
int drmSyncobjExportSyncFile(int fd, uint32_t handle, int *sync_file_fd);
int drmSyncobjWait(int fd, uint32_t *handles, unsigned num_handles,
                   int64_t timeout_nsec, unsigned flags, uint32_t *first_signaled);
int drmSyncobjReset(int fd, const uint32_t *handles, uint32_t handle_count);
int drmSyncobjSignal(int fd, const uint32_t *handles, uint32_t handle_count);
int drmSyncobjTimelineSignal(int fd, const uint32_t *handles, uint64_t *points, uint32_t handle_count);
int drmSyncobjTimelineWait(int fd, uint32_t *handles, uint64_t *points,
                           unsigned num_handles, int64_t timeout_nsec,
                           unsigned flags, uint32_t *first_signaled);
int drmSyncobjQuery(int fd, uint32_t *handles, uint64_t *points, uint32_t handle_count);
int drmSyncobjQuery2(int fd, uint32_t *handles, uint64_t *points,
                     uint32_t handle_count, uint32_t flags);
int drmSyncobjTransfer(int fd, uint32_t dst_handle, uint64_t dst_point,
                       uint32_t src_handle, uint64_t src_point, uint32_t flags);

/* Darwin helpers for Tb375fcMali BO open/mmap/munmap/zero */
int   panthor_darwin_open_render_node(void);
void  panthor_darwin_close_render_node(int fd);
void *panthor_darwin_mmap_bo(int fd, uint64_t mmap_offset, size_t size);
int   panthor_darwin_munmap_bo(int fd, void *addr, size_t size);
void  panthor_darwin_zero_bo_range(int fd, uint32_t bo_handle, uint64_t offset, uint64_t size);

#ifdef __cplusplus
}
#endif

#endif /* _XF86DRM_H_ */
