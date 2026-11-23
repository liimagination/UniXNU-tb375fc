// SPDX-License-Identifier: BSD-2-Clause-Patent
// Board probe, IOSurface ABI accessors, and late display registration glue for macOS 27.3 (26C5035r).
#include "OriginalLegacyFramebufferABI.h"
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceMemory.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSData.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <pexpert/pexpert.h>

extern kmod_info_t g_kernel_kmod_info;
extern "C" void _GLOBAL__sub_I_NativeLegacyDisplay_cpp(void) __asm("__GLOBAL__sub_I_NativeLegacyDisplay.cpp");
extern "C" void _GLOBAL__sub_I_NativeGOPPresenter_cpp(void) __asm("__GLOBAL__sub_I_NativeGOPPresenter.cpp");
extern "C" IOService *native_legacy_display_allocate(void);
extern "C" void native_swiftshader_accelerator_probe(void);

extern "C" IOMemoryDescriptor *iosurface_get_memory_descriptor(const void *)
    __asm("__ZNK9IOSurface19getMemoryDescriptorEv");
extern "C" uint32_t iosurface_get_width(const void *)
    __asm("__ZNK9IOSurface8getWidthEv");
extern "C" uint32_t iosurface_get_height(const void *)
    __asm("__ZNK9IOSurface9getHeightEv");
extern "C" uint32_t iosurface_get_bytes_per_row(const void *)
    __asm("__ZNK9IOSurface14getBytesPerRowEv");
extern "C" size_t iosurface_get_alloc_size(const void *)
    __asm("__ZNK9IOSurface12getAllocSizeEv");
extern "C" uint32_t iosurface_get_pixel_format(const void *)
    __asm("__ZNK9IOSurface14getPixelFormatEv");
extern "C" uint32_t iosurface_get_plane_count(const void *)
    __asm("__ZNK9IOSurface13getPlaneCountEv");

extern "C" {
struct NativeLegacyDisplayState {
    uint64_t stage;
    int64_t  meta_status;
    uint64_t object;
    uint64_t object_size;
    uint64_t init_result;
    uint64_t probe_match;
    uint64_t parent_match;
    uint64_t start_result;
    uint64_t released;
    uint64_t guard_slot;
    uint64_t gop_map;
    uint64_t gop_virtual;
    uint64_t gop_physical;
    uint64_t gop_bytes;
    uint32_t gop_width;
    uint32_t gop_height;
    uint64_t client_count;
    int64_t  client_status;
    uint64_t swap_validated;
    uint64_t swap_transferred;
    uint64_t swap_retired;
    uint64_t vsync_ticks;
};

volatile NativeLegacyDisplayState gNativeLegacyDisplayState = {0, -1};
}

extern "C" IOMemoryDescriptor *native_legacy_surface_descriptor(OSObject *surface) {
    return surface ? iosurface_get_memory_descriptor(surface) : nullptr;
}
extern "C" uint64_t native_legacy_surface_width(OSObject *surface) {
    return surface ? iosurface_get_width(surface) : 0;
}
extern "C" uint64_t native_legacy_surface_height(OSObject *surface) {
    return surface ? iosurface_get_height(surface) : 0;
}
extern "C" uint64_t native_legacy_surface_stride(OSObject *surface) {
    if (!surface) return 0;
    uint64_t row = iosurface_get_bytes_per_row(surface);
    uint64_t w = iosurface_get_width(surface);
    uint32_t fmt = iosurface_get_pixel_format(surface);
    if ((fmt == 0x26424741U || fmt == 0x42475241U) && w > 0 && row < w * 4) {
        row = w * 4;
    }
    return row;
}
extern "C" uint64_t native_legacy_surface_allocation(OSObject *surface) {
    return surface ? iosurface_get_alloc_size(surface) : 0;
}
extern "C" uint32_t native_legacy_surface_format(OSObject *surface) {
    return surface ? iosurface_get_pixel_format(surface) : 0;
}
extern "C" uint32_t native_legacy_surface_planes(OSObject *surface) {
    return surface ? iosurface_get_plane_count(surface) : 0;
}

extern "C" __attribute__((noreturn)) void native_legacy_backend_guard(unsigned slot, void *obj, void *caller) {
    gNativeLegacyDisplayState.guard_slot = slot;
    panic("NativeLegacyDisplay: unimplemented virtual slot %u (this=%p caller=%p)", slot, obj, caller);
}

extern "C" bool native_legacy_gop_mapping_observed(IOService *service, IOMemoryMap *map, uint32_t width, uint32_t height) {
    if (!service || !map) return false;
    gNativeLegacyDisplayState.gop_map = reinterpret_cast<uint64_t>(map);
    gNativeLegacyDisplayState.gop_virtual = map->getVirtualAddress();
    gNativeLegacyDisplayState.gop_physical = map->getPhysicalAddress();
    gNativeLegacyDisplayState.gop_bytes = map->getLength();
    gNativeLegacyDisplayState.gop_width = width;
    gNativeLegacyDisplayState.gop_height = height;
    return true;
}

extern "C" void native_legacy_registration_observed(IOService *) {
    gNativeLegacyDisplayState.stage = 6;
}
extern "C" void native_legacy_swap_stage(unsigned) {}
extern "C" void native_legacy_client_enter(task_t, uint32_t type) {
    gNativeLegacyDisplayState.client_count = gNativeLegacyDisplayState.client_count + 1;
    if (gNativeLegacyDisplayState.client_count <= 16)
        IOLog("NativeLegacyDisplay: client request type=%u\n", type);
}
extern "C" void native_legacy_client_observed(IOReturn status, IOUserClient *) {
    gNativeLegacyDisplayState.client_status = status;
    if (gNativeLegacyDisplayState.client_count <= 16)
        IOLog("NativeLegacyDisplay: client result=%#x\n", status);
}
extern "C" void native_legacy_pm_observed(unsigned, IOReturn, unsigned long) {}
extern "C" void native_legacy_frame_observed(unsigned, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t) {}
extern "C" void native_legacy_frame_colors_observed(const uint32_t *, const uint64_t *, uint64_t) {}
extern "C" void native_legacy_swap_validation_observed(bool accepted, unsigned reason, const void *, const void *) {
    static unsigned observed[2];
    if (__atomic_fetch_add(&observed[accepted], 1, __ATOMIC_RELAXED) < 16)
        IOLog("NativeLegacyDisplay: swap validated=%d reason=%u\n", accepted, reason);
    if (accepted) gNativeLegacyDisplayState.swap_validated = gNativeLegacyDisplayState.swap_validated + 1;
}
extern "C" void native_legacy_surface_rejected(uint64_t w, uint64_t h, uint32_t fmt, uint32_t planes,
                                               uint64_t stride, uint64_t alloc, uint64_t len) {
    IOLog("NativeLegacyDisplay: surface rejected %llux%llu fmt=0x%x planes=%u stride=%llu alloc=%llu len=%llu\n",
          w, h, fmt, planes, stride, alloc, len);
}
extern "C" bool native_legacy_prepare_scanout(void) { return true; }
extern "C" void native_legacy_transfer_observed(IOReturn status, const void *, uint64_t bytes) {
    static unsigned observed;
    if (__atomic_fetch_add(&observed, 1, __ATOMIC_RELAXED) < 16)
        IOLog("NativeLegacyDisplay: transfer result=%#x bytes=%llu\n", status, bytes);
    if (status == kIOReturnSuccess) gNativeLegacyDisplayState.swap_transferred = gNativeLegacyDisplayState.swap_transferred + 1;
}
extern "C" void native_legacy_copy_timing_observed(uint64_t, uint64_t) {}
extern "C" void native_legacy_retirement_observed(IOService *) {
    gNativeLegacyDisplayState.swap_retired = gNativeLegacyDisplayState.swap_retired + 1;
    if (gNativeLegacyDisplayState.swap_retired <= 10)
        IOLog("NativeLegacyDisplay: frame retired\n");
}
extern "C" void native_legacy_gpu_fallback_observed(int) {}
extern "C" void native_legacy_vsync_observed(unsigned event, unsigned) {
    if (event == 1) gNativeLegacyDisplayState.vsync_ticks = gNativeLegacyDisplayState.vsync_ticks + 1;
}

extern "C" bool native_adreno_fences(uint32_t *submitted, uint32_t *retired) {
    if (submitted) *submitted = 0;
    if (retired) *retired = 0;
    return false;
}
extern "C" bool native_adreno_fences_detail(uint32_t *a, uint32_t *b, uint32_t *c) {
    if (a) *a = 0;
    if (b) *b = 0;
    if (c) *c = 0;
    return false;
}
extern "C" bool native_gpu_trace_enabled(void) { return false; }
extern "C" void native_gpu_trace_record(uint32_t, uint32_t, uint64_t, uint64_t, uint64_t, uint64_t,
                                        uint64_t, uint64_t, uint64_t, uint64_t, uint64_t) {}
extern "C" bool native_legacy_gpu_offer(void *, IOMemoryDescriptor *, uint64_t, uint32_t, uint32_t, uint32_t) {
    return false;
}
extern "C" uint32_t native_ufs_dpu_map(unsigned, const uint64_t *, unsigned) { return 0; }
extern "C" {
volatile uint32_t native_ufs_dpu_state = 0;
}

extern "C" void native_legacy_display_probe(void) {
    unsigned phase = 0;
    if (!PE_parse_boot_argn("offline-legacy-phase", &phase, sizeof(phase)) || phase == 0) {
        return;
    }
    static bool probed_once = false;
    if (probed_once) return;
    probed_once = true;

    OSReturn kr_surf = OSKextLoadKextWithIdentifier("com.apple.iokit.IOSurface");
    OSReturn kr_av   = OSKextLoadKextWithIdentifier("com.apple.iokit.IOAVFamily");
    OSReturn kr_mfb  = OSKextLoadKextWithIdentifier("com.apple.iokit.IOMobileGraphicsFamily");
    IOLog("NativeLegacyDisplay: kext load iosurface=0x%x ioav=0x%x iomfb=0x%x\n",
          kr_surf, kr_av, kr_mfb);
    if (kr_surf != kOSReturnSuccess || kr_av != kOSReturnSuccess || kr_mfb != kOSReturnSuccess) {
        gNativeLegacyDisplayState.meta_status = -3;
        return;
    }

    OSDictionary *surf_match = IOService::serviceMatching("IOSurfaceRoot");
    if (surf_match) {
        IOService *surf_root = IOService::waitForMatchingService(surf_match, 5000000000ULL);
        surf_match->release();
        IOLog("NativeLegacyDisplay: IOSurfaceRoot ready=%d\n", surf_root != nullptr);
        OSSafeReleaseNULL(surf_root);
    }

    static bool registered = false;
    if (!registered) {
        void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
        if (!handle) {
            gNativeLegacyDisplayState.meta_status = -2;
            IOLog("NativeLegacyDisplay: preModLoad failed\n");
            return;
        }
        _GLOBAL__sub_I_NativeLegacyDisplay_cpp();
        _GLOBAL__sub_I_NativeGOPPresenter_cpp();
        OSReturn post_kr = OSMetaClass::postModLoad(handle);
        gNativeLegacyDisplayState.meta_status = post_kr;
        IOLog("NativeLegacyDisplay: postModLoad status=0x%x\n", post_kr);
        if (post_kr != kOSReturnSuccess) {
            return;
        }
        registered = true;
    }
    gNativeLegacyDisplayState.stage = 1;

    PE_Video video = {};
    if (PE_current_console(&video) != 0 || !video.v_baseAddr || !video.v_width || !video.v_height) {
        gNativeLegacyDisplayState.stage = 2;
        IOLog("NativeLegacyDisplay: late_probe stage=%llu gop=%ux%u phys=0x%llx virt=0x%llx bytes=0x%llx init=%llu attach=%llu start=%llu\n",
              gNativeLegacyDisplayState.stage, gNativeLegacyDisplayState.gop_width, gNativeLegacyDisplayState.gop_height,
              gNativeLegacyDisplayState.gop_physical, gNativeLegacyDisplayState.gop_virtual, gNativeLegacyDisplayState.gop_bytes,
              gNativeLegacyDisplayState.init_result, gNativeLegacyDisplayState.parent_match, gNativeLegacyDisplayState.start_result);
        return;
    }

    IOService *root = IOService::getPlatform();
    if (!root) return;

    IOService *provider = OSTypeAlloc(IOService);
    if (!provider || !provider->init()) {
        OSSafeReleaseNULL(provider);
        return;
    }
    provider->setName("display0");
    uint32_t dotPitch = 273;
    if (OSData *data = OSData::withBytes(&dotPitch, sizeof(dotPitch))) {
        provider->setProperty("dot-pitch", data);
        data->release();
    }
    provider->setProperty("BuiltIn", kOSBooleanTrue);
    IODeviceMemory *mem = IODeviceMemory::withRange(video.v_baseAddr, video.v_rowBytes * video.v_height);
    if (mem) {
        OSArray *arr = OSArray::withObjects((const OSObject **)&mem, 1, 1);
        if (arr) {
            provider->setDeviceMemory(arr);
            arr->release();
        }
        mem->release();
    }
    provider->attach(root);
    provider->start(root);

    IOService *display = native_legacy_display_allocate();
    if (!display) {
        provider->release();
        return;
    }
    gNativeLegacyDisplayState.object = reinterpret_cast<uint64_t>(display);
    gNativeLegacyDisplayState.object_size = sizeof(IOMobileFramebufferLegacy);

    OSDictionary *props = OSDictionary::withCapacity(4);
    bool inited = props && display->init(props);
    OSSafeReleaseNULL(props);
    gNativeLegacyDisplayState.init_result = inited ? 1 : 0;
    if (!inited || phase < 2) {
        display->release();
        provider->release();
        gNativeLegacyDisplayState.released = 1;
        gNativeLegacyDisplayState.stage = 8;
        IOLog("NativeLegacyDisplay: late_probe stage=%llu gop=%ux%u phys=0x%llx virt=0x%llx bytes=0x%llx init=%llu attach=%llu start=%llu\n",
              gNativeLegacyDisplayState.stage, gNativeLegacyDisplayState.gop_width, gNativeLegacyDisplayState.gop_height,
              gNativeLegacyDisplayState.gop_physical, gNativeLegacyDisplayState.gop_virtual, gNativeLegacyDisplayState.gop_bytes,
              gNativeLegacyDisplayState.init_result, gNativeLegacyDisplayState.parent_match, gNativeLegacyDisplayState.start_result);
        return;
    }

    SInt32 score = 0;
    IOService *matched = display->probe(provider, &score);
    gNativeLegacyDisplayState.probe_match = (matched == display) ? 1 : 0;
    bool attached = display->attach(provider);
    gNativeLegacyDisplayState.parent_match = attached ? 1 : 0;
    gNativeLegacyDisplayState.stage = 5;

    bool started = attached && display->start(provider);
    gNativeLegacyDisplayState.start_result = started ? 1 : 0;
    if (started && phase >= 4) {
        unsigned reg = 0;
        if (PE_parse_boot_argn("offline-legacy-register", &reg, sizeof(reg)) && reg == 1) {
            display->registerService();
        }
        native_swiftshader_accelerator_probe();
    } else {
        if (attached) display->detach(provider);
        display->release();
        provider->release();
        gNativeLegacyDisplayState.released = 1;
        gNativeLegacyDisplayState.stage = 8;
    }
    IOLog("NativeLegacyDisplay: late_probe stage=%llu gop=%ux%u phys=0x%llx virt=0x%llx bytes=0x%llx init=%llu attach=%llu start=%llu\n",
          gNativeLegacyDisplayState.stage, gNativeLegacyDisplayState.gop_width, gNativeLegacyDisplayState.gop_height,
          gNativeLegacyDisplayState.gop_physical, gNativeLegacyDisplayState.gop_virtual, gNativeLegacyDisplayState.gop_bytes,
          gNativeLegacyDisplayState.init_result, gNativeLegacyDisplayState.parent_match, gNativeLegacyDisplayState.start_result);
}

extern "C" void native_legacy_display_late_probe(void) {
    unsigned phase = 0;
    if (!PE_parse_boot_argn("offline-legacy-phase", &phase, sizeof(phase)) || phase != 4) {
        return;
    }
    native_legacy_display_probe();
}
