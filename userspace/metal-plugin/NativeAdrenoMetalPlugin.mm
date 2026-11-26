// SPDX-License-Identifier: MIT
// TB375FC macOS 27.3 In-Process Metal Plugin Bundle (arm64e, PAC-bridged).
// Runs m2v (87a869f) + Mesa PanVK (libvulkan_panfrost.dylib) directly inside each
// Metal client process (WindowServer, Dock, Finder, WallpaperImageExtension, etc.),
// eliminating the legacy Unix-socket RPC daemon bottleneck completely.

#import <Foundation/Foundation.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <mach/mach_time.h>
#include <objc/runtime.h>
#include <os/log.h>
#include <pthread.h>
#include <pwd.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <vector>

#include "adreno-metal-src/IOGPUMetalDeviceShim.h"
#include "adreno-metal-src/NativeBridgeMetalFrontend.h"
#include "adreno-metal-src/bridge/NativeMetalObjects.h"
#include "../mali-panvk/Tb375fcMaliABI.h"

extern "C" int sandbox_check(pid_t pid, const char *operation, int type, ...);

namespace {

static bool bootArgToken(const char *key, char *out, size_t outSize) {
    char args[1100] = {};
    size_t sz = sizeof(args) - 1;
    if (sysctlbyname("kern.bootargs", args, &sz, nullptr, 0) != 0) return false;
    const size_t klen = strlen(key);
    for (const char *p = args; *p;) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        const char *end = p;
        while (*end && *end != ' ' && *end != '\t') ++end;
        if ((size_t)(end - p) >= klen && strncmp(p, key, klen) == 0) {
            if (out && outSize) {
                size_t vlen = (size_t)(end - (p + klen));
                if (vlen >= outSize) vlen = outSize - 1;
                memcpy(out, p + klen, vlen);
                out[vlen] = '\0';
            }
            return true;
        }
        p = end;
    }
    return false;
}

static bool bootArgIsZero(const char *prefixWithEq) {
    char val[32] = {};
    return bootArgToken(prefixWithEq, val, sizeof(val)) && strcmp(val, "0") == 0;
}

static io_connect_t maliLogConnect() {
    static io_connect_t sConn = MACH_PORT_NULL;
    static dispatch_once_t sOnce;
    dispatch_once(&sOnce, ^{
        CFMutableDictionaryRef match = IOServiceMatching(TB375FC_MALI_SERVICE_NAME);
        if (!match) return;
        io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, match);
        if (svc == IO_OBJECT_NULL) return;
        io_connect_t c = MACH_PORT_NULL;
        if (IOServiceOpen(svc, mach_task_self(), 0, &c) == KERN_SUCCESS && c != MACH_PORT_NULL) {
            sConn = c;
        }
        IOObjectRelease(svc);
    });
    return sConn;
}

static void maliKernelLog(const char *fmt, ...) {
    Tb375fcMaliDebugLogIn in = {};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(in.msg, sizeof(in.msg), fmt, ap);
    va_end(ap);
    os_log(OS_LOG_DEFAULT, "%{public}s", in.msg);
    io_connect_t c = maliLogConnect();
    if (c != MACH_PORT_NULL) {
        (void)IOConnectCallStructMethod(c, MALI_SEL_DEBUG_LOG, &in, sizeof(in), nullptr, nullptr);
    }
}

static std::string canonicalizePath(const char *path) {
    if (!path || !path[0]) return {};
    std::string s(path);
    while (s.size() > 1 && s.back() == '/') s.pop_back();
    char resolved[PATH_MAX] = {};
    if (realpath(s.c_str(), resolved)) {
        std::string r(resolved);
        while (r.size() > 1 && r.back() == '/') r.pop_back();
        return r;
    }
    // If leaf does not exist yet, resolve longest existing ancestor so /var -> /private/var matches sandbox rules
    size_t slash = s.rfind('/');
    if (slash != std::string::npos && slash > 0) {
        std::string parent = canonicalizePath(s.substr(0, slash).c_str());
        if (!parent.empty()) return parent + s.substr(slash);
    }
    return s;
}

static bool canWriteDirQuietly(const char *rawDir, bool useSandboxCheck = true) {
    if (!rawDir || !rawDir[0]) return false;
    std::string dir = canonicalizePath(rawDir);
    if (dir.empty()) return false;
    if (useSandboxCheck && sandbox_check(getpid(), nullptr, 0) == 1) {
        std::string probe = dir + "/.write-probe";
        if (sandbox_check(getpid(), "file-write-create", 1 | 0x40000000, probe.c_str()) != 0) {
            return false;
        }
    }
    mkdir(dir.c_str(), 0700);
    return access(dir.c_str(), W_OK) == 0;
}

static bool findPanvkLibrary(char *out, size_t outSize) {
    Dl_info info = {};
    if (dladdr((const void *)&findPanvkLibrary, &info) && info.dli_fname) {
        const char *slash = strrchr(info.dli_fname, '/');
        if (slash) {
            size_t dirLen = (size_t)(slash - info.dli_fname);
            if (dirLen + sizeof("/libvulkan_panfrost.dylib") < outSize) {
                memcpy(out, info.dli_fname, dirLen);
                strcpy(out + dirLen, "/libvulkan_panfrost.dylib");
                if (access(out, R_OK) == 0) return true;
            }
        }
    }
    static const char *const kFallbacks[] = {
        "/Library/GPUBundles/NativeAdrenoMetal.bundle/Contents/MacOS/libvulkan_panfrost.dylib",
        "/private/var/ngpu/lib/libvulkan_panfrost.dylib",
    };
    for (const char *cand : kFallbacks) {
        if (access(cand, R_OK) == 0) {
            strlcpy(out, cand, outSize);
            return true;
        }
    }
    return false;
}

static void startWindowServerHeartbeat() {
    pthread_t th;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&th, &attr, [](void *) -> void * {
        pthread_setname_np("mali-ws-hb");
        uint64_t prevCommits = 0;
        uint64_t prevNs = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
        for (;;) {
            sleep(10);
            char status[192] = {};
            native_bridge_metal_status(status, sizeof(status));
            unsigned long long curCommits = 0, curFail = 0;
            sscanf(status, "c%llu/%llu", &curCommits, &curFail);
            uint64_t nowNs = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
            double dt = (nowNs > prevNs) ? (double)(nowNs - prevNs) / 1e9 : 10.0;
            uint64_t delta = (curCommits >= prevCommits) ? (curCommits - prevCommits) : 0;
            double fps = (dt > 0.0) ? ((double)delta / dt) : 0.0;
            prevCommits = curCommits;
            prevNs = nowNs;
            maliKernelLog("[mali-ws] dt=%.1fs commits=+%llu fps=%.1f | %.110s",
                          dt, (unsigned long long)delta, fps, status);
        }
        return nullptr;
    }, nullptr);
    pthread_attr_destroy(&attr);
}

} // namespace

@interface IOGPUMetalDevice : NSObject
@end

@interface NativeAdrenoMetalDevice : IOGPUMetalDevice
- (id)initWithAcceleratorPort:(unsigned)port;
@end

@implementation NativeAdrenoMetalDevice
- (id)initWithAcceleratorPort:(unsigned)port {
    static NSMutableArray *sKept = nil;
    static dispatch_once_t sOnce;
    dispatch_once(&sOnce, ^{ sKept = [NSMutableArray array]; });
    @synchronized(sKept) {
        [sKept addObject:self];
    }
    if (!port) return nil;

    static id sDevice = nil;
    static bool sTried = false;
    @synchronized(sKept) {
        if (sTried) return sDevice;
        sTried = true;

        const char *prog = getprogname() ? getprogname() : "?";
        const bool isWS = (strcmp(prog, "WindowServer") == 0);

        // Configure PanVK & QuartzCore environment before loading PanVK / m2v.
        // Imported IOSurfaces use VK_IMAGE_TILING_LINEAR (never AFBC per panvk_image.c), so leaving
        // AFBC enabled by default compresses all private OPTIMAL render targets (50-70% less DRAM BW!).
        // Pass boot-arg panvk-debug=noafbc to disable AFBC if needed for diagnostics.
        char panvkDbg[64] = {};
        if (bootArgToken("panvk-debug=", panvkDbg, sizeof(panvkDbg)) && panvkDbg[0]) {
            setenv("PANVK_DEBUG", panvkDbg, 1);
        }
        setenv("CA_DISABLE_FRAMEBUFFER_COMPRESSION", "1", 1);
        setenv("CA_DISPLAY_EDR_HEADROOM", "1", 1);
        setenv("CA_FP16_MIN_HEADROOM", "1e30", 1);
        setenv("CA_ALLOW_EDR_MIRRORING", "0", 1);
        setenv("METAL2VULKAN_COMPILE_ON_ASKING_THREAD", "1", 1);
        CFPreferencesSetAppValue(CFSTR("CAEnableFramebufferCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
        CFPreferencesSetAppValue(CFSTR("CAEnableIOSurfaceCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);

        char panvkPath[1024] = {};
        if (!findPanvkLibrary(panvkPath, sizeof(panvkPath))) {
            maliKernelLog("[mali-metal] %s(%d) FAIL: libvulkan_panfrost.dylib not found", prog, (int)getpid());
            return nil;
        }

        uint64_t regID = 0;
        (void)IORegistryEntryGetRegistryEntryID((io_registry_entry_t)port, &regID);

        // Build candidate writable shader-cache directories.
        // External HOME / pw_dir paths check sandbox_check first to avoid deny spam;
        // the process's OWN Darwin per-process cache/temp dirs (C/ and T/) are tested directly
        // with mkdir + access(W_OK) so sandboxed daemons (WindowServer, iconservicesagent,
        // NotificationCenter, GeneralSettings, Widgets) get real disk caches for m2v + PanVK!
        struct CacheCand { std::string path; bool checkSandbox; };
        std::vector<CacheCand> candidates;
        auto addCandidate = [&](const std::string &raw, bool checkSb) {
            std::string c = canonicalizePath(raw.c_str());
            if (c.empty()) return;
            for (const auto &existing : candidates) {
                if (existing.path == c) return;
            }
            candidates.push_back({c, checkSb});
        };
        {
            // 1. Container/User HOME (App Sandbox widgets, GeneralSettings, NotificationCenter, Calendar, etc.)
            if (const char *home = getenv("HOME"); home && home[0]) {
                std::string croot = canonicalizePath((std::string(home) + "/Library/Caches").c_str());
                if (canWriteDirQuietly(croot.c_str(), true)) {
                    addCandidate(croot + "/org.unixnu.metal", true);
                }
            }
            // 2. pw_dir fallback (unsandboxed user daemons where HOME may be unset)
            passwd pw = {}, *found = nullptr;
            std::vector<char> pwbuf(16384);
            if (getpwuid_r(getuid(), &pw, pwbuf.data(), pwbuf.size(), &found) == 0 && found && pw.pw_dir && pw.pw_dir[0]) {
                std::string croot = canonicalizePath((std::string(pw.pw_dir) + "/Library/Caches").c_str());
                if (canWriteDirQuietly(croot.c_str(), true)) {
                    addCandidate(croot + "/org.unixnu.metal", true);
                }
            }
            // 3. Per-process Darwin user cache dir (/private/var/folders/.../C/com.apple.metal/m2v or C/m2v)
            char cdir[PATH_MAX] = {};
            if (confstr(_CS_DARWIN_USER_CACHE_DIR, cdir, sizeof(cdir) - 32) && cdir[0]) {
                std::string rcdir = canonicalizePath(cdir);
                if (!rcdir.empty()) {
                    std::string metalDir = rcdir + "/com.apple.metal";
                    if (canWriteDirQuietly(metalDir.c_str(), false)) {
                        addCandidate(metalDir + "/m2v", false);
                    }
                    addCandidate(rcdir + "/m2v", false);
                }
            }
            // 4. Per-process Darwin temp dir (/private/var/folders/.../T/m2v)
            char tdir[PATH_MAX] = {};
            if (confstr(_CS_DARWIN_USER_TEMP_DIR, tdir, sizeof(tdir) - 32) && tdir[0]) {
                std::string rtdir = canonicalizePath(tdir);
                if (!rtdir.empty()) addCandidate(rtdir + "/m2v", false);
            }
            if (const char *tmpdir = getenv("TMPDIR"); tmpdir && tmpdir[0]) {
                std::string rtdir = canonicalizePath(tmpdir);
                if (!rtdir.empty()) addCandidate(rtdir + "/m2v", false);
            }
            // 5. System /private/var/tmp fallback (WindowServer & root daemons)
            char vtmp[128] = {};
            snprintf(vtmp, sizeof(vtmp), "/private/var/tmp/tb375fc-m2v-%u", (unsigned)getuid());
            addCandidate(vtmp, true);
        }

        id dev = nil;
        std::string chosenCache;
        for (const auto &cand : candidates) {
            if (!canWriteDirQuietly(cand.path.c_str(), cand.checkSandbox)) continue;
            std::string mesaDir = cand.path + "/panvk";
            if (canWriteDirQuietly(mesaDir.c_str(), cand.checkSandbox)) {
                setenv("MESA_SHADER_CACHE_DIR", mesaDir.c_str(), 1);
            }
            @try {
                dev = native_bridge_metal_open(panvkPath, cand.path.c_str(), regID, nullptr);
            } @catch (NSException *ex) {
                dev = nil;
            }
            if (dev) {
                chosenCache = cand.path;
                break;
            }
        }

        if (!dev) {
            setenv("METAL2VULKAN_SHADER_CACHE_OPTIONAL", "1", 1);
            setenv("MESA_SHADER_CACHE_DISABLE", "true", 1);
            const char *fallbackPath = candidates.empty() ? "/private/var/tmp/m2v-nocache" : candidates[0].path.c_str();
            @try {
                dev = native_bridge_metal_open(panvkPath, fallbackPath, regID, nullptr);
            } @catch (NSException *ex) {
                dev = nil;
            }
            if (dev) chosenCache = "optional(in-mem)";
        }

        if (!dev) {
            maliKernelLog("[mali-metal] %s(%d,uid=%u) OPEN FAIL: %.150s",
                          prog, (int)getpid(), (unsigned)getuid(), nmb_last_error());
            return nil;
        }

        iogpu::install(dev, (io_service_t)port);

        native_bridge_metal_problem_hook = [](const char *problem) {
            maliKernelLog("[mali-problem] %s(%d): %.120s",
                          getprogname() ? getprogname() : "?", (int)getpid(), problem ? problem : "");
        };

        maliKernelLog("[mali-metal] %s(%d,uid=%u) OPEN OK reg=0x%llx cache=%.64s",
                      prog, (int)getpid(), (unsigned)getuid(),
                      (unsigned long long)regID, chosenCache.c_str());

        if (isWS) {
            startWindowServerHeartbeat();
        }

        sDevice = dev;
        return dev;
    }
}
@end

// ---- WindowServer Frame Pacing Optimizations (from upstream AdrenoMetal) ----
// 1. CA_FORCE_TRIPLE_BUFFERED_LATENCY: renders a frame ahead instead of stalling after >1 refresh frames.
__attribute__((constructor)) static void tb375fcTripleBufferInit() {
    const char *prog = getprogname();
    if (!prog || strcmp(prog, "WindowServer") != 0 || bootArgIsZero("ca-triple-buffer=")) return;
    auto setOption = reinterpret_cast<void (*)(unsigned, bool)>(dlsym(RTLD_DEFAULT, "CASetDebugOption"));
    auto optionName = reinterpret_cast<const char *(*)(unsigned)>(dlsym(RTLD_DEFAULT, "CAGetDebugOptionEnvVariableName"));
    unsigned index = ~0u;
    for (unsigned i = 0; optionName && i < 1024; ++i) {
        const char *name = optionName(i);
        if (!name) break;
        if (strcmp(name, "CA_FORCE_TRIPLE_BUFFERED_LATENCY") == 0) { index = i; break; }
    }
    const bool ok = (setOption && index != ~0u);
    if (ok) setOption(index, true);
    maliKernelLog("[mali-ws] CA_FORCE_TRIPLE_BUFFERED_LATENCY %s (idx=%u)", ok ? "ON" : "not-found", index);
}

// 2. CAWindowServerDisplay freeWakeup: eliminates SkyLight's 16.7ms VBL grid quantization penalty.
static std::atomic<uint64_t> gWsUpdateBegan{0}, gWsLastAsk{0};
static double wsHostSeconds(uint64_t t) {
    static mach_timebase_info_data_t base;
    if (!base.denom) mach_timebase_info(&base);
    return (double)t * base.numer / base.denom / 1e9;
}
static uint64_t wsHostTicks(double seconds) {
    static mach_timebase_info_data_t base;
    if (!base.denom) mach_timebase_info(&base);
    return (uint64_t)(seconds * 1e9 * base.denom / base.numer);
}
__attribute__((constructor)) static void tb375fcFreeWakeupInit() {
    const char *prog = getprogname();
    if (!prog || strcmp(prog, "WindowServer") != 0 || bootArgIsZero("ca-free-wakeup=")) return;
    unsigned hz = 120;
    char hzBuf[16] = {};
    if (bootArgToken("vsync=", hzBuf, sizeof(hzBuf))) {
        int v = atoi(hzBuf);
        if (v >= 30 && v <= 240) hz = (unsigned)v;
    }
    const double refresh = 1.0 / hz;
    Class display = objc_getClass("CAWindowServerDisplay");
    static SEL wakeSel = sel_registerName("nextWakeupTime");
    static SEL beginSel = sel_registerName("beginExternalUpdate:usingSoftwareRenderer:");
    Method wake = display ? class_getInstanceMethod(display, wakeSel) : nullptr;
    Method begin = display ? class_getInstanceMethod(display, beginSel) : nullptr;
    if (!wake || !begin || method_getTypeEncoding(wake)[0] != 'd' || method_getNumberOfArguments(begin) != 4) {
        maliKernelLog("[mali-ws] freeWakeup skipped (missing selector)");
        return;
    }
    static auto wakeGrid = reinterpret_cast<double (*)(id, SEL)>(method_getImplementation(wake));
    static auto beginOriginal = reinterpret_cast<uint64_t (*)(id, SEL, void *, uint64_t)>(method_getImplementation(begin));
    method_setImplementation(begin, imp_implementationWithBlock(^uint64_t(id self, void *update, uint64_t software) {
        const uint64_t now = mach_absolute_time(), asked = gWsLastAsk.load(std::memory_order_relaxed);
        gWsUpdateBegan.store(asked && now - asked < wsHostTicks(0.003) ? asked : now, std::memory_order_relaxed);
        return beginOriginal(self, beginSel, update, software);
    }));
    method_setImplementation(wake, imp_implementationWithBlock(^double(id self) {
        const double grid = wakeGrid(self, wakeSel);
        gWsLastAsk.store(mach_absolute_time(), std::memory_order_relaxed);
        const uint64_t began = gWsUpdateBegan.load(std::memory_order_relaxed);
        if (grid <= 0 || !began) return grid;
        const double now = wsHostSeconds(mach_absolute_time()), start = wsHostSeconds(began);
        if (now - start > 1.0) return grid;
        const double due = start + refresh - 0.0005, ahead = start + 1.25 * refresh + 0.001;
        if (now < due) return due < grid ? due : grid;
        return now > ahead ? now : ahead;
    }));
    maliKernelLog("[mali-ws] freeWakeup ON (%u Hz, %.2f ms)", hz, refresh * 1e3);
}

// 3. MenuBarAgent CenterHUD / Dynamic Island downward offset (boot-arg island-y=<pt>, default 17.5pt).
// Moves the top-edge-hugging Mac17,11 CenterHUD pill down so it aligns vertically with menu bar icons.
__attribute__((constructor)) static void tb375fcCenterHudOffsetInit() {
    const char *prog = getprogname();
    if (!prog || strcmp(prog, "MenuBarAgent") != 0) return;
    double dy = 17.5;
    char yBuf[16] = {};
    if (bootArgToken("island-y=", yBuf, sizeof(yBuf))) {
        dy = atof(yBuf);
    }
    if (dy <= 0.0 || dy > 48.0) return;
    Class screenCls = objc_getClass("NSScreen");
    if (!screenCls) return;
    static const char *const kSels[] = {
        "_centerHUDHardwareFrame",
        "_centerHUDDefaultFrame",
        "_centerHUDMinimalObstructionFrame",
    };
    unsigned hooked = 0;
    for (const char *sname : kSels) {
        SEL sel = sel_registerName(sname);
        Method m = class_getInstanceMethod(screenCls, sel);
        if (!m) continue;
        const char *enc = method_getTypeEncoding(m);
        if (!enc || (strncmp(enc, "{CGRect=", 8) != 0 && strncmp(enc, "{_NSRect=", 9) != 0) ||
            method_getNumberOfArguments(m) != 2) continue;
        using RectFn = NSRect (*)(id, SEL);
        RectFn orig = reinterpret_cast<RectFn>(method_getImplementation(m));
        const char *label = sname;
        __block bool logged = false;
        method_setImplementation(m, imp_implementationWithBlock(^NSRect(id self) {
            NSRect r = orig(self, sel);
            if (r.size.width > 1.0 && r.size.height > 1.0) {
                NSRect shifted = r;
                if (r.origin.y > 100.0) shifted.origin.y -= dy;      // AppKit bottom-up
                else                    shifted.origin.y += dy;      // CG top-down
                if (!logged) {
                    logged = true;
                    maliKernelLog("[mali-island] %s orig=(%.1f,%.1f,%.1f,%.1f) -> y=%.1f",
                                  label, r.origin.x, r.origin.y, r.size.width, r.size.height, shifted.origin.y);
                }
                return shifted;
            }
            return r;
        }));
        ++hooked;
    }
    maliKernelLog("[mali-island] MenuBarAgent hook=%u dy=%.1fpt", hooked, dy);
    if (hooked > 0) {
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1500000000LL)), dispatch_get_main_queue(), ^{
            [[NSNotificationCenter defaultCenter] postNotificationName:@"_NSScreenDidChangeSafeApertureNotification" object:nil];
        });
    }
}
