// SPDX-License-Identifier: MIT
// TB375FC M7 out-of-process SwiftShader Metal GPU daemon (arm64, com.apple.security.cs.allow-jit).
// Connects to kernel IOAccelSwiftShaderUserClient with SWIFTSHADER_DAEMON_OPEN_TYPE,
// runs a single shared metal2vulkan + SwiftShader LLVM 16.0.0 JIT instance for all macOS clients,
// and dispatches zero-copy shared-memory + batched Metal RPCs.
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <libkern/OSCacheControl.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/ucontext.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/wait.h>
#include <dirent.h>
#include <libproc.h>
#include <poll.h>
#include <spawn.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SwiftShaderIpc.h"
#include "NativeMetalObjects.h"
#include "NmbWireCodec.h"

// m2v-v2 bridge compatibility helpers
static size_t nmb_command_buffer_debug_state(const NmbCommandBuffer *cb, char *out, size_t size) {
    if (!out || size == 0) return 0;
    if (!cb) { snprintf(out, size, "null"); return strlen(out); }
    uint32_t st = nmb_command_buffer_status(cb);
    char errbuf[256] = {};
    nmb_command_buffer_error(cb, errbuf, sizeof(errbuf));
    snprintf(out, size, "status=%u err=%s", st, errbuf[0] ? errbuf : "none");
    return strlen(out);
}

// Wire-compatible 14-field NmbTextureDescriptor (matches existing NativeAdrenoMetal.bundle RPC client)
struct NmbTextureDescriptorWire {
    uint64_t type, format, width, height, depth, levels, samples, array_length, usage, options;
    uint64_t rotation, framebuffer_only, is_drawable;
    uint64_t swizzle;
};
static_assert(sizeof(NmbTextureDescriptorWire) == 14 * sizeof(uint64_t), "wire tex desc size");

static inline bool read_tex_desc(m7wire::Reader &r, NmbTextureDescriptor &d) {
    NmbTextureDescriptorWire w{};
    if (!r.get_pod(w)) return false;
    d.type = w.type; d.format = w.format; d.width = w.width; d.height = w.height;
    d.depth = w.depth; d.levels = w.levels; d.samples = w.samples; d.array_length = w.array_length;
    d.usage = w.usage; d.options = w.options; d.rotation = w.rotation;
    d.framebuffer_only = w.framebuffer_only; d.is_drawable = w.is_drawable; d.swizzle = w.swizzle;
    d.allow_gpu_optimized_contents = 1;
    return true;
}

static inline void write_tex_desc(m7wire::Writer &out, const NmbTextureDescriptor &d) {
    NmbTextureDescriptorWire w{};
    w.type = d.type; w.format = d.format; w.width = d.width; w.height = d.height;
    w.depth = d.depth; w.levels = d.levels; w.samples = d.samples; w.array_length = d.array_length;
    w.usage = d.usage; w.options = d.options; w.rotation = d.rotation;
    w.framebuffer_only = d.framebuffer_only; w.is_drawable = d.is_drawable; w.swizzle = d.swizzle;
    out.put_pod(w);
}

using namespace m7wire;

namespace {

static io_connect_t gConn = MACH_PORT_NULL;
static NmbDevice *gDevice = nullptr;      // Dedicated SwiftShader VkDevice/VkQueue for WindowServer (client=1)
static NmbDevice *gBgDevice = nullptr;    // Separate SwiftShader VkDevice/VkQueue for background clients (client>1)
static bool gRealMali = false;            // True when running on real Mali-G615 hardware (panvk)
static std::mutex gVkMu;                  // Mutex for gDevice (WindowServer)
static std::mutex gBgVkMu;                // Separate mutex for gBgDevice (background clients)
static std::atomic<uint64_t> gNextHandle{1};
static std::atomic<uint64_t> gRpcCount{0};
static std::atomic<uint64_t> gCommitCount{0};
static std::atomic<uint64_t> gRenderPsoCount{0};
static std::atomic<uint64_t> gComputePsoCount{0};
static std::atomic<int32_t> gActiveWorkers{0};
static std::atomic<int32_t> gInPso{0};
static std::atomic<uint64_t> gC1Rpcs{0};
static std::atomic<uint64_t> gC1Commits{0};
static std::atomic<uint32_t> gC1LastOp{0};
static std::atomic<uint64_t> gC1LastOpMs{0};
static int gConsoleFd = -1;

static uint64_t monotonic_ms() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void gpud_log(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;
    fwrite(buf, 1, (size_t)n, stderr);
    fflush(stderr);
    if (gConsoleFd >= 0) {
        (void)write(gConsoleFd, buf, (size_t)n);
    }
}

static void wait_cb_completed_locked(NmbCommandBuffer *cb, uint32_t client_id, uint64_t cbid, uint64_t commit_no) {
    uint64_t t0 = monotonic_ms();
    for (;;) {
        int rc = nmb_command_buffer_wait(cb, 10000);
        if (rc != 1) break;
        char dbgbuf[512] = {};
        nmb_command_buffer_debug_state(cb, dbgbuf, sizeof(dbgbuf));
        gpud_log("[gpud] c%u commit #%llu (cbid=%llu) STILL RUNNING %llums [%s]\n",
                 client_id, (unsigned long long)commit_no, (unsigned long long)cbid,
                 (unsigned long long)(monotonic_ms() - t0), dbgbuf);
    }
}

static kern_return_t daemon_shmem_map(uint64_t shmem_id, uint64_t *va_out, uint64_t *len_out) {
    uint64_t in[1] = { shmem_id };
    uint64_t out[2] = {};
    uint32_t out_cnt = 2;
    kern_return_t kr = IOConnectCallScalarMethod(gConn, SEL_DAEMON_SHMEM_MAP, in, 1, out, &out_cnt);
    if (kr == KERN_SUCCESS) {
        *va_out = out[0];
        *len_out = out[1];
    }
    return kr;
}

static void daemon_shmem_unmap(uint64_t shmem_id) {
    uint64_t in[1] = { shmem_id };
    (void)IOConnectCallScalarMethod(gConn, SEL_DAEMON_SHMEM_UNMAP, in, 1, nullptr, nullptr);
}

static void daemon_post_event(uint32_t client_id, uint64_t cb_id, uint64_t val, uint64_t kind) {
    uint64_t in[4] = { (uint64_t)client_id, cb_id, val, kind };
    (void)IOConnectCallScalarMethod(gConn, SEL_DAEMON_POST_EVENT, in, 4, nullptr, nullptr);
}

// Callback contexts for async completions & shmem deallocators
struct AsyncCbCtx {
    uint32_t client_id;
    uint64_t cb_id;
    uint64_t kind;
};

static void cb_completed_trampoline(void *ctx_raw) {
    auto *ctx = static_cast<AsyncCbCtx *>(ctx_raw);
    daemon_post_event(ctx->client_id, ctx->cb_id, 0, ctx->kind);
    delete ctx;
}

static void cb_shared_event_trampoline(void *ctx_raw, uint64_t value) {
    auto *ctx = static_cast<AsyncCbCtx *>(ctx_raw);
    daemon_post_event(ctx->client_id, ctx->cb_id, value, EVT_SHARED_EVENT);
    delete ctx;
}

struct ShmemDeallocCtx {
    uint32_t client_id;
    uint64_t shmem_id;
    uint64_t cb_id;
};

static void shmem_dealloc_trampoline(void *ctx_raw, void *, uint64_t) {
    auto *ctx = static_cast<ShmemDeallocCtx *>(ctx_raw);
    daemon_shmem_unmap(ctx->shmem_id);
    if (ctx->cb_id != 0) {
        daemon_post_event(ctx->client_id, ctx->cb_id, 0, EVT_NOCOPY_RELEASE);
    }
    delete ctx;
}

// Per-client typed handle tables with shared_ptr custom deleters so concurrent
// worker threads never hit use-after-free even on client disconnect.
struct ClientHandles {
    std::mutex mu;
    std::unordered_map<uint64_t, std::shared_ptr<NmbBuffer>> buffers;
    std::unordered_map<uint64_t, std::shared_ptr<NmbHeap>> heaps;
    std::unordered_map<uint64_t, std::shared_ptr<NmbTexture>> textures;
    std::unordered_map<uint64_t, std::shared_ptr<NmbLibrary>> libraries;
    std::unordered_map<uint64_t, std::shared_ptr<NmbFunction>> functions;
    std::unordered_map<uint64_t, std::shared_ptr<NmbComputePipeline>> compute_psos;
    std::unordered_map<uint64_t, std::shared_ptr<NmbRenderPipeline>> render_psos;
    std::unordered_map<uint64_t, std::shared_ptr<NmbSampler>> samplers;
    std::unordered_map<uint64_t, std::shared_ptr<NmbDepthStencil>> depth_stencils;
    std::unordered_map<uint64_t, std::shared_ptr<NmbBinaryArchive>> archives;
    std::unordered_map<uint64_t, std::shared_ptr<NmbQueue>> queues;
    std::unordered_map<uint64_t, std::shared_ptr<NmbCommandBuffer>> cbs;
    std::unordered_map<uint64_t, std::shared_ptr<NmbEvent>> events;
    std::unordered_map<uint64_t, std::shared_ptr<NmbFence>> fences;
    // Encoders are owned until OP_ENC_END / OP_ENC_DISCARD / client close
    std::unordered_map<uint64_t, NmbEncoder *> encoders;
    std::unordered_map<NmbEncoder *, uint64_t> enc_cur_pid;
    std::unordered_map<uint64_t, std::string> fn_names;
    std::unordered_map<uint64_t, std::string> pso_names;
    std::unordered_map<std::string, std::shared_ptr<NmbRenderPipeline>> pso_alias_cache;
    std::unordered_set<uint64_t> tcim_fids;
    std::unordered_set<uint64_t> tcim_pids;

    void clear_all() {
        std::lock_guard<std::mutex> lk(mu);
        for (auto &kv : encoders) {
            if (kv.second) nmb_encoder_discard(kv.second);
        }
        encoders.clear();
        enc_cur_pid.clear();
        fn_names.clear();
        pso_names.clear();
        pso_alias_cache.clear();
        tcim_fids.clear();
        tcim_pids.clear();
        cbs.clear();
        queues.clear();
        render_psos.clear();
        compute_psos.clear();
        samplers.clear();
        depth_stencils.clear();
        archives.clear();
        textures.clear();
        buffers.clear();
        heaps.clear();
        functions.clear();
        libraries.clear();
        fences.clear();
        events.clear();
    }
};

static std::mutex gClientsMu;
static std::unordered_map<uint32_t, std::shared_ptr<ClientHandles>> gClients;

// Track latest WindowServer scanout IOSurface for CDC `fbshot` command
static std::mutex gScanoutMu;
static std::shared_ptr<NmbBuffer> gScanoutBuf;
static uint64_t gScanoutBid = 0, gScanoutW = 0, gScanoutH = 0, gScanoutBpr = 0, gScanoutFmt = 0, gScanoutOff = 0;
static std::atomic<bool> gFbshotPending{false};
static std::atomic<bool> gFbshotReady{false};

static bool dump_scanout_ppm_locked(const char *path, uint64_t *outW = nullptr, uint64_t *outH = nullptr, uint64_t *outFmt = nullptr);

#include "mali_vk_selftest.h"

static void record_ws_scanout(uint32_t client_id, uint64_t bid, const std::shared_ptr<NmbBuffer> &b,
                              uint64_t w, uint64_t h, uint64_t bpr, uint64_t fmt, uint64_t off) {
    (void)client_id;
    if (!b || w < 1000 || h < 600 || bpr < w * 4) return;
    if (fmt != 70 && fmt != 71 && fmt != 80 && fmt != 81 && fmt != 115) return;
    if (!nmb_buffer_contents(b.get())) return;
    std::lock_guard<std::mutex> lk(gScanoutMu);
    if (gScanoutBuf && gScanoutFmt == 115 && fmt != 115) return;
    gScanoutBuf = b;
    gScanoutBid = bid;
    gScanoutW = w;
    gScanoutH = h;
    gScanoutBpr = bpr;
    gScanoutFmt = fmt;
    gScanoutOff = off;
}

static uint8_t f16_to_srgb_u8(uint16_t h) {
    uint32_t sign = (h >> 15) & 1u;
    uint32_t exp  = (h >> 10) & 0x1Fu;
    uint32_t mant = h & 0x3FFu;
    if (sign || exp == 0) return 0;
    float f = 0.0f;
    if (exp == 31) f = 1.0f;
    else {
        uint32_t bits = ((exp + 112u) << 23) | (mant << 13);
        memcpy(&f, &bits, sizeof(f));
    }
    if (f <= 0.0f) return 0;
    if (f >= 1.0f) return 255;
    float s = powf(f, 1.0f / 2.2f);
    int v = (int)(s * 255.0f + 0.5f);
    return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
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

static bool dump_scanout_ppm_locked(const char *path, uint64_t *outW, uint64_t *outH, uint64_t *outFmt) {
    if (!gScanoutBuf || !gScanoutW || !gScanoutH) return false;
    uint8_t *raw_base = (uint8_t *)nmb_buffer_contents(gScanoutBuf.get());
    if (!raw_base) return false;
    arm64_dcache_clean_inval(raw_base + gScanoutOff, (size_t)(gScanoutH * gScanoutBpr));
    const uint8_t *base = raw_base + gScanoutOff;
    uint64_t dw = gScanoutW / 2, dh = gScanoutH / 2;
    FILE *fp = fopen(path, "wb");
    if (!fp) return false;
    fprintf(fp, "P6\n%llu %llu\n255\n", (unsigned long long)dw, (unsigned long long)dh);
    std::vector<uint8_t> row(dw * 3);
    for (uint64_t y = 0; y < dh; ++y) {
        const uint8_t *src = base + (y * 2) * gScanoutBpr;
        for (uint64_t x = 0; x < dw; ++x) {
            if (gScanoutFmt == 115) {
                const uint16_t *p = (const uint16_t *)(src + (x * 2) * 8);
                row[x * 3 + 0] = f16_to_srgb_u8(p[0]);
                row[x * 3 + 1] = f16_to_srgb_u8(p[1]);
                row[x * 3 + 2] = f16_to_srgb_u8(p[2]);
            } else if (gScanoutFmt == 80 || gScanoutFmt == 81) {
                const uint8_t *p = src + (x * 2) * 4;
                row[x * 3 + 0] = p[2];
                row[x * 3 + 1] = p[1];
                row[x * 3 + 2] = p[0];
            } else {
                const uint8_t *p = src + (x * 2) * 4;
                row[x * 3 + 0] = p[0];
                row[x * 3 + 1] = p[1];
                row[x * 3 + 2] = p[2];
            }
        }
        fwrite(row.data(), 1, row.size(), fp);
    }
    fclose(fp);
    if (outW) *outW = gScanoutW;
    if (outH) *outH = gScanoutH;
    if (outFmt) *outFmt = gScanoutFmt;
    return true;
}

// Shared file-backed library cache (saves RAM when WindowServer & clients open default.metallib)
struct CachedLibEntry {
    off_t size;
    time_t mtime;
    std::shared_ptr<NmbLibrary> lib;
};
static std::mutex gFileLibMu;
static std::unordered_map<std::string, CachedLibEntry> gFileLibCache;

static std::shared_ptr<ClientHandles> get_client(uint32_t client_id) {
    std::lock_guard<std::mutex> lk(gClientsMu);
    auto it = gClients.find(client_id);
    if (it != gClients.end()) return it->second;
    auto c = std::make_shared<ClientHandles>();
    gClients[client_id] = c;
    return c;
}

static void remove_client(uint32_t client_id) {
    std::shared_ptr<ClientHandles> c;
    {
        std::lock_guard<std::mutex> lk(gClientsMu);
        auto it = gClients.find(client_id);
        if (it != gClients.end()) {
            c = it->second;
            gClients.erase(it);
        }
    }
    if (c) {
        c->clear_all();
        gpud_log("[gpud] cleaned up handles for closed client_id=%u\n", client_id);
    }
}

template <typename T, void (*Rel)(T *)>
static uint64_t put_handle(std::mutex &mu, std::unordered_map<uint64_t, std::shared_ptr<T>> &m, T *raw) {
    if (!raw) return 0;
    uint64_t id = gNextHandle.fetch_add(1, std::memory_order_relaxed);
    std::shared_ptr<T> sp(raw, [](T *p) { if (p) Rel(p); });
    std::lock_guard<std::mutex> lk(mu);
    m.emplace(id, std::move(sp));
    return id;
}

template <typename T>
static std::shared_ptr<T> get_handle(std::mutex &mu, std::unordered_map<uint64_t, std::shared_ptr<T>> &m, uint64_t id) {
    if (!id) return nullptr;
    std::lock_guard<std::mutex> lk(mu);
    auto it = m.find(id);
    return (it == m.end()) ? nullptr : it->second;
}

template <typename T>
static void drop_handle(std::mutex &mu, std::unordered_map<uint64_t, std::shared_ptr<T>> &m, uint64_t id) {
    if (!id) return;
    std::shared_ptr<T> dropped;
    {
        std::lock_guard<std::mutex> lk(mu);
        auto it = m.find(id);
        if (it != m.end()) {
            dropped = std::move(it->second);
            m.erase(it);
        }
    }
    // dropped releases outside lock
}

static bool replay_encoder_batch(ClientHandles &ch, NmbEncoder *enc, Reader &r) {
    uint32_t count = r.get_u32();
    if (!r.ok) return false;
    bool skip_tcim = false;
    for (uint32_t i = 0; i < count; ++i) {
        uint16_t sub = r.get_u16();
        if (!r.ok) return false;
        bool ok = true;
        switch (sub) {
            case SUB_SET_BUFFER: {
                uint32_t stage = r.get_u32();
                uint64_t bid = r.get_u64();
                uint64_t off = r.get_u64();
                uint32_t idx = r.get_u32();
                if (!r.ok) return false;
                auto buf = get_handle(ch.mu, ch.buffers, bid);
                ok = nmb_encoder_set_buffer(enc, stage, buf.get(), off, idx);
                break;
            }
            case SUB_SET_BUFFER_OFFSET: {
                uint32_t stage = r.get_u32();
                uint64_t off = r.get_u64();
                uint32_t idx = r.get_u32();
                if (!r.ok) return false;
                ok = nmb_encoder_set_buffer_offset(enc, stage, off, idx);
                break;
            }
            case SUB_SET_BYTES: {
                uint32_t stage = r.get_u32();
                uint32_t idx = r.get_u32();
                const uint8_t *ptr = nullptr; size_t len = 0;
                if (!r.get_bytes_view(&ptr, &len)) return false;
                ok = nmb_encoder_set_bytes(enc, stage, ptr, len, idx);
                break;
            }
            case SUB_SET_TEXTURE: {
                uint32_t stage = r.get_u32();
                uint64_t tid = r.get_u64();
                uint32_t idx = r.get_u32();
                if (!r.ok) return false;
                auto tex = get_handle(ch.mu, ch.textures, tid);
                ok = nmb_encoder_set_texture(enc, stage, tex.get(), idx);
                break;
            }
            case SUB_SET_SAMPLER: {
                uint32_t stage = r.get_u32();
                uint64_t sid = r.get_u64();
                uint32_t idx = r.get_u32();
                if (!r.ok) return false;
                auto samp = get_handle(ch.mu, ch.samplers, sid);
                ok = nmb_encoder_set_sampler(enc, stage, samp.get(), idx);
                break;
            }
            case SUB_UPDATE_FENCE: {
                uint64_t fid = r.get_u64();
                uint64_t stages = r.get_u64();
                if (!r.ok) return false;
                auto f = get_handle(ch.mu, ch.fences, fid);
                ok = nmb_encoder_update_fence(enc, f.get(), stages);
                break;
            }
            case SUB_WAIT_FENCE: {
                uint64_t fid = r.get_u64();
                uint64_t stages = r.get_u64();
                if (!r.ok) return false;
                auto f = get_handle(ch.mu, ch.fences, fid);
                ok = nmb_encoder_wait_fence(enc, f.get(), stages);
                break;
            }
            case SUB_COMPUTE_PIPELINE: {
                uint64_t pid = r.get_u64();
                if (!r.ok) return false;
                auto pso = get_handle(ch.mu, ch.compute_psos, pid);
                ok = nmb_compute_set_pipeline(enc, pso.get());
                break;
            }
            case SUB_COMPUTE_TG_MEM: {
                uint32_t len = r.get_u32();
                uint32_t idx = r.get_u32();
                if (!r.ok) return false;
                ok = nmb_compute_set_threadgroup_memory(enc, len, idx);
                break;
            }
            case SUB_COMPUTE_DISPATCH: {
                uint64_t sz[3], pg[3];
                for (int k = 0; k < 3; ++k) sz[k] = r.get_u64();
                for (int k = 0; k < 3; ++k) pg[k] = r.get_u64();
                bool threads = (r.get_u8() != 0);
                if (!r.ok) return false;
                ok = nmb_compute_dispatch(enc, sz, pg, threads);
                break;
            }
            case SUB_COMPUTE_DISPATCH_IND: {
                uint64_t bid = r.get_u64();
                uint64_t off = r.get_u64();
                uint64_t pg[3];
                for (int k = 0; k < 3; ++k) pg[k] = r.get_u64();
                if (!r.ok) return false;
                auto buf = get_handle(ch.mu, ch.buffers, bid);
                ok = nmb_compute_dispatch_indirect(enc, buf.get(), off, pg);
                break;
            }
            case SUB_RENDER_PIPELINE: {
                uint64_t pid = r.get_u64();
                if (!r.ok) return false;
                bool is_heavy = false;
                {
                    std::lock_guard<std::mutex> lk(ch.mu);
                    ch.enc_cur_pid[enc] = pid;
                    is_heavy = (ch.tcim_pids.count(pid) != 0);
                }
                if (is_heavy) {
                    ok = true;
                } else {
                    auto pso = get_handle(ch.mu, ch.render_psos, pid);
                    ok = nmb_render_set_pipeline(enc, pso.get());
                }
                break;
            }
            case SUB_RENDER_DEPTH_STENCIL: {
                uint64_t dsid = r.get_u64();
                if (!r.ok) return false;
                auto ds = get_handle(ch.mu, ch.depth_stencils, dsid);
                ok = nmb_render_set_depth_stencil(enc, ds.get());
                break;
            }
            case SUB_RENDER_VIEWPORT: {
                double vp[6];
                for (int k = 0; k < 6; ++k) vp[k] = r.get_f64();
                if (!r.ok) return false;
                ok = nmb_render_set_viewport(enc, vp);
                break;
            }
            case SUB_RENDER_VIEWPORTS: {
                uint32_t cnt = r.get_u32();
                if (!r.ok || cnt > 64) return false;
                std::vector<double> vps(cnt * 6);
                for (size_t k = 0; k < vps.size(); ++k) vps[k] = r.get_f64();
                if (!r.ok) return false;
                ok = nmb_render_set_viewports(enc, vps.data(), cnt);
                break;
            }
            case SUB_RENDER_SCISSOR: {
                uint64_t sc[4];
                for (int k = 0; k < 4; ++k) sc[k] = r.get_u64();
                if (!r.ok) return false;
                ok = nmb_render_set_scissor(enc, sc);
                break;
            }
            case SUB_RENDER_SCISSORS: {
                uint32_t cnt = r.get_u32();
                if (!r.ok || cnt > 64) return false;
                std::vector<uint64_t> scs(cnt * 4);
                for (size_t k = 0; k < scs.size(); ++k) scs[k] = r.get_u64();
                if (!r.ok) return false;
                ok = nmb_render_set_scissors(enc, scs.data(), cnt);
                break;
            }
            case SUB_RENDER_RASTER: {
                uint64_t cull = r.get_u64(), wind = r.get_u64(), fill = r.get_u64();
                if (!r.ok) return false;
                ok = nmb_render_set_raster(enc, cull, wind, fill);
                break;
            }
            case SUB_RENDER_BLEND_COLOR: {
                float c[4];
                for (int k = 0; k < 4; ++k) c[k] = r.get_f32();
                if (!r.ok) return false;
                ok = nmb_render_set_blend_color(enc, c);
                break;
            }
            case SUB_RENDER_STENCIL_REF: {
                uint32_t f = r.get_u32(), b = r.get_u32();
                if (!r.ok) return false;
                ok = nmb_render_set_stencil_reference(enc, f, b);
                break;
            }
            case SUB_RENDER_DEPTH_BIAS: {
                float b = r.get_f32(), s = r.get_f32(), cl = r.get_f32();
                if (!r.ok) return false;
                ok = nmb_render_set_depth_bias(enc, b, s, cl);
                break;
            }
            case SUB_RENDER_VISIBILITY: {
                uint64_t mode = r.get_u64(), off = r.get_u64();
                if (!r.ok) return false;
                ok = nmb_render_set_visibility_mode(enc, mode, off);
                break;
            }
            case SUB_RENDER_LINE_WIDTH: {
                float w = r.get_f32();
                if (!r.ok) return false;
                ok = nmb_render_set_line_width(enc, w);
                break;
            }
            case SUB_RENDER_TESS_BUF: {
                uint64_t bid = r.get_u64(), off = r.get_u64(), stride = r.get_u64();
                if (!r.ok) return false;
                auto buf = get_handle(ch.mu, ch.buffers, bid);
                ok = nmb_render_set_tessellation_factor_buffer(enc, buf.get(), off, stride);
                break;
            }
            case SUB_RENDER_TESS_SCALE: {
                float s = r.get_f32();
                if (!r.ok) return false;
                ok = nmb_render_set_tessellation_factor_scale(enc, s);
                break;
            }
            case SUB_RENDER_TG_MEM: {
                uint64_t len = r.get_u64(), off = r.get_u64();
                uint32_t idx = r.get_u32();
                if (!r.ok) return false;
                ok = nmb_render_set_threadgroup_memory_length(enc, len, off, idx);
                break;
            }
            case SUB_RENDER_DISPATCH_TILE: {
                uint64_t th[3];
                for (int k = 0; k < 3; ++k) th[k] = r.get_u64();
                if (!r.ok) return false;
                bool is_heavy = false;
                {
                    std::lock_guard<std::mutex> lk(ch.mu);
                    is_heavy = (ch.tcim_pids.count(ch.enc_cur_pid[enc]) != 0);
                }
                ok = is_heavy ? true : nmb_render_dispatch_threads_per_tile(enc, th);
                break;
            }
            case SUB_RENDER_DISPATCH_TILE_REG: {
                uint64_t th[3], reg[6];
                for (int k = 0; k < 3; ++k) th[k] = r.get_u64();
                for (int k = 0; k < 6; ++k) reg[k] = r.get_u64();
                if (!r.ok) return false;
                bool is_heavy = false;
                {
                    std::lock_guard<std::mutex> lk(ch.mu);
                    is_heavy = (ch.tcim_pids.count(ch.enc_cur_pid[enc]) != 0);
                }
                ok = is_heavy ? true : nmb_render_dispatch_threads_per_tile_in_region(enc, th, reg);
                break;
            }
            case SUB_RENDER_DRAW: {
                uint64_t prim = r.get_u64();
                uint32_t start = r.get_u32(), cnt = r.get_u32(), inst = r.get_u32(), bi = r.get_u32();
                if (!r.ok) return false;
                uint64_t cur_pid = 0;
                bool is_heavy = false;
                std::string pname;
                {
                    std::lock_guard<std::mutex> lk(ch.mu);
                    cur_pid = ch.enc_cur_pid[enc];
                    is_heavy = (ch.tcim_pids.count(cur_pid) != 0);
                    auto it = ch.pso_names.find(cur_pid);
                    if (it != ch.pso_names.end()) pname = it->second;
                }
                if (is_heavy) {
                    static std::atomic<uint32_t> skipLog{0};
                    if (skipLog.fetch_add(1, std::memory_order_relaxed) < 40)
                        gpud_log("[gpud] SKIPPED heavy draw pid=%llu (%s) prim=%llu cnt=%u\n",
                                 (unsigned long long)cur_pid, pname.c_str(), (unsigned long long)prim, cnt);
                    ok = true;
                } else {
                    static std::atomic<uint32_t> drawLog{0};
                    if (drawLog.fetch_add(1, std::memory_order_relaxed) < 64)
                        gpud_log("[gpud] draw pid=%llu (%s) prim=%llu cnt=%u\n",
                                 (unsigned long long)cur_pid, pname.c_str(), (unsigned long long)prim, cnt);
                    ok = nmb_render_draw(enc, prim, start, cnt, inst, bi);
                }
                break;
            }
            case SUB_RENDER_DRAW_INDEXED: {
                uint64_t prim = r.get_u64();
                uint32_t cnt = r.get_u32();
                uint64_t itype = r.get_u64(), bid = r.get_u64(), off = r.get_u64();
                uint32_t inst = r.get_u32();
                int32_t bv = r.get_i32();
                uint32_t bi = r.get_u32();
                if (!r.ok) return false;
                uint64_t cur_pid = 0;
                bool is_heavy = false;
                std::string pname;
                {
                    std::lock_guard<std::mutex> lk(ch.mu);
                    cur_pid = ch.enc_cur_pid[enc];
                    is_heavy = (ch.tcim_pids.count(cur_pid) != 0);
                    auto it = ch.pso_names.find(cur_pid);
                    if (it != ch.pso_names.end()) pname = it->second;
                }
                if (is_heavy) {
                    static std::atomic<uint32_t> skipIdxLog{0};
                    if (skipIdxLog.fetch_add(1, std::memory_order_relaxed) < 40)
                        gpud_log("[gpud] SKIPPED heavy idx-draw pid=%llu (%s) prim=%llu cnt=%u\n",
                                 (unsigned long long)cur_pid, pname.c_str(), (unsigned long long)prim, cnt);
                    ok = true;
                } else {
                    static std::atomic<uint32_t> drawIdxLog{0};
                    if (drawIdxLog.fetch_add(1, std::memory_order_relaxed) < 64)
                        gpud_log("[gpud] idx-draw pid=%llu (%s) prim=%llu cnt=%u\n",
                                 (unsigned long long)cur_pid, pname.c_str(), (unsigned long long)prim, cnt);
                    auto buf = get_handle(ch.mu, ch.buffers, bid);
                    ok = nmb_render_draw_indexed(enc, prim, cnt, itype, buf.get(), off, inst, bv, bi);
                }
                break;
            }
            case SUB_RENDER_DRAW_PATCHES: {
                uint32_t cp = r.get_u32(), ps = r.get_u32(), pc = r.get_u32();
                uint64_t pib = r.get_u64(), pio = r.get_u64();
                uint64_t cib = r.get_u64(), cio = r.get_u64();
                uint32_t inst = r.get_u32(), bi = r.get_u32();
                uint64_t indb = r.get_u64(), indo = r.get_u64();
                if (!r.ok) return false;
                bool is_heavy = false;
                {
                    std::lock_guard<std::mutex> lk(ch.mu);
                    is_heavy = (ch.tcim_pids.count(ch.enc_cur_pid[enc]) != 0);
                }
                if (is_heavy) {
                    ok = true;
                } else {
                    auto bpi = get_handle(ch.mu, ch.buffers, pib);
                    auto bci = get_handle(ch.mu, ch.buffers, cib);
                    auto bind = get_handle(ch.mu, ch.buffers, indb);
                    ok = nmb_render_draw_patches(enc, cp, ps, pc, bpi.get(), pio, bci.get(), cio, inst, bi, bind.get(), indo);
                }
                break;
            }
            case SUB_BLIT_COPY_BUF: {
                uint64_t sbid = r.get_u64(), soff = r.get_u64(), dbid = r.get_u64(), doff = r.get_u64(), sz = r.get_u64();
                if (!r.ok) return false;
                auto sb = get_handle(ch.mu, ch.buffers, sbid);
                auto db = get_handle(ch.mu, ch.buffers, dbid);
                ok = nmb_blit_copy_buffer(enc, sb.get(), soff, db.get(), doff, sz);
                break;
            }
            case SUB_BLIT_FILL_BUF: {
                uint64_t bid = r.get_u64(), st = r.get_u64(), en = r.get_u64();
                uint8_t val = r.get_u8();
                if (!r.ok) return false;
                auto b = get_handle(ch.mu, ch.buffers, bid);
                ok = nmb_blit_fill_buffer(enc, b.get(), st, en, val);
                break;
            }
            case SUB_BLIT_COPY_TEX: {
                uint64_t stid = r.get_u64(), ssl = r.get_u64(), slv = r.get_u64();
                uint64_t so[3], sz[3], do_[3];
                for (int k = 0; k < 3; ++k) so[k] = r.get_u64();
                for (int k = 0; k < 3; ++k) sz[k] = r.get_u64();
                uint64_t dtid = r.get_u64(), dsl = r.get_u64(), dlv = r.get_u64();
                for (int k = 0; k < 3; ++k) do_[k] = r.get_u64();
                if (!r.ok) return false;
                auto st = get_handle(ch.mu, ch.textures, stid);
                auto dt = get_handle(ch.mu, ch.textures, dtid);
                ok = nmb_blit_copy_texture(enc, st.get(), ssl, slv, so, sz, dt.get(), dsl, dlv, do_);
                break;
            }
            case SUB_BLIT_BUF_TO_TEX: {
                uint64_t bid = r.get_u64(), off = r.get_u64(), bpr = r.get_u64(), bpi = r.get_u64();
                uint64_t sz[3], org[3];
                for (int k = 0; k < 3; ++k) sz[k] = r.get_u64();
                uint64_t tid = r.get_u64(), sl = r.get_u64(), lv = r.get_u64();
                for (int k = 0; k < 3; ++k) org[k] = r.get_u64();
                if (!r.ok) return false;
                auto b = get_handle(ch.mu, ch.buffers, bid);
                auto t = get_handle(ch.mu, ch.textures, tid);
                ok = nmb_blit_buffer_to_texture(enc, b.get(), off, bpr, bpi, sz, t.get(), sl, lv, org);
                break;
            }
            case SUB_BLIT_TEX_TO_BUF: {
                uint64_t tid = r.get_u64(), sl = r.get_u64(), lv = r.get_u64();
                uint64_t org[3], sz[3];
                for (int k = 0; k < 3; ++k) org[k] = r.get_u64();
                for (int k = 0; k < 3; ++k) sz[k] = r.get_u64();
                uint64_t bid = r.get_u64(), off = r.get_u64(), bpr = r.get_u64(), bpi = r.get_u64();
                if (!r.ok) return false;
                auto t = get_handle(ch.mu, ch.textures, tid);
                auto b = get_handle(ch.mu, ch.buffers, bid);
                ok = nmb_blit_texture_to_buffer(enc, t.get(), sl, lv, org, sz, b.get(), off, bpr, bpi);
                break;
            }
            case SUB_BLIT_GEN_MIPS: {
                uint64_t tid = r.get_u64();
                if (!r.ok) return false;
                auto t = get_handle(ch.mu, ch.textures, tid);
                ok = nmb_blit_generate_mipmaps(enc, t.get());
                break;
            }
            default:
                return false;
        }
        if (!ok) return false;
    }
    return true;
}

static void handle_rpc(uint32_t client_id, const uint8_t *req_buf, size_t req_len, Writer &out) {
    Reader r(req_buf, req_len);
    uint32_t op = r.get_u32();
    if (!r.ok) { reply_err(out, "truncated request"); return; }

    if (op == OP_CLIENT_CLOSED) {
        remove_client(client_id);
        reply_ok(out);
        return;
    }

    auto ch = get_client(client_id);
    NmbDevice *dev = (client_id == 1 || !gBgDevice) ? gDevice : gBgDevice;
    std::mutex &dev_mu = (dev == gDevice) ? gVkMu : gBgVkMu;
    gRpcCount.fetch_add(1, std::memory_order_relaxed);
    if (client_id == 1) {
        gC1Rpcs.fetch_add(1, std::memory_order_relaxed);
        gC1LastOp.store(op, std::memory_order_relaxed);
        gC1LastOpMs.store(monotonic_ms(), std::memory_order_relaxed);
    }
    struct WorkerGuard {
        WorkerGuard() { gActiveWorkers.fetch_add(1, std::memory_order_relaxed); }
        ~WorkerGuard() { gActiveWorkers.fetch_sub(1, std::memory_order_relaxed); }
    } _wg;

    auto hold_fn = [&](uint64_t id, std::vector<std::shared_ptr<NmbFunction>> &keep) -> NmbFunction * {
        auto sp = get_handle(ch->mu, ch->functions, id);
        if (!sp) return nullptr;
        keep.push_back(sp);
        return sp.get();
    };
    auto hold_arch = [&](uint64_t id, std::vector<std::shared_ptr<NmbBinaryArchive>> &keep) -> NmbBinaryArchive * {
        auto sp = get_handle(ch->mu, ch->archives, id);
        if (!sp) return nullptr;
        keep.push_back(sp);
        return sp.get();
    };

    switch (op) {
        // ---- Device & queries ----
        case OP_DEVICE_OPEN: {
            gpud_log("[gpud] OP_DEVICE_OPEN client_id=%u\n", client_id);
            reply_ok(out);
            out.put_u64(1); // single shared device handle
            break;
        }
        case OP_DEVICE_RELEASE: {
            reply_ok(out);
            break;
        }
        case OP_DEVICE_NAME: {
            char buf[256] = {};
            nmb_device_name(dev, buf, sizeof(buf));
            reply_ok(out);
            out.put_str_nonnull(buf);
            break;
        }
        case OP_DEVICE_VENDOR_NAME: {
            char buf[256] = {};
            nmb_device_vendor_name(dev, buf, sizeof(buf));
            reply_ok(out);
            out.put_str_nonnull(buf);
            break;
        }
        case OP_DEVICE_MAX_BUFFER_LENGTH: {
            reply_ok(out);
            out.put_u64(nmb_device_max_buffer_length(dev));
            break;
        }
        case OP_DEVICE_MAX_THREADS_PER_TG: {
            uint64_t tg[3] = {0, 0, 0};
            if (!nmb_device_max_threads_per_threadgroup(dev, tg)) {
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
                for (int i = 0; i < 3; ++i) out.put_u64(tg[i]);
            }
            break;
        }
        case OP_DEVICE_ANSWER_LIST: {
            std::vector<std::pair<std::string, uint32_t>> list;
            for (size_t i = 0; ; ++i) {
                uint32_t kind = 0;
                const char *name = nmb_device_answer_selector(i, &kind);
                if (!name) break;
                list.emplace_back(name, kind);
            }
            reply_ok(out);
            out.put_u32((uint32_t)list.size());
            for (const auto &it : list) {
                out.put_str_nonnull(it.first.c_str());
                out.put_u32(it.second);
            }
            break;
        }
        case OP_DEVICE_ANSWER: {
            std::string sel = r.get_str();
            uint64_t arg = r.get_u64();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            uint64_t val = 0;
            if (!nmb_device_answer(dev, sel.c_str(), arg, &val)) {
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
                out.put_u64(val);
            }
            break;
        }
        case OP_DEVICE_LINEAR_BPR: {
            uint64_t fmt = r.get_u64(), w = r.get_u64();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            uint64_t bpr = nmb_device_linear_bytes_per_row(dev, fmt, w);
            if (!bpr) reply_err(out, nmb_last_error());
            else { reply_ok(out); out.put_u64(bpr); }
            break;
        }
        case OP_DEVICE_HEAP_TEX_SIZE_ALIGN: {
            NmbTextureDescriptor d{};
            if (!read_tex_desc(r, d)) { reply_err(out, "bad args"); break; }
            uint64_t sa[2] = {0, 0};
            if (!nmb_device_heap_texture_size_and_align(dev, &d, sa)) {
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
                out.put_u64(sa[0]); out.put_u64(sa[1]);
            }
            break;
        }
        case OP_DEVICE_HEAP_BUF_SIZE_ALIGN: {
            uint64_t len = r.get_u64(), opts = r.get_u64();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            uint64_t sa[2] = {0, 0};
            if (!nmb_device_heap_buffer_size_and_align(dev, len, opts, sa)) {
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
                out.put_u64(sa[0]); out.put_u64(sa[1]);
            }
            break;
        }
        case OP_DEVICE_CURRENT_ALLOC_SIZE: {
            reply_ok(out);
            out.put_u64(nmb_device_current_allocated_size(dev));
            break;
        }
        case OP_DEVICE_FAMILY_WAIVERS: {
            char buf[2048] = {};
            nmb_device_family_waivers(dev, buf, sizeof(buf));
            reply_ok(out);
            out.put_str_nonnull(buf);
            break;
        }

        // ---- Buffers ----
        case OP_BUFFER_NEW_SHMEM: {
            uint64_t sid = r.get_u64(), aligned = r.get_u64(), req_len = r.get_u64(), opts = r.get_u64();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            uint64_t dva = 0, mapped = 0;
            kern_return_t kr = daemon_shmem_map(sid, &dva, &mapped);
            if (kr != KERN_SUCCESS || !dva || mapped < aligned) {
                reply_err(out, "daemon shmem map failed");
                break;
            }
            auto *ctx = new ShmemDeallocCtx{ client_id, sid, 0 };
            NmbBuffer *b = nullptr;
            {
                std::lock_guard<std::mutex> dlk(dev_mu);
                b = nmb_buffer_new_no_copy_releasing(dev, (void *)dva, aligned, opts, shmem_dealloc_trampoline, ctx);
                if (!b) {
                    // Retry with StorageModeShared if Managed/other shared flag was refused
                    uint64_t shared_opts = (opts & ~0xF0ULL);
                    if (shared_opts != opts) {
                        b = nmb_buffer_new_no_copy_releasing(dev, (void *)dva, aligned, shared_opts, shmem_dealloc_trampoline, ctx);
                    }
                }
            }
            if (!b) {
                daemon_shmem_unmap(sid);
                delete ctx;
                gpud_log("[gpud] OP_BUFFER_NEW_SHMEM failed len=%llu aligned=%llu opts=0x%llx err=%s\n",
                         (unsigned long long)req_len, (unsigned long long)aligned, (unsigned long long)opts, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t hid = put_handle<NmbBuffer, nmb_buffer_release>(ch->mu, ch->buffers, b);
                reply_ok(out);
                out.put_u64(hid);
                out.put_u64(nmb_buffer_gpu_address(b));
            }
            break;
        }
        case OP_BUFFER_NEW_PRIVATE: {
            uint64_t len = r.get_u64(), opts = r.get_u64();
            uint8_t has_init = r.get_u8();
            const uint8_t *ptr = nullptr; size_t blen = 0;
            if (has_init) {
                if (!r.get_bytes_view(&ptr, &blen)) { reply_err(out, "bad bytes"); break; }
            }
            if (!r.ok) { reply_err(out, "bad args"); break; }
            NmbBuffer *b = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); b = nmb_buffer_new(dev, len, opts, has_init ? ptr : nullptr); }
            if (!b) {
                reply_err(out, nmb_last_error());
            } else {
                uint64_t hid = put_handle<NmbBuffer, nmb_buffer_release>(ch->mu, ch->buffers, b);
                reply_ok(out);
                out.put_u64(hid);
                out.put_u64(nmb_buffer_gpu_address(b));
            }
            break;
        }
        case OP_BUFFER_NEW_NOCOPY: {
            uint64_t sid = r.get_u64(), page_off = r.get_u64(), aligned = r.get_u64();
            uint64_t req_len = r.get_u64(), opts = r.get_u64(), cb_id = r.get_u64();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            uint64_t dva = 0, mapped = 0;
            kern_return_t kr = daemon_shmem_map(sid, &dva, &mapped);
            if (kr != KERN_SUCCESS || !dva || mapped < aligned) {
                reply_err(out, "daemon nocopy shmem map failed");
                break;
            }
            uint64_t import_len = (req_len & 4095ULL) == 0 ? req_len : aligned;
            auto *ctx = new ShmemDeallocCtx{ client_id, sid, cb_id };
            NmbBuffer *b = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); b = nmb_buffer_new_no_copy_releasing(dev, (void *)(dva + page_off), import_len, opts, shmem_dealloc_trampoline, ctx); }
            if (!b) {
                daemon_shmem_unmap(sid);
                delete ctx;
                gpud_log("[gpud] OP_BUFFER_NEW_NOCOPY failed off=%llu len=%llu aligned=%llu opts=0x%llx err=%s\n",
                         (unsigned long long)page_off, (unsigned long long)req_len,
                         (unsigned long long)aligned, (unsigned long long)opts, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t hid = put_handle<NmbBuffer, nmb_buffer_release>(ch->mu, ch->buffers, b);
                reply_ok(out);
                out.put_u64(hid);
                out.put_u64(nmb_buffer_gpu_address(b));
            }
            break;
        }
        case OP_BUFFER_RELEASE: {
            uint64_t bid = r.get_u64();
            {
                std::lock_guard<std::mutex> lk(gScanoutMu);
                if (bid == gScanoutBid) {
                    gScanoutBuf.reset();
                    gScanoutBid = 0;
                }
            }
            drop_handle(ch->mu, ch->buffers, bid);
            reply_ok(out);
            break;
        }
        case OP_BUFFER_GPU_ADDRESS: {
            auto b = get_handle(ch->mu, ch->buffers, r.get_u64());
            reply_ok(out);
            out.put_u64(b ? nmb_buffer_gpu_address(b.get()) : 0);
            break;
        }
        case OP_BUFFER_SET_PURGEABLE: {
            uint64_t id = r.get_u64(), st = r.get_u64();
            auto b = get_handle(ch->mu, ch->buffers, id);
            if (!b) { reply_err(out, "invalid buffer"); break; }
            uint64_t prev = nmb_buffer_set_purgeable_state(b.get(), st);
            reply_ok(out); out.put_u64(prev);
            break;
        }
        case OP_BUFFER_HOLD: {
            auto b = get_handle(ch->mu, ch->buffers, r.get_u64());
            reply_ok(out);
            out.put_u8(b && nmb_buffer_hold(b.get()) ? 1 : 0);
            break;
        }
        case OP_BUFFER_UNHOLD: {
            auto b = get_handle(ch->mu, ch->buffers, r.get_u64());
            if (b) nmb_buffer_unhold(b.get());
            reply_ok(out);
            break;
        }
        case OP_BUFFER_MAKE_ALIASABLE: {
            auto b = get_handle(ch->mu, ch->buffers, r.get_u64());
            reply_ok(out);
            out.put_u8(b && nmb_buffer_make_aliasable(b.get()) ? 1 : 0);
            break;
        }
        case OP_BUFFER_IS_ALIASABLE: {
            auto b = get_handle(ch->mu, ch->buffers, r.get_u64());
            reply_ok(out);
            out.put_u8(b && nmb_buffer_is_aliasable(b.get()) ? 1 : 0);
            break;
        }

        // ---- Heaps ----
        case OP_HEAP_NEW: {
            uint64_t type = r.get_u64(), sz = r.get_u64(), opts = r.get_u64();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            NmbHeap *h = nmb_heap_new(dev, type, sz, opts);
            if (!h) reply_err(out, nmb_last_error());
            else {
                uint64_t hid = put_handle<NmbHeap, nmb_heap_release>(ch->mu, ch->heaps, h);
                reply_ok(out); out.put_u64(hid);
            }
            break;
        }
        case OP_HEAP_RELEASE: {
            drop_handle(ch->mu, ch->heaps, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_HEAP_STATS: {
            auto h = get_handle(ch->mu, ch->heaps, r.get_u64());
            reply_ok(out);
            out.put_u64(h ? nmb_heap_size(h.get()) : 0);
            out.put_u64(h ? nmb_heap_used_size(h.get()) : 0);
            out.put_u64(h ? nmb_heap_allocated_size(h.get()) : 0);
            break;
        }
        case OP_HEAP_MAX_AVAIL: {
            uint64_t id = r.get_u64(), align = r.get_u64();
            auto h = get_handle(ch->mu, ch->heaps, id);
            reply_ok(out);
            out.put_u64(h ? nmb_heap_max_available_size(h.get(), align) : 0);
            break;
        }
        case OP_HEAP_SET_PURGEABLE: {
            uint64_t id = r.get_u64(), st = r.get_u64();
            auto h = get_handle(ch->mu, ch->heaps, id);
            if (!h) { reply_err(out, "invalid heap"); break; }
            reply_ok(out); out.put_u64(nmb_heap_set_purgeable_state(h.get(), st));
            break;
        }
        case OP_HEAP_NEW_BUFFER: {
            uint64_t id = r.get_u64(), len = r.get_u64(), opts = r.get_u64(), off = r.get_u64();
            auto h = get_handle(ch->mu, ch->heaps, id);
            if (!h) { reply_err(out, "invalid heap"); break; }
            NmbBuffer *b = nmb_heap_new_buffer(h.get(), len, opts, off);
            if (!b) reply_err(out, nmb_last_error());
            else {
                uint64_t hid = put_handle<NmbBuffer, nmb_buffer_release>(ch->mu, ch->buffers, b);
                reply_ok(out);
                out.put_u64(hid);
                out.put_u64(nmb_buffer_gpu_address(b));
            }
            break;
        }
        case OP_HEAP_NEW_TEXTURE: {
            uint64_t id = r.get_u64();
            NmbTextureDescriptor d{};
            if (!read_tex_desc(r, d)) { reply_err(out, "bad args"); break; }
            uint64_t off = r.get_u64();
            auto h = get_handle(ch->mu, ch->heaps, id);
            if (!h) { reply_err(out, "invalid heap"); break; }
            NmbTexture *t = nmb_heap_new_texture(h.get(), &d, off);
            if (!t) reply_err(out, nmb_last_error());
            else {
                uint64_t tid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, t);
                reply_ok(out); out.put_u64(tid);
            }
            break;
        }

        // ---- Textures ----
        case OP_TEXTURE_NEW: {
            NmbTextureDescriptor d{};
            if (!read_tex_desc(r, d)) { reply_err(out, "bad args"); break; }
            NmbTexture *t = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); t = nmb_texture_new(dev, &d); }
            if (!t) reply_err(out, nmb_last_error());
            else {
                uint64_t tid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, t);
                reply_ok(out); out.put_u64(tid);
            }
            break;
        }
        case OP_TEXTURE_NEW_ON_BUFFER: {
            uint64_t bid = r.get_u64();
            NmbTextureDescriptor d{};
            if (!read_tex_desc(r, d)) { reply_err(out, "bad args"); break; }
            uint64_t off = r.get_u64(), bpr = r.get_u64();
            bool iosurf = (r.get_u8() != 0);
            auto b = get_handle(ch->mu, ch->buffers, bid);
            if (!b) { reply_err(out, "invalid buffer"); break; }
            if (iosurf) {
                uint64_t bpp = (d.format == 70 || d.format == 71 || d.format == 80 || d.format == 81) ? 4 :
                               (d.format == 115) ? 8 : 0;
                if (bpp && d.width > 0 && bpr < d.width * bpp) {
                    static uint32_t fixedBprCount = 0;
                    if (++fixedBprCount <= 16)
                        gpud_log("[gpud] iosurf tex %llux%llu fmt=%llu: fixed bpr %llu -> %llu (off=0x%llx->0)\n",
                                 (unsigned long long)d.width, (unsigned long long)d.height,
                                 (unsigned long long)d.format, (unsigned long long)bpr,
                                 (unsigned long long)(d.width * bpp), (unsigned long long)off);
                    bpr = d.width * bpp;
                    off = 0;
                } else {
                    static uint32_t dispTexCount = 0;
                    if (++dispTexCount <= 16)
                        gpud_log("[gpud] iosurf tex %llux%llu fmt=%llu off=0x%llx->0 bpr=%llu\n",
                                 (unsigned long long)d.width, (unsigned long long)d.height,
                                 (unsigned long long)d.format, (unsigned long long)off, (unsigned long long)bpr);
                    if (off != 0) off = 0;
                }
            }
            NmbTexture *t = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); t = nmb_texture_new_on_buffer(b.get(), &d, off, bpr, iosurf); }
            if (!t) {
                gpud_log("[gpud] tex_on_buf FAILED client=%u (%llux%llu fmt=%llu off=0x%llx bpr=%llu iosurf=%d): %s\n",
                         client_id, (unsigned long long)d.width, (unsigned long long)d.height,
                         (unsigned long long)d.format, (unsigned long long)off, (unsigned long long)bpr,
                         (int)iosurf, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                if (iosurf) record_ws_scanout(client_id, bid, b, d.width, d.height, bpr, d.format, off);
                uint64_t tid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, t);
                reply_ok(out); out.put_u64(tid);
            }
            break;
        }
        case OP_TEXTURE_NEW_ON_SURFACE: {
            uint64_t bid = r.get_u64();
            NmbTextureDescriptor d{};
            if (!read_tex_desc(r, d)) { reply_err(out, "bad args"); break; }
            uint64_t off = r.get_u64(), bpr = r.get_u64();
            uint32_t pfmt = r.get_u32();
            uint64_t plane = r.get_u64(), w = r.get_u64(), h = r.get_u64();
            uint64_t coff = r.get_u64(), cbpr = r.get_u64();
            auto b = get_handle(ch->mu, ch->buffers, bid);
            if (!b) { reply_err(out, "invalid buffer"); break; }
            if (pfmt == 0x26424741U && w > 0 && bpr < w * 4) {
                bpr = w * 4;
                off = 0;
            }
            NmbTexture *t = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); t = nmb_texture_new_on_surface(b.get(), &d, off, bpr, pfmt, plane, w, h, coff, cbpr); }
            if (!t) {
                gpud_log("[gpud] tex_on_surf FAILED client=%u (%llux%llu pfmt=0x%x off=0x%llx bpr=%llu): %s\n",
                         client_id, (unsigned long long)w, (unsigned long long)h, pfmt,
                         (unsigned long long)off, (unsigned long long)bpr, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                record_ws_scanout(client_id, bid, b, w, h, bpr, d.format, off);
                uint64_t tid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, t);
                reply_ok(out); out.put_u64(tid);
            }
            break;
        }
        case OP_TEXTURE_NEW_LINEAR: {
            uint64_t bid = r.get_u64();
            NmbTextureDescriptor d{};
            if (!read_tex_desc(r, d)) { reply_err(out, "bad args"); break; }
            uint64_t off = r.get_u64(), bpr = r.get_u64(), bpi = r.get_u64();
            auto b = get_handle(ch->mu, ch->buffers, bid);
            if (!b) { reply_err(out, "invalid buffer"); break; }
            NmbTexture *t = nmb_texture_new_linear(b.get(), &d, off, bpr, bpi);
            if (!t) reply_err(out, nmb_last_error());
            else {
                uint64_t tid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, t);
                reply_ok(out); out.put_u64(tid);
            }
            break;
        }
        case OP_TEXTURE_NEW_VIEW: {
            uint64_t tid = r.get_u64(), fmt = r.get_u64(), kind = r.get_u64();
            uint64_t lv[2] = { r.get_u64(), r.get_u64() };
            uint64_t sl[2] = { r.get_u64(), r.get_u64() };
            bool whole = (r.get_u8() != 0);
            auto t = get_handle(ch->mu, ch->textures, tid);
            if (!t) { reply_err(out, "invalid texture"); break; }
            NmbTexture *v = nmb_texture_new_view(t.get(), fmt, kind, lv, sl, whole);
            if (!v) reply_err(out, nmb_last_error());
            else {
                uint64_t vid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, v);
                reply_ok(out); out.put_u64(vid);
            }
            break;
        }
        case OP_TEXTURE_NEW_VIEW_SWIZZLE: {
            uint64_t tid = r.get_u64(), fmt = r.get_u64(), kind = r.get_u64();
            uint64_t lv[2] = { r.get_u64(), r.get_u64() };
            uint64_t sl[2] = { r.get_u64(), r.get_u64() };
            uint64_t swiz = r.get_u64();
            auto t = get_handle(ch->mu, ch->textures, tid);
            if (!t) { reply_err(out, "invalid texture"); break; }
            NmbTexture *v = nmb_texture_new_view_swizzle(t.get(), fmt, kind, lv, sl, swiz);
            if (!v) reply_err(out, nmb_last_error());
            else {
                uint64_t vid = put_handle<NmbTexture, nmb_texture_release>(ch->mu, ch->textures, v);
                reply_ok(out); out.put_u64(vid);
            }
            break;
        }
        case OP_TEXTURE_RELEASE: {
            drop_handle(ch->mu, ch->textures, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_TEXTURE_DESCRIBE: {
            auto t = get_handle(ch->mu, ch->textures, r.get_u64());
            NmbTextureDescriptor d{};
            if (t) nmb_texture_describe(t.get(), &d);
            reply_ok(out); write_tex_desc(out, d);
            break;
        }
        case OP_TEXTURE_REPLACE: {
            uint64_t tid = r.get_u64();
            uint64_t reg[6];
            for (int k = 0; k < 6; ++k) reg[k] = r.get_u64();
            uint64_t lv = r.get_u64(), sl = r.get_u64(), bpr = r.get_u64(), bpi = r.get_u64();
            const uint8_t *ptr = nullptr; size_t blen = 0;
            if (!r.get_bytes_view(&ptr, &blen)) { reply_err(out, "bad bytes"); break; }
            auto t = get_handle(ch->mu, ch->textures, tid);
            if (!t) { reply_err(out, "invalid texture"); break; }
            bool rep_ok = false;
            { std::lock_guard<std::mutex> dlk(dev_mu); rep_ok = nmb_texture_replace(t.get(), reg, lv, sl, ptr, bpr, bpi, blen); }
            if (!rep_ok)
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_TEXTURE_GET_BYTES: {
            uint64_t tid = r.get_u64(), bpr = r.get_u64(), bpi = r.get_u64(), len = r.get_u64();
            uint64_t reg[6];
            for (int k = 0; k < 6; ++k) reg[k] = r.get_u64();
            uint64_t lv = r.get_u64(), sl = r.get_u64();
            if (!r.ok || len > 32ULL * 1024 * 1024) { reply_err(out, "bad args"); break; }
            auto t = get_handle(ch->mu, ch->textures, tid);
            if (!t) { reply_err(out, "invalid texture"); break; }
            std::vector<uint8_t> tmp(len);
            bool get_ok = false;
            { std::lock_guard<std::mutex> dlk(dev_mu); get_ok = nmb_texture_get_bytes(t.get(), tmp.data(), bpr, bpi, len, reg, lv, sl); }
            if (!get_ok)
                reply_err(out, nmb_last_error());
            else {
                reply_ok(out);
                out.put_bytes(tmp.data(), tmp.size());
            }
            break;
        }
        case OP_TEXTURE_SET_PURGEABLE: {
            uint64_t id = r.get_u64(), st = r.get_u64();
            auto t = get_handle(ch->mu, ch->textures, id);
            if (!t) { reply_err(out, "invalid texture"); break; }
            reply_ok(out); out.put_u64(nmb_texture_set_purgeable_state(t.get(), st));
            break;
        }
        case OP_TEXTURE_HOLD: {
            auto t = get_handle(ch->mu, ch->textures, r.get_u64());
            reply_ok(out); out.put_u8(t && nmb_texture_hold(t.get()) ? 1 : 0);
            break;
        }
        case OP_TEXTURE_UNHOLD: {
            auto t = get_handle(ch->mu, ch->textures, r.get_u64());
            if (t) nmb_texture_unhold(t.get());
            reply_ok(out);
            break;
        }
        case OP_TEXTURE_ALLOCATED_SIZE: {
            auto t = get_handle(ch->mu, ch->textures, r.get_u64());
            reply_ok(out); out.put_u64(t ? nmb_texture_allocated_size(t.get()) : 0);
            break;
        }
        case OP_TEXTURE_MAKE_ALIASABLE: {
            auto t = get_handle(ch->mu, ch->textures, r.get_u64());
            reply_ok(out); out.put_u8(t && nmb_texture_make_aliasable(t.get()) ? 1 : 0);
            break;
        }
        case OP_TEXTURE_IS_ALIASABLE: {
            auto t = get_handle(ch->mu, ch->textures, r.get_u64());
            reply_ok(out); out.put_u8(t && nmb_texture_is_aliasable(t.get()) ? 1 : 0);
            break;
        }

        // ---- Libraries & Functions ----
        case OP_LIBRARY_NEW_BYTES: {
            bool air = (r.get_u8() != 0);
            const uint8_t *ptr = nullptr; size_t blen = 0;
            if (!r.get_bytes_view(&ptr, &blen)) { reply_err(out, "bad bytes"); break; }
            NmbLibrary *lib = nmb_library_new(dev, ptr, blen, air);
            if (!lib) reply_err(out, nmb_last_error());
            else {
                uint64_t lid = put_handle<NmbLibrary, nmb_library_release>(ch->mu, ch->libraries, lib);
                reply_ok(out); out.put_u64(lid);
            }
            break;
        }
        case OP_LIBRARY_NEW_FILE: {
            std::string path = r.get_str();
            if (!r.ok || path.empty()) { reply_err(out, "bad path"); break; }
            struct stat st{};
            if (stat(path.c_str(), &st) != 0 || st.st_size <= 0) {
                reply_err(out, "library file stat failed");
                break;
            }
            std::string cache_key = (dev == gDevice ? "ws:" : "bg:") + path;
            std::shared_ptr<NmbLibrary> cached;
            {
                std::lock_guard<std::mutex> lk(gFileLibMu);
                auto it = gFileLibCache.find(cache_key);
                if (it != gFileLibCache.end() && it->second.size == st.st_size && it->second.mtime == st.st_mtime) {
                    cached = it->second.lib;
                }
            }
            if (!cached) {
                int fd = open(path.c_str(), O_RDONLY);
                if (fd < 0) { reply_err(out, "library file open failed"); break; }
                void *mapped = mmap(nullptr, (size_t)st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
                close(fd);
                if (mapped == MAP_FAILED) { reply_err(out, "library file mmap failed"); break; }
                NmbLibrary *raw = nmb_library_new(dev, mapped, (size_t)st.st_size, false);
                munmap(mapped, (size_t)st.st_size);
                if (!raw) { reply_err(out, nmb_last_error()); break; }
                cached = std::shared_ptr<NmbLibrary>(raw, [](NmbLibrary *p) { if (p) nmb_library_release(p); });
                {
                    std::lock_guard<std::mutex> lk(gFileLibMu);
                    gFileLibCache[cache_key] = { st.st_size, st.st_mtime, cached };
                }
                gpud_log("[gpud] loaded metallib '%s' (%lld bytes, %u funcs)\n",
                         path.c_str(), (long long)st.st_size, nmb_library_function_count(cached.get()));
            }
            uint64_t lid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lk(ch->mu);
                ch->libraries.emplace(lid, std::move(cached));
            }
            reply_ok(out); out.put_u64(lid);
            break;
        }
        case OP_LIBRARY_RELEASE: {
            drop_handle(ch->mu, ch->libraries, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_LIBRARY_FUNC_COUNT: {
            auto lib = get_handle(ch->mu, ch->libraries, r.get_u64());
            reply_ok(out); out.put_u32(lib ? nmb_library_function_count(lib.get()) : 0);
            break;
        }
        case OP_LIBRARY_FUNC_NAME: {
            uint64_t lid = r.get_u64(); uint32_t idx = r.get_u32();
            auto lib = get_handle(ch->mu, ch->libraries, lid);
            if (!lib) { reply_err(out, "invalid library"); break; }
            size_t need = nmb_library_function_name(lib.get(), idx, nullptr, 0);
            std::string s(need, '\0');
            if (need) nmb_library_function_name(lib.get(), idx, s.data(), need + 1);
            reply_ok(out); out.put_str_nonnull(s.c_str());
            break;
        }
        case OP_FUNCTION_NEW:
        case OP_FUNCTION_NEW_ZERO: {
            uint64_t lid = r.get_u64();
            std::string name = r.get_str();
            if (!r.ok) { reply_err(out, "bad args"); break; }
            auto lib = get_handle(ch->mu, ch->libraries, lid);
            if (!lib) { reply_err(out, "invalid library"); break; }
            NmbFunction *fn = (op == OP_FUNCTION_NEW)
                ? nmb_function_new(lib.get(), name.c_str())
                : nmb_function_new_zero_constants(lib.get(), name.c_str());
            if (!fn) {
                gpud_log("[gpud] fn FAILED client=%u name=%s: %s\n", client_id, name.c_str(), nmb_last_error());
                reply_err(out, nmb_last_error());
            }
            else {
                uint64_t fid = put_handle<NmbFunction, nmb_function_release>(ch->mu, ch->functions, fn);
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->fn_names[fid] = name;
                }
                reply_ok(out); out.put_u64(fid);
            }
            break;
        }
        case OP_FUNCTION_NEW_SPEC:
        case OP_FUNCTION_NEW_SPEC_ZERO: {
            uint64_t lid = r.get_u64();
            std::string name = r.get_str();
            uint32_t cnt = r.get_u32();
            if (!r.ok || cnt > 4096) { reply_err(out, "bad args"); break; }
            std::vector<std::string> cnames(cnt);
            std::vector<bool> has_names(cnt);
            std::vector<NmbConstant> cs(cnt);
            for (uint32_t i = 0; i < cnt; ++i) {
                cs[i].type = r.get_u64();
                cs[i].index = r.get_u64();
                bool hn = false;
                if (!r.get_opt_str(hn, cnames[i])) break;
                has_names[i] = hn;
                cs[i].name = hn ? cnames[i].c_str() : nullptr;
                const uint8_t *bp = nullptr; size_t bl = 0;
                if (!r.get_bytes_view(&bp, &bl)) break;
                cs[i].bytes = bp;
                cs[i].length = bl;
            }
            if (!r.ok) { reply_err(out, "bad constants"); break; }
            auto lib = get_handle(ch->mu, ch->libraries, lid);
            if (!lib) { reply_err(out, "invalid library"); break; }
            NmbFunction *fn = (op == OP_FUNCTION_NEW_SPEC)
                ? nmb_function_new_specialized(lib.get(), name.c_str(), cs.data(), cnt)
                : nmb_function_new_specialized_zero_rest(lib.get(), name.c_str(), cs.data(), cnt);
            if (!fn) {
                gpud_log("[gpud] fn FAILED client=%u name=%s: %s\n", client_id, name.c_str(), nmb_last_error());
                reply_err(out, nmb_last_error());
            }
            else {
                uint64_t fid = put_handle<NmbFunction, nmb_function_release>(ch->mu, ch->functions, fn);
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->fn_names[fid] = name;
                }
                reply_ok(out); out.put_u64(fid);
            }
            break;
        }
        case OP_FUNCTION_RELEASE: {
            drop_handle(ch->mu, ch->functions, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_FUNCTION_INFO: {
            auto fn = get_handle(ch->mu, ch->functions, r.get_u64());
            reply_ok(out);
            out.put_u64(fn ? nmb_function_type(fn.get()) : 0);
            out.put_u64(fn ? nmb_function_patch_type(fn.get()) : 0);
            out.put_i64(fn ? nmb_function_patch_control_point_count(fn.get()) : -1);
            out.put_u32(fn ? nmb_function_constant_count(fn.get()) : 0);
            break;
        }
        case OP_FUNCTION_CONST_AT: {
            uint64_t fid = r.get_u64(); uint32_t at = r.get_u32();
            auto fn = get_handle(ch->mu, ch->functions, fid);
            if (!fn) { reply_err(out, "invalid function"); break; }
            size_t nsz = nmb_function_constant_name_size(fn.get(), at);
            std::string cname(nsz + 1, '\0');
            uint64_t idx = 0, dtype = 0; bool req = false;
            if (!nmb_function_constant_at(fn.get(), at, cname.data(), cname.size(), &idx, &dtype, &req)) {
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
                out.put_str_nonnull(cname.c_str());
                out.put_u64(idx); out.put_u64(dtype); out.put_u8(req ? 1 : 0);
            }
            break;
        }
        case OP_FUNCTION_SET_SPEC_NAME: {
            uint64_t fid = r.get_u64();
            bool hn = false; std::string sname;
            if (!r.get_opt_str(hn, sname)) { reply_err(out, "bad args"); break; }
            auto fn = get_handle(ch->mu, ch->functions, fid);
            if (!fn || !nmb_function_set_specialized_name(fn.get(), hn ? sname.c_str() : nullptr))
                reply_err(out, nmb_last_error());
            else {
                if (hn) {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->fn_names[fid] = sname;
                    // Keep A2Xghfc/Xghfc (218,964 LLVM IR instructions) bypassed: m2v's CPU AIR->SPIR-V
                    // translator stalls on it regardless of Mali vs SwiftShader.
                    if (sname == "A2Xghfc" || sname == "Xghfc") {
                        ch->tcim_fids.insert(fid);
                    } else {
                        ch->tcim_fids.erase(fid);
                    }
                }
                static std::atomic<uint32_t> specNameLog{0};
                if (client_id == 1 || specNameLog.fetch_add(1, std::memory_order_relaxed) < 16)
                    gpud_log("[gpud] fn_set_spec_name client=%u fid=%llu -> '%s'\n",
                             client_id, (unsigned long long)fid, hn ? sname.c_str() : "<null>");
                reply_ok(out);
            }
            break;
        }

        // ---- Pipelines, Samplers, DepthStencil, BinaryArchive ----
        case OP_COMPUTE_PSO_NEW: {
            uint64_t fid = r.get_u64();
            std::vector<std::shared_ptr<NmbBinaryArchive>> keep_a;
            auto to_ar = [&](uint64_t id){ return hold_arch(id, keep_a); };
            DecodedPipelineOpts<decltype(to_ar)> opts;
            if (!opts.read(r, to_ar)) {
                reply_err(out, "bad opts"); break;
            }
            auto fn = get_handle(ch->mu, ch->functions, fid);
            if (!fn) { reply_err(out, "invalid function"); break; }
            uint64_t t0 = monotonic_ms();
            gInPso.fetch_add(1, std::memory_order_relaxed);
            if (client_id == 1) gpud_log("[gpud] c1 compute PSO START\n");
            NmbComputePipeline *pso = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); pso = nmb_compute_pipeline_new_with_options(dev, fn.get(), opts.ptr()); }
            gInPso.fetch_sub(1, std::memory_order_relaxed);
            uint64_t dt = monotonic_ms() - t0;
            if (!pso) {
                gpud_log("[gpud] compute PSO FAILED client=%u (%llums): %s\n", client_id, (unsigned long long)dt, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t c = gComputePsoCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (client_id == 1 || c <= 15 || (c % 25) == 0)
                    gpud_log("[gpud] compute PSO #%llu compiled (client=%u %llums)\n", (unsigned long long)c, client_id, (unsigned long long)dt);
                uint64_t pid = put_handle<NmbComputePipeline, nmb_compute_pipeline_release>(ch->mu, ch->compute_psos, pso);
                reply_ok(out);
                out.put_u64(pid);
                out.put_u64(nmb_compute_pipeline_max_threads(pso));
                out.put_u64(nmb_compute_pipeline_execution_width(pso));
            }
            break;
        }
        case OP_COMPUTE_PSO_NEW_DESC: {
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            std::vector<std::shared_ptr<NmbBinaryArchive>> keep_a;
            auto to_fn = [&](uint64_t id) { return hold_fn(id, keep_f); };
            auto to_ar = [&](uint64_t id) { return hold_arch(id, keep_a); };
            DecodedComputePsoDesc<decltype(to_fn)> desc;
            DecodedPipelineOpts<decltype(to_ar)> opts;
            if (!desc.read(r, to_fn) || !opts.read(r, to_ar)) { reply_err(out, "bad desc"); break; }
            uint64_t t0 = monotonic_ms();
            gInPso.fetch_add(1, std::memory_order_relaxed);
            if (client_id == 1) gpud_log("[gpud] c1 computeDesc PSO START\n");
            NmbComputePipeline *pso = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); pso = nmb_compute_pipeline_new_with_descriptor(dev, &desc.d, opts.ptr()); }
            gInPso.fetch_sub(1, std::memory_order_relaxed);
            uint64_t dt = monotonic_ms() - t0;
            if (!pso) {
                gpud_log("[gpud] computeDesc PSO FAILED client=%u (%llums): %s\n", client_id, (unsigned long long)dt, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t c = gComputePsoCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (client_id == 1 || c <= 15 || (c % 25) == 0)
                    gpud_log("[gpud] compute PSO #%llu compiled (client=%u %llums)\n", (unsigned long long)c, client_id, (unsigned long long)dt);
                uint64_t pid = put_handle<NmbComputePipeline, nmb_compute_pipeline_release>(ch->mu, ch->compute_psos, pso);
                reply_ok(out);
                out.put_u64(pid);
                out.put_u64(nmb_compute_pipeline_max_threads(pso));
                out.put_u64(nmb_compute_pipeline_execution_width(pso));
            }
            break;
        }
        case OP_COMPUTE_PSO_RELEASE: {
            drop_handle(ch->mu, ch->compute_psos, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_COMPUTE_PSO_INFO: {
            auto pso = get_handle(ch->mu, ch->compute_psos, r.get_u64());
            reply_ok(out);
            out.put_u64(pso ? nmb_compute_pipeline_max_threads(pso.get()) : 0);
            out.put_u64(pso ? nmb_compute_pipeline_execution_width(pso.get()) : 0);
            break;
        }
        case OP_RENDER_PSO_NEW: {
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            std::vector<std::shared_ptr<NmbBinaryArchive>> keep_a;
            bool is_tcim = false;
            std::string pso_label;
            auto to_fn = [&](uint64_t id) {
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    if (ch->tcim_fids.count(id) != 0) is_tcim = true;
                    auto it = ch->fn_names.find(id);
                    std::string fn_lbl = (it != ch->fn_names.end()) ? it->second : ("fid" + std::to_string(id));
                    if (!pso_label.empty()) pso_label += "+";
                    pso_label += fn_lbl;
                }
                return hold_fn(id, keep_f);
            };
            auto to_ar = [&](uint64_t id) { return hold_arch(id, keep_a); };
            DecodedRenderPsoDesc<decltype(to_fn)> desc;
            DecodedPipelineOpts<decltype(to_ar)> opts;
            if (!desc.read(r, to_fn) || !opts.read(r, to_ar)) { reply_err(out, "bad desc"); break; }
            if (is_tcim) {
                uint64_t pid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->tcim_pids.insert(pid);
                    ch->pso_names[pid] = pso_label;
                }
                gpud_log("[gpud] BYPASSED heavy render PSO pid=%llu (%s) (client=%u 0ms)\n",
                         (unsigned long long)pid, pso_label.c_str(), client_id);
                reply_ok(out); out.put_u64(pid);
                break;
            }
            // Fast 0ms aliasing for slow 16-tap bicubic (*S1*) & Tcim*_Isrc* shaders to their
            // already-compiled bilinear equivalents (saves ~17.5 minutes of AIR->SPIR-V compile
            // and prevents Compile(Budget(Time)) failures while rendering full desktop layers!).
            {
                std::string alias_lbl;
                size_t s1_pos = pso_label.find("S1");
                if (s1_pos != std::string::npos) {
                    alias_lbl = pso_label;
                    alias_lbl.erase(s1_pos, 2);
                } else if (pso_label.find("+Tcim") != std::string::npos &&
                           pso_label.find("_Isrc") != std::string::npos) {
                    alias_lbl = pso_label;
                    size_t p = alias_lbl.find("+Tcim");
                    alias_lbl.replace(p, 5, "+Timg");
                    size_t m1 = alias_lbl.find("M1");
                    if (m1 != std::string::npos) alias_lbl.erase(m1, 2);
                }
                if (!alias_lbl.empty()) {
                    std::string k_exact = alias_lbl + "|" + std::to_string(desc.d.colors[0].format) + "|" + std::to_string(desc.d.colors[0].blending);
                    std::string k_fmt   = alias_lbl + "|" + std::to_string(desc.d.colors[0].format);
                    std::shared_ptr<NmbRenderPipeline> aliased;
                    {
                        std::lock_guard<std::mutex> lk(ch->mu);
                        auto it = ch->pso_alias_cache.find(k_exact);
                        if (it == ch->pso_alias_cache.end()) it = ch->pso_alias_cache.find(k_fmt);
                        if (it != ch->pso_alias_cache.end()) aliased = it->second;
                    }
                    if (aliased) {
                        uint64_t pid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
                        {
                            std::lock_guard<std::mutex> lk(ch->mu);
                            ch->render_psos[pid] = aliased;
                            ch->pso_names[pid] = pso_label + "=>" + alias_lbl;
                        }
                        gpud_log("[gpud] ALIASED render PSO pid=%llu (%s => %s) (client=%u 0ms)\n",
                                 (unsigned long long)pid, pso_label.c_str(), alias_lbl.c_str(), client_id);
                        reply_ok(out); out.put_u64(pid);
                        break;
                    }
                }
            }
            uint64_t t0 = monotonic_ms();
            gInPso.fetch_add(1, std::memory_order_relaxed);
            if (client_id == 1) gpud_log("[gpud] c1 render PSO START (%s)\n", pso_label.c_str());
            NmbRenderPipeline *pso = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); pso = nmb_render_pipeline_new_with_options(dev, &desc.d, opts.ptr()); }
            gInPso.fetch_sub(1, std::memory_order_relaxed);
            uint64_t dt = monotonic_ms() - t0;
            if (!pso) {
                gpud_log("[gpud] render PSO FAILED client=%u (%s %llums): %s\n",
                         client_id, pso_label.c_str(), (unsigned long long)dt, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t c = gRenderPsoCount.fetch_add(1, std::memory_order_relaxed) + 1;
                uint64_t pid = put_handle<NmbRenderPipeline, nmb_render_pipeline_release>(ch->mu, ch->render_psos, pso);
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->pso_names[pid] = pso_label;
                    auto sp = ch->render_psos[pid];
                    std::string k_exact = pso_label + "|" + std::to_string(desc.d.colors[0].format) + "|" + std::to_string(desc.d.colors[0].blending);
                    std::string k_fmt   = pso_label + "|" + std::to_string(desc.d.colors[0].format);
                    ch->pso_alias_cache[k_exact] = sp;
                    if (ch->pso_alias_cache.find(k_fmt) == ch->pso_alias_cache.end()) {
                        ch->pso_alias_cache[k_fmt] = sp;
                    }
                }
                if (client_id == 1 || c <= 15 || (c % 25) == 0)
                    gpud_log("[gpud] render PSO #%llu pid=%llu (%s) compiled (client=%u %llums)\n",
                             (unsigned long long)c, (unsigned long long)pid, pso_label.c_str(), client_id, (unsigned long long)dt);
                reply_ok(out); out.put_u64(pid);
            }
            break;
        }
        case OP_TILE_PSO_NEW: {
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            std::vector<std::shared_ptr<NmbBinaryArchive>> keep_a;
            bool is_tcim = false;
            std::string pso_label;
            auto to_fn = [&](uint64_t id) {
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    if (ch->tcim_fids.count(id) != 0) is_tcim = true;
                    auto it = ch->fn_names.find(id);
                    std::string fn_lbl = (it != ch->fn_names.end()) ? it->second : ("fid" + std::to_string(id));
                    if (!pso_label.empty()) pso_label += "+";
                    pso_label += fn_lbl;
                }
                return hold_fn(id, keep_f);
            };
            auto to_ar = [&](uint64_t id) { return hold_arch(id, keep_a); };
            DecodedTilePsoDesc<decltype(to_fn)> desc;
            DecodedPipelineOpts<decltype(to_ar)> opts;
            if (!desc.read(r, to_fn) || !opts.read(r, to_ar)) { reply_err(out, "bad desc"); break; }
            if (is_tcim) {
                uint64_t pid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->tcim_pids.insert(pid);
                    ch->pso_names[pid] = pso_label;
                }
                gpud_log("[gpud] BYPASSED heavy tile PSO pid=%llu (%s) (client=%u 0ms)\n",
                         (unsigned long long)pid, pso_label.c_str(), client_id);
                reply_ok(out); out.put_u64(pid);
                break;
            }
            NmbRenderPipeline *pso = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); pso = nmb_tile_pipeline_new(dev, &desc.d, opts.ptr()); }
            if (!pso) {
                gpud_log("[gpud] tile PSO FAILED client=%u: %s\n", client_id, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t pid = put_handle<NmbRenderPipeline, nmb_render_pipeline_release>(ch->mu, ch->render_psos, pso);
                {
                    std::lock_guard<std::mutex> lk(ch->mu);
                    ch->pso_names[pid] = pso_label;
                }
                reply_ok(out); out.put_u64(pid);
            }
            break;
        }
        case OP_RENDER_PSO_RELEASE: {
            drop_handle(ch->mu, ch->render_psos, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_RENDER_PSO_REFLECTION: {
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            auto to_fn = [&](uint64_t id) { return hold_fn(id, keep_f); };
            DecodedRenderPsoDesc<decltype(to_fn)> desc;
            if (!desc.read(r, to_fn)) { reply_err(out, "bad desc"); break; }
            NmbReflection *ref = nmb_render_pipeline_reflection(&desc.d);
            if (!ref) reply_err(out, nmb_last_error());
            else {
                const char *js = nmb_reflection_json(ref);
                reply_ok(out); out.put_str_nonnull(js);
                nmb_reflection_release(ref);
            }
            break;
        }
        case OP_COMPUTE_PSO_REFLECTION: {
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            auto to_fn = [&](uint64_t id) { return hold_fn(id, keep_f); };
            DecodedComputePsoDesc<decltype(to_fn)> desc;
            if (!desc.read(r, to_fn)) { reply_err(out, "bad desc"); break; }
            NmbReflection *ref = nmb_compute_pipeline_reflection(&desc.d);
            if (!ref) reply_err(out, nmb_last_error());
            else {
                const char *js = nmb_reflection_json(ref);
                reply_ok(out); out.put_str_nonnull(js);
                nmb_reflection_release(ref);
            }
            break;
        }
        case OP_SAMPLER_NEW: {
            NmbSamplerDescriptor d{};
            if (!r.get_pod(d)) { reply_err(out, "bad args"); break; }
            NmbSampler *s = nmb_sampler_new(dev, &d);
            if (!s) reply_err(out, nmb_last_error());
            else {
                uint64_t sid = put_handle<NmbSampler, nmb_sampler_release>(ch->mu, ch->samplers, s);
                reply_ok(out); out.put_u64(sid);
            }
            break;
        }
        case OP_SAMPLER_RELEASE: {
            drop_handle(ch->mu, ch->samplers, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_DEPTH_STENCIL_NEW: {
            NmbDepthStencilDescriptor d{};
            if (!r.get_pod(d)) { reply_err(out, "bad args"); break; }
            NmbDepthStencil *ds = nmb_depth_stencil_new(dev, &d);
            if (!ds) reply_err(out, nmb_last_error());
            else {
                uint64_t dsid = put_handle<NmbDepthStencil, nmb_depth_stencil_release>(ch->mu, ch->depth_stencils, ds);
                reply_ok(out); out.put_u64(dsid);
            }
            break;
        }
        case OP_DEPTH_STENCIL_RELEASE: {
            drop_handle(ch->mu, ch->depth_stencils, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_ARCHIVE_NEW: {
            bool hp = false; std::string path;
            if (!r.get_opt_str(hp, path)) { reply_err(out, "bad args"); break; }
            NmbBinaryArchive *a = nmb_binary_archive_new(dev, hp ? path.c_str() : nullptr);
            if (!a) reply_err(out, nmb_last_error());
            else {
                uint64_t aid = put_handle<NmbBinaryArchive, nmb_binary_archive_release>(ch->mu, ch->archives, a);
                reply_ok(out); out.put_u64(aid);
            }
            break;
        }
        case OP_ARCHIVE_RELEASE: {
            drop_handle(ch->mu, ch->archives, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_ARCHIVE_SERIALIZE: {
            uint64_t aid = r.get_u64();
            std::string path = r.get_str();
            auto a = get_handle(ch->mu, ch->archives, aid);
            if (!a || !nmb_binary_archive_serialize(a.get(), path.c_str()))
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_ARCHIVE_ADD_RENDER: {
            uint64_t aid = r.get_u64();
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            auto to_fn = [&](uint64_t id) { return hold_fn(id, keep_f); };
            DecodedRenderPsoDesc<decltype(to_fn)> desc;
            if (!desc.read(r, to_fn)) { reply_err(out, "bad desc"); break; }
            auto a = get_handle(ch->mu, ch->archives, aid);
            if (!a || !nmb_binary_archive_add_render(a.get(), &desc.d))
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_ARCHIVE_ADD_COMPUTE: {
            uint64_t aid = r.get_u64();
            std::vector<std::shared_ptr<NmbFunction>> keep_f;
            auto to_fn = [&](uint64_t id) { return hold_fn(id, keep_f); };
            DecodedComputePsoDesc<decltype(to_fn)> desc;
            if (!desc.read(r, to_fn)) { reply_err(out, "bad desc"); break; }
            auto a = get_handle(ch->mu, ch->archives, aid);
            if (!a || !nmb_binary_archive_add_compute_descriptor(a.get(), &desc.d))
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }

        // ---- Queues, CommandBuffers, Events, Fences ----
        case OP_QUEUE_NEW: {
            uint64_t max_cbs = r.get_u64();
            NmbQueue *q = max_cbs ? nmb_queue_new_with_max_command_buffer_count(dev, (size_t)max_cbs)
                                  : nmb_queue_new(dev);
            if (!q) reply_err(out, nmb_last_error());
            else {
                uint64_t qid = put_handle<NmbQueue, nmb_queue_release>(ch->mu, ch->queues, q);
                reply_ok(out); out.put_u64(qid);
            }
            break;
        }
        case OP_QUEUE_RELEASE: {
            drop_handle(ch->mu, ch->queues, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_CB_NEW: {
            auto q = get_handle(ch->mu, ch->queues, r.get_u64());
            if (!q) { reply_err(out, "invalid queue"); break; }
            NmbCommandBuffer *cb = nmb_command_buffer_new(q.get());
            if (!cb) reply_err(out, nmb_last_error());
            else {
                uint64_t cbid = put_handle<NmbCommandBuffer, nmb_command_buffer_release>(ch->mu, ch->cbs, cb);
                reply_ok(out); out.put_u64(cbid);
            }
            break;
        }
        case OP_CB_RELEASE: {
            drop_handle(ch->mu, ch->cbs, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_CB_USES_BUFFER: {
            uint64_t cbid = r.get_u64(), bid = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            auto b = get_handle(ch->mu, ch->buffers, bid);
            int res = (cb && b) ? nmb_command_buffer_uses_buffer(cb.get(), b.get()) : 0;
            reply_ok(out); out.put_i32(res);
            break;
        }
        case OP_CB_USES_TEXTURE: {
            uint64_t cbid = r.get_u64(), tid = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            auto t = get_handle(ch->mu, ch->textures, tid);
            int res = (cb && t) ? nmb_command_buffer_uses_texture(cb.get(), t.get()) : 0;
            reply_ok(out); out.put_i32(res);
            break;
        }
        case OP_CB_ON_COMPLETED: {
            uint64_t cbid = r.get_u64(), cb_tok = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            if (!cb) { reply_err(out, "invalid cb"); break; }
            auto *ctx = new AsyncCbCtx{ client_id, cb_tok, EVT_CB_COMPLETED };
            if (!nmb_command_buffer_on_completed(cb.get(), cb_completed_trampoline, ctx)) {
                delete ctx;
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
            }
            break;
        }
        case OP_CB_ON_SCHEDULED: {
            uint64_t cbid = r.get_u64(), cb_tok = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            if (!cb) { reply_err(out, "invalid cb"); break; }
            auto *ctx = new AsyncCbCtx{ client_id, cb_tok, EVT_CB_SCHEDULED };
            if (!nmb_command_buffer_on_scheduled(cb.get(), cb_completed_trampoline, ctx)) {
                delete ctx;
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
            }
            break;
        }
        case OP_CB_COMMIT: {
            uint64_t cbid = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            if (!cb) { reply_err(out, "invalid cb"); break; }
            bool ok = false;
            {
                std::lock_guard<std::mutex> dlk(dev_mu);
                ok = nmb_command_buffer_commit(cb.get());
            }
            if (!ok) {
                gpud_log("[gpud] commit FAILED client=%u cbid=%llu: %s\n", client_id, (unsigned long long)cbid, nmb_last_error());
                reply_err(out, nmb_last_error());
            } else {
                uint64_t c = gCommitCount.fetch_add(1, std::memory_order_relaxed) + 1;
                if (client_id == 1) {
                    gC1Commits.fetch_add(1, std::memory_order_relaxed);
                    // Wait up to 15000ms outside dev_mu so 1472x920 RGBA16Float frames
                    // finish SwiftShader rendering + shadow->IOSurface copy-back BEFORE
                    // WindowServer swaps the IOSurface to NativeLegacyDisplay!
                    uint64_t wt0 = monotonic_ms();
                    int wrc = nmb_command_buffer_wait(cb.get(), 15000);
                    uint64_t wdt = monotonic_ms() - wt0;
                    if (wrc == 1) {
                        char dbgbuf[256] = {};
                        nmb_command_buffer_debug_state(cb.get(), dbgbuf, sizeof(dbgbuf));
                        gpud_log("[gpud] c1 commit #%llu (cbid=%llu) >15000ms [%s], continuing async\n",
                                 (unsigned long long)c, (unsigned long long)cbid, dbgbuf);
                    } else if (c <= 60 || (c % 25) == 0) {
                        gpud_log("[gpud] c1 commit #%llu DONE in %llums (wrc=%d)\n",
                                 (unsigned long long)c, (unsigned long long)wdt, wrc);
                    }
                    if (c == 30 || c == 100 || c == 250 || gFbshotPending.load(std::memory_order_relaxed)) {
                        std::lock_guard<std::mutex> slk(gScanoutMu);
                        if (dump_scanout_ppm_locked("/tmp/fb.ppm")) {
                            gFbshotPending.store(false, std::memory_order_relaxed);
                            gFbshotReady.store(true, std::memory_order_relaxed);
                        }
                    }
                }
                if (c <= 60 || (c % 25) == 0) gpud_log("[gpud] commit #%llu OK (client=%u cbid=%llu)\n", (unsigned long long)c, client_id, (unsigned long long)cbid);
                reply_ok(out);
            }
            break;
        }
        case OP_CB_WAIT_SCHEDULED: {
            auto cb = get_handle(ch->mu, ch->cbs, r.get_u64());
            uint64_t t0 = monotonic_ms();
            bool ok = cb && nmb_command_buffer_wait_scheduled(cb.get());
            uint64_t dt = monotonic_ms() - t0;
            if (client_id == 1 && dt >= 500)
                gpud_log("[gpud] c1 OP_CB_WAIT_SCHEDULED %llums ok=%d\n", (unsigned long long)dt, ok ? 1 : 0);
            if (!ok)
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_CB_WAIT: {
            uint64_t cbid = r.get_u64(), timeout_ms = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            if (!cb) { reply_err(out, "invalid cb"); break; }
            uint64_t t0 = monotonic_ms();
            int rc = 0;
            if (client_id == 1 && timeout_ms == 0) {
                uint64_t st0 = nmb_command_buffer_status(cb.get());
                gpud_log("[gpud] c1 OP_CB_WAIT START cbid=%llu st=%llu\n",
                         (unsigned long long)cbid, (unsigned long long)st0);
                for (int iter = 0; ; ++iter) {
                    rc = nmb_command_buffer_wait(cb.get(), 10000);
                    if (rc != 1) break;
                    uint64_t st = nmb_command_buffer_status(cb.get());
                    char errbuf[128] = {};
                    nmb_command_buffer_error(cb.get(), errbuf, sizeof(errbuf));
                    char dbgbuf[512] = {};
                    nmb_command_buffer_debug_state(cb.get(), dbgbuf, sizeof(dbgbuf));
                    gpud_log("[gpud] c1 OP_CB_WAIT STILL WAITING cbid=%llu %llums st=%llu err='%s' [%s]\n",
                             (unsigned long long)cbid, (unsigned long long)(monotonic_ms() - t0),
                             (unsigned long long)st, errbuf, dbgbuf);
                }
            } else if (timeout_ms == 0) {
                for (int iter = 0; ; ++iter) {
                    rc = nmb_command_buffer_wait(cb.get(), 15000);
                    if (rc != 1) break;
                    char dbgbuf[256] = {};
                    nmb_command_buffer_debug_state(cb.get(), dbgbuf, sizeof(dbgbuf));
                    gpud_log("[gpud] c%u OP_CB_WAIT STILL WAITING cbid=%llu %llums [%s]\n",
                             client_id, (unsigned long long)cbid,
                             (unsigned long long)(monotonic_ms() - t0), dbgbuf);
                }
            } else {
                rc = nmb_command_buffer_wait(cb.get(), timeout_ms);
            }
            uint64_t dt = monotonic_ms() - t0;
            if (client_id == 1 && dt >= 500)
                gpud_log("[gpud] c1 OP_CB_WAIT DONE cbid=%llu %llums rc=%d st=%llu\n",
                         (unsigned long long)cbid, (unsigned long long)dt, rc,
                         (unsigned long long)nmb_command_buffer_status(cb.get()));
            reply_ok(out); out.put_i32(rc);
            break;
        }
        case OP_CB_STATUS: {
            auto cb = get_handle(ch->mu, ch->cbs, r.get_u64());
            uint64_t st = cb ? nmb_command_buffer_status(cb.get()) : 5;
            uint64_t mcode = cb ? nmb_command_buffer_metal_error_code(cb.get()) : 0;
            double gs = cb ? nmb_command_buffer_gpu_time(cb.get(), false) : 0.0;
            double ge = cb ? nmb_command_buffer_gpu_time(cb.get(), true) : 0.0;
            char errbuf[256] = {};
            if (cb) nmb_command_buffer_error(cb.get(), errbuf, sizeof(errbuf));
            reply_ok(out);
            out.put_u64(st); out.put_u64(mcode);
            out.put_f64(gs); out.put_f64(ge);
            out.put_str_nonnull(errbuf);
            break;
        }
        case OP_CB_SIGNAL_EVENT: {
            uint64_t cbid = r.get_u64(), eid = r.get_u64(), val = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            auto ev = get_handle(ch->mu, ch->events, eid);
            if (client_id == 1)
                gpud_log("[gpud] c1 SIGNAL_EVENT cbid=%llu eid=%llu val=%llu\n",
                         (unsigned long long)cbid, (unsigned long long)eid, (unsigned long long)val);
            if (!cb || !ev || !nmb_command_buffer_signal_event(cb.get(), ev.get(), val))
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_CB_WAIT_EVENT: {
            uint64_t cbid = r.get_u64(), eid = r.get_u64(), val = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            auto ev = get_handle(ch->mu, ch->events, eid);
            if (client_id == 1)
                gpud_log("[gpud] c1 WAIT_EVENT cbid=%llu eid=%llu val=%llu cur=%llu\n",
                         (unsigned long long)cbid, (unsigned long long)eid, (unsigned long long)val,
                         (unsigned long long)(ev ? nmb_shared_event_value(ev.get()) : 0));
            if (!cb || !ev || !nmb_command_buffer_wait_event(cb.get(), ev.get(), val))
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_EVENT_NEW: {
            bool shared = (r.get_u8() != 0);
            NmbEvent *ev = nmb_event_new(dev, shared);
            if (!ev) reply_err(out, nmb_last_error());
            else {
                uint64_t eid = put_handle<NmbEvent, nmb_event_release>(ch->mu, ch->events, ev);
                reply_ok(out); out.put_u64(eid);
            }
            break;
        }
        case OP_EVENT_RELEASE: {
            drop_handle(ch->mu, ch->events, r.get_u64());
            reply_ok(out);
            break;
        }
        case OP_SHARED_EVENT_VALUE: {
            auto ev = get_handle(ch->mu, ch->events, r.get_u64());
            reply_ok(out); out.put_u64(ev ? nmb_shared_event_value(ev.get()) : 0);
            break;
        }
        case OP_SHARED_EVENT_SET_VALUE: {
            uint64_t eid = r.get_u64(), val = r.get_u64();
            auto ev = get_handle(ch->mu, ch->events, eid);
            if (!ev || !nmb_shared_event_set_value(ev.get(), val))
                reply_err(out, nmb_last_error());
            else
                reply_ok(out);
            break;
        }
        case OP_SHARED_EVENT_WAIT: {
            uint64_t eid = r.get_u64(), val = r.get_u64(), to = r.get_u64();
            auto ev = get_handle(ch->mu, ch->events, eid);
            bool reached = ev && nmb_shared_event_wait(ev.get(), val, to);
            reply_ok(out); out.put_u8(reached ? 1 : 0);
            break;
        }
        case OP_SHARED_EVENT_NOTIFY: {
            uint64_t eid = r.get_u64(), val = r.get_u64(), cb_tok = r.get_u64();
            auto ev = get_handle(ch->mu, ch->events, eid);
            if (!ev) { reply_err(out, "invalid event"); break; }
            auto *ctx = new AsyncCbCtx{ client_id, cb_tok, EVT_SHARED_EVENT };
            if (!nmb_shared_event_notify(ev.get(), val, cb_shared_event_trampoline, ctx)) {
                delete ctx;
                reply_err(out, nmb_last_error());
            } else {
                reply_ok(out);
            }
            break;
        }
        case OP_FENCE_NEW: {
            NmbFence *f = nmb_fence_new(dev);
            if (!f) reply_err(out, nmb_last_error());
            else {
                uint64_t fid = put_handle<NmbFence, nmb_fence_release>(ch->mu, ch->fences, f);
                reply_ok(out); out.put_u64(fid);
            }
            break;
        }
        case OP_FENCE_RELEASE: {
            drop_handle(ch->mu, ch->fences, r.get_u64());
            reply_ok(out);
            break;
        }

        // ---- Encoders ----
        case OP_ENC_COMPUTE_NEW: {
            auto cb = get_handle(ch->mu, ch->cbs, r.get_u64());
            if (!cb) { reply_err(out, "invalid cb"); break; }
            NmbEncoder *enc = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); enc = nmb_compute_encoder_new(cb.get()); }
            if (!enc) reply_err(out, nmb_last_error());
            else {
                uint64_t eid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
                std::lock_guard<std::mutex> lk(ch->mu);
                ch->encoders[eid] = enc;
                reply_ok(out); out.put_u64(eid);
            }
            break;
        }
        case OP_ENC_BLIT_NEW: {
            auto cb = get_handle(ch->mu, ch->cbs, r.get_u64());
            if (!cb) { reply_err(out, "invalid cb"); break; }
            NmbEncoder *enc = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); enc = nmb_blit_encoder_new(cb.get()); }
            if (!enc) reply_err(out, nmb_last_error());
            else {
                uint64_t eid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
                std::lock_guard<std::mutex> lk(ch->mu);
                ch->encoders[eid] = enc;
                reply_ok(out); out.put_u64(eid);
            }
            break;
        }
        case OP_ENC_RENDER_NEW: {
            uint64_t cbid = r.get_u64();
            auto cb = get_handle(ch->mu, ch->cbs, cbid);
            std::vector<std::shared_ptr<NmbTexture>> keep_t;
            std::vector<std::shared_ptr<NmbBuffer>> keep_b;
            auto to_tex = [&](uint64_t id) -> NmbTexture * {
                auto sp = get_handle(ch->mu, ch->textures, id);
                if (!sp) return nullptr;
                keep_t.push_back(sp);
                return sp.get();
            };
            auto to_buf = [&](uint64_t id) -> NmbBuffer * {
                auto sp = get_handle(ch->mu, ch->buffers, id);
                if (!sp) return nullptr;
                keep_b.push_back(sp);
                return sp.get();
            };
            NmbRenderPassDescriptor rp{};
            if (!read_render_pass_desc(r, rp, to_tex, to_buf) || !cb) {
                reply_err(out, "invalid render pass desc"); break;
            }
            NmbEncoder *enc = nullptr;
            { std::lock_guard<std::mutex> dlk(dev_mu); enc = nmb_render_encoder_new(cb.get(), &rp); }
            if (!enc) reply_err(out, nmb_last_error());
            else {
                uint64_t eid = gNextHandle.fetch_add(1, std::memory_order_relaxed);
                std::lock_guard<std::mutex> lk(ch->mu);
                ch->encoders[eid] = enc;
                reply_ok(out); out.put_u64(eid);
            }
            break;
        }
        case OP_ENC_BATCH: {
            uint64_t eid = r.get_u64();
            bool do_end = (r.get_u8() != 0);
            NmbEncoder *enc = nullptr;
            {
                std::lock_guard<std::mutex> lk(ch->mu);
                auto it = ch->encoders.find(eid);
                if (it != ch->encoders.end()) {
                    enc = it->second;
                    if (do_end) ch->encoders.erase(it);
                }
            }
            if (!enc) { reply_err(out, "invalid encoder"); break; }
            bool ok = false;
            {
                std::lock_guard<std::mutex> dlk(dev_mu);
                ok = replay_encoder_batch(*ch, enc, r);
                if (do_end) {
                    if (ok) ok = nmb_encoder_end(enc);
                    else nmb_encoder_discard(enc);
                }
            }
            if (!ok) reply_err(out, nmb_last_error());
            else reply_ok(out);
            break;
        }
        case OP_ENC_END: {
            uint64_t eid = r.get_u64();
            NmbEncoder *enc = nullptr;
            {
                std::lock_guard<std::mutex> lk(ch->mu);
                auto it = ch->encoders.find(eid);
                if (it != ch->encoders.end()) { enc = it->second; ch->encoders.erase(it); }
            }
            if (!enc || !nmb_encoder_end(enc)) reply_err(out, nmb_last_error());
            else reply_ok(out);
            break;
        }
        case OP_ENC_DISCARD: {
            uint64_t eid = r.get_u64();
            NmbEncoder *enc = nullptr;
            {
                std::lock_guard<std::mutex> lk(ch->mu);
                auto it = ch->encoders.find(eid);
                if (it != ch->encoders.end()) { enc = it->second; ch->encoders.erase(it); }
            }
            if (enc) nmb_encoder_discard(enc);
            reply_ok(out);
            break;
        }
        case OP_ENC_TILE_SIZE: {
            uint64_t eid = r.get_u64();
            NmbEncoder *enc = nullptr;
            {
                std::lock_guard<std::mutex> lk(ch->mu);
                auto it = ch->encoders.find(eid);
                if (it != ch->encoders.end()) enc = it->second;
            }
            uint64_t sz[2] = {0, 0};
            if (!enc || !nmb_render_tile_size(enc, sz)) reply_err(out, nmb_last_error());
            else {
                reply_ok(out); out.put_u64(sz[0]); out.put_u64(sz[1]);
            }
            break;
        }

        default:
            reply_err(out, "unknown RPC opcode");
            break;
    }
}

static void worker_loop(int worker_id) {
    // 16 MiB virtual capacity (demand-zero pages: only touched pages use RAM!)
    const size_t kBufCap = 16ULL * 1024 * 1024;
    void *req_mem = mmap(nullptr, kBufCap, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (req_mem == MAP_FAILED) {
        gpud_log("[gpud] worker %d mmap failed\n", worker_id);
        return;
    }
    Writer reply;
    reply.reserve(4096);

    for (;;) {
        uint64_t in[2] = { (uint64_t)(uintptr_t)req_mem, (uint64_t)kBufCap };
        uint64_t out[3] = {};
        uint32_t out_cnt = 3;
        kern_return_t kr = IOConnectCallScalarMethod(gConn, SEL_DAEMON_WAIT_REQ, in, 2, out, &out_cnt);
        if (kr != KERN_SUCCESS) {
            if (kr == kIOReturnAborted) break;
            gpud_log("[gpud] worker %d WAIT_REQ err=0x%x\n", worker_id, kr);
            usleep(10000);
            continue;
        }
        uint64_t call_id = out[0];
        uint32_t client_id = (uint32_t)out[1];
        uint64_t req_len = out[2];

        reply.clear();
        handle_rpc(client_id, static_cast<const uint8_t *>(req_mem), (size_t)req_len, reply);

        // Synthetic client close does not have a blocked caller waiting on SEND_REPLY
        if (req_len == sizeof(uint32_t)) {
            uint32_t first_op = 0;
            memcpy(&first_op, req_mem, sizeof(first_op));
            if (first_op == OP_CLIENT_CLOSED) continue;
        }

        uint64_t rep_in[4] = {
            call_id,
            (uint64_t)(uintptr_t)reply.buf.data(),
            (uint64_t)reply.buf.size(),
            (uint64_t)KERN_SUCCESS
        };
        (void)IOConnectCallScalarMethod(gConn, SEL_DAEMON_SEND_REPLY, rep_in, 4, nullptr, nullptr);
    }

    munmap(req_mem, kBufCap);
}

}  // namespace

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    signal(SIGPIPE, SIG_IGN);
    (void)setpriority(PRIO_PROCESS, 0, -20);
    setenv("METAL2VULKAN_COMPILE_ON_ASKING_THREAD", "1", 1);
    gConsoleFd = open("/dev/console", O_WRONLY | O_NOCTTY);

    gpud_log("[gpud] tb375fc-gpu-daemon starting pid=%d uid=%d\n", getpid(), getuid());

    // Ensure persistent shader + JIT cache directories exist on APFS Data volume (/private/var/db)
    mkdir("/private/var/db", 0755);
    const char *jit_cache_dir = "/private/var/db/tb375fc-jit-cache";
    mkdir(jit_cache_dir, 0700);
    setenv("TB375FC_JIT_DISK_CACHE", "1", 1);
    setenv("TB375FC_JIT_DISK_CACHE_DIR", jit_cache_dir, 1);
    setenv("TB375FC_JIT_DISK_CACHE_MAX_MB", "256", 1);

    // Count preexisting JIT cache entries + persist boot counter across reboots
    {
        unsigned jit_files = 0;
        uint64_t jit_bytes = 0;
        if (DIR *d = opendir(jit_cache_dir)) {
            while (struct dirent *de = readdir(d)) {
                if (de->d_name[0] == '.') continue;
                std::string fp = std::string(jit_cache_dir) + "/" + de->d_name;
                struct stat st;
                if (lstat(fp.c_str(), &st) == 0 && S_ISREG(st.st_mode)) {
                    ++jit_files;
                    jit_bytes += (uint64_t)st.st_size;
                }
            }
            closedir(d);
        }
        int prev_cnt = 0;
        if (FILE *f = fopen("/private/var/db/tb375fc-persist-bootcount.txt", "r")) {
            (void)fscanf(f, "%d", &prev_cnt);
            fclose(f);
        }
        int next_cnt = prev_cnt + 1;
        if (FILE *f = fopen("/private/var/db/tb375fc-persist-bootcount.txt", "w")) {
            fprintf(f, "%d\n", next_cnt);
            fflush(f);
            fsync(fileno(f));
            fclose(f);
        }
        gpud_log("[gpud] persist_bootcount=%d->%d jit_cache_preexisting=%u files (%llu KiB) dir=%s\n",
                 prev_cnt, next_cnt, jit_files, (unsigned long long)(jit_bytes >> 10), jit_cache_dir);
    }

    const char *cache_dir = "/private/var/db/tb375fc-m2v-cache";
    mkdir(cache_dir, 0755);

    // 1. Open Vulkan ICD + metal2vulkan v2 device (prefer real Mali-G615 panvk if installed)
    {
        auto crash_handler = [](int sig, siginfo_t *si, void *uctx) {
            ucontext_t *uc = (ucontext_t *)uctx;
            uint64_t pc = uc ? uc->uc_mcontext->__ss.__pc : 0;
            uint64_t lr = uc ? uc->uc_mcontext->__ss.__lr : 0;
            Dl_info di{};
            dladdr((void *)pc, &di);
            uint64_t off = di.dli_fbase ? (pc - (uint64_t)di.dli_fbase) : 0;
            char buf[512];
            int n = snprintf(buf, sizeof(buf),
                "[gpud] CRASH sig=%d code=%d addr=%p pc=0x%llx lr=0x%llx mod=%s+0x%llx sym=%s\n",
                sig, si ? si->si_code : 0, si ? si->si_addr : nullptr,
                (unsigned long long)pc, (unsigned long long)lr,
                di.dli_fname ? di.dli_fname : "?", (unsigned long long)off,
                di.dli_sname ? di.dli_sname : "?");
            int con = open("/dev/console", O_WRONLY | O_NOCTTY);
            if (con >= 0) {
                if (n > 0) (void)write(con, buf, (size_t)n);
                void *bt[12];
                int nb = backtrace(bt, 12);
                for (int i = 0; i < nb; ++i) {
                    Dl_info bi{};
                    dladdr(bt[i], &bi);
                    uint64_t boff = bi.dli_fbase ? ((uint64_t)bt[i] - (uint64_t)bi.dli_fbase) : 0;
                    int m = snprintf(buf, sizeof(buf), "[gpud]   #%d %p %s+0x%llx (%s)\n",
                        i, bt[i], bi.dli_fname ? bi.dli_fname : "?",
                        (unsigned long long)boff, bi.dli_sname ? bi.dli_sname : "?");
                    if (m > 0) (void)write(con, buf, (size_t)m);
                }
                close(con);
            }
            _exit(128 + sig);
        };
        struct sigaction sa{};
        sa.sa_sigaction = crash_handler;
        sa.sa_flags = SA_SIGINFO;
        sigaction(SIGILL, &sa, nullptr);
        sigaction(SIGSEGV, &sa, nullptr);
        sigaction(SIGBUS, &sa, nullptr);
        sigaction(SIGABRT, &sa, nullptr);
    }
    const char *loader_path = "/usr/lib/libvk_swiftshader.dylib";
    if (access("/usr/lib/libvulkan_panfrost.dylib", R_OK) == 0) {
        loader_path = "/usr/lib/libvulkan_panfrost.dylib";
        setenv("PANVK_DEBUG", "noafbc,linear", 1);
        // Wait up to 8s for Tb375fcMali IOService to finish GPUEB + CSF MCU bring-up
        for (int w = 0; w < 80; ++w) {
            io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault, IOServiceMatching("Tb375fcMali"));
            if (svc != IO_OBJECT_NULL) {
                IOObjectRelease(svc);
                gpud_log("[gpud] Tb375fcMali IOService FOUND after %dms! Opening panvk...\n", w * 100);
                break;
            }
            if (w == 79) gpud_log("[gpud] WARN: Tb375fcMali IOService not found after 8000ms\n");
            usleep(100000);
        }
        run_mali_vk_selftest(loader_path, gpud_log);
    }
    gDevice = nmb_device_open(loader_path, cache_dir);
    if (!gDevice) {
        const char *err = nmb_last_error();
        gpud_log("[gpud] FATAL: nmb_device_open(%s) failed:\n", loader_path);
        // Log full error in 300-byte chunks so nothing gets truncated on CDC
        size_t elen = err ? strlen(err) : 0;
        for (size_t off = 0; off < elen; off += 300) {
            gpud_log("[gpud]   err[%zu]: %.300s\n", off, err + off);
        }
        return 1;
    }
    char dev_name[128] = {};
    nmb_device_name(gDevice, dev_name, sizeof(dev_name));
    gpud_log("[gpud] SwiftShader device OPEN (WS): '%s' maxBuf=%llu MiB\n",
             dev_name, (unsigned long long)(nmb_device_max_buffer_length(gDevice) >> 20));

    // On real Mali-G615 hardware, share the single hardware Vulkan device (saves ~100 MiB RAM & avoids extra AS/CSG).
    // Only open a separate BG device on CPU SwiftShader.
    if (!strstr(dev_name, "Mali")) {
        const char *bg_cache_dir = "/private/var/db/tb375fc-m2v-cache-bg";
        mkdir(bg_cache_dir, 0755);
        gBgDevice = nmb_device_open(loader_path, bg_cache_dir);
        if (gBgDevice) {
            gpud_log("[gpud] SwiftShader device OPEN (BG): isolated VkQueue for client > 1\n");
        } else {
            gpud_log("[gpud] WARN: BG SwiftShader device failed (%s), falling back to shared\n", nmb_last_error());
        }
    } else {
        gRealMali = true;
        gpud_log("[gpud] Real Mali-G615 detected: using single shared HW Vulkan device (heavy blur PSOs enabled)\n");
    }

    // Quick startup self-test for VK_EXT_external_memory_host zero-copy import
    {
        void *page = nullptr;
        if (posix_memalign(&page, 4096, 4096) == 0 && page) {
            memset(page, 0x5A, 4096);
            NmbBuffer *tb = nmb_buffer_new_no_copy(gDevice, page, 4096, 0);
            if (tb) {
                gpud_log("[gpud] VK_EXT_external_memory_host self-test: PASS!\n");
                nmb_buffer_release(tb);
            } else {
                gpud_log("[gpud] VK_EXT_external_memory_host self-test: FAIL (%s)\n", nmb_last_error());
            }
            free(page);
        }
    }

    // 1-second real Mali-G615 hardware pixel render-clear self-test (verifies GPU actually writes pixels!)
    {
        void *tmem = nullptr;
        size_t tsz = 64 * 64 * 4;
        if (posix_memalign(&tmem, 4096, tsz) == 0 && tmem) {
            memset(tmem, 0, tsz);
            NmbBuffer *b = nmb_buffer_new_no_copy(gDevice, tmem, tsz, 0);
            NmbTextureDescriptor td{};
            td.type = 2; /* 2D */
            td.format = 80; /* BGRA8Unorm */
            td.width = 64; td.height = 64; td.depth = 1;
            td.levels = 1; td.samples = 1; td.array_length = 1;
            td.usage = 0x5; /* ShaderRead | RenderTarget */
            NmbTexture *tx = b ? nmb_texture_new_on_buffer(b, &td, 0, 64 * 4, true) : nullptr;
            NmbQueue *q = tx ? nmb_queue_new(gDevice) : nullptr;
            NmbCommandBuffer *cb = q ? nmb_command_buffer_new(q) : nullptr;
            if (cb) {
                NmbRenderPassDescriptor rpd{};
                rpd.colors[0].texture = tx;
                rpd.colors[0].load = 2;  /* MTLLoadActionClear */
                rpd.colors[0].store = 1; /* MTLStoreActionStore */
                rpd.colors[0].clear[0] = 1.0; rpd.colors[0].clear[1] = 0.0;
                rpd.colors[0].clear[2] = 0.0; rpd.colors[0].clear[3] = 1.0;
                rpd.width = 64; rpd.height = 64; rpd.array_length = 1; rpd.samples = 1;
                NmbEncoder *enc = nmb_render_encoder_new(cb, &rpd);
                if (enc) {
                    nmb_encoder_end(enc);
                    bool cok = nmb_command_buffer_commit(cb);
                    int wrc = cok ? nmb_command_buffer_wait(cb, 3000) : -1;
                    arm64_dcache_clean_inval(tmem, tsz);
                    uint32_t px0 = ((const uint32_t *)tmem)[0];
                    uint32_t pxMid = ((const uint32_t *)tmem)[32 * 64 + 32];
                    gpud_log("[gpud] MALI PIXEL SELF-TEST: commit=%d wait=%d px0=0x%08x pxMid=0x%08x (want 0xffff0000) -> %s\n",
                             (int)cok, wrc, px0, pxMid, (px0 == 0xffff0000u && pxMid == 0xffff0000u) ? "PASS!" : "FAIL!");
                } else {
                    gpud_log("[gpud] MALI PIXEL SELF-TEST: encoder_new FAIL (%s)\n", nmb_last_error());
                }
            } else {
                gpud_log("[gpud] MALI PIXEL SELF-TEST: setup FAIL (%s)\n", nmb_last_error());
            }
            if (cb) nmb_command_buffer_release(cb);
            if (q)  nmb_queue_release(q);
            if (tx) nmb_texture_release(tx);
            if (b)  nmb_buffer_release(b);
            free(tmem);
        }
    }

    // 2. Connect to kernel IOAccelSwiftShader service (retry up to 10s if IOKit is still publishing)
    io_service_t service = IO_OBJECT_NULL;
    for (int tries = 0; tries < 100 && service == IO_OBJECT_NULL; ++tries) {
        service = IOServiceGetMatchingService(kIOMainPortDefault, IOServiceMatching(SWIFTSHADER_SERVICE_CLASS));
        if (service == IO_OBJECT_NULL) usleep(100000);
    }
    if (service == IO_OBJECT_NULL) {
        gpud_log("[gpud] FATAL: IOServiceMatching(%s) not found!\n", SWIFTSHADER_SERVICE_CLASS);
        return 2;
    }

    kern_return_t kr = IOServiceOpen(service, mach_task_self(), SWIFTSHADER_DAEMON_OPEN_TYPE, &gConn);
    IOObjectRelease(service);
    if (kr != KERN_SUCCESS || gConn == MACH_PORT_NULL) {
        gpud_log("[gpud] FATAL: IOServiceOpen(DAEMON) failed kr=0x%x\n", kr);
        return 3;
    }

    // Restore /Library/GPUBundles -> real APFS GPUBundles (with SF_RESTRICTED + KernelExtensionManagement)
    // after rc.temporaryDataVolume mounted tmpfs over /System/Volumes/Data.
    {
        (void)mkdir("/Library", 0755);
        (void)mkdir("/Library/GPUBundles", 0755);
        uint64_t lout[1] = { 0 };
        uint32_t lout_cnt = 1;
        kern_return_t lkr = IOConnectCallScalarMethod(gConn, SEL_DAEMON_LINK_GPUBUNDLES,
                                                      nullptr, 0, lout, &lout_cnt);
        struct statfs sfs{};
        const char *bpath = "/Library/GPUBundles/NativeAdrenoMetal.bundle";
        int sret = statfs(bpath, &sfs);
        struct stat st{};
        int bstat = stat("/Library/GPUBundles/NativeAdrenoMetal.bundle/Contents/MacOS/NativeAdrenoMetal", &st);
        using rootless_fn_t = int (*)(const char *, const char *);
        auto rootless_fn = reinterpret_cast<rootless_fn_t>(dlsym(RTLD_DEFAULT, "rootless_check_trusted_class"));
        int rtrust = rootless_fn ? rootless_fn(bpath, "KernelExtensionManagement") : -999;
        gpud_log("[gpud] LINK_GPUBUNDLES kr=0x%x err=%llu statfs=%d(%s) bin_stat=%d(flags=0x%x) rootless_trust=%d\n",
                 lkr, (unsigned long long)lout[0], sret, sret == 0 ? sfs.f_fstypename : "?",
                 bstat, bstat == 0 ? st.st_flags : 0, rtrust);
    }

    // Data volume is now provisioned offline via make-macos-data-seed.py (macos-data-seed.dbr)
    // with full Templates/Data ownership, --skip-setup, local admin user, and --auto-login.
    // Write readiness sentinel file so BootCacheControl can return immediately.
    if (int rfd = open("/private/var/tmp/tb375fc-gpud.ready", O_CREAT | O_WRONLY | O_TRUNC, 0644); rfd >= 0) {
        char msg[32];
        int len = snprintf(msg, sizeof(msg), "%d\n", getpid());
        (void)write(rfd, msg, (size_t)len);
        close(rfd);
    }

    // Unlink our Data-volume overlay /private/var/ngpu/sbin/BootCacheControl so
    // launchd's post-early-boot `cache-tag` resolves to the sealed OS-volume /usr/sbin/BootCacheControl
    // instead of a Data-volume binary (which deadlocks AppleSystemPolicy <-> syspolicyd!).
    {
        int urc = unlink("/private/var/ngpu/sbin/BootCacheControl");
        gpud_log("[gpud] unlinked Data-volume BootCacheControl overlay (rc=%d, errno=%d) for cache-tag\n",
                 urc, urc == 0 ? 0 : errno);
    }

    // Raise kern.maxvnodes from default 1024 to 32768 to eliminate "vnode: table is full" thrashing.
    {
        int maxvnodes = 32768;
        int src = sysctlbyname("kern.maxvnodes", nullptr, nullptr, &maxvnodes, sizeof(maxvnodes));
        gpud_log("[gpud] sysctl kern.maxvnodes=32768 rc=%d errno=%d\n", src, src == 0 ? 0 : errno);
    }

    // Watchdog for stuck OS-volume `BootCacheControl tag` blocking launchd's cache-tag boot task.
    {
        pthread_t bc_th{};
        pthread_create(&bc_th, nullptr, [](void *) -> void * {
            for (int round = 0; round < 3; ++round) {
                sleep(12);
                pid_t pids[512];
                int npids = proc_listallpids(pids, (int)sizeof(pids));
                if (npids > 512) npids = 512;
                for (int i = 0; i < npids; ++i) {
                    char name[256] = {};
                    if (proc_name(pids[i], name, sizeof(name)) > 0 && strcmp(name, "BootCacheControl") == 0) {
                        gpud_log("[gpud] killing stuck BootCacheControl pid=%d (SIGKILL) to unblock launchd\n", (int)pids[i]);
                        (void)kill(pids[i], SIGKILL);
                    }
                }
            }
            return nullptr;
        }, nullptr);
        pthread_detach(bc_th);
    }

    gpud_log("[gpud] CONNECTED to IOAccelSwiftShader! Launching 8 RPC worker threads (8 MiB stack each)...\n");

    // Interactive USB CDC root shell over /dev/console (kernel tb375fc_km_try_cinput delivers host USB CDC TX bytes here)
    {
        pthread_t sh_th{};
        pthread_create(&sh_th, nullptr, [](void *) -> void * {
            auto run_cmd = [](const char *tag, char *const argv[], int max_lines, int timeout_ms) {
                int pfd[2];
                if (pipe(pfd) != 0) return;
                posix_spawn_file_actions_t fa;
                posix_spawn_file_actions_init(&fa);
                posix_spawn_file_actions_addclose(&fa, pfd[0]);
                posix_spawn_file_actions_adddup2(&fa, pfd[1], STDOUT_FILENO);
                posix_spawn_file_actions_adddup2(&fa, pfd[1], STDERR_FILENO);
                posix_spawn_file_actions_addclose(&fa, pfd[1]);
                pid_t child = 0;
                extern char **environ;
                int rc = posix_spawn(&child, argv[0], &fa, nullptr, argv, environ);
                posix_spawn_file_actions_destroy(&fa);
                close(pfd[1]);
                if (rc != 0) {
                    gpud_log("[cdc-sh:%s] posix_spawn err=%d\n", tag, rc);
                    close(pfd[0]);
                    return;
                }
                fcntl(pfd[0], F_SETFL, O_NONBLOCK);
                uint64_t deadline = monotonic_ms() + (uint64_t)timeout_ms;
                std::string acc;
                int lines = 0;
                char buf[1024];
                while (monotonic_ms() < deadline && lines < max_lines) {
                    struct pollfd pf = { pfd[0], POLLIN, 0 };
                    int pr = poll(&pf, 1, 200);
                    if (pr > 0) {
                        ssize_t n = read(pfd[0], buf, sizeof(buf));
                        if (n <= 0) break;
                        acc.append(buf, (size_t)n);
                        size_t nl;
                        while (lines < max_lines && (nl = acc.find('\n')) != std::string::npos) {
                            std::string line = acc.substr(0, nl);
                            acc.erase(0, nl + 1);
                            gpud_log("[cdc-sh:%s] %s\n", tag, line.c_str());
                            lines++;
                        }
                    } else if (pr < 0 && errno != EINTR) {
                        break;
                    }
                }
                if (!acc.empty() && lines < max_lines) {
                    gpud_log("[cdc-sh:%s] %s\n", tag, acc.c_str());
                }
                close(pfd[0]);
                kill(child, SIGKILL);
                int st = 0;
                (void)waitpid(child, &st, 0);
                gpud_log("[cdc-sh] --- END %s (%d lines) ---\n", tag, lines);
            };

            int cfd = open("/dev/console", O_RDONLY | O_NOCTTY);
            if (cfd < 0) {
                gpud_log("[cdc-sh] open(/dev/console) failed errno=%d\n", errno);
                return nullptr;
            }
            gpud_log("[cdc-sh] READY (cmds: ps | ls <dir> | cat <file> | kill <pid> [sig] | sample <pid> | run <bin> [args...] | sh <cmd>)\n");
            std::string line;
            char ch = 0;
            while (true) {
                ssize_t n = read(cfd, &ch, 1);
                if (n <= 0) {
                    if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
                        close(cfd);
                        usleep(200000);
                        cfd = open("/dev/console", O_RDONLY | O_NOCTTY);
                        if (cfd < 0) { usleep(500000); continue; }
                    } else {
                        usleep(50000);
                    }
                    continue;
                }
                if (ch == '\r' || ch == '\n') {
                    while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) line.pop_back();
                    size_t sp = 0;
                    while (sp < line.size() && (line[sp] == ' ' || line[sp] == '\t')) sp++;
                    if (sp) line.erase(0, sp);
                    if (line.empty()) continue;
                    gpud_log("[cdc-sh]> %s\n", line.c_str());

                    if (line == "ps") {
                        pid_t pids[512];
                        int npids = proc_listallpids(pids, (int)sizeof(pids));
                        if (npids > 512) npids = 512;
                        gpud_log("[cdc-sh:ps] count=%d\n", npids);
                        for (int i = 0; i < npids; ++i) {
                            char name[256] = {};
                            proc_name(pids[i], name, sizeof(name));
                            gpud_log("[cdc-sh:ps] %d %s\n", (int)pids[i], name);
                        }
                        gpud_log("[cdc-sh] --- END ps ---\n");
                    } else if (line.rfind("ls ", 0) == 0) {
                        std::string p = line.substr(3);
                        DIR *d = opendir(p.c_str());
                        if (!d) {
                            gpud_log("[cdc-sh:ls] opendir(%s) errno=%d\n", p.c_str(), errno);
                        } else {
                            while (struct dirent *de = readdir(d)) {
                                std::string full = p + "/" + de->d_name;
                                struct stat st{};
                                if (lstat(full.c_str(), &st) == 0) {
                                    char lbuf[256] = {};
                                    if (S_ISLNK(st.st_mode)) {
                                        ssize_t ln = readlink(full.c_str(), lbuf, sizeof(lbuf) - 1);
                                        if (ln > 0) lbuf[ln] = '\0';
                                        gpud_log("[cdc-sh:ls] %04o %u:%u %lld %s -> %s\n",
                                                 st.st_mode & 07777, st.st_uid, st.st_gid, (long long)st.st_size, de->d_name, lbuf);
                                    } else {
                                        gpud_log("[cdc-sh:ls] %04o %u:%u %lld %s%s\n",
                                                 st.st_mode & 07777, st.st_uid, st.st_gid, (long long)st.st_size, de->d_name,
                                                 S_ISDIR(st.st_mode) ? "/" : "");
                                    }
                                }
                            }
                            closedir(d);
                        }
                        gpud_log("[cdc-sh] --- END ls ---\n");
                    } else if (line.rfind("cat ", 0) == 0) {
                        std::string p = line.substr(4);
                        int fd = open(p.c_str(), O_RDONLY);
                        if (fd < 0) {
                            gpud_log("[cdc-sh:cat] open(%s) errno=%d\n", p.c_str(), errno);
                        } else {
                            char buf[4096];
                            ssize_t rn = read(fd, buf, sizeof(buf) - 1);
                            close(fd);
                            if (rn > 0) {
                                buf[rn] = '\0';
                                gpud_log("[cdc-sh:cat] %s\n", buf);
                            }
                        }
                        gpud_log("[cdc-sh] --- END cat ---\n");
                    } else if (line.rfind("kill ", 0) == 0) {
                        int pid = 0, sig = SIGTERM;
                        sscanf(line.c_str() + 5, "%d %d", &pid, &sig);
                        int krc = (pid > 1) ? kill((pid_t)pid, sig) : -1;
                        gpud_log("[cdc-sh:kill] kill(%d, %d) rc=%d errno=%d\n", pid, sig, krc, krc == 0 ? 0 : errno);
                    } else if (line.rfind("sample ", 0) == 0) {
                        std::string p = line.substr(7);
                        char *argv[] = { (char *)"/usr/bin/sample", (char *)p.c_str(), (char *)"1", (char *)"-mayDie", nullptr };
                        run_cmd("sample", argv, 100, 15000);
                    } else if (line.rfind("run ", 0) == 0) {
                        std::vector<std::string> toks;
                        size_t i = 4;
                        while (i < line.size()) {
                            while (i < line.size() && line[i] == ' ') i++;
                            if (i >= line.size()) break;
                            size_t j = i;
                            while (j < line.size() && line[j] != ' ') j++;
                            toks.push_back(line.substr(i, j - i));
                            i = j;
                        }
                        if (!toks.empty()) {
                            std::vector<char *> argv;
                            for (auto &t : toks) argv.push_back((char *)t.c_str());
                            argv.push_back(nullptr);
                            run_cmd("run", argv.data(), 120, 15000);
                        }
                    } else if (line == "selftest") {
                        run_mali_vk_selftest("/usr/lib/libvulkan_panfrost.dylib", gpud_log);
                        gpud_log("[cdc-sh] --- END selftest ---\n");
                    } else if (line == "fbshot") {
                        bool wrote_ppm = false;
                        uint64_t sw = 0, sh = 0, sfmt = 0;
                        {
                            std::lock_guard<std::mutex> lk(gScanoutMu);
                            wrote_ppm = dump_scanout_ppm_locked("/tmp/fb.ppm", &sw, &sh, &sfmt);
                        }
                        if (!wrote_ppm) {
                            gFbshotPending.store(true, std::memory_order_relaxed);
                            for (int wi = 0; wi < 40 && gFbshotPending.load(std::memory_order_relaxed); ++wi) {
                                usleep(100000);
                            }
                            if (!gFbshotPending.load(std::memory_order_relaxed) || gFbshotReady.load(std::memory_order_relaxed)) {
                                wrote_ppm = (access("/tmp/fb.ppm", R_OK) == 0);
                            }
                        }
                        if (!wrote_ppm) {
                            gpud_log("[cdc-sh:fbshot] no live scanout available\n");
                        } else {
                            char *sargv[] = { (char *)"/usr/bin/sips", (char *)"-s", (char *)"format", (char *)"jpeg",
                                              (char *)"-s", (char *)"formatOptions", (char *)"55",
                                              (char *)"/tmp/fb.ppm", (char *)"-o", (char *)"/tmp/fb.jpg", nullptr };
                            run_cmd("fbshot-sips", sargv, 10, 10000);
                            gpud_log("[cdc-sh:fbshot] OK (from %llux%llu fmt=%llu)\n",
                                     (unsigned long long)sw, (unsigned long long)sh, (unsigned long long)sfmt);
                            char *bargv[] = { (char *)"/bin/sh", (char *)"-c",
                                              (char *)"echo B64S; base64 -b 100 -i /tmp/fb.jpg | awk '{print \"B64:\" $0}'; echo B64E", nullptr };
                            run_cmd("fbshot-b64", bargv, 400, 20000);
                        }
                        gpud_log("[cdc-sh] --- END fbshot ---\n");
                    } else if (line.rfind("sh ", 0) == 0) {
                        std::string cmd = line.substr(3);
                        char *argv[] = { (char *)"/bin/sh", (char *)"-c", (char *)cmd.c_str(), nullptr };
                        run_cmd("sh", argv, 120, 15000);
                    } else {
                        gpud_log("[cdc-sh] unknown cmd: %s\n", line.c_str());
                    }
                    line.clear();
                } else if (ch >= 0x20 && ch < 0x7f && line.size() < 512) {
                    line.push_back(ch);
                }
            }
            return nullptr;
        }, nullptr);
        pthread_detach(sh_th);
    }

    // Run early-boot sampling only when explicitly enabled for diagnostics.
    const char *boot_sample = getenv("TB375FC_GPU_BOOT_SAMPLE");
    if (boot_sample && strcmp(boot_sample, "1") == 0) {
        pthread_t diag_th{};
        pthread_create(&diag_th, nullptr, [](void *) -> void * {
            sleep(75);
            if (gRpcCount.load(std::memory_order_relaxed) != 0) return nullptr;
            gpud_log("[gpud-diag] 0 GPU RPCs after 75s; scanning processes via libproc...\n");

            pid_t ws_pid = 0, od_pid = 0, lw_pid = 0;
            pid_t pids[512];
            int npids = proc_listallpids(pids, (int)sizeof(pids));
            if (npids > 512) npids = 512;
            gpud_log("[gpud-diag] total pids=%d\n", npids);
            for (int i = 0; i < npids; ++i) {
                char name[256] = {};
                if (proc_name(pids[i], name, sizeof(name)) > 0) {
                    if (strcmp(name, "WindowServer") == 0) ws_pid = pids[i];
                    else if (strcmp(name, "opendirectoryd") == 0) od_pid = pids[i];
                    else if (strcmp(name, "loginwindow") == 0) lw_pid = pids[i];
                    if (strstr(name, "WindowServer") || strstr(name, "loginwindow") ||
                        strstr(name, "opendirectory") || strstr(name, "Setup") ||
                        strstr(name, "display") || strstr(name, "Dock") || strstr(name, "Finder")) {
                        gpud_log("[gpud-diag] proc pid=%d name=%s\n", (int)pids[i], name);
                    }
                }
            }

            auto run_bounded = [](const char *tag, char *const argv[], int max_lines, int timeout_ms) {
                int pfd[2];
                if (pipe(pfd) != 0) return;
                posix_spawn_file_actions_t fa;
                posix_spawn_file_actions_init(&fa);
                posix_spawn_file_actions_addclose(&fa, pfd[0]);
                posix_spawn_file_actions_adddup2(&fa, pfd[1], STDOUT_FILENO);
                posix_spawn_file_actions_adddup2(&fa, pfd[1], STDERR_FILENO);
                posix_spawn_file_actions_addclose(&fa, pfd[1]);
                pid_t child = 0;
                extern char **environ;
                int rc = posix_spawn(&child, argv[0], &fa, nullptr, argv, environ);
                posix_spawn_file_actions_destroy(&fa);
                close(pfd[1]);
                if (rc != 0) {
                    gpud_log("[gpud-diag] %s posix_spawn err=%d\n", tag, rc);
                    close(pfd[0]);
                    return;
                }
                fcntl(pfd[0], F_SETFL, O_NONBLOCK);
                uint64_t deadline = monotonic_ms() + (uint64_t)timeout_ms;
                std::string acc;
                int lines = 0;
                char buf[1024];
                while (monotonic_ms() < deadline && lines < max_lines) {
                    struct pollfd pf = { pfd[0], POLLIN, 0 };
                    int pr = poll(&pf, 1, 250);
                    if (pr > 0) {
                        ssize_t n = read(pfd[0], buf, sizeof(buf));
                        if (n <= 0) break;
                        acc.append(buf, (size_t)n);
                        size_t nl;
                        while (lines < max_lines && (nl = acc.find('\n')) != std::string::npos) {
                            std::string line = acc.substr(0, nl);
                            acc.erase(0, nl + 1);
                            gpud_log("[gpud-diag:%s] %s\n", tag, line.c_str());
                            lines++;
                        }
                    } else if (pr < 0 && errno != EINTR) {
                        break;
                    }
                }
                close(pfd[0]);
                kill(child, SIGKILL);
                int st = 0;
                (void)waitpid(child, &st, 0);
                gpud_log("[gpud-diag] %s finished (%d lines)\n", tag, lines);
            };

            if (ws_pid > 0) {
                char pbuf[16];
                snprintf(pbuf, sizeof(pbuf), "%d", (int)ws_pid);
                char *argv[] = { (char *)"/usr/bin/sample", pbuf, (char *)"1", (char *)"-mayDie", nullptr };
                run_bounded("ws-sample", argv, 80, 15000);
            }
            if (od_pid > 0) {
                char pbuf[16];
                snprintf(pbuf, sizeof(pbuf), "%d", (int)od_pid);
                char *argv[] = { (char *)"/usr/bin/sample", pbuf, (char *)"1", (char *)"-mayDie", nullptr };
                run_bounded("od-sample", argv, 80, 15000);
            }
            if (lw_pid > 0) {
                char pbuf[16];
                snprintf(pbuf, sizeof(pbuf), "%d", (int)lw_pid);
                char *argv[] = { (char *)"/usr/bin/sample", pbuf, (char *)"1", (char *)"-mayDie", nullptr };
                run_bounded("lw-sample", argv, 60, 15000);
            }
            gpud_log("[gpud-diag] DONE (ws=%d od=%d lw=%d)\n", (int)ws_pid, (int)od_pid, (int)lw_pid);
            return nullptr;
        }, nullptr);
        pthread_detach(diag_th);
    }

    // Periodic heartbeat so we can immediately see whether WindowServer is compiling, waiting, or idle
    {
        pthread_t hb_th{};
        pthread_create(&hb_th, nullptr, [](void *) -> void * {
            using CreateAssertionFn = int (*)(CFStringRef, uint32_t, CFStringRef, uint32_t *);
            using DeclareUserActFn = int (*)(CFStringRef, uint32_t, uint32_t *);
            auto create_assertion = (CreateAssertionFn)dlsym(RTLD_DEFAULT, "IOPMAssertionCreateWithName");
            auto declare_user_act = (DeclareUserActFn)dlsym(RTLD_DEFAULT, "IOPMAssertionDeclareUserActivity");
            uint32_t disp_assert_id = 0;
            uint32_t user_act_id = 0;
            bool disp_assert_ok = false;
            bool disp_assert_err_logged = false;
            for (;;) {
                sleep(15);
                uint64_t rpcs = gRpcCount.load(std::memory_order_relaxed);
                if (rpcs == 0) continue;
                if (declare_user_act) {
                    (void)declare_user_act(CFSTR("TB375FC Display KeepAlive"), 0, &user_act_id);
                }
                if (!disp_assert_ok) {
                    if (!create_assertion) {
                        if (!disp_assert_err_logged) {
                            gpud_log("[gpud] IOPMAssertionCreateWithName dlsym=NULL\n");
                            disp_assert_err_logged = true;
                        }
                    } else {
                        int rc = create_assertion(CFSTR("PreventUserIdleDisplaySleep"), 255, CFSTR("TB375FC Display"), &disp_assert_id);
                        if (rc == 0) {
                            disp_assert_ok = true;
                            gpud_log("[gpud] PreventUserIdleDisplaySleep assertion active (id=%u)\n", disp_assert_id);
                        } else if (!disp_assert_err_logged) {
                            gpud_log("[gpud] PreventUserIdleDisplaySleep rc=0x%x\n", rc);
                            disp_assert_err_logged = true;
                        }
                    }
                }
                uint64_t now = monotonic_ms();
                uint64_t last_ms = gC1LastOpMs.load(std::memory_order_relaxed);
                uint64_t ago_s = (last_ms && now >= last_ms) ? (now - last_ms) / 1000ULL : 0;
                gpud_log("[gpud] HB rpc=%llu c1_rpc=%llu c1_last_op=%u(%llus ago) active=%d in_pso=%d rpso=%llu cpso=%llu commits=%llu c1_commits=%llu\n",
                         (unsigned long long)rpcs,
                         (unsigned long long)gC1Rpcs.load(std::memory_order_relaxed),
                         gC1LastOp.load(std::memory_order_relaxed),
                         (unsigned long long)ago_s,
                         gActiveWorkers.load(std::memory_order_relaxed),
                         gInPso.load(std::memory_order_relaxed),
                         (unsigned long long)gRenderPsoCount.load(std::memory_order_relaxed),
                         (unsigned long long)gComputePsoCount.load(std::memory_order_relaxed),
                         (unsigned long long)gCommitCount.load(std::memory_order_relaxed),
                         (unsigned long long)gC1Commits.load(std::memory_order_relaxed));
            }
            return nullptr;
        }, nullptr);
        pthread_detach(hb_th);
    }

    // 3. Launch 8 worker threads with 8 MiB stack (AIR->SPIR-V translator recurses deeply on large shaders)
    constexpr int kWorkers = 8;
    pthread_t threads[kWorkers] = {};
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 8u << 20);
    for (int i = 0; i < kWorkers; ++i) {
        pthread_create(&threads[i], &attr, [](void *arg) -> void * {
            worker_loop((int)(intptr_t)arg);
            return nullptr;
        }, (void *)(intptr_t)i);
    }
    pthread_attr_destroy(&attr);

    for (int i = 0; i < kWorkers; ++i) {
        pthread_join(threads[i], nullptr);
    }

    IOServiceClose(gConn);
    if (gBgDevice) nmb_device_release(gBgDevice);
    nmb_device_release(gDevice);
    return 0;
}
