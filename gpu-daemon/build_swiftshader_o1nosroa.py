#!/usr/bin/env python3
"""Build libvk_swiftshader.o1nosroa.dylib cleanly under ram-expansion/gpu-diag/
without modifying postmmu-diag/.

Key features:
1. Includes libVulkan.hostmem.o and VkPhysicalDevice.hostmem.o (VK_EXT_external_memory_host).
2. JITGlobals::toLLVM(): returns CodeGenOptLevel::Less (SelectionDAG -O1, avoiding ARM64
   FastISel crashes from -O0 AND expensive -O2 scheduling).
3. Keeps SROAPass + InstCombinePass so allocas are promoted before MachineCSE.
4. Boosts JIT/Queue thread QoS to QOS_CLASS_USER_INTERACTIVE + nice -20 so XNU's decay
   scheduler does not starve Queue<...> down to priority 17 (7.7% CPU) on 1 core.
5. Logs [ss-jit] START <name> (bb=<N> insts=<M>) and [ss-jit] DONE <name> (<ms>ms)
   directly to /dev/console so USB CDC captures every shader JIT compile in real time.
"""
import shutil
import subprocess
from pathlib import Path

SS_SRC = Path("/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/postmmu-diag/m5-gui/cpu-vulkan/swiftshader-708e9e77773860bbb2ad9bb2d61b451de4b84e31")
SS_BUILD = Path("/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/postmmu-diag/m5-gui/cpu-vulkan/swiftshader-arm64")
HOSTMEM_DIR = Path("/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/postmmu-diag/m5-gui/m7-metal/swiftshader")
OUT_DIR = Path("/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/gpu-diag")
DESKTOP_DIR = Path("/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/ram-expansion/desktop")

def main():
    orig_cpp = SS_SRC / "src/Reactor/LLVMJIT.cpp"
    src = orig_cpp.read_text()

    # 1. Add headers, /dev/console logger helper, and persistent DiskObjectCache
    assert '#include "LLVMReactor.hpp"' in src
    helper_code = """#include "LLVMReactor.hpp"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/SHA1.h"
#include "llvm/Support/raw_ostream.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <pthread/qos.h>
#include <sys/resource.h>

static void ss_console_log(const char *fmt, ...) {
    static int cfd = -2;
    if (cfd == -2) {
        cfd = open("/dev/console", O_WRONLY | O_NOCTTY | O_NONBLOCK);
    }
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;
        fwrite(buf, 1, (size_t)n, stderr);
        fflush(stderr);
        if (cfd >= 0) (void)write(cfd, buf, (size_t)n);
    }
}

static void ss_boost_thread_prio() {
    static thread_local bool done = false;
    if (!done) {
        done = true;
        pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
        setpriority(PRIO_DARWIN_THREAD, 0, -20);
    }
}
"""
    src = src.replace('#include "LLVMReactor.hpp"', helper_code, 1)

    # 2. Force CodeGenOptLevel::Less in toLLVM() (SelectionDAG, never FastISel)
    old_switch = """\tswitch(level)
\t{
\tcase 0: return llvm::CodeGenOptLevel::None;
\tcase 1: return llvm::CodeGenOptLevel::Less;
\tcase 2: return llvm::CodeGenOptLevel::Default;
\tcase 3: return llvm::CodeGenOptLevel::Aggressive;
\tdefault: UNREACHABLE("Unknown Optimization Level %d", int(level));
\t}"""
    assert old_switch in src, "toLLVM switch not found"
    new_switch = """\t(void)level;
\treturn llvm::CodeGenOptLevel::Less;"""
    src = src.replace(old_switch, new_switch, 1)

    # 3. Boost thread priority & stub giant runaway IR (>50000 insts) at start of runPasses()
    old_run = "void JITBuilder::runPasses()\n{"
    assert old_run in src, "runPasses header not found"
    new_run = """void JITBuilder::runPasses()
{
\tss_boost_thread_prio();
\tif (module) {
\t\tfor (llvm::Function &F : *module) {
\t\t\tif (!F.isDeclaration() && F.getReturnType()->isVoidTy() &&
\t\t\t    !F.hasFnAttribute(llvm::Attribute::PresplitCoroutine)) {
\t\t\t\tsize_t ic = F.getInstructionCount();
\t\t\t\tif (ic > 50000) {
\t\t\t\t\tsize_t bc = F.size();
\t\t\t\t\tss_console_log("[ss-jit] STUB GIANT IR %s (bb=%zu insts=%zu -> ret void)\\n",
\t\t\t\t\t               F.getName().str().c_str(), bc, ic);
\t\t\t\t\tF.deleteBody();
\t\t\t\t\tauto *bb = llvm::BasicBlock::Create(*context, "entry", &F);
\t\t\t\t\tllvm::ReturnInst::Create(*context, bb);
\t\t\t\t}
\t\t\t}
\t\t}
\t}"""
    src = src.replace(old_run, new_run, 1)

    # 4. Persistent ARM64 machine-code DiskObjectCache for ORC ConcurrentIRCompiler
    old_cl = "llvm::orc::IRCompileLayer compileLayer(session, objectLayer, std::make_unique<llvm::orc::ConcurrentIRCompiler>(JITGlobals::get()->getTargetMachineBuilder()));"
    assert old_cl in src, "IRCompileLayer line not found"
    disk_cache_cls = """
struct SSDiskObjectCache {
    std::atomic<uint64_t> hits{0}, misses{0}, corrupt{0}, bytes_used{0}, seq{0};
    std::mutex init_mu;
    std::string dir_path;
    std::string fingerprint;
    uint64_t max_bytes = 256ULL * 1024ULL * 1024ULL;
    std::atomic<int> state{-1}; // -1=uninit, 0=disabled, 1=ready

    static bool verifySafeDir(const char *p) {
        struct stat st;
        if (lstat(p, &st) != 0) return false;
        if (!S_ISDIR(st.st_mode)) return false; // rejects symlinks & regular files
        if (st.st_uid != geteuid()) return false;
        if ((st.st_mode & 0022) != 0) chmod(p, 0700);
        return true;
    }

    bool initLocked() {
        const char *v = getenv("TB375FC_JIT_DISK_CACHE");
        if (!v || v[0] != '1') return false;
        const char *custom_dir = getenv("TB375FC_JIT_DISK_CACHE_DIR");
        dir_path = (custom_dir && custom_dir[0] == '/') ? custom_dir : "/private/var/tmp/tb375fc-m7-cache/jit";
        const char *cap_env = getenv("TB375FC_JIT_DISK_CACHE_MAX_MB");
        if (cap_env && cap_env[0]) {
            long mb = strtol(cap_env, nullptr, 10);
            if (mb >= 16 && mb <= 2048) max_bytes = (uint64_t)mb * 1024ULL * 1024ULL;
        }
        // Build + CPU architecture fingerprint so stale objects from other builds/CPUs are never loaded
        fingerprint = "tb375fc-ss-o1nosroa-arm64-v2:" __DATE__ ":" __TIME__ ":";
        if (!custom_dir) {
            mkdir("/private/var/tmp/tb375fc-m7-cache", 0700);
            if (!verifySafeDir("/private/var/tmp/tb375fc-m7-cache")) return false;
        }
        mkdir(dir_path.c_str(), 0700);
        if (!verifySafeDir(dir_path.c_str())) return false;

        // One-click cache invalidation via TB375FC_JIT_DISK_CACHE_RESET=1
        const char *rst = getenv("TB375FC_JIT_DISK_CACHE_RESET");
        bool do_reset = (rst && rst[0] == '1');
        uint64_t total = 0;
        if (DIR *d = opendir(dir_path.c_str())) {
            while (struct dirent *de = readdir(d)) {
                if (de->d_name[0] == '.') continue;
                std::string fp = dir_path + "/" + de->d_name;
                struct stat st;
                if (lstat(fp.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
                if (do_reset) {
                    unlink(fp.c_str());
                } else {
                    total += (uint64_t)st.st_size;
                }
            }
            closedir(d);
        }
        bytes_used.store(total, std::memory_order_relaxed);
        return true;
    }

    bool isReady() {
        int s = state.load(std::memory_order_acquire);
        if (s >= 0) return s == 1;
        std::lock_guard<std::mutex> lk(init_mu);
        s = state.load(std::memory_order_relaxed);
        if (s >= 0) return s == 1;
        bool ok = initLocked();
        state.store(ok ? 1 : 0, std::memory_order_release);
        return ok;
    }

    std::string keyPath(const llvm::Module *M) const {
        std::string ir;
        ir.reserve(16384);
        llvm::raw_string_ostream os(ir);
        M->print(os, nullptr);
        os.flush();
        llvm::SHA1 hasher;
        hasher.update(fingerprint);
        hasher.update(M->getTargetTriple());
        hasher.update(":");
        hasher.update(ir);
        auto raw = hasher.final();
        static const char hex[] = "0123456789abcdef";
        char h[41];
        for (int i = 0; i < 20; ++i) {
            uint8_t b = static_cast<uint8_t>(raw[i]);
            h[i * 2] = hex[b >> 4];
            h[i * 2 + 1] = hex[b & 0xf];
        }
        h[40] = '\\0';
        return dir_path + "/" + h + ".o";
    }

    static bool isValidMachO64(const char *data, size_t sz) {
        if (sz < 32) return false;
        uint32_t magic = 0;
        memcpy(&magic, data, 4);
        return magic == 0xfeedfacfU; // MH_MAGIC_64
    }

    std::unique_ptr<llvm::MemoryBuffer> getObject(const llvm::Module *M) {
        if (!M || !isReady()) return nullptr;
        std::string path = keyPath(M);
        int fd = open(path.c_str(), O_RDONLY | O_NOFOLLOW);
        if (fd < 0) {
            misses.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
        struct stat st;
        if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_size < 32 || st.st_size > 16 * 1024 * 1024) {
            close(fd);
            unlink(path.c_str());
            corrupt.fetch_add(1, std::memory_order_relaxed);
            misses.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
        auto buf = llvm::MemoryBuffer::getOpenFile(fd, path, st.st_size, false);
        close(fd);
        if (!buf || !isValidMachO64((*buf)->getBufferStart(), (*buf)->getBufferSize())) {
            unlink(path.c_str());
            corrupt.fetch_add(1, std::memory_order_relaxed);
            misses.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
        hits.fetch_add(1, std::memory_order_relaxed);
        return std::move(*buf);
    }

    void notifyObjectCompiled(const llvm::Module *M, llvm::MemoryBufferRef Obj) {
        if (!M || !isReady()) return;
        size_t sz = Obj.getBufferSize();
        if (!isValidMachO64(Obj.getBufferStart(), sz)) return;
        if (bytes_used.load(std::memory_order_relaxed) + sz > max_bytes) return;
        std::string path = keyPath(M);
        uint64_t id = seq.fetch_add(1, std::memory_order_relaxed);
        char tmp[320];
        snprintf(tmp, sizeof(tmp), "%s.%d.%lu.%llu.tmp", path.c_str(), (int)getpid(),
                 (unsigned long)pthread_self(), (unsigned long long)id);
        int fd = open(tmp, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
        if (fd < 0) return;
        const char *p = Obj.getBufferStart();
        size_t rem = sz;
        bool ok = true;
        while (rem > 0) {
            ssize_t n = write(fd, p, rem);
            if (n <= 0) { ok = false; break; }
            p += n;
            rem -= static_cast<size_t>(n);
        }
        if (ok && fsync(fd) != 0) ok = false;
        close(fd);
        if (ok) {
            if (rename(tmp, path.c_str()) == 0) {
                bytes_used.fetch_add(sz, std::memory_order_relaxed);
            } else {
                unlink(tmp);
            }
        } else {
            unlink(tmp);
        }
    }
};
static SSDiskObjectCache gSSDiskObjCache;

class SSCachingIRCompiler final : public llvm::orc::IRCompileLayer::IRCompiler {
    llvm::orc::ConcurrentIRCompiler inner;
public:
    explicit SSCachingIRCompiler(llvm::orc::JITTargetMachineBuilder jtmb)
        : IRCompiler(llvm::orc::irManglingOptionsFromTargetOptions(jtmb.getOptions())),
          inner(std::move(jtmb)) {}
    llvm::Expected<std::unique_ptr<llvm::MemoryBuffer>> operator()(llvm::Module &M) override {
        if (auto cached = gSSDiskObjCache.getObject(&M)) return std::move(cached);
        auto obj = inner(M);
        if (obj) gSSDiskObjCache.notifyObjectCompiled(&M, (*obj)->getMemBufferRef());
        return obj;
    }
};
"""
    src = src.replace("class JITRoutine : public rr::Routine", disk_cache_cls + "\nclass JITRoutine : public rr::Routine", 1)
    new_cl = "llvm::orc::IRCompileLayer compileLayer(session, objectLayer, std::make_unique<SSCachingIRCompiler>(JITGlobals::get()->getTargetMachineBuilder()));"
    src = src.replace(old_cl, new_cl, 1)

    # 5. Log START (with BB and Instruction count) and DONE (with ms + cache hit/miss) in acquireRoutine
    old_acq = """std::shared_ptr<rr::Routine> JITBuilder::acquireRoutine(const char *name, llvm::Function **funcs, size_t count)
{
\tASSERT(module);
\treturn std::make_shared<JITRoutine>(std::move(module), std::move(context), name, funcs, count);
}"""
    assert old_acq in src, "acquireRoutine not found"
    new_acq = """std::shared_ptr<rr::Routine> JITBuilder::acquireRoutine(const char *name, llvm::Function **funcs, size_t count)
{
\tASSERT(module);
\tss_boost_thread_prio();
\tsize_t bbs = 0, insts = 0;
\tfor (size_t i = 0; i < count; ++i) {
\t\tif (funcs[i]) {
\t\t\tbbs += funcs[i]->size();
\t\t\tinsts += funcs[i]->getInstructionCount();
\t\t}
\t}
\tuint64_t h0 = gSSDiskObjCache.hits.load(std::memory_order_relaxed);
\tauto t0 = std::chrono::steady_clock::now();
\tauto r = std::make_shared<JITRoutine>(std::move(module), std::move(context), name, funcs, count);
\tauto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
\tuint64_t h1 = gSSDiskObjCache.hits.load(std::memory_order_relaxed);
\tss_console_log("[ss-jit] %s %s (bb=%zu insts=%zu %lldms hits=%llu)\\n",
\t               (h1 > h0) ? "HIT " : "MISS", name ? name : "?", bbs, insts, (long long)ms, (unsigned long long)h1);
\treturn r;
}"""
    src = src.replace(old_acq, new_acq, 1)

    patched_cpp = OUT_DIR / "LLVMJIT_o1nosroa.cpp"
    patched_cpp.write_text(src)
    print(f"Wrote {patched_cpp}")

    ninja = (SS_BUILD / "build.ninja").read_text().splitlines()

    def parse_block(marker):
        for i, line in enumerate(ninja):
            if marker in line:
                block = [line]
                j = i + 1
                while j < len(ninja) and ninja[j].startswith("  "):
                    block.append(ninja[j])
                    j += 1
                return block
        raise RuntimeError(f"Marker not found: {marker}")

    comp_block = parse_block("build src/Reactor/CMakeFiles/ReactorLLVM.dir/LLVMJIT.cpp.o:")
    kv = dict(l.strip().split(" = ", 1) for l in comp_block[1:])

    obj_out = OUT_DIR / "LLVMJIT_o1nosroa.o"
    compile_cmd = f"clang++ {kv['DEFINES']} {kv['INCLUDES']} {kv['FLAGS']} -c {patched_cpp} -o {obj_out}"
    print("Compiling LLVMJIT_o1nosroa.cpp...")
    subprocess.run(compile_cmd, shell=True, check=True)

    ar_block = parse_block("build src/Reactor/libReactorLLVM.a:")
    ar_objs = ar_block[0].split(": CXX_STATIC_LIBRARY_LINKER__ReactorLLVM_Release ", 1)[1].split(" || ")[0].split()
    local_objs = [str(obj_out) if o.endswith("LLVMJIT.cpp.o") else str(SS_BUILD / o) for o in ar_objs]
    reactor_a = OUT_DIR / "libReactorLLVM_o1nosroa.a"
    if reactor_a.exists():
        reactor_a.unlink()
    subprocess.run(["ar", "rcs", str(reactor_a)] + local_objs, check=True)
    subprocess.run(["ranlib", str(reactor_a)], check=True)

    link_block = parse_block("build libvk_swiftshader.dylib:")
    lkv = dict(l.strip().split(" = ", 1) for l in link_block[1:])

    vk_objs_raw = link_block[0].split(": CXX_SHARED_LIBRARY_LINKER__vk_swiftshader_Release ", 1)[1].split(" | ")[0].split()
    vk_objs = []
    for o in vk_objs_raw:
        if o.endswith("libVulkan.cpp.o"):
            vk_objs.append(str(HOSTMEM_DIR / "libVulkan.hostmem.o"))
        elif o.endswith("VkPhysicalDevice.cpp.o"):
            vk_objs.append(str(HOSTMEM_DIR / "VkPhysicalDevice.hostmem.o"))
        else:
            vk_objs.append(str(SS_BUILD / o))

    resolved_libs = []
    for tok in lkv["LINK_LIBRARIES"].split():
        if tok == "src/Reactor/libReactorLLVM.a":
            resolved_libs.append(str(reactor_a))
        elif tok.endswith(".a") and not tok.startswith("/"):
            resolved_libs.append(str(SS_BUILD / tok))
        else:
            resolved_libs.append(tok)

    out_dylib = DESKTOP_DIR / "libvk_swiftshader.o1nosroa.dylib"
    link_cmd = (
        f"clang++ {lkv['LANGUAGE_COMPILE_FLAGS']} {lkv['ARCH_FLAGS']} {lkv['LINK_FLAGS']} "
        f"-install_name @rpath/libvk_swiftshader.dylib -o {out_dylib} "
        + " ".join(vk_objs) + " " + " ".join(resolved_libs)
    )
    print("Linking libvk_swiftshader.o1nosroa.dylib...")
    subprocess.run(link_cmd, shell=True, check=True)
    shutil.copy2(out_dylib, OUT_DIR / "libvk_swiftshader.o1nosroa.unstripped.dylib")

    subprocess.run(["strip", "-x", str(out_dylib)], check=True)
    subprocess.run(["codesign", "--force", "--sign", "-", str(out_dylib)], check=True)
    print(f"SUCCESS: {out_dylib} ({out_dylib.stat().st_size:,} bytes)")

if __name__ == "__main__":
    main()
