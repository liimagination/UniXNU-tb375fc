#!/Users/imagination/Documents/Unixnu/.tools/venv/bin/python
"""Build isolated libTb375fcDesktopStorage_rw.a with bounded UFS LUN2 PersistOverlay support (P2).

Strictly isolated in local-artifacts/macos27-firstboot/ram-expansion/kernel/ufs-rw/.
Does NOT modify postmmu-diag/m5-gui/desktop-root/ or m5-v14/.
"""
import json
from pathlib import Path
import runpy
import subprocess

HERE = Path(__file__).resolve().parent
RAM_DIR = HERE.parents[1]
BASE = RAM_DIR.parent
M5 = BASE / "postmmu-diag/m5-gui/m5-v14"
DROOT = BASE / "postmmu-diag/m5-gui/desktop-root"
STORAGE_RO = DROOT / "storage-build"
UPSTREAM_RAMBLOCK = BASE / "upstream/drivers/RamBlock"


def main():
    builder = runpy.run_path(str(M5.parent / "build_m5_kernel_and_pack.py"))
    ks = builder["XNU_OBJ"]
    decoder = json.JSONDecoder()
    cpp, _ = decoder.raw_decode((ks / "iokit/DEVELOPMENT/IOPlatformExpert.cpo.json").read_text())
    c, _ = decoder.raw_decode((ks / "osfmk/DEVELOPMENT/startup.o.json").read_text())

    # 1. Create isolated NativeRamBlock.cpp from m5-v14/NativeRamBlock.cpp without the 7 stub hooks,
    #    and add a concise summary IOLog at the end of attachStore().
    nrb_txt = (M5 / "NativeRamBlock.cpp").read_text()
    old_stubs = """extern "C" {
unsigned native_ufs_store_mode(void) { return 0; }
int native_ufs_store_open(uint64_t, uint64_t *, const char **) { return 0; }
int native_ufs_store_read(uint64_t, uint32_t, void *) { return 0; }
int native_ufs_store_write(uint64_t, uint32_t, const void *) { return 0; }
int native_ufs_store_write_cached(uint64_t, uint32_t, const void *) { return 0; }
int native_ufs_store_sync(void) { return 0; }
unsigned native_ufs_store_gpt_check(void) { return 0; }
IOService *tb375fc_create_ram_block(void) { return new NativeRamBlock; }
}"""
    new_stubs = """extern "C" {
IOService *tb375fc_create_ram_block(void) { return new NativeRamBlock; }
}"""
    assert old_stubs in nrb_txt, "Expected 7 native_ufs_store_* stubs in m5-v14/NativeRamBlock.cpp"
    nrb_txt = nrb_txt.replace(old_stubs, new_stubs, 1)

    old_open = """        StoreOps ops;
        persist::Open result = overlayBits && stage ? overlay.open(ops, pages, seedHash, mode == 1, overlayBits, discardRequested())
                                                          : persist::Open::Failed;"""
    new_open = """        StoreOps ops;
        bool autoUpgrade = false;
        if (mode == 2 && !discardRequested() && pages >= 524288ULL && stage) {
          if (ops.read(0, 1, stage)) {
            bool allZero = true;
            for (size_t zi = 0; zi < persist::kBlock; ++zi) {
              if (stage[zi] != 0) { allZero = false; break; }
            }
            if (allZero || (first == 33128000ULL && pages == 16777216ULL && memcmp(stage, "SHNGDATA", 8) != 0)) {
              autoUpgrade = true;
              IOLog("NativeRamBlock: initializing fresh %llu-page window at lba=%llu (one-time fresh init)\\n", pages, first);
            } else if (memcmp(stage, "SHNGDATA", 8) == 0 &&
                       persist::get32(stage + 8) == 1 &&
                       persist::get32(stage + 12) == persist::kHeaderBytes &&
                       persist::get32(stage + 60) == persist::crc32(stage, 60)) {
              const uint64_t oldPages = persist::get64(stage + 16);
              if (oldPages > 0 && oldPages < pages) {
                autoUpgrade = true;
                IOLog("NativeRamBlock: upgrading smaller %llu-page SHNGDATA overlay -> %llu pages (one-time fresh init)\\n", oldPages, pages);
              }
            }
          }
        }
        persist::Open result = overlayBits && stage ? overlay.open(ops, pages, seedHash, mode == 1, overlayBits, discardRequested() || autoUpgrade)
                                                          : persist::Open::Failed;"""
    assert old_open in nrb_txt, "Expected overlay.open call in NativeRamBlock.cpp"
    nrb_txt = nrb_txt.replace(old_open, new_open, 1)

    old_end = """        native_data_persist_present = overlay.present;
        native_data_persist_gpt_after = native_ufs_store_gpt_check();
      }
    }
    IOLockUnlock(gate);"""
    new_end = """        native_data_persist_present = overlay.present;
        native_data_persist_gpt_after = native_ufs_store_gpt_check();
        IOLog("NativeRamBlock: attachStore done mode=%u state=%u header=%u gpt=%u->%u lba=%llu pages=%llu present=%llu direct=%d wb=%d\\n",
              mode, native_data_persist_state, native_data_persist_header,
              native_data_persist_gpt, native_data_persist_gpt_after,
              native_data_persist_first_lba, native_data_persist_pages,
              native_data_persist_present, direct ? 1 : 0, writeBack ? 1 : 0);
      }
    }
    IOLockUnlock(gate);"""
    assert old_end in nrb_txt, "Expected attachStore tail in NativeRamBlock.cpp"
    nrb_txt = nrb_txt.replace(old_end, new_end, 1)

    nrb_src = HERE / "NativeRamBlock.cpp"
    nrb_src.write_text(nrb_txt)

    flags = [
        "-I", str(builder["STORAGE_HDR"]),
        "-I", str(HERE),
        "-I", str(UPSTREAM_RAMBLOCK),
        "-I", str(M5),
        "-I", str(builder["XNU_SRC"] / "osfmk"),
        "-include", str(M5 / "macos-target.h"),
        "-g0", "-Os", "-fno-sanitize=signed-integer-overflow", "-Wno-error",
    ]

    # 2. Compile NativeRamBlock.cpp -> audit vtable -> globalize -> strip __mod_init_func
    nrb_o = HERE / "NativeRamBlock.o"
    nrb_fixed = HERE / "NativeRamBlock.fixed.o"
    nrb_late = HERE / "NativeRamBlock.late.o"
    print("Compiling NativeRamBlock.cpp...", flush=True)
    subprocess.run(builder["build_cmd_from_record"](cpp, nrb_src, nrb_o, flags), cwd=cpp["directory"], check=True)
    audit = builder["audit_and_repair_storage_vtable"](nrb_o, "__ZTV14NativeRamBlock", nrb_fixed)
    print(f"NativeRamBlock vtable audit: {audit}", flush=True)
    builder["globalize_symbol_in_macho"](nrb_fixed, "__GLOBAL__sub_I_NativeRamBlock.cpp")
    subprocess.run([builder["OBJCOPY"], "--remove-section=__DATA,__mod_init_func", str(nrb_fixed), str(nrb_late)], check=True)

    # 3. Compile Tb375fcPersistStore.cpp
    ps_src = HERE / "Tb375fcPersistStore.cpp"
    ps_o = HERE / "Tb375fcPersistStore.o"
    print("Compiling Tb375fcPersistStore.cpp...", flush=True)
    subprocess.run(builder["build_cmd_from_record"](cpp, ps_src, ps_o, flags), cwd=cpp["directory"], check=True)

    # 4. Compile Tb375fcUfsTransport.c (with bounded write-once window + WRITE(10) + SYNCHRONIZE CACHE(10))
    ut_src = HERE / "Tb375fcUfsTransport.c"
    ut_o = HERE / "Tb375fcUfsTransport.o"
    print("Compiling Tb375fcUfsTransport.c...", flush=True)
    subprocess.run(builder["build_cmd_from_record"](c, ut_src, ut_o, flags), cwd=c["directory"], check=True)

    # 5. Assemble libTb375fcDesktopStorage_rw.a reusing the verified RO desktop objects
    reused = [
        STORAGE_RO / "Tb375fcUfs.late.o",
        STORAGE_RO / "DesktopRootKey.o",
        STORAGE_RO / "SourceRootPolicies.o",
        STORAGE_RO / "SourceRootBinding.o",
        STORAGE_RO / "verified_root.o",
        STORAGE_RO / "xnu_crypto.o",
        M5 / "objs/IOBlockStorageDevice.late.o",
    ]
    for p in reused:
        assert p.is_file(), f"Missing {p}"

    target = HERE / "libTb375fcDesktopStorage_rw.a"
    stage = HERE / "storage_rw.pending.a"
    stage.unlink(missing_ok=True)
    subprocess.run([
        builder["AR"], "rcs", str(stage),
        *map(str, reused), str(nrb_late), str(ps_o), str(ut_o),
    ], check=True)
    stage.replace(target)
    print(f"PASS: built {target} ({target.stat().st_size:,} B)", flush=True)


if __name__ == "__main__":
    main()
