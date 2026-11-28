#!/usr/bin/env python3
"""Apply post-MMU diag, late-blocks (3 cyan progress blocks), and 4-byte surgical SDT fallback patch.

Fix over legacy apply-late-blocks-and-sdt.py:
  _ml_unsafe_kernel_text ends at +0x30 (4-byte `bl _ml_unsafe_kernel_text.cold.1`).
  +0x34 is the entry point of compiler-outlined `_OUTLINED_FUNCTION_0` (`mov x4,#0; mov x5,#0; b _kernel_debug`).
  The legacy 8-byte patch at +0x30 overwrote +0x34 with `b _ml_unsafe_kernel_text+0x1c` (`ldp x29,x30; retab`),
  causing instant IB-key PAC panics at `_ml_unsafe_kernel_text+0x20` whenever ktrace/tailspind called `_OUTLINED_FUNCTION_0`.
  This fixed script puts the 8-byte stub (`mov w0,#0; b _ml_unsafe_kernel_text+0x1c`) in an empty cave at `entry_va + 0x920`
  and patches ONLY the 4 bytes at `_ml_unsafe_kernel_text + 0x30` with `b <cave>`, leaving `_OUTLINED_FUNCTION_0` intact.
"""
from pathlib import Path
import shutil
import struct
import subprocess
import sys

DIAG = Path("/Users/imagination/Documents/Unixnu/local-artifacts/macos27-firstboot/postmmu-diag")
NM = "/Users/imagination/tools/LLVM-19.1.7-macOS-X64/bin/llvm-nm"


def branch(from_va, to_va, link=False):
    delta = (to_va - from_va) // 4
    assert (to_va - from_va) % 4 == 0 and -(1 << 25) <= delta < (1 << 25)
    return (0x94000000 if link else 0x14000000) | (delta & 0x3FFFFFF)


def mov_x9_u64(val):
    return struct.pack(
        "<IIII",
        0xD2800009 | (((val >> 0) & 0xFFFF) << 5),
        0xF2A00009 | (((val >> 16) & 0xFFFF) << 5),
        0xF2C00009 | (((val >> 32) & 0xFFFF) << 5),
        0xF2E00009 | (((val >> 48) & 0xFFFF) << 5),
    )


def load_segments(data):
    segs = []
    offset = 32
    for _ in range(struct.unpack_from("<I", data, 16)[0]):
        cmd, size = struct.unpack_from("<II", data, offset)
        if cmd == 0x19:
            _, vmaddr, vmsize, fileoff, filesize = struct.unpack_from("<16sQQQQ", data, offset + 8)
            if filesize:
                segs.append((vmaddr, vmsize, fileoff, filesize))
        offset += size
    return segs


def file_offset(segs, va):
    for vmaddr, vmsize, fileoff, filesize in segs:
        if vmaddr <= va < vmaddr + min(vmsize, filesize):
            return fileoff + (va - vmaddr)
    raise ValueError(f"VA {va:#x} not in segments")


def load_kernel_symbols(path):
    by_addr = {}
    by_name = {}
    out = subprocess.check_output([NM, "-n", str(path)], text=True)
    for line in out.splitlines():
        if line.startswith("Symbols for ") and not line.startswith("Symbols for com.apple.kernel:"):
            break
        parts = line.split()
        if len(parts) == 3 and all(c in "0123456789abcdefABCDEF" for c in parts[0]):
            va = int(parts[0], 16)
            by_addr.setdefault(va, parts[2])
            by_name[parts[2]] = va
    return by_addr, by_name


def find_call(data, segs, by_addr, by_name, fn_name, target_name):
    starts = sorted(by_addr)
    fn_va = by_name[fn_name]
    fn_end = next(a for a in starts if a > fn_va)
    target_va = by_name[target_name]
    hits = []
    for va in range(fn_va, fn_end, 4):
        word = struct.unpack_from("<I", data, file_offset(segs, va))[0]
        if word >> 26 == 0b100101:
            imm = word & 0x3FFFFFF
            delta = imm - (1 << 26) if imm & (1 << 25) else imm
            if va + delta * 4 == target_va:
                hits.append(va)
    assert len(hits) == 1, f"Expected 1 call from {fn_name} to {target_name}, got {len(hits)}"
    return hits[0], target_va


def unixthread_pc(data):
    offset = 32
    for _ in range(struct.unpack_from("<I", data, 16)[0]):
        cmd, size = struct.unpack_from("<II", data, offset)
        if cmd == 0x5:
            return struct.unpack_from("<Q", data, offset + 16 + 32 * 8)[0]
        offset += size
    raise ValueError("No LC_UNIXTHREAD")


def main():
    output = Path(sys.argv[1]).resolve()
    src = output / "xnu-cdc-board.macho"
    dst = output / "xnu-cdc-board-diag.macho"
    shutil.copy2(src, dst)

    subprocess.run(
        [
            sys.executable,
            str(DIAG / "patch-xnu-postmmu-diag.py"),
            str(dst),
            "--uefi",
            "local-artifacts/kernel27-rebuild/mu-silicium",
        ],
        check=True,
    )

    data = bytearray(dst.read_bytes())
    segs = load_segments(data)
    by_addr, by_name = load_kernel_symbols(dst)

    vinfo_va = by_name.get("_boot_progress_panic_vinfo", by_name["_vinfo"])
    entry_va = unixthread_pc(data)

    # Inline verified 420-byte late-blocks progress templates (byte-for-byte identical across Boot 70 & Boot 161;
    # vinfo_va and branch instructions are re-patched dynamically immediately below).
    cave1 = bytearray.fromhex(
        "090194d2e994a1f209fedff2e9fffff22d0d40b9bf011071430300542c0140b9"
        "9fa10071e3020054290940f9a90200b429110d8b29010491ebff9f520be0bf72"
        "8f0280520c7438d52ee179922e7808d5df3f03d50a7438d5aa0000370a0480d2"
        "cb4500b84a0500f1c1ffff5429010d8bef050071a1feff540c7418d59f3f03d5"
        "70701914"
    )
    cave2 = bytearray.fromhex(
        "fe0f1ff8dc150094fe0741f8090194d2e994a1f209fedff2e9fffff22d0d40b9"
        "bf011071a30300542c0140b99fa1007143030054290940f9090300b429110d8b"
        "29010891ebff9f520be0bf72600000340b008052ebffbf728f0280520c7438d5"
        "2ee179922e7808d5df3f03d50a7438d5aa0000370a0480d2cb4500b84a0500f1"
        "c1ffff5429010d8bef050071a1feff540c7418d59f3f03d5c0035fd6"
    )
    cave3 = bytearray.fromhex(
        "090194d2e994a1f209fedff2e9fffff22d0d40b9bf011071430300542c0140b9"
        "9fa10071e3020054290940f9a90200b429110d8b29010c91ebff9f520be0bf72"
        "8f0280520c7438d52ee179922e7808d5df3f03d50a7438d5aa0000370a0480d2"
        "cb4500b84a0500f1c1ffff5429010d8bef050071a1feff540c7418d59f3f03d5"
        "c0ab1814"
    )

    c1_va = entry_va + 0x680
    c2_va = entry_va + 0x780
    c3_va = entry_va + 0x880
    c4_va = entry_va + 0x920  # 8-byte cave for SDT fallback (after cave3 ends at +0x904)

    # Find call sites dynamically from symbol table
    pe_call_va, pe_target_va = find_call(data, segs, by_addr, by_name, "_kernel_bootstrap_thread", "_PE_init_iokit")
    armcpu_call_va, ml_proc_reg_va = find_call(data, segs, by_addr, by_name, "__ZN6ArmCpu5startEP9IOService", "_ml_processor_register")
    bsd_call_va, bsd_target_va = find_call(data, segs, by_addr, by_name, "_bsd_init", "_IOFindBSDRoot")

    # Patch vinfo_va and branch targets in cave1, cave2, cave3
    cave1[0:16] = mov_x9_u64(vinfo_va)
    struct.pack_into("<I", cave1, 0x80, branch(c1_va + 0x80, pe_target_va, link=False))

    struct.pack_into("<I", cave2, 0x04, branch(c2_va + 0x04, ml_proc_reg_va, link=True))
    cave2[0x0C:0x1C] = mov_x9_u64(vinfo_va)

    cave3[0:16] = mov_x9_u64(vinfo_va)
    struct.pack_into("<I", cave3, 0x80, branch(c3_va + 0x80, bsd_target_va, link=False))

    # Build 8-byte cave4 for SDT fallback: mov w0, #0; b (_ml_unsafe_kernel_text + 0x1c)
    ml_unsafe_va = by_name["_ml_unsafe_kernel_text"]
    epilogue_va = ml_unsafe_va + 0x1C  # ldp x29, x30, [sp], #0x10; retab
    cave4 = bytearray(struct.pack("<II", 0x52800000, branch(c4_va + 4, epilogue_va, link=False)))

    for c_va, blob in ((c1_va, cave1), (c2_va, cave2), (c3_va, cave3), (c4_va, cave4)):
        foff = file_offset(segs, c_va)
        assert not any(data[foff : foff + len(blob)]), f"Cave at {c_va:#x} not empty"
        data[foff : foff + len(blob)] = blob

    struct.pack_into("<I", data, file_offset(segs, pe_call_va), branch(pe_call_va, c1_va, link=True))
    struct.pack_into("<I", data, file_offset(segs, armcpu_call_va), branch(armcpu_call_va, c2_va, link=True))
    struct.pack_into("<I", data, file_offset(segs, bsd_call_va), branch(bsd_call_va, c3_va, link=True))

    # Apply 4-byte surgical SDT fallback patch ONLY at _ml_unsafe_kernel_text + 0x30 (do NOT touch +0x34!)
    sdt_va = ml_unsafe_va + 0x30
    sdt_foff = file_offset(segs, sdt_va)
    w0, w1 = struct.unpack_from("<II", data, sdt_foff)
    assert (w0 >> 26) == 0b100101 and w1 == 0xD2800004, f"Unexpected SDT bytes at {sdt_va:#x}: {w0:#x} {w1:#x}"
    struct.pack_into("<I", data, sdt_foff, branch(sdt_va, c4_va, link=False))
    # Verify +0x34 (_OUTLINED_FUNCTION_0) is completely untouched
    assert struct.unpack_from("<I", data, sdt_foff + 4)[0] == 0xD2800004

    dst.write_bytes(data)
    print(f"PASS: patched {dst.name} with postmmu-diag + CDC panic hooks + 3 late-blocks + 4-byte surgical SDT fallback")
    print(f"  vinfo_va      = {vinfo_va:#x}")
    print(f"  cave1 (IOKit) = {c1_va:#x} <- {pe_call_va:#x} -> {pe_target_va:#x}")
    print(f"  cave2 (CPU0)  = {c2_va:#x} <- {armcpu_call_va:#x} -> {ml_proc_reg_va:#x}")
    print(f"  cave3 (Root)  = {c3_va:#x} <- {bsd_call_va:#x} -> {bsd_target_va:#x}")
    print(f"  cave4 (SDT)   = {c4_va:#x} <- {sdt_va:#x} -> {epilogue_va:#x} (+0x34 _OUTLINED_FUNCTION_0 preserved!)")


if __name__ == "__main__":
    main()
