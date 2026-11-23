#!/Users/imagination/Documents/Unixnu/.tools/venv/bin/python
"""Build M7-v4-based XNU kernel with start.s bootstrap-pagetable clamp for up to 8 GiB RAM.

Strictly isolated in local-artifacts/macos27-firstboot/ram-expansion/kernel/.
Read-only against m7-metal/kernel-out-v4/ and XNU_SRC/XNU_OBJ.
"""
import hashlib
import json
import pathlib
import shutil
import struct
import subprocess
import sys

KDIR = pathlib.Path(__file__).resolve().parent
RAM_DIR = KDIR.parent
BASE = RAM_DIR.parent
M7_V4_K = BASE / 'postmmu-diag/m5-gui/m7-metal/kernel-out-v4'
TREE = BASE / 'out/tb375fc/macos-27.3-26C5035r/tb375fc/tree'
XNU_SRC = TREE / '.cache/xnu-work'
XNU_OBJ = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837'
I_DIR = BASE / 'postmmu-diag/tb375fc-usb-cdc/xnu-integration'
LLVM_BIN = pathlib.Path('/Users/imagination/tools/LLVM-19.1.7-macOS-X64/bin')
OBJDUMP = str(LLVM_BIN / 'llvm-objdump')
AR = str(LLVM_BIN / 'llvm-ar')
PY = '/Users/imagination/Documents/Unixnu/.tools/venv/bin/python'

# Slots 318/320/361/362 are declared pure-virtual on IOMobileFramebuffer (base of
# IOMobileFramebufferLegacy); callers such as s_supported_frame_info sign blraa with
# IOMobileFramebuffer's PAC discriminators, NOT IOMobileFramebufferLegacy's ___cxa_pure_virtual!
IOMFB_BASE_PAC_FIXES = {
    318: (0xd5cf, 0xb486),
    320: (0x7dc3, 0xbd70),
    361: (0xf329, 0x358b),
    362: (0xcd03, 0xe1af),
}


def patch_native_legacy_display_pac(src_obj: pathlib.Path, dst_obj: pathlib.Path):
    raw = bytearray(src_obj.read_bytes())
    _, _, _, _, ncmds, _ = struct.unpack_from('<6I', raw, 0)
    cmd_off = 32
    symoff = nsyms = stroff = 0
    sections = []
    for _ in range(ncmds):
        cmd, cmdsize = struct.unpack_from('<II', raw, cmd_off)
        if cmd == 0x19:
            nsects = struct.unpack_from('<I', raw, cmd_off + 64)[0]
            for i in range(nsects):
                s = cmd_off + 72 + 80 * i
                addr, size, offset = struct.unpack_from('<QQI', raw, s + 32)
                sections.append({'addr': addr, 'offset': offset})
        elif cmd == 0x2:
            symoff, nsyms, stroff, _ = struct.unpack_from('<4I', raw, cmd_off + 8)
        cmd_off += cmdsize
    ztv_addr = ztv_sect = None
    for i in range(nsyms):
        strx, _, n_sect, _, n_value = struct.unpack_from('<IBBHQ', raw, symoff + 16 * i)
        end = raw.index(b'\0', stroff + strx)
        if raw[stroff + strx:end] == b'__ZTV19NativeLegacyDisplay':
            ztv_addr = n_value
            ztv_sect = sections[n_sect - 1]
            break
    assert ztv_addr is not None, '__ZTV19NativeLegacyDisplay not found'
    file_base = ztv_sect['offset'] + (ztv_addr - ztv_sect['addr']) + 16
    for slot, (old_div, new_div) in IOMFB_BASE_PAC_FIXES.items():
        pos = file_base + slot * 8
        w = struct.unpack_from('<Q', raw, pos)[0]
        cur_div = (w >> 32) & 0xffff
        assert cur_div == old_div, f'slot[{slot}] expected div {old_div:#x}, got {cur_div:#x}'
        struct.pack_into('<Q', raw, pos, (w & 0xffff0000ffffffff) | (new_div << 32))
    dst_obj.write_bytes(bytes(raw))


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_display(output: pathlib.Path) -> pathlib.Path:
    output.mkdir(parents=True, exist_ok=True)
    print('[2/4] Building NativeLegacyDisplay (direct kickSwapQueue + PAC 318/320/361/362) + LegacyDisplayProbe (&BGA stride)...')
    import runpy
    m5_gui = BASE / 'postmmu-diag/m5-gui'
    m5_builder = runpy.run_path(str(m5_gui / 'build_m5_kernel_and_pack.py'))
    m4_dir = m5_builder['M3M4_DIR']
    m4 = runpy.run_path(str(m4_dir / 'build-m3m4.py'))
    kc_image = m5_builder['Image']

    disp_src = KDIR / 'NativeLegacyDisplay.cpp'
    disp_o = output / 'NativeLegacyDisplay.o'
    disp_late_raw = output / 'NativeLegacyDisplay.late.raw.o'
    patched_disp_o = output / 'NativeLegacyDisplay.late.o'
    probe_src = KDIR / 'LegacyDisplayProbe.cpp'
    probe_o = output / 'LegacyDisplayProbe.o'

    crec = json.JSONDecoder().raw_decode((XNU_OBJ / 'iokit/DEVELOPMENT/IOPlatformExpert.cpo.json').read_text())[0]
    extra = [
        f'-I{m4_dir}', f'-I{BASE / "upstream/drivers/GopFramebuffer"}',
        f'-I{BASE / "upstream/drivers/AdrenoMetal"}',
        f'-I{BASE / "postmmu-diag/m5-gui/m7-metal/common"}',
        '-include', str(m4_dir / 'm4-bootargs.h'),
        '-DNATIVE_LEGACY_MACOS=1', '-DNATIVE_LEGACY_CANCEL_SWAPS=1',
        '-DNATIVE_LEGACY_DEFAULT_SURFACE=1',
    ]
    for src_f, out_f in ((disp_src, disp_o), (probe_src, probe_o)):
        cargs = []
        for tok in crec['arguments']:
            if tok == crec['file']:
                cargs.append(str(src_f))
            elif tok == crec['output']:
                cargs.append(str(out_f))
            else:
                cargs.append(tok)
        subprocess.run(cargs + extra, cwd=crec['directory'], check=True)
        print(f'      PASS: compiled {out_f.name} ({out_f.stat().st_size} B)')

    kc_info = m4['inspect_kc_symbols'](kc_image(m4['KC_PATH'].read_bytes()))
    # Patch both blraa and braa (tail-call!) PAC discriminators + vtable slots directly in our isolated copy
    src_m4_fn = (m4_dir / 'build-m3m4.py').read_text()
    # Accept {294, 365, 416, 423, 479} since internal kickSwapQueue (slot 411) is now a direct non-virtual call!
    ns = dict(m4)
    fn_code = src_m4_fn[src_m4_fn.index('def patch_object_vtable_and_init(') : src_m4_fn.index('def write_bindings_and_adapters(')]
    fn_code = fn_code.replace('(insn & 0xfffffc00) == 0xd73f0800', '(insn & 0xffdf_fc00) == 0xd71f0800')
    fn_code = fn_code.replace('{294, 365, 411, 416, 423, 479}', '{294, 365, 416, 423, 479}')
    exec(fn_code, ns)
    ns['patch_object_vtable_and_init'](disp_o, kc_info['vtable_divs'])
    m4['globalize_symbol_in_macho'](disp_o, '__GLOBAL__sub_I_NativeLegacyDisplay.cpp')
    subprocess.run([m4['OBJCOPY'], '--remove-section=__DATA,__mod_init_func',
                    str(disp_o), str(disp_late_raw)], check=True)
    patch_native_legacy_display_pac(disp_late_raw, patched_disp_o)

    patched_disp_a = output / 'libNativeLegacyDisplay.a'
    shutil.copyfile(M7_V4_K / 'libNativeLegacyDisplay.a', patched_disp_a)
    subprocess.run([AR, 'rcs', str(patched_disp_a), str(patched_disp_o), str(probe_o)], check=True)
    print('      PASS: patched NativeLegacyDisplay (direct kickSwapQueue + PAC) + LegacyDisplayProbe (&BGA stride)!')

    return patched_disp_a


def main():
    print('[1/4] Patching start.s (clamp temporary bootstrap_pagetables to <=448 MiB inside DDR Mid 2)...')
    orig_start = XNU_SRC / 'osfmk/arm64/start.s'
    src = orig_start.read_text()
    anchor = '\t/*\n\t * Setup the V=P bootstrap mapping\n\t * x5 - total number of L2 entries to allocate\n\t */\n\tlsr\t\tx5,  x24, #(ARM_TT_L2_SHIFT)'
    assert src.count(anchor) == 1, 'Anchor not found uniquely in start.s'
    replacement = (
        '\t/*\n'
        '\t * TB375FC RAM expansion: clamp temporary bootstrap_pagetables mapping to\n'
        '\t * <= 448 MiB (0x1C000000, strictly inside DDR Mid 2 and below TEE at 0x70000000).\n'
        '\t * HeadSize (~285 MiB for an 8 GiB span) fits inside 448 MiB, and arm_vm_init()\n'
        '\t * builds cpu_tte across full BA_MEM_SIZE skipping darwinboot-memory-excluded.\n'
        '\t */\n'
        '\tmovz\t\tx6, #0x1c00, lsl #16\n'
        '\tcmp\t\tx24, x6\n'
        '\tcsel\t\tx24, x6, x24, hi\n\n'
        + anchor
    )
    patched_start = KDIR / 'start.s'
    mpidr_old = (
        '#if HAS_CLUSTER\n'
        '\tand\t\tx0, x15, #0xFFFF\t\t\t\t\t// CPU number in Affinity0, cluster ID in Affinity1\n'
        '#else\n'
        '\tand\t\tx0, x15, #0xFF\t\t\t\t\t\t// CPU number is in MPIDR Affinity Level 0\n'
        '#endif'
    )
    assert src.count(mpidr_old) == 1, 'MPIDR mask block not found uniquely in start.s'
    mpidr_new = '\tand\t\tx0, x15, #0xFFFF\t\t\t\t\t// TB375FC MT6897: core ID is in Aff1:Aff0 (0x000..0x700)'
    patched_start.write_text(src.replace(anchor, replacement, 1).replace(mpidr_old, mpidr_new, 1))

    rec = json.JSONDecoder().raw_decode((XNU_OBJ / 'osfmk/DEVELOPMENT/start.o.json').read_text())[0]
    start_o = KDIR / 'start.o'
    args = []
    rep_src = rep_out = False
    for tok in rec['arguments']:
        if tok == rec['file']:
            args.append(str(patched_start))
            rep_src = True
        elif tok == rec['output']:
            args.append(str(start_o))
            rep_out = True
        else:
            args.append(tok)
    assert rep_src and rep_out, 'Failed to rewrite start.o.json arguments'
    cmd = [args[0], '-iquote', str(orig_start.parent), *args[1:]]
    subprocess.run(cmd, cwd=rec['directory'], check=True)

    dis = subprocess.check_output([OBJDUMP, '--disassemble-symbols=_start_first_cpu', str(start_o)], text=True)
    assert '#0x1c000000' in dis and 'csel\tx24, x6, x24, hi' in dis, f'Clamp not found in start.o disasm:\n{dis}'
    print('      PASS: start.o compiled & clamp verified in _start_first_cpu!')

    print('[1b/4] Compiling power-prep/Tb375fcPower.c (MT6375 charger + MM8013 + disp_pwm0)...')
    pwr_src = BASE / 'power-prep/Tb375fcPower.c'
    pwr_o = KDIR / 'Tb375fcPower.o'
    prec = json.JSONDecoder().raw_decode((XNU_OBJ / 'osfmk/DEVELOPMENT/startup.o.json').read_text())[0]
    pargs = []
    prep_src = prep_out = False
    for tok in prec['arguments']:
        if tok == prec['file']:
            pargs.append(str(pwr_src))
            prep_src = True
        elif tok == prec['output']:
            pargs.append(str(pwr_o))
            prep_out = True
        else:
            pargs.append(tok)
    assert prep_src and prep_out, 'Failed to rewrite startup.o.json arguments'
    subprocess.run(pargs, cwd=prec['directory'], check=True)
    print(f'      PASS: compiled {pwr_o.name} ({pwr_o.stat().st_size} B)')

    patched_disp_a = build_display(KDIR)

    lines = (M7_V4_K / 'link.filelist').read_text().splitlines()
    matches = [i for i, x in enumerate(lines) if pathlib.Path(x).name == 'start.o']
    assert len(matches) == 1, f'Expected 1 start.o in link.filelist, got {len(matches)}'
    lines[matches[0]] = str(start_o)
    lines.append(str(pwr_o))
    filelist = KDIR / 'link.filelist'
    filelist.write_text('\n'.join(lines) + '\n')

    native_macho = KDIR / 'native-cdc-klog.macho'
    link_cmd = json.loads((M7_V4_K / 'link-command.json').read_text())
    link_cmd = [
        x.replace(str(M7_V4_K / 'link.filelist'), str(filelist))
         .replace(str(M7_V4_K / 'native-cdc-klog.macho'), str(native_macho))
         .replace(str(M7_V4_K / 'libNativeLegacyDisplay.a'), str(patched_disp_a))
        for x in link_cmd
    ]
    subprocess.run(link_cmd, cwd=XNU_OBJ, check=True)

    def get_mod_init(p):
        out = subprocess.check_output([OBJDUMP, '-h', str(p)], text=True)
        return [l.split()[2] for l in out.splitlines() if '__mod_init_func' in l and len(l.split()) >= 3]

    ref_mod_init = get_mod_init(M7_V4_K / 'native-cdc-klog.macho')
    mod_init_sizes = get_mod_init(native_macho)
    assert mod_init_sizes == ref_mod_init, f'__mod_init_func mismatch: {mod_init_sizes} != {ref_mod_init}'
    print(f'      PASS: linked {native_macho.name} ({native_macho.stat().st_size} B), __mod_init_func={mod_init_sizes} (matches M7 v4)')

    print('[3/4] Running emit_prelink.py against 26C5035r/kernelcache.decompressed...')
    board_macho = KDIR / 'xnu-cdc-board.macho'
    board_json = KDIR / 'xnu-cdc-board.json'
    board_map = KDIR / 'xnu-cdc-board.map.json'
    diag_macho = KDIR / 'xnu-cdc-board-diag.macho'
    for p in (board_macho, board_json, board_map, diag_macho):
        if p.exists():
            p.unlink()

    prelink_cmd = json.loads((M7_V4_K / 'prelink-command.json').read_text())
    prelink_cmd = [
        x.replace(str(M7_V4_K / 'native-cdc-klog.macho'), str(native_macho))
         .replace(str(M7_V4_K / 'xnu-cdc-board.macho'), str(board_macho))
        for x in prelink_cmd
    ]
    subprocess.run(prelink_cmd, cwd=BASE.parents[1], check=True)

    meta = json.loads(board_json.read_text())
    assert meta['missing'] == {}, f'Prelink missing: {meta["missing"]}'
    assert meta['stubbed'] == {}, f'Prelink stubbed: {meta["stubbed"]}'
    print(f'      PASS: emit_prelink clean (missing=0, stubbed=0, {board_macho.stat().st_size} B)')

    print('[4/4] Applying apply_late_blocks_and_sdt_fixed.py -> xnu-cdc-board-diag.macho...')
    subprocess.run([PY, str(KDIR / 'apply_late_blocks_and_sdt_fixed.py'), str(KDIR)], cwd=BASE.parents[1], check=True)
    assert diag_macho.exists()
    report = {
        'base_kernel': 'M7 v4 + patched start.o (<=448 MiB bootstrap_pagetables clamp)',
        'diag_macho': str(diag_macho),
        'diag_macho_bytes': diag_macho.stat().st_size,
        'diag_macho_sha256': sha256_file(diag_macho),
        'mod_init_func_sizes': mod_init_sizes,
        'prelink_missing': meta['missing'],
        'prelink_stubbed': meta['stubbed'],
    }
    (KDIR / 'kernel-build-report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
