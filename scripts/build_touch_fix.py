#!/Users/imagination/Documents/Unixnu/.tools/venv/bin/python
"""Recompile Nvt36532Protocol.c and Tb375fcTouchProbe.cpp and update libTb375fcTouch_fixed.a."""
import json
from pathlib import Path
import shlex
import subprocess

FIX_DIR = Path(__file__).resolve().parent
OBJS_DIR = FIX_DIR / 'objs'
KDIR = FIX_DIR.parent
RAM_DIR = KDIR.parent
BASE = RAM_DIR.parent
TOUCH_DIR = BASE / 'touch-prep'
TREE = BASE / 'out/tb375fc/macos-27.3-26C5035r/tb375fc/tree'
XNU_OBJ = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837'
LLVM_AR = '/Users/imagination/tools/LLVM-19.1.7-macOS-X64/bin/llvm-ar'


def load_compile_flags(json_path: Path):
    obj = json.loads(json_path.read_text().rstrip(',\n '))
    cmd = obj['arguments'] if 'arguments' in obj else shlex.split(obj['command'])
    compiler = cmd[0]
    flags, i = [], 1
    while i < len(cmd):
        tok = cmd[i]
        if tok in ('-c', '-o') or tok.endswith('.cpp') or tok.endswith('.c'):
            i += 2 if tok in ('-c', '-o') else 1
            continue
        if tok in ('-MF', '-MT', '-MJ'):
            i += 2
            continue
        if tok in ('-MD', '-MMD'):
            i += 1
            continue
        flags.append(tok)
        i += 1
    return compiler, flags, obj.get('directory', str(json_path.parent))


def main():
    c_compiler, c_flags, c_cwd = load_compile_flags(XNU_OBJ / 'bsd/DEVELOPMENT/bsd_init.o.json')
    cxx_compiler, cxx_flags, cxx_cwd = load_compile_flags(XNU_OBJ / 'iokit/DEVELOPMENT/IOPlatformExpert.cpo.json')

    inc_flags = [
        f'-I{FIX_DIR}',
        f'-I{TOUCH_DIR / "include"}',
        f'-I{TOUCH_DIR / "src"}',
        f'-I{TOUCH_DIR / "hid"}',
    ]

    nvt_o = OBJS_DIR / 'Nvt36532Protocol.o'
    probe_o = OBJS_DIR / 'Tb375fcTouchProbe.o'

    subprocess.run([
        c_compiler, *c_flags, *inc_flags, '-Wno-unused-parameter',
        '-c', str(FIX_DIR / 'Nvt36532Protocol.c'), '-o', str(nvt_o),
    ], cwd=c_cwd, check=True)
    print(f'PASS: compiled {nvt_o} ({nvt_o.stat().st_size:,} B)')

    subprocess.run([
        cxx_compiler, *cxx_flags, *inc_flags, '-Wno-unused-parameter',
        '-c', str(FIX_DIR / 'Tb375fcTouchProbe.cpp'), '-o', str(probe_o),
    ], cwd=cxx_cwd, check=True)
    print(f'PASS: compiled {probe_o} ({probe_o.stat().st_size:,} B)')

    archive = FIX_DIR / 'libTb375fcTouch_fixed.a'
    members = [
        'Mt6897Spi.o',
        'Nvt36532Protocol.o',
        'Tb375fcDispProbe.o',
        'Tb375fcMaliProbe.o',
        'Tb375fcTouchHid.late.o',
        'Tb375fcTouchProbe.o',
        'hid-adapters.o',
        'hid-bindings.o',
    ]
    paths = [OBJS_DIR / m for m in members]
    for p in paths:
        assert p.is_file(), f'Missing {p}'

    stage = FIX_DIR / 'libTb375fcTouch_fixed.pending.a'
    stage.unlink(missing_ok=True)
    subprocess.run([LLVM_AR, 'rcs', str(stage), *map(str, paths)], check=True)
    stage.replace(archive)
    print(f'PASS: updated {archive} ({archive.stat().st_size:,} B)')


if __name__ == '__main__':
    main()
