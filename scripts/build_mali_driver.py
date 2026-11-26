#!/Users/imagination/Documents/Unixnu/.tools/venv/bin/python
"""Compile Tb375fcMaliDriver.cpp (arm64e kernel object) and update libTb375fcTouch_mali4.a."""
import json
from pathlib import Path
import shutil
import subprocess

MALI_DIR = Path(__file__).resolve().parent
KDIR = MALI_DIR.parent
RAM_DIR = KDIR.parent
BASE = RAM_DIR.parent
TREE = BASE / 'out/tb375fc/macos-27.3-26C5035r/tb375fc/tree'
XNU_OBJ = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837'
LLVM_AR = '/Users/imagination/tools/LLVM-19.1.7-macOS-X64/bin/llvm-ar'


def main():
    crec = json.JSONDecoder().raw_decode((XNU_OBJ / 'iokit/DEVELOPMENT/IOPlatformExpert.cpo.json').read_text())[0]
    src = MALI_DIR / 'Tb375fcMaliDriver.cpp'
    obj = MALI_DIR / 'Tb375fcMaliDriver.o'
    cargs = []
    for tok in crec['arguments']:
        if tok == crec['file']:
            cargs.append(str(src))
        elif tok == crec['output']:
            cargs.append(str(obj))
        else:
            cargs.append(tok)
    cargs.append('-Wno-unused-parameter')
    subprocess.run(cargs, cwd=crec['directory'], check=True)
    print(f'PASS: compiled {obj} ({obj.stat().st_size:,} B)')

    base_a = KDIR / 'touch-fix/libTb375fcTouch_fixed.a'
    mali_a = MALI_DIR / 'libTb375fcTouch_mali4.a'
    assert base_a.is_file(), f'Missing {base_a}'
    shutil.copyfile(base_a, mali_a)
    subprocess.run([LLVM_AR, 'd', str(mali_a), 'Tb375fcMaliProbe.o'], check=True)
    subprocess.run([LLVM_AR, 'rcs', str(mali_a), str(obj)], check=True)
    print(f'PASS: updated {mali_a} ({mali_a.stat().st_size:,} B)')


if __name__ == '__main__':
    main()
