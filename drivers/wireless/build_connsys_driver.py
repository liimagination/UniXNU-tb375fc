#!/Users/imagination/Documents/Unixnu/.tools/venv/bin/python
"""Compile Tb375fcConnSys.cpp (arm64e kernel object) for TB375FC XNU."""
import json
from pathlib import Path
import subprocess

CONNSYS_DIR = Path(__file__).resolve().parent
KDIR = CONNSYS_DIR.parent
RAM_DIR = KDIR.parent
BASE = RAM_DIR.parent
TREE = BASE / 'out/tb375fc/macos-27.3-26C5035r/tb375fc/tree'
XNU_OBJ = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837'


def main():
    crec = json.JSONDecoder().raw_decode(
        (XNU_OBJ / 'iokit/DEVELOPMENT/IOPlatformExpert.cpo.json').read_text()
    )[0]
    src = CONNSYS_DIR / 'Tb375fcConnSys.cpp'
    obj = CONNSYS_DIR / 'Tb375fcConnSys.o'
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


if __name__ == '__main__':
    main()
