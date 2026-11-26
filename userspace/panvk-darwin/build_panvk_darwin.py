#!/usr/bin/env python3
"""Cross-compile Mesa panvk (Mali-G615 Valhall v10 CSF) for arm64-apple-macos."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

BASE = Path(__file__).resolve().parent
SOURCE = BASE / "mesa-src"
OUT = BASE / "build-arm64"
VENV_BIN = BASE / "mesa-venv/bin"
HOST_TOOLS_BIN = BASE / "host-tools/bin"
HOST_CLC_BIN = BASE / "build-host-clc"

def read(cmd):
    return subprocess.check_output([str(c) for c in cmd], text=True).strip()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--configure-only", action="store_true")
    ap.add_argument("--arm64e", action="store_true", help="Build arm64e-apple-macos15.0 for in-process macOS 27 Metal plugin")
    args = ap.parse_args()

    out_dir = BASE / ("build-arm64e" if args.arm64e else "build-arm64")
    sdk = read(["xcrun", "--sdk", "macosx", "--show-sdk-path"])
    find = lambda tool: read(["xcrun", "--sdk", "macosx", "--find", tool])
    triple = "arm64e-apple-macos15.0" if args.arm64e else "arm64-apple-macos14.0"
    flags = ["-target", triple, "-isysroot", sdk, "-fstack-protector-strong", "-DPANTHOR_DARWIN=1"]

    cross = BASE / ("mesa-panvk-arm64e.cross" if args.arm64e else "mesa-panvk-arm64.cross")
    cross.write_text(
        "[binaries]\n"
        f"c = {find('clang')!r}\n"
        f"cpp = {find('clang++')!r}\n"
        f"objc = {find('clang')!r}\n"
        f"ar = {find('ar')!r}\n"
        f"strip = {find('strip')!r}\n"
        "pkg-config = '/usr/bin/false'\n"
        "[host_machine]\n"
        "system = 'darwin'\n"
        "cpu_family = 'aarch64'\n"
        "cpu = 'arm64'\n"
        "endian = 'little'\n"
        "[properties]\n"
        "needs_exe_wrapper = true\n"
        "[built-in options]\n"
        + "".join(
            f"{k} = {flags!r}\n"
            for k in ("c_args", "cpp_args", "objc_args", "c_link_args", "cpp_link_args", "objc_link_args")
        )
    )

    # Put host mesa_clc, vtn_bindgen2, panfrost_compile, glslangValidator, meson, ninja on PATH
    env = os.environ.copy()
    env["PATH"] = ":".join([
        str(HOST_CLC_BIN / "src/compiler/clc"),
        str(HOST_CLC_BIN / "src/compiler/spirv"),
        str(HOST_CLC_BIN / "src/panfrost/clc"),
        str(HOST_TOOLS_BIN),
        str(VENV_BIN),
        env.get("PATH", ""),
    ])
    env.update(CC=find("clang"), CXX=find("clang++"), CFLAGS="", CXXFLAGS="", CPPFLAGS="", LDFLAGS="")

    options = [
        "-Dgallium-drivers=",
        "-Dvulkan-drivers=panfrost",
        "-Dplatforms=macos",
        "-Dmesa-clc=system",
        "-Dprecomp-compiler=system",
        "-Dglx=disabled",
        "-Degl=disabled",
        "-Dgbm=disabled",
        "-Dopengl=false",
        "-Dgles1=disabled",
        "-Dgles2=disabled",
        "-Dllvm=disabled",
        "-Dxmlconfig=disabled",
        "-Dshader-cache=enabled",
        "-Dtools=",
        "-Dbuild-tests=false",
        "-Dvideo-codecs=",
        "-Dvulkan-layers=",
        "-Dlibunwind=disabled",
        "-Dlmsensors=disabled",
        "-Dvalgrind=disabled",
        "-Dexpat=disabled",
        "-Dzstd=disabled",
        "-Dbuildtype=release",
        "-Db_ndebug=true",
    ]

    out_dir.mkdir(parents=True, exist_ok=True)
    meson = VENV_BIN / "meson"
    if not (out_dir / "build.ninja").exists():
        cmd = [meson, "setup", out_dir, SOURCE, "--wrap-mode=nodownload", "--cross-file", cross] + options
    else:
        cmd = [meson, "configure", out_dir] + options

    print("Running:", " ".join(str(x) for x in cmd), flush=True)
    r = subprocess.run([str(x) for x in cmd], env=env)
    if r.returncode != 0:
        sys.exit(r.returncode)
    if args.configure_only:
        print("PASS: panvk configure OK")
        return

    r = subprocess.run([str(VENV_BIN / "ninja"), "-C", str(out_dir), "-j6", "src/panfrost/vulkan/libvulkan_panfrost.dylib"], env=env)
    if r.returncode != 0:
        sys.exit(r.returncode)

    lib = out_dir / "src/panfrost/vulkan/libvulkan_panfrost.dylib"
    print(f"PASS: built {lib} ({lib.stat().st_size} bytes)")
    syms = subprocess.check_output(["nm", "-gU", str(lib)], text=True)
    for s in ("_vk_icdGetInstanceProcAddr", "_vkGetInstanceProcAddr"):
        print(f"  symbol {s}: {'OK' if s in syms else 'MISSING!'}")

if __name__ == "__main__":
    main()
