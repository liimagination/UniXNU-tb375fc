#!/usr/bin/env python3
"""Build TB375FC In-Process arm64e NativeAdrenoMetal.bundle (m2v 87a869f + PAC rewrite + Mesa PanVK)."""
import json
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import sys

PLUGIN_DIR = Path(__file__).resolve().parent
RAM_DIR = PLUGIN_DIR.parent
BASE = RAM_DIR.parent
WORKSPACE = BASE.parents[1]
UPSTREAM = BASE / 'upstream'

ADRENO_SRC = PLUGIN_DIR / 'adreno-metal-src'
BRIDGE_A = RAM_DIR / 'gpu-diag/bridge-v2/target/arm64e-apple-darwin/release/libadreno_metal_bridge.a'
PANVK_DYLIB = RAM_DIR / 'mali-panvk/build-arm64e/src/panfrost/vulkan/libvulkan_panfrost.dylib'
VULKAN_INCLUDE = RAM_DIR / 'mali-panvk/mesa-src/include'
OBJCOPY = WORKSPACE / '.tools/clang21/bin/llvm-objcopy'


def prepare_frontend() -> Path:
    src = (ADRENO_SRC / 'NativeBridgeMetalFrontend.mm').read_text()
    # Convert 1x-sample MTLResourceStorageModeMemoryless -> MTLResourceStorageModePrivate unconditionally
    # so iconservicesagent / RenderBox / ControlCenter 1x memoryless attachments never fail on m2v!
    old_blk = (
        ' if(singleSample() && out->samples>1) {\n'
        '  out->samples=1;\n'
        '  if(out->type==MTLTextureType2DMultisample)out->type=MTLTextureType2D;\n'
        '  if(out->type==MTLTextureType2DMultisampleArray)out->type=MTLTextureType2DArray;\n'
        '  // A preserved attachment may be copied to its resolve target after the pass.\n'
        '  if((out->options&MTLResourceStorageModeMask)==MTLResourceStorageModeMemoryless)\n'
        '   out->options=(out->options&~MTLResourceStorageModeMask)|MTLResourceStorageModePrivate;\n'
        ' }'
    )
    new_blk = (
        ' if(singleSample() && out->samples>1) {\n'
        '  out->samples=1;\n'
        '  if(out->type==MTLTextureType2DMultisample)out->type=MTLTextureType2D;\n'
        '  if(out->type==MTLTextureType2DMultisampleArray)out->type=MTLTextureType2DArray;\n'
        ' }\n'
        ' if((out->options&MTLResourceStorageModeMask)==MTLResourceStorageModeMemoryless)\n'
        '  out->options=(out->options&~MTLResourceStorageModeMask)|MTLResourceStorageModePrivate;'
    )
    assert src.count(old_blk) == 1, 'Expected exact singleSample() block in NativeBridgeMetalFrontend.mm'
    src = src.replace(old_blk, new_blk, 1)
    # Chunked 1 MiB pread() fallback for newLibraryWithFile: in case NSDataReadingMappedIfSafe hits decmpfs ubc_upl_map kr=0x3
    old_lib = (
        '- (id<MTLLibrary>)newLibraryWithFile:(NSString *)path error:(NSError **)error {\n'
        ' NSData *bytes=path?[NSData dataWithContentsOfFile:path options:NSDataReadingMappedIfSafe error:error]:nil;'
    )
    new_lib = (
        'static NSData *nbReadLibraryFile(NSString *path, NSError **error) {\n'
        ' NSData *bytes=path?[NSData dataWithContentsOfFile:path options:NSDataReadingMappedIfSafe error:error]:nil;\n'
        ' if(bytes && bytes.length>0)return bytes;\n'
        ' if(!path)return nil;\n'
        ' const int fd=open(path.fileSystemRepresentation,O_RDONLY);\n'
        ' if(fd<0)return nil;\n'
        ' struct stat st={};\n'
        ' if(fstat(fd,&st)!=0 || st.st_size<=0){close(fd);return nil;}\n'
        ' NSMutableData *buf=[NSMutableData dataWithLength:(NSUInteger)st.st_size];\n'
        ' if(!buf){close(fd);return nil;}\n'
        ' uint8_t *dst=(uint8_t *)buf.mutableBytes;\n'
        ' off_t off=0;\n'
        ' while(off<st.st_size){\n'
        '  size_t want=(size_t)MIN((off_t)(1<<20),st.st_size-off);\n'
        '  ssize_t got=pread(fd,dst+off,want,off);\n'
        '  if(got<=0){close(fd);return nil;}\n'
        '  off+=got;\n'
        ' }\n'
        ' close(fd);\n'
        ' if(error)*error=nil;\n'
        ' return buf;\n'
        '}\n'
        '- (id<MTLLibrary>)newLibraryWithFile:(NSString *)path error:(NSError **)error {\n'
        ' NSData *bytes=nbReadLibraryFile(path,error);'
    )
    assert src.count(old_lib) == 1, 'Expected exact newLibraryWithFile: in NativeBridgeMetalFrontend.mm'
    src = src.replace(old_lib, new_lib, 1)
    dst = PLUGIN_DIR / 'NativeBridgeMetalFrontend_patched.mm'
    dst.write_text(src)
    return dst


def main():
    assert BRIDGE_A.is_file(), f'Missing {BRIDGE_A}'
    assert PANVK_DYLIB.is_file(), f'Missing {PANVK_DYLIB}'
    assert OBJCOPY.is_file(), f'Missing {OBJCOPY}'

    frontend_mm = prepare_frontend()
    plugin_mm = PLUGIN_DIR / 'NativeAdrenoMetalPlugin.mm'

    out_dir = PLUGIN_DIR / 'build'
    if out_dir.exists():
        shutil.rmtree(out_dir)
    bundle = out_dir / 'NativeAdrenoMetal.bundle'
    contents = bundle / 'Contents'
    macos_dir = contents / 'MacOS'
    res_dir = contents / 'Resources'
    macos_dir.mkdir(parents=True)
    res_dir.mkdir(parents=True)

    sdk = subprocess.check_output(['xcrun', '--sdk', 'macosx', '--show-sdk-path'], text=True).strip()

    # 1. Rename callback-taking C imports in Rust staticlib to PAC-signing wrappers in RustPacBoundary.c
    pac_renames = {
        '_pthread_create': '_nmb_pac_pthread_create',
        '__tlv_atexit': '_nmb_pac_tlv_atexit',
        '_atexit': '_nmb_pac_atexit',
        '___cxa_atexit': '_nmb_pac_cxa_atexit',
        '__dyld_register_func_for_add_image': '_nmb_pac_dyld_register_func_for_add_image',
        '__Unwind_Backtrace': '_nmb_pac_Unwind_Backtrace',
        '_dlsym': '_nmb_pac_dlsym',
    }
    pac_bridge = out_dir / 'libadreno_metal_bridge_pac.a'
    subprocess.run([
        str(OBJCOPY),
        *[f'--redefine-sym={a}={b}' for a, b in pac_renames.items()],
        str(BRIDGE_A), str(pac_bridge),
    ], check=True)

    # 2. Compile RustPacBoundary.c as arm64e
    boundary_o = out_dir / 'RustPacBoundary.o'
    subprocess.run([
        'xcrun', 'clang', '-target', 'arm64e-apple-macos15.0', '-isysroot', sdk,
        '-O2', '-g', '-std=c11', '-fstack-protector-strong',
        f'-I{VULKAN_INCLUDE}',
        '-c', str(ADRENO_SRC / 'bridge/RustPacBoundary.c'),
        '-o', str(boundary_o),
    ], check=True)

    # 3. Link arm64e NativeAdrenoMetal bundle executable with link map for rust-pac-rewrite.py
    exe = macos_dir / 'NativeAdrenoMetal'
    link_map = out_dir / 'NativeAdrenoMetal.map'
    subprocess.run([
        'xcrun', 'clang++', '-target', 'arm64e-apple-macos15.0', '-isysroot', sdk,
        '-O2', '-DNDEBUG', '-std=gnu++17', '-fobjc-arc', '-fstack-protector-strong',
        '-Wno-deprecated-declarations', '-Wno-unused-parameter',
        '-DNATIVE_METAL_BUILD_ID="tb375fc-mali-panvk-inproc-87a869f"',
        '-bundle',
        f'-I{ADRENO_SRC}', f'-I{ADRENO_SRC / "bridge"}', f'-I{VULKAN_INCLUDE}',
        '-x', 'objective-c++', str(plugin_mm), str(frontend_mm),
        '-x', 'none', str(boundary_o), str(pac_bridge),
        f'-Wl,-map,{link_map}',
        '-Wl,-dead_strip',
        '-lobjc',
        '-framework', 'Foundation', '-framework', 'Metal', '-framework', 'IOSurface',
        '-framework', 'IOKit', '-framework', 'CoreFoundation',
        '-F', f'{sdk}/System/Library/PrivateFrameworks', '-framework', 'IOGPU',
        '-o', str(exe),
    ], check=True)

    # 4. Rewrite Rust indirect blr/br instructions through nmb_pac_call_x0..29 trampolines
    subprocess.run([
        sys.executable, str(UPSTREAM / 'scripts/rust-pac-rewrite.py'),
        str(exe), str(link_map), 'libadreno_metal_bridge_pac.a',
    ], check=True)
    subprocess.run(['strip', '-x', str(exe)], check=True)

    # 5. Stage arm64e libvulkan_panfrost.dylib inside Contents/MacOS/, self-bind weak imports, and 4K-sign
    panvk_dst = macos_dir / 'libvulkan_panfrost.dylib'
    shutil.copyfile(PANVK_DYLIB, panvk_dst)
    subprocess.run(['strip', '-x', str(panvk_dst)], check=True)
    subprocess.run([
        sys.executable, str(UPSTREAM / 'scripts/macho-self-bind-weak-imports.py'),
        str(panvk_dst),
    ], check=True)
    subprocess.run([
        'codesign', '--force', '--sign', '-', '--pagesize', '4096',
        '--identifier', 'org.tb375fc.panvk', str(panvk_dst),
    ], check=True)
    subprocess.run(['codesign', '--verify', '--strict', str(panvk_dst)], check=True)
    panvk_detail = subprocess.run(['codesign', '--display', '--verbose=4', str(panvk_dst)],
                                  capture_output=True, text=True, check=True).stderr
    panvk_cdhash = re.search(r'^CDHash=([a-f0-9]{40})$', panvk_detail, re.M)[1]
    assert re.search(r'^Page size=4096$', panvk_detail, re.M)

    # 6. Write Info.plist & gpu-build.json and 4K-sign NativeAdrenoMetal.bundle
    (res_dir / 'gpu-build.json').write_text(json.dumps({
        'runtime': 'm2v-87a869f-inproc',
        'driver': 'mesa-panvk-arm64e',
    }, indent=2) + '\n')
    (contents / 'Info.plist').write_bytes(plistlib.dumps(dict(
        CFBundleIdentifier='org.blu-sharky.NativeAdrenoMetal',
        CFBundleExecutable='NativeAdrenoMetal',
        CFBundlePackageType='BNDL',
        CFBundleVersion='1',
        CFBundleShortVersionString='1',
        NSPrincipalClass='NativeAdrenoMetalDevice',
        LSMinimumSystemVersion='15.0',
    ), fmt=plistlib.FMT_XML))

    subprocess.run([
        'codesign', '--force', '--sign', '-', '--pagesize', '4096',
        '--identifier', 'org.blu-sharky.NativeAdrenoMetal', str(bundle),
    ], check=True)
    subprocess.run(['codesign', '--verify', '--strict', str(bundle)], check=True)
    bundle_detail = subprocess.run(['codesign', '--display', '--verbose=4', str(bundle)],
                                   capture_output=True, text=True, check=True).stderr
    bundle_cdhash = re.search(r'^CDHash=([a-f0-9]{40})$', bundle_detail, re.M)[1]
    assert re.search(r'^Page size=4096$', bundle_detail, re.M)

    # Verify arm64e headers on both Mach-Os
    for bin_p in (exe, panvk_dst):
        hdr = bin_p.read_bytes()[:16]
        magic = int.from_bytes(hdr[0:4], 'little')
        cputype = int.from_bytes(hdr[4:8], 'little')
        subtype = int.from_bytes(hdr[8:12], 'little') & 0xffffff
        assert (magic, cputype, subtype) == (0xfeedfacf, 0x100000c, 2), f'Not arm64e: {bin_p}'

    # Clean intermediate pac_bridge & map to save disk space, keep bundle + trust.json
    pac_bridge.unlink(missing_ok=True)
    link_map.unlink(missing_ok=True)
    boundary_o.unlink(missing_ok=True)

    trust = {
        'bundle_cdhash': bundle_cdhash,
        'panvk_cdhash': panvk_cdhash,
        'exe_bytes': exe.stat().st_size,
        'panvk_bytes': panvk_dst.stat().st_size,
    }
    (out_dir / 'trust.json').write_text(json.dumps(trust, indent=2) + '\n')
    print(subprocess.check_output(['otool', '-L', str(exe)], text=True))
    print(subprocess.check_output(['otool', '-L', str(panvk_dst)], text=True))
    print('PASS:', json.dumps(trust, indent=2))


if __name__ == '__main__':
    main()
