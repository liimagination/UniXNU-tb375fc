#!/Users/imagination/Documents/Unixnu/.tools/venv/bin/python
"""Build Full macOS 27.3 Desktop Tier 3 (7.03 GiB RAM + MT6375 Charging + 1472x920 Display + Isolated gBgDevice).

Strictly isolated in local-artifacts/macos27-firstboot/ram-expansion/desktop/.
Does NOT modify postmmu-diag/m5-gui/desktop-root/ or m7-metal/.
"""
import argparse
import hashlib
import importlib.util
import json
import lzma
import os
from pathlib import Path
import re
import runpy
import shutil
import struct
import subprocess
import sys
import tempfile

DESKTOP_DIR = Path(__file__).resolve().parent
RAM_DIR = DESKTOP_DIR.parent
BASE = RAM_DIR.parent
WORKSPACE = BASE.parents[1]
MTOOLS = WORKSPACE / '.tools/bin'
M5_GUI = BASE / 'postmmu-diag/m5-gui'
DROOT = M5_GUI / 'desktop-root'
M7_DIR = M5_GUI / 'm7-metal'
V4 = M7_DIR / 'kernel-out-v4'
PACK_M7 = M7_DIR / 'pack'
UPSTREAM = BASE / 'upstream'

DARWINBOOT_EFI = RAM_DIR / 'uefi/build/DarwinBoot/RELEASE_CLANGPDB/AARCH64/DarwinBoot.efi'
DIAG_DAEMON = RAM_DIR / 'gpu-diag/tb375fc-gpu-daemon'
TIER3_KDIR = RAM_DIR / 'kernel'
PACK_DIR = RAM_DIR / 'pack'

WIN_SIZE = 0x1E9600000  # Tier 3: full 8 GB DRAM (~7.03 GiB usable)
BOOT = '::/EFI/UniXNU/'
ARGS_PREFIX = '#define XNULZ_KERNEL_ARGS    L"'


def imported(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def tool(name, image, *args):
    subprocess.run([str(MTOOLS / name), '-i', str(image), *map(str, args)], check=True)


def signed_cdhash(path: Path) -> str:
    subprocess.run(['codesign', '--verify', '--strict', str(path)], check=True)
    out = subprocess.check_output(['codesign', '-dvvv', str(path)], stderr=subprocess.STDOUT, text=True)
    m = re.search(r'^CDHash=([0-9a-fA-F]{40})$', out, re.M)
    assert m, f'CDHash not found for {path}:\n{out}'
    return m.group(1).lower()


def lzma_alone(raw: bytes, dict_size: int, nice_len: int = 273) -> bytes:
    lc, lp, pb = 2, 2, 2
    opts = [{
        'id': lzma.FILTER_LZMA1,
        'dict_size': dict_size,
        'lc': lc, 'lp': lp, 'pb': pb,
        'mode': lzma.MODE_NORMAL,
        'nice_len': nice_len,
        'mf': lzma.MF_BT4,
    }]
    packed = lzma.compress(raw, format=lzma.FORMAT_ALONE, filters=opts)
    packed = packed[:5] + struct.pack('<Q', len(raw)) + packed[13:]
    assert packed[0] == (pb * 5 + lp) * 9 + lc
    assert struct.unpack_from('<Q', packed, 5)[0] == len(raw)
    assert lzma.decompress(packed, format=lzma.FORMAT_ALONE) == raw
    return packed


def build_kernel(output: Path, rebuild_display: bool, touch_stage1: bool = False, persist_storage: bool = False, mali_driver: bool = False) -> Path:
    kout = output / 'kernel-out'
    kout.mkdir(parents=True, exist_ok=True)
    diag = kout / 'xnu-cdc-board-diag.macho'
    if not rebuild_display and not touch_stage1 and not persist_storage and diag.is_file() and diag.stat().st_size == 142868480:
        print(f'[1/5] Reusing prelinked Full Desktop Tier 3 kernel: {diag}', flush=True)
        return diag

    print('[1/5] Linking Full Desktop Tier 3 kernel (7.03 GiB + Tb375fcPower + 1472x920 + DesktopRoot + IOKitBSDInit + tzinit dirs)...', flush=True)
    v4_ns = runpy.run_path(str(M7_DIR / 'kernel/build_m7_kernel.py'))
    builder, m4 = v4_ns['builder'], v4_ns['m4']
    uuids = {}
    for role in ('OS', 'SystemOS', 'App'):
        uuids[role] = json.loads((DROOT / 'policies' / role / 'verification.json').read_text())['volume_uuid']

    # Compile patched kern_exec.o: create tz/log/run/tmp dirs, write pre-launchd disabled.plist
    # (watchdogd/fairplayd/CommCenter — NOT clocksyncd, which now runs with our virtual en0!),
    # and late-init Tb375fcDummyEth so IONetworkingFamily publishes en0 -> IOTimeSyncClockManager -> WindowServer.
    kexec_rec = json.JSONDecoder().raw_decode((builder['XNU_OBJ'] / 'bsd/DEVELOPMENT/kern_exec.o.json').read_text())[0]
    kexec_src_txt = (DROOT / 'desktop-kernel/kern_exec.c').read_text()

    old_shim = '\tif (!error) native_owner_macos_boot_shims(p, scratch);'
    new_shim = (
        '\tif (!error) native_owner_macos_boot_shims(p, scratch);\n'
        '\tif (!error) {\n'
        '\t\textern void tb375fc_dummy_eth_probe(void);\n'
        '\t\ttb375fc_dummy_eth_probe();\n'
        '\t}\n'
        '\tif (!error) (void)native_gpu_addon_parents(p, scratch, "/private/var/db/timezone/tz/");\n'
        '\tif (!error) (void)native_gpu_addon_parents(p, scratch, "/private/var/db/icutz/");\n'
        '\tif (!error) (void)native_gpu_addon_parents(p, scratch, "/private/var/log/");\n'
        '\tif (!error) (void)native_gpu_addon_parents(p, scratch, "/private/var/run/");\n'
        '\tif (!error) (void)native_gpu_addon_parents(p, scratch, "/private/var/tmp/");\n'
        '\tif (!error) {\n'
        '\t\tvfs_context_t wctx = vfs_context_current();\n'
        '\t\tstatic const char wd_plist[] =\n'
        '\t\t    "<?xml version=\\"1.0\\" encoding=\\"UTF-8\\"?>\\n"\n'
        '\t\t    "<!DOCTYPE plist PUBLIC \\"-//Apple//DTD PLIST 1.0//EN\\" \\"http://www.apple.com/DTDs/PropertyList-1.0.dtd\\">\\n"\n'
        '\t\t    "<plist version=\\"1.0\\"><dict>"\n'
        '\t\t    "<key>com.apple.watchdogd</key><true/>"\n'
        '\t\t    "<key>com.apple.fairplayd.A2</key><true/>"\n'
        '\t\t    "<key>com.apple.accessoryupdaterd</key><true/>"\n'
        '\t\t    "<key>com.apple.identityservicesd</key><true/>"\n'
        '\t\t    "<key>com.apple.CommCenter</key><true/>"\n'
        '\t\t    "<key>com.apple.metadata.mds</key><true/>"\n'
        '\t\t    "<key>com.apple.metadata.mds.index</key><true/>"\n'
        '\t\t    "<key>com.apple.metadata.mds.scan</key><true/>"\n'
        '\t\t    "<key>com.apple.mobileassetd</key><true/>"\n'
        '\t\t    "<key>com.apple.cloudd</key><true/>"\n'
        '\t\t    "<key>com.apple.appstored</key><true/>"\n'
        '\t\t    "<key>com.apple.backupd</key><true/>"\n'
        '\t\t    "<key>com.apple.backupd-helper</key><true/>"\n'
        '\t\t    "<key>com.apple.softwareupdated</key><true/>"\n'
        '\t\t    "<key>com.apple.triald.system</key><true/>"\n'
        '\t\t    "<key>com.apple.searchpartyd</key><true/>"\n'
        '\t\t    "<key>com.apple.biomed</key><true/>"\n'
        '\t\t    "<key>com.apple.aned</key><true/>"\n'
        '\t\t    "<key>com.apple.aneuserd</key><true/>"\n'
        '\t\t    "<key>com.apple.audio.audiomxd</key><true/>"\n'
        '\t\t    "<key>com.apple.audio.coreaudiod</key><true/>"\n'
        '\t\t    "<key>com.apple.tailspind</key><true/>"\n'
        '\t\t    "<key>com.apple.ReportCrash.Root</key><true/>"\n'
        '\t\t    "<key>com.apple.ReportCrash</key><true/>"\n'
        '\t\t    "<key>com.apple.ReportCrashService</key><true/>"\n'
        '\t\t    "<key>com.apple.SubmitDiagInfo</key><true/>"\n'
        '\t\t    "<key>com.apple.osanalyticshelper</key><true/>"\n'
        '\t\t    "<key>com.apple.spindump</key><true/>"\n'
        '\t\t    "<key>com.apple.systemstats.analysis</key><true/>"\n'
        '\t\t    "<key>com.apple.symptomsd-diag</key><true/>"\n'
        '\t\t    "<key>com.apple.analyticsd</key><true/>"\n'
        '\t\t    "<key>com.apple.coreduetd</key><true/>"\n'
        '\t\t    "<key>com.apple.PerfPowerServices</key><true/>"\n'
        '\t\t    "<key>com.apple.PerfPowerTelemetryClientRegistrationService</key><true/>"\n'
        '\t\t    "<key>com.apple.findmybeaconingd</key><true/>"\n'
        '\t\t    "<key>com.apple.findmydeviced</key><true/>"\n'
        '\t\t    "<key>com.apple.geod</key><true/>"\n'
        '\t\t    "<key>com.apple.airportd</key><true/>"\n'
        '\t\t    "<key>com.apple.bluetoothd</key><true/>"\n'
        '\t\t    "<key>com.apple.mediaremoted</key><true/>"\n'
        '\t\t    "<key>com.apple.uarpd</key><true/>"\n'
        '\t\t    "<key>com.apple.uarpassetmanagerd</key><true/>"\n'
        '\t\t    "<key>com.apple.XProtect.daemon.scan</key><true/>"\n'
        '\t\t    "<key>com.apple.modelcatalogd</key><true/>"\n'
        '\t\t    "</dict></plist>\\n";\n'
        '\t\tstatic const char user_disabled_plist[] =\n'
        '\t\t    "<?xml version=\\"1.0\\" encoding=\\"UTF-8\\"?>\\n"\n'
        '\t\t    "<!DOCTYPE plist PUBLIC \\"-//Apple//DTD PLIST 1.0//EN\\" \\"http://www.apple.com/DTDs/PropertyList-1.0.dtd\\">\\n"\n'
        '\t\t    "<plist version=\\"1.0\\"><dict>"\n'
        '\t\t    "<key>com.apple.spotlightknowledged</key><true/>"\n'
        '\t\t    "<key>com.apple.spotlightknowledged.updater</key><true/>"\n'
        '\t\t    "<key>com.apple.corespotlightd</key><true/>"\n'
        '\t\t    "<key>com.apple.parsecd</key><true/>"\n'
        '\t\t    "<key>com.apple.Siri.agent</key><true/>"\n'
        '\t\t    "<key>com.apple.siriactionsd</key><true/>"\n'
        '\t\t    "<key>com.apple.siriknowledged</key><true/>"\n'
        '\t\t    "<key>com.apple.assistantd</key><true/>"\n'
        '\t\t    "<key>com.apple.sirittsd</key><true/>"\n'
        '\t\t    "<key>com.apple.photolibraryd</key><true/>"\n'
        '\t\t    "<key>com.apple.photoanalysisd</key><true/>"\n'
        '\t\t    "<key>com.apple.mediaanalysisd</key><true/>"\n'
        '\t\t    "<key>com.apple.itunescloudd</key><true/>"\n'
        '\t\t    "<key>com.apple.cloudd</key><true/>"\n'
        '\t\t    "<key>com.apple.bird</key><true/>"\n'
        '\t\t    "<key>com.apple.routined</key><true/>"\n'
        '\t\t    "<key>com.apple.suggestd</key><true/>"\n'
        '\t\t    "<key>com.apple.duetexpertd</key><true/>"\n'
        '\t\t    "<key>com.apple.knowledge-agent</key><true/>"\n'
        '\t\t    "<key>com.apple.chronod</key><true/>"\n'
        '\t\t    "<key>com.apple.ScreenTimeAgent</key><true/>"\n'
        '\t\t    "<key>com.apple.triald</key><true/>"\n'
        '\t\t    "<key>com.apple.wallpaper.export</key><true/>"\n'
        '\t\t    "<key>com.apple.contactsd</key><true/>"\n'
        '\t\t    "<key>com.apple.contactsdonationagent</key><true/>"\n'
        '\t\t    "<key>com.apple.calaccessd</key><true/>"\n'
        '\t\t    "<key>com.apple.remindd</key><true/>"\n'
        '\t\t    "<key>com.apple.syncdefaultsd</key><true/>"\n'
        '\t\t    "<key>com.apple.imagent</key><true/>"\n'
        '\t\t    "<key>com.apple.sharingd</key><true/>"\n'
        '\t\t    "<key>com.apple.rapportd</key><true/>"\n'
        '\t\t    "<key>com.apple.nearbyd</key><true/>"\n'
        '\t\t    "<key>com.apple.familycircled</key><true/>"\n'
        '\t\t    "<key>com.apple.avconferenced</key><true/>"\n'
        '\t\t    "<key>com.apple.deleted</key><true/>"\n'
        '\t\t    "<key>com.apple.UsageTrackingAgent</key><true/>"\n'
        '\t\t    "<key>com.apple.weatherd</key><true/>"\n'
        '\t\t    "<key>com.apple.newsd</key><true/>"\n'
        '\t\t    "</dict></plist>\\n";\n'
        '\t\t(void)native_gpu_addon_parents(p, scratch, "/private/var/db/com.apple.xpc.launchd/disabled.plist");\n'
        '\t\tint werr1 = native_gpu_addon_write("/private/var/db/com.apple.xpc.launchd/disabled.plist",\n'
        '\t\t    (const uint8_t *)wd_plist, sizeof(wd_plist) - 1, 0644, wctx);\n'
        '\t\tint werr2 = native_gpu_addon_write("/private/var/db/com.apple.xpc.launchd/disabled.migrated",\n'
        '\t\t    (const uint8_t *)"", 0, 0644, wctx);\n'
        '\t\tint werr3 = native_gpu_addon_write("/private/var/db/com.apple.xpc.launchd/disabled.501.plist",\n'
        '\t\t    (const uint8_t *)user_disabled_plist, sizeof(user_disabled_plist) - 1, 0644, wctx);\n'
        '\t\tprintf("tb375fc: pre-launchd disabled.plist=%d migrated=%d disabled.501=%d\\n", werr1, werr2, werr3);\n'
        '\t}'
    )
    if touch_stage1:
        new_shim += (
            '\n\tif (!error) {\n'
            '\t\tuint32_t touch_stage = 0;\n'
            '\t\textern void tb375fc_touch_probe(void);\n'
            '\t\tif (PE_parse_boot_argn("touch", &touch_stage, sizeof(touch_stage)) && touch_stage >= 1 && touch_stage <= 4) {\n'
            '\t\t\ttb375fc_touch_probe();\n'
            '\t\t}\n'
            '\t}'
        )
    assert old_shim in kexec_src_txt, 'Expected native_owner_macos_boot_shims in kern_exec.c'
    kexec_src = kout / 'kern_exec.c'
    kexec_src.write_text(kexec_src_txt.replace(old_shim, new_shim, 1))
    kexec_obj = kout / 'kern_exec.o'
    kexec_args = builder['build_cmd_from_record'](
        kexec_rec, kexec_src, kexec_obj,
        [
            '-UXNU_OWNER_DATA_MOUNT', '-DXNU_OWNER_DATA_MOUNT=1',
            '-UXNU_OWNER_BOOT_MANIFEST', '-DXNU_OWNER_BOOT_MANIFEST=1',
            '-UXNU_NATIVE_RAM_BLOCK', '-DXNU_NATIVE_RAM_BLOCK=1',
            '-UXNU_OWNER_CACHE_MOUNT', '-DXNU_OWNER_CACHE_MOUNT=1',
            '-UXNU_OWNER_UNION_SYSTEMOS', '-DXNU_OWNER_UNION_SYSTEMOS=1',
            f'-DXNU_OWNER_DATA_VOLUME_UUID="{builder["DATA_VOLUME_UUID"]}"',
            f'-DXNU_OWNER_ROOT_VOLUME_UUID="{uuids["OS"]}"',
            f'-DXNU_OWNER_CACHE_VOLUME_UUID="{uuids["SystemOS"]}"',
            f'-DXNU_OWNER_APP_VOLUME_UUID="{uuids["App"]}"',
        ],
    )
    kexec_args[1:1] = ['-I', str(builder['V1'] / 'include'), '-iquote', str(Path(kexec_rec['file']).parent)]
    subprocess.run(kexec_args, cwd=kexec_rec['directory'], check=True)

    # Compile patched IOKitBSDInit.cpo so native_owner_publish_boot_manifest updates the ADT's
    # Image4 boot-manifest-hash instead of failing with EAUTH (status=3) when it already exists.
    iokit_rec = json.JSONDecoder().raw_decode((builder['XNU_OBJ'] / 'iokit/DEVELOPMENT/IOKitBSDInit.cpo.json').read_text())[0]
    iokit_orig = Path(iokit_rec['file'])
    if not iokit_orig.is_absolute():
        iokit_orig = Path(iokit_rec['directory']) / iokit_orig
    iokit_src_txt = iokit_orig.read_text()
    old_chk = 'if (chosen->getProperty("boot-manifest-hash") || chosen->getProperty("owner-boot-manifest-domain")) {'
    new_chk = 'if (chosen->getProperty("owner-boot-manifest-domain")) {'
    assert old_chk in iokit_src_txt, 'Expected boot-manifest-hash check in IOKitBSDInit.cpp'
    iokit_src = kout / 'IOKitBSDInit.cpp'
    iokit_src.write_text(iokit_src_txt.replace(old_chk, new_chk, 1))
    iokit_obj = kout / 'IOKitBSDInit.cpo'
    iokit_args = builder['build_cmd_from_record'](
        iokit_rec, iokit_src, iokit_obj,
        ['-UXNU_OWNER_BOOT_MANIFEST', '-DXNU_OWNER_BOOT_MANIFEST=1'],
    )
    iokit_args[1:1] = ['-iquote', str(iokit_orig.parent)]
    subprocess.run(iokit_args, cwd=iokit_rec['directory'], check=True)

    # Compile patched IOUserClient.cpo so IOServices with NativeSandboxGrantAll=true (Tb375fcMali)
    # are accessible to all sandboxed macOS Metal clients in-process (WindowServer, Dock, Finder, apps).
    iouc_obj = None
    if mali_driver:
        iouc_rec = json.JSONDecoder().raw_decode((builder['XNU_OBJ'] / 'iokit/DEVELOPMENT/IOUserClient.cpo.json').read_text())[0]
        iouc_orig = Path(iouc_rec['file'])
        if not iouc_orig.is_absolute():
            iouc_orig = Path(iouc_rec['directory']) / iouc_orig
        iouc_src_txt = iouc_orig.read_text()
        old_grant = 'native_sandbox_grant(IOService * service)\n{\n\tOSString * list = OSDynamicCast(OSString, service->getProperty("NativeSandboxGrantProcess"));'
        new_grant = (
            'native_sandbox_grant(IOService * service)\n{\n'
            '\tif (service->getProperty("NativeSandboxGrantAll") == kOSBooleanTrue) {\n'
            '\t\treturn true;\n'
            '\t}\n'
            '\tOSString * list = OSDynamicCast(OSString, service->getProperty("NativeSandboxGrantProcess"));'
        )
        assert old_grant in iouc_src_txt, 'Expected native_sandbox_grant in IOUserClient.cpp'
        iouc_src = kout / 'IOUserClient.cpp'
        iouc_src.write_text(iouc_src_txt.replace(old_grant, new_grant, 1))
        iouc_obj = kout / 'IOUserClient.cpo'
        iouc_args = builder['build_cmd_from_record'](iouc_rec, iouc_src, iouc_obj, [])
        iouc_args[1:1] = ['-iquote', str(iouc_orig.parent)]
        subprocess.run(iouc_args, cwd=iouc_rec['directory'], check=True)
        print('      Compiled IOUserClient.cpo with NativeSandboxGrantAll support', flush=True)

    # Compile Tb375fcDummyEth.cpp (virtual en0 IOEthernetController) with late-init + IONetworkingFamily KC bindings
    eth_src = DESKTOP_DIR / 'Tb375fcDummyEth.cpp'
    assert eth_src.is_file(), f'Missing {eth_src}'
    pe_rec = m4['XNU_OBJ'] / 'iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'
    compiler, cflags, pe_cwd = m4['load_compile_flags'](pe_rec)
    macosx_sdk = subprocess.check_output(['xcrun', '--sdk', 'macosx', '--show-sdk-path'], text=True).strip()
    eth_obj = kout / 'Tb375fcDummyEth.o'
    subprocess.run([
        compiler, *cflags,
        f'-isystem{macosx_sdk}/System/Library/Frameworks/Kernel.framework/Headers',
        '-Wno-unused-parameter', '-c', str(eth_src), '-o', str(eth_obj),
    ], cwd=pe_cwd, check=True)
    m4['globalize_symbol_in_macho'](eth_obj, '__GLOBAL__sub_I_Tb375fcDummyEth.cpp')
    eth_late = kout / 'Tb375fcDummyEth.late.o'
    subprocess.run([m4['OBJCOPY'], '--remove-section=__DATA,__mod_init_func', str(eth_obj), str(eth_late)], check=True)

    # Extract IONetworkingFamily symbols from KC_PATH and generate eth-bindings.o / eth-addresses.o
    Image = builder['Image']
    kc_raw = m4['KC_PATH'].read_bytes()
    kc = Image(kc_raw)
    plan = json.loads((v4_ns['upstream_root'] / 'scripts/data/ioaccelerator-base-abi-26C5035r.json').read_text())
    slide = int(plan['collection_slide'], 16)
    ionet_syms = {}
    for kext_name, (_, off) in kc.entries.items():
        if 'IONetworkingFamily' in kext_name:
            sub = Image(kc_raw, off)
            for va, names in sub.symbols().items():
                for n in names:
                    ionet_syms[n] = va
    nm_undefs = subprocess.check_output([builder['NM'], '-u', str(eth_late)], text=True).splitlines()
    net_undefs = sorted({
        l.strip().split()[-1] for l in nm_undefs if l.strip()
        if any(tok in l for tok in ('IONetwork', 'IOEthernet', 'IOMbuf'))
    })
    eth_text_asm = '.text\n.p2align 2\n'
    eth_addr_asm = ''
    for sym in net_undefs:
        if sym in ionet_syms:
            addr = ionet_syms[sym] + slide
            if sym.endswith('gMetaClassE') or sym.startswith('__ZTV') or sym.startswith('__ZTI'):
                eth_addr_asm += f'.globl {sym}\n.set {sym}, {addr:#x}\n'
            else:
                eth_addr_asm += f'.globl {sym}.original\n.set {sym}.original, {addr:#x}\n'
                eth_text_asm += f'.globl {sym}\n{sym}:\n bti c\n b {sym}.original\n'
        else:
            assert '_RESERVEDIONetworkController' in sym, f'Unexpected missing IONetworkingFamily symbol: {sym}'
            eth_text_asm += f'.globl {sym}\n{sym}:\n bti c\n ret\n'
    eth_bind_s, eth_addr_s = kout / 'eth-bindings.S', kout / 'eth-addresses.S'
    eth_bind_o, eth_addr_o = kout / 'eth-bindings.o', kout / 'eth-addresses.o'
    eth_bind_s.write_text(eth_text_asm)
    eth_addr_s.write_text(eth_addr_asm)
    ios_sdk = subprocess.check_output(['xcrun', '--sdk', 'iphoneos', '--show-sdk-path'], text=True).strip()
    for s_path, o_path in ((eth_bind_s, eth_bind_o), (eth_addr_s, eth_addr_o)):
        subprocess.run(['xcrun', 'clang', '--target=arm64e-apple-ios27.0.0', '-mkernel', '-isysroot', ios_sdk,
                        '-c', str(s_path), '-o', str(o_path)], check=True)
    print(f'      Compiled Tb375fcDummyEth ({len(net_undefs)} IONetworkingFamily bindings)', flush=True)

    droot_k = DROOT / 'desktop-kernel'
    src_lines = (droot_k / 'link.filelist').read_text().splitlines()
    start_o = TIER3_KDIR / 'start.o'
    pwr_o = TIER3_KDIR / 'Tb375fcPower.o'
    disp_a = TIER3_KDIR / 'libNativeLegacyDisplay.a'
    if rebuild_display:
        display_builder = imported(TIER3_KDIR / 'build_ram8g_kernel.py', 'display_builder')
        disp_a = display_builder.build_display(kout / 'display')
    storage_a = (TIER3_KDIR / 'ufs-rw/libTb375fcDesktopStorage_rw.a') if persist_storage else (DROOT / 'storage-build/libTb375fcDesktopStorage.a')
    for p in (start_o, pwr_o, disp_a, storage_a):
        assert p.is_file(), f'Missing {p}'

    matches = [i for i, x in enumerate(src_lines) if Path(x).name == 'start.o']
    assert len(matches) == 1, f'Expected 1 start.o, got {len(matches)}'
    src_lines[matches[0]] = str(start_o)
    imatches = [i for i, x in enumerate(src_lines) if Path(x).name == 'IOKitBSDInit.cpo']
    assert len(imatches) == 1, f'Expected 1 IOKitBSDInit.cpo, got {len(imatches)}'
    src_lines[imatches[0]] = str(iokit_obj)
    kmatches = [i for i, x in enumerate(src_lines) if Path(x).name == 'kern_exec.o']
    assert len(kmatches) == 1, f'Expected 1 kern_exec.o, got {len(kmatches)}'
    src_lines[kmatches[0]] = str(kexec_obj)
    if iouc_obj is not None:
        umatches = [i for i, x in enumerate(src_lines) if Path(x).name == 'IOUserClient.cpo']
        assert len(umatches) == 1, f'Expected 1 IOUserClient.cpo, got {len(umatches)}'
        src_lines[umatches[0]] = str(iouc_obj)
    vmk_o = TIER3_KDIR / 'vm_kern.o'
    if vmk_o.is_file():
        vmatches = [i for i, x in enumerate(src_lines) if Path(x).name == 'vm_kern.o']
        assert len(vmatches) == 1, f'Expected 1 vm_kern.o, got {len(vmatches)}'
        src_lines[vmatches[0]] = str(vmk_o)
    src_lines.extend([str(pwr_o), str(eth_late), str(eth_bind_o), str(eth_addr_o)])
    filelist = kout / 'link.filelist'
    filelist.write_text('\n'.join(src_lines) + '\n')

    armcpu_a = TIER3_KDIR / 'libArmCpu.a'
    orig_armcpu_a = str(BASE / 'out/tb375fc/macos-27.3-26C5035r/tb375fc/tree/build/armcpu/libArmCpu.a')
    assert armcpu_a.is_file(), f'Missing {armcpu_a}'
    native = kout / 'native-cdc-klog.macho'
    link = json.loads((V4 / 'link-command.json').read_text())
    link = [
        arg.replace(str(V4 / 'link.filelist'), str(filelist))
           .replace(str(V4 / 'native-cdc-klog.macho'), str(native))
           .replace(str(M5_GUI / 'm5-v14/libTb375fcStorage.a'), str(storage_a))
           .replace(str(V4 / 'libNativeLegacyDisplay.a'), str(disp_a))
           .replace(orig_armcpu_a, str(armcpu_a))
        for arg in link
    ]
    if touch_stage1:
        touch_dir = BASE / 'touch-prep'
        mali4_touch = TIER3_KDIR / 'mali-bringup/libTb375fcTouch_mali4.a'
        fixed_touch = TIER3_KDIR / 'touch-fix/libTb375fcTouch_fixed.a'
        if mali_driver and mali4_touch.is_file():
            touch_archive = mali4_touch
        elif fixed_touch.is_file():
            touch_archive = fixed_touch
        else:
            touch_archive = touch_dir / 'build/libTb375fcTouch.a'
            qualification = json.loads((touch_dir / 'build/qualification-report.json').read_text())
            assert digest(touch_archive.read_bytes()) == qualification['archive_sha256']
        link.extend(['-Xlinker', '-force_load', '-Xlinker', str(touch_archive)])
    subprocess.run(link, cwd=builder['XNU_OBJ'], check=True)

    board = kout / 'xnu-cdc-board.macho'
    for p in (board, board.with_suffix('.json'), board.with_suffix('.map.json'), diag):
        if p.exists():
            p.unlink()

    prelink = json.loads((V4 / 'prelink-command.json').read_text())
    prelink = [
        arg.replace(str(V4 / 'native-cdc-klog.macho'), str(native))
           .replace(str(V4 / 'xnu-cdc-board.macho'), str(board))
        for arg in prelink
    ]
    subprocess.run(prelink, cwd=builder['W'], check=True)
    report = json.loads(board.with_suffix('.json').read_text())
    assert report['missing'] == {} and report['stubbed'] == {}, f'Unresolved symbols: {report}'

    # Sync all 400 PAC vtable discriminators of __ZTV15Tb375fcDummyEth with __ZTV20IOEthernetController
    board_raw = bytearray(board.read_bytes())
    board_kc = Image(board_raw)
    board_syms = {}
    for _, (_, off) in board_kc.entries.items():
        for va, names in Image(board_raw, off).symbols().items():
            for n in names:
                board_syms[n] = va
    eth_vptr = board_syms['__ZTV15Tb375fcDummyEth'] + 16
    base_vptr = board_syms['__ZTV20IOEthernetController'] + 16
    bfix = board_kc.fixups()
    patched_divs = 0
    for idx in range(400):
        eslot = eth_vptr + idx * 8
        bslot = base_vptr + idx * 8
        if eslot in bfix and bslot in bfix:
            _, _, _, ediv, _ = bfix[eslot]
            _, _, _, bdiv, _ = bfix[bslot]
            if ediv != bdiv:
                off = board_kc.offset(eslot, 8)
                word, = struct.unpack_from('<Q', board_raw, off)
                struct.pack_into('<Q', board_raw, off, (word & ~(0xffff << 32)) | (bdiv << 32))
                patched_divs += 1
    if patched_divs:
        board.write_bytes(board_raw)
    print(f'      Synced Tb375fcDummyEth vtable PAC discriminators ({patched_divs}/400 patched)', flush=True)

    subprocess.run([builder['PY'], str(TIER3_KDIR / 'apply_late_blocks_and_sdt_fixed.py'), str(kout)],
                   cwd=builder['W'], check=True)

    symbols = subprocess.check_output([builder['NM'], str(diag)], text=True)
    if touch_stage1:
        touch_builder = imported(touch_dir / 'hid/build_touch_driver.py', 'touch_abi')
        abi_manifest = json.loads((touch_dir / 'hid/hid-abi-manifest.json').read_text())
        verification = touch_builder.verify_prelinked_touch_vtable(diag, abi_manifest)
        assert verification['total_slots_verified'] == 334
        report_path = kout / 'touch-abi-report.json'
        report_path.write_text(json.dumps(verification, indent=2) + '\n')
        print('      PASS: revalidated all 334 touch HID ABI slots in the final kernel', flush=True)
    for name in ('_native_verified_block_manifest_sha384', '_native_owner_publish_boot_manifest',
                 '_native_owner_cache_checkpoint', '_native_owner_macos_app_cryptex',
                 '_native_source_root_policies_load',
                 '__ZTV18IOAccelSwiftShader', '__ZTV28IOAccelSwiftShaderUserClient',
                 '_native_swiftshader_accelerator_probe', '__GLOBAL__sub_I_IOAccelSwiftShader.cpp',
                 '__ZTV15Tb375fcDummyEth', '_tb375fc_dummy_eth_probe', '__GLOBAL__sub_I_Tb375fcDummyEth.cpp'):
        assert any(line.endswith(' ' + name) for line in symbols.splitlines()), name
    if persist_storage:
        for name in ('_native_ufs_store_open', '_native_ufs_store_write', '_tb375fc_ufs_set_write_window'):
            assert any(line.endswith(' ' + name) for line in symbols.splitlines()), name
        print('      PASS: verified bounded UFS PersistOverlay symbols in final kernel', flush=True)
    print(f'      PASS: {diag} ({diag.stat().st_size} B, sha256={digest(diag.read_bytes())[:16]}...)', flush=True)
    return diag


def build_xnulz(layout_h: Path, lazy_code_hashes: bool, smp: bool = False, extra_boot_args=None) -> Path:
    print('[3/5] Building desktop XnuLz.efi with measured Layout.h...', flush=True)
    work = layout_h.parent / 'xnulz-work'
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    source = M7_DIR / 'xnulz-m7'
    for directory in ('XnuLzPkg', 'Conf'):
        shutil.copytree(source / directory, work / directory, dirs_exist_ok=True)
    app = work / 'XnuLzPkg/Application/XnuLz'
    shutil.copyfile(layout_h, app / 'Layout.h')
    src = app / 'XnuLz.c'
    lines = src.read_text().splitlines(keepends=True)
    index, = (n for n, line in enumerate(lines) if line.startswith(ARGS_PREFIX))
    args = lines[index].rstrip().removeprefix(ARGS_PREFIX).removesuffix('"').split()
    assert all(not arg.startswith('apfs-boot-shim=') for arg in args)
    # Keep apfs-boot-shim=1 so mount-phase-1/2 succeed without Apple Silicon iSC xART,
    # and pass wdt=-1 to disable Apple hardware watchdogd panics.
    args.append('apfs-boot-shim=1')
    args.append('wdt=-1')
    args.append('validation_disables=0x10')
    args.append('fb-half=2')
    if lazy_code_hashes:
        args.append('cs-lazy=1')
    if smp:
        args.append('smp=1')
    if extra_boot_args:
        args.extend(extra_boot_args)
    boot_args = ' '.join(args)
    lines[index] = ARGS_PREFIX + boot_args + '"\n'
    src.write_text(''.join(lines))

    tools = WORKSPACE / 'Mu_Basecore/BaseTools'
    env = os.environ.copy()
    env.update(
        WORKSPACE=str(work),
        PACKAGES_PATH=f'{work}:{tools.parent}',
        EDK_TOOLS_PATH=str(tools),
        CLANG_BIN=str(WORKSPACE / '.tools/clang21/bin') + '/',
        PYTHON_COMMAND=str(WORKSPACE / '.tools/venv/bin/python'),
    )
    env['PATH'] = os.pathsep.join(map(str, [
        tools / 'BinWrappers/PosixLike', tools / 'Source/C/bin',
        WORKSPACE / '.tools/venv/bin', env['PATH'],
    ]))
    with (work / 'build.log').open('w') as log:
        subprocess.run(['build', '-a', 'AARCH64', '-b', 'RELEASE', '-t', 'CLANGPDB',
                        '-p', 'XnuLzPkg/XnuLzPkg.dsc'], cwd=work, env=env,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    efi = work / 'Build/XnuLzPkg/RELEASE_CLANGPDB/AARCH64/XnuLz.efi'
    assert (boot_args + '\0').encode('utf-16le') in efi.read_bytes()
    print(f'      PASS: {efi} (boot-args: {boot_args})', flush=True)
    return efi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DESKTOP_DIR)
    parser.add_argument('--daemon', type=Path, default=DIAG_DAEMON)
    parser.add_argument('--data-seed', type=Path)
    parser.add_argument('--cpu-only', action='store_true')
    parser.add_argument('--rebuild-display', action='store_true')
    parser.add_argument('--menu-timeout', type=int)
    parser.add_argument('--lazy-code-hashes', action='store_true',
                        help='authenticate original signed blocks at first use')
    parser.add_argument('--smp', action='store_true',
                        help='enable 8-core PSCI SMP via smp=1 boot-arg')
    parser.add_argument('--touch-stage1', action='store_true',
                        help='integrate and run only the read-only touch MMIO snapshot (alias for --touch-stage 1)')
    parser.add_argument('--touch-stage', type=int, choices=[1, 2, 3, 4], default=None,
                        help='integrate Tb375fcTouch and pass touch=N (1=snapshot, 2=spi2 init, 3=trim ID, 4=HID)')
    parser.add_argument('--persist-storage', action='store_true',
                        help='link isolated bounded UFS LUN2 PersistOverlay storage archive (boot-arg gated via data-persist=1|2)')
    parser.add_argument('--mali-driver', action='store_true',
                        help='link Tb375fcMaliDriver IOUserClient (libTb375fcTouch_mali4.a) for mali=4')
    parser.add_argument('--panvk-icd', type=Path, default=None,
                        help='pack Mesa panvk libvulkan_panfrost.dylib instead of SwiftShader')
    parser.add_argument('--inproc-bundle', type=Path, default=None,
                        help='pack self-contained arm64e in-process NativeAdrenoMetal.bundle (contains libvulkan_panfrost.dylib)')
    parser.add_argument('--extra-boot-arg', action='append', default=[],
                        help='additional XNU boot-arg (repeatable)')
    parser.add_argument('--het-clusters', action='store_true',
                        help='tag ADT cpu0..3 as E-cluster (A510) and cpu4..7 as P-cluster (A715)')
    parser.add_argument('--no-notch', action='store_true',
                        help='clear ADT /product/partially-occluded-display & has-round-cornered-mac-display so flat LCD hides MacBook notch/island')
    args = parser.parse_args()
    if args.menu_timeout is not None and args.menu_timeout < 1:
        parser.error('--menu-timeout must leave time to enter the menu')
    if args.data_seed is not None and not args.data_seed.is_file():
        parser.error('--data-seed must be an existing file')
    output = args.output.resolve()
    touch_stage = args.touch_stage if args.touch_stage is not None else (1 if args.touch_stage1 else 0)
    if touch_stage > 0:
        if output.exists() and any(output.iterdir()):
            parser.error('--touch-stage requires a new or empty output directory')
        if any(arg.split('=', 1)[0] == 'touch' for arg in args.extra_boot_arg):
            parser.error('--touch-stage supplies its own touch=N boot argument')
        args.extra_boot_arg.append(f'touch={touch_stage}')
    output.mkdir(parents=True, exist_ok=True)
    diag_kernel = build_kernel(output, args.rebuild_display, touch_stage > 0, persist_storage=args.persist_storage, mali_driver=args.mali_driver)
    if args.lazy_code_hashes:
        symbols = subprocess.check_output(['xcrun', 'nm', str(diag_kernel)], text=True)
        for name in ('_cs_native4k_lazy', '_cs_native4k_lazy_authenticate'):
            if not any(line.endswith(' ' + name) for line in symbols.splitlines()):
                raise ValueError(f'Kernel lacks deferred code authentication symbol {name}')

    print('[2/5] Assembling inner FAT (Desktop OS UUID + RootPolicies + Tier 3 7.03 GiB ADT + latest GPU daemon)...', flush=True)
    packer = imported(BASE / 'pack-vendor-fat.py', 'pack_vendor')
    pack_tiers = imported(PACK_DIR / 'pack_ram_tiers.py', 'pack_tiers')
    pack_m7 = imported(PACK_M7 / 'pack_m7_35mib.py', 'pack_m7')
    dpayload = imported(DROOT / 'build_desktop_payload.py', 'dpayload')
    adt_helpers = imported(
        BASE / 'postmmu-diag/launchd-root/restore-v1/35m-launchd/pid1-gate-v1/generate_manifest_adt.py',
        'adt_helpers',
    )

    cpu_only = args.cpu_only
    os_uuid = json.loads((DROOT / 'policies/OS/verification.json').read_text())['volume_uuid']
    active_daemon = (DESKTOP_DIR / 'tb375fc-cdc-shell') if cpu_only else args.daemon
    daemon_hash = signed_cdhash(active_daemon)
    custom_launcher = RAM_DIR / 'gpu-diag/BootCacheControl'
    launcher = custom_launcher if custom_launcher.is_file() else (M7_DIR / 'daemon/BootCacheControl')
    custom_ss = DESKTOP_DIR / 'libvk_swiftshader.o1nosroa.dylib'
    if not custom_ss.is_file():
        custom_ss = DESKTOP_DIR / 'libvk_swiftshader.o0.dylib'
    swiftshader = custom_ss if custom_ss.is_file() else (M7_DIR / 'swiftshader/libvk_swiftshader.hostmem.dylib')
    bundle = args.inproc_bundle.resolve() if args.inproc_bundle else (M7_DIR / 'plugin/NativeAdrenoMetal.bundle')
    if args.inproc_bundle:
        assert bundle.is_dir(), f'Missing in-process bundle: {bundle}'
        print(f'[build] In-Process Metal Bundle: {bundle}')
    else:
        print(f"[build] SwiftShader: {swiftshader.name}")

    baseline_fat_lzma = PACK_M7 / 'board-m7-v3/readback/fatpart.lzma'
    base_v4_dir = PACK_M7 / 'board-m7-v4'
    v4_report = json.loads((base_v4_dir / 'verification.json').read_text())
    v4_img = (base_v4_dir / 'vendor_boot_a-m7-v4.img').read_bytes()

    with tempfile.TemporaryDirectory(dir=output, prefix='work-') as tmp:
        work = Path(tmp)
        validator = pack_tiers.build_adt_validator(work)

        src_inner = work / 'old.img'
        with lzma.open(baseline_fat_lzma) as packed, src_inner.open('wb') as dst:
            shutil.copyfileobj(packed, dst)

        image = work / 'inner.img'
        with image.open('wb') as dst:
            dst.truncate(src_inner.stat().st_size)
        tool('mformat', image, '-v', 'XNURAM', '::')
        for path in ('::/EFI', BOOT.rstrip('/')):
            tool('mmd', image, path)

        # Build updated gpuaddon
        if cpu_only:
            addon_entries = [
                ('sbin/BootCacheControl', 0o755, launcher.read_bytes()),
                ('sbin/tb375fc-gpu-daemon', 0o755, active_daemon.read_bytes()),
            ]
        else:
            if args.inproc_bundle:
                print(f"[build] In-Process mode: packing {bundle.name} + {active_daemon.name} (no RPC GPU daemon)")
                addon_entries = [
                    ('sbin/BootCacheControl', 0o755, launcher.read_bytes()),
                    ('sbin/tb375fc-gpu-daemon', 0o755, active_daemon.read_bytes()),
                ]
            elif args.panvk_icd and args.panvk_icd.is_file():
                print(f"[build] Vulkan ICD: Mesa panvk ({args.panvk_icd.name})")
                addon_entries = [
                    ('sbin/BootCacheControl', 0o755, launcher.read_bytes()),
                    ('sbin/tb375fc-gpu-daemon', 0o755, active_daemon.read_bytes()),
                    ('lib/libvulkan_panfrost.dylib', 0o755, args.panvk_icd.read_bytes()),
                ]
            else:
                addon_entries = [
                    ('sbin/BootCacheControl', 0o755, launcher.read_bytes()),
                    ('sbin/tb375fc-gpu-daemon', 0o755, active_daemon.read_bytes()),
                    ('lib/libvk_swiftshader.dylib', 0o755, swiftshader.read_bytes()),
                ]
            for path in sorted(bundle.rglob('*')):
                if not path.is_file():
                    continue
                rel = path.relative_to(bundle).as_posix()
                mode = 0o755 if (path.name == 'NativeAdrenoMetal' or path.suffix == '.dylib') else 0o644
                addon_entries.append((f'GPUBundles/NativeAdrenoMetal.bundle/{rel}', mode, path.read_bytes()))
            touch_fw = DESKTOP_DIR / 'novatek_ts_fw_tm.bin'
            if touch_fw.is_file():
                addon_entries.append(('novatek_ts_fw_tm.bin', 0o644, touch_fw.read_bytes()))
            mali_fw = DESKTOP_DIR / 'mali_csffw.bin'
            if mali_fw.is_file() and any(a.startswith('mali=') for a in args.extra_boot_arg):
                addon_entries.append(('mali_csffw.bin', 0o644, mali_fw.read_bytes()))
        addon_p = work / 'kernel.macho.gpuaddon'
        addon_p.write_bytes(pack_m7.make_addon(addon_entries))

        # Build updated trustcache (include daemon, launcher, bundle, and embedded/external Vulkan ICD)
        orig_trust = work / 'orig.trustcache'
        new_trust = work / 'kernel.macho.trustcache'
        tool('mcopy', src_inner, BOOT + 'kernel.macho.trustcache', orig_trust)
        trust_hashes = [daemon_hash]
        if launcher == custom_launcher:
            trust_hashes.append(signed_cdhash(launcher))
        if not cpu_only and args.inproc_bundle:
            trust_hashes.append(signed_cdhash(bundle))
            inproc_panvk = bundle / 'Contents/MacOS/libvulkan_panfrost.dylib'
            if inproc_panvk.is_file():
                trust_hashes.append(signed_cdhash(inproc_panvk))
            print(f'[build] TrustCache hashes (in-proc): {trust_hashes}')
        elif not cpu_only and args.panvk_icd and args.panvk_icd.is_file():
            trust_hashes.append(signed_cdhash(args.panvk_icd))
        elif not cpu_only and swiftshader == custom_ss:
            trust_hashes.append(signed_cdhash(swiftshader))
        subprocess.run([
            sys.executable,
            str(UPSTREAM / 'scripts/trust-append-module.py'),
            str(orig_trust), str(new_trust),
            *trust_hashes,
        ], check=True)

        # Build updated ADT: Desktop OS UUID + RootPolicies + Tier 3 7.03 GiB RAM
        # + /defaults/vmm-present=1 (so watchdogd exits 0 instead of SIGTRAP) + sleep-disabled=1
        orig_adt = work / 'orig.adt'
        tool('mcopy', src_inner, BOOT + 'kernel.macho.adt', orig_adt)
        dt_bytes = dpayload.desktop_tree(orig_adt.read_bytes(), os_uuid)
        dt_bytes = pack_tiers.patch_adt(dt_bytes, WIN_SIZE, adt_helpers, validator)
        tree, end = adt_helpers.parse(dt_bytes)
        assert end == len(dt_bytes)
        d_idx = next(i for i, c in enumerate(tree[1]) if dict(c[0]).get('name', b'').rstrip(b'\0') == b'defaults')
        d_props = [(k, v) for k, v in tree[1][d_idx][0] if k not in ('vmm-present', 'sleep-disabled')]
        d_props.append(('vmm-present', struct.pack('<I', 1)))
        d_props.append(('sleep-disabled', struct.pack('<I', 1)))
        tree[1][d_idx] = (d_props, tree[1][d_idx][1])
        if args.no_notch:
            p_idx = next(i for i, c in enumerate(tree[1]) if dict(c[0]).get('name', b'').rstrip(b'\0') == b'product')
            p_props = [(k, v) for k, v in tree[1][p_idx][0]
                       if k not in ('partially-occluded-display', 'has-round-cornered-mac-display')]
            p_props.append(('partially-occluded-display', struct.pack('<I', 0)))
            p_props.append(('has-round-cornered-mac-display', struct.pack('<I', 0)))
            tree[1][p_idx] = (p_props, tree[1][p_idx][1])
            print('      ADT /product notch/island disabled: partially-occluded-display=0, has-round-cornered-mac-display=0', flush=True)
        if args.het_clusters:
            c_idx = next(i for i, c in enumerate(tree[1]) if dict(c[0]).get('name', b'').rstrip(b'\0') == b'cpus')
            new_cpus = []
            for child in tree[1][c_idx][1]:
                cname = dict(child[0]).get('name', b'').rstrip(b'\0').decode()
                cprops = [(k, v) for k, v in child[0] if k != 'cluster-type']
                if cname in ('cpu0', 'cpu1', 'cpu2', 'cpu3'):
                    cprops.append(('cluster-type', struct.pack('<I', ord('E'))))
                elif cname in ('cpu4', 'cpu5', 'cpu6', 'cpu7'):
                    cprops.append(('cluster-type', struct.pack('<I', ord('P'))))
                new_cpus.append((cprops, child[1]))
            tree[1][c_idx] = (tree[1][c_idx][0], new_cpus)
            print('      ADT heterogeneous clusters enabled: cpu0..3=E (A510), cpu4..7=P (A715)', flush=True)
        dt_bytes = adt_helpers.encode(tree)
        # Re-validate ADT with DarwinBoot validator
        import ctypes
        w_out = ctypes.c_uint64(0)
        p_out = ctypes.c_int(0)
        e_out = ctypes.c_size_t(0)
        a_out = ctypes.c_size_t(0)
        assert validator.validate_adt(dt_bytes, len(dt_bytes), ctypes.byref(w_out), ctypes.byref(p_out), ctypes.byref(e_out), ctypes.byref(a_out)) == 1
        assert w_out.value == WIN_SIZE and a_out.value == 1
        adt_p = work / 'kernel.macho.adt'
        adt_p.write_bytes(dt_bytes)

        # Copy md0-test.dbr (prefer 256m ultraslim official Data seed so full SwiftShader + Data seed fits in 35 MiB outer.img!)
        custom_seed = args.data_seed or (DESKTOP_DIR / 'macos-data-seed-256m-ultraslim.dbr')
        if not custom_seed.is_file():
            custom_seed = DESKTOP_DIR / 'macos-data-seed-512m-slim.dbr'
        if not custom_seed.is_file():
            custom_seed = DESKTOP_DIR / 'macos-data-seed-512m.dbr'
        if custom_seed.is_file():
            print(f'      Using proper macOS Data seed: {custom_seed} ({custom_seed.stat().st_size} B)', flush=True)
            shutil.copyfile(custom_seed, work / 'md0-test.dbr')
        else:
            tool('mcopy', src_inner, BOOT + 'md0-test.dbr', work / 'md0-test.dbr')
        shutil.copyfile(diag_kernel, work / 'kernel.macho')
        shutil.copyfile(DROOT / 'desktop-root-policies.bin', work / 'kernel.macho.vroot')

        names = ['kernel.macho', 'kernel.macho.adt', 'kernel.macho.gpuaddon',
                 'kernel.macho.trustcache', 'kernel.macho.vroot', 'md0-test.dbr']
        need_bytes = sum((work / n).stat().st_size for n in names) + (16 << 20)
        if need_bytes > image.stat().st_size:
            fat_size = ((need_bytes + (4 << 20) - 1) // (4 << 20)) * (4 << 20)
            image.unlink()
            with image.open('wb') as dst:
                dst.truncate(fat_size)
            tool('mformat', image, '-v', 'XNURAM', '::')
            for path in ('::/EFI', BOOT.rstrip('/')):
                tool('mmd', image, path)
        for name in sorted(names):
            tool('mcopy', image, work / name, BOOT + name)
            rb = work / ('rb-' + name)
            tool('mcopy', image, BOOT + name, rb)
            assert rb.read_bytes() == (work / name).read_bytes()

        raw = bytearray(image.read_bytes())
        layout = {'VOLUME_SIZE': len(raw)}
        for key, name in (('KERNEL', 'kernel.macho'), ('ADT', 'kernel.macho.adt')):
            data = (work / name).read_bytes()
            off = raw.find(data)
            assert off >= 0 and raw.find(data, off + 1) == -1, name
            layout[key + '_OFF'] = off
            layout[key + '_SIZE'] = len(data)
        kstart = layout['KERNEL_OFF']
        raw[kstart:kstart + layout['KERNEL_SIZE']] = bytes(layout['KERNEL_SIZE'])
        image.write_bytes(raw)
        layout_h = output / 'Layout.h'
        layout_h.write_text(''.join(f'#define KNULZ_{k} {v}ULL\n' for k, v in layout.items()))
        print(f'      OS UUID={os_uuid} layout={layout}', flush=True)

        xnulz_efi = build_xnulz(layout_h, args.lazy_code_hashes, smp=args.smp, extra_boot_args=args.extra_boot_arg)

        print('[4/5] Compressing kernel.lzma (32 MiB) and fatpart.lzma (32 MiB)...', flush=True)
        kernel_lzma_bytes = lzma_alone(diag_kernel.read_bytes(), 32 << 20, nice_len=273)
        kernel_lzma = work / 'kernel.lzma'
        kernel_lzma.write_bytes(kernel_lzma_bytes)

        fatpart_lzma_bytes = lzma_alone(bytes(raw), 32 << 20, nice_len=273)
        fatpart_lzma = work / 'fatpart.lzma'
        fatpart_lzma.write_bytes(fatpart_lzma_bytes)
        decoder = xnulz_efi.parents[4] / 'XnuLzPkg/Application/XnuLz/Lzma/Sdk/C'
        verifier = work / 'verify_lzmadec'
        subprocess.run(['xcrun', 'clang', '-O2', '-I', str(decoder),
                        str(M5_GUI / 'cpu-vulkan/verify_lzmadec.c'),
                        str(decoder / 'LzmaDec.c'), '-o', str(verifier)], check=True)
        for packed, expected in ((kernel_lzma, diag_kernel), (fatpart_lzma, image)):
            decoded = work / 'decoded.raw'
            subprocess.run([str(verifier), str(packed), str(decoded)], check=True)
            assert digest(decoded.read_bytes()) == digest(expected.read_bytes()), packed.name
            decoded.unlink()

        print('[5/5] Packing vendor_boot_a-desktop-tier3.img (strict 35 MiB proven outer FAT)...', flush=True)
        fat_slice = slice(v4_report['fat_offset_in_ramdisk'],
                          v4_report['fat_offset_in_ramdisk'] + v4_report['fat_bytes'])
        outer = work / 'outer.img'
        outer.write_bytes(packer.vendor_boot.parse(v4_img)['ramdisk'][fat_slice])

        cfg = work / 'unixnu.toml'
        tool('mcopy', outer, BOOT + 'unixnu.toml', cfg)
        cfg_txt = cfg.read_text()
        assert cfg_txt.count('M7 v4 SwiftShader') == 2
        cfg_txt = cfg_txt.replace('M7 v4 SwiftShader', 'Desktop OS Tier3')
        if args.menu_timeout is not None:
            cfg_txt, changed = re.subn(r'^timeout\s*=\s*\d+$',
                                       f'timeout = {args.menu_timeout}', cfg_txt, flags=re.M)
            assert changed == 1, 'Expected one menu timeout'
        cfg.write_text(cfg_txt)

        outer_files = {
            'unixnu.toml': cfg,
            'Payloads/XnuLz.efi': xnulz_efi,
            'Payloads/DarwinBoot.efi': DARWINBOOT_EFI,
            'Payloads/kernel.lzma': kernel_lzma,
            'Payloads/fatpart.lzma': fatpart_lzma,
        }
        total_outer_payload = sum(src.stat().st_size for src in outer_files.values())
        print(f'      Outer payload: {total_outer_payload:,} B (kernel={len(kernel_lzma_bytes):,}, fatpart={len(fatpart_lzma_bytes):,})', flush=True)
        for rel, src in outer_files.items():
            pack_m7.replace(outer, src, BOOT + rel)

        stock_vb = (BASE / 'current-stock/vendor_boot_a.img').read_bytes()
        outer_bytes = outer.read_bytes()
        out_bytes, meta = packer.pack(stock_vb, outer_bytes)
        assert len(out_bytes) == len(stock_vb) == 67108864
        assert meta['fat_bytes'] == v4_report['fat_bytes']
        assert packer.vendor_boot.parse(out_bytes)['ramdisk'][fat_slice] == outer_bytes
        outer_hashes = {}
        for name, source in outer_files.items():
            readback = work / ('verify-' + source.name)
            tool('mcopy', outer, BOOT + name, readback)
            data = readback.read_bytes()
            assert data == source.read_bytes(), name
            outer_hashes[name] = digest(data)
        out_img = output / 'vendor_boot_a-desktop-tier3.img'
        staged = work / out_img.name
        staged.write_bytes(out_bytes)
        staged.replace(out_img)

        summary = {
            'image_path': str(out_img),
            'image_bytes': len(out_bytes),
            'image_sha256': digest(out_bytes),
            'os_uuid': os_uuid,
            'kernel_sha256': digest(diag_kernel.read_bytes()),
            'kernel_lzma_bytes': len(kernel_lzma_bytes),
            'fatpart_lzma_bytes': len(fatpart_lzma_bytes),
            'daemon_cdhash': daemon_hash,
            'data_seed': str(custom_seed) if custom_seed.is_file() else 'M7 fallback',
            'data_seed_sha256': digest((work / 'md0-test.dbr').read_bytes()),
            'menu_timeout': args.menu_timeout,
            'lazy_code_hashes': args.lazy_code_hashes,
            'smp': args.smp,
            'extra_boot_args': args.extra_boot_arg,
            'outer_file_sha256': outer_hashes,
            'layout': layout,
        }
        (output / 'desktop-tier3-manifest.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
