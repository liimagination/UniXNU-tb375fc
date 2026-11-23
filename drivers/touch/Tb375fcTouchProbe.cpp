/* SPDX-License-Identifier: MIT */
/*
 * TB375FC Staged Touch Kernel Probe (MT6897 SPI2 + Novatek NT36532 + macOS IOHIDDevice).
 *
 * OFF by default! Enable via boot-args:
 *   touch=1 : Stage 1 — Non-invasive MMIO register snapshot only (ZERO writes!)
 *   touch=2 : Stage 2 — Enable SPI2 clocks, configure GPIO 9/11..14/60, init SPI2 IPMv2
 *   touch=3 : Stage 3 — Reset & probe NT36532 over SPI2 (prints live 6-byte Trim ID!)
 *   touch=4 : Stage 4 — Load novatek_ts_fw_{tm,boe}.bin, download SRAM FW, register
 *                       Tb375fcTouchHid (IOHIDDevice), and stream live touch -> HID pointer!
 *
 * Optional tuning boot-args:
 *   touch-rot=0..3       : Landscape orientation transform (default 0)
 *   touch-manual-cs=0..1 : 0=HW CS + IPMv2 PAUSE_EN (default), 1=GPIO12 manual CS
 *   touch-26m=0..1       : 0=mainpll_d6_d2 182MHz (default), 1=tck_26m 26MHz
 *   touch-log=0..1       : Verbose live contact coordinate logging on CDC (default 1)
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/OSKextLib.h>
#include <libkern/c++/OSMetaClass.h>
#include <mach/kmod.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>

#include "Mt6897Spi.h"
#include "Mt6897TouchBoard.h"
#include "Nvt36532Protocol.h"

/* Minimal BSD VFS prototypes for reading firmware file from /private/var/ngpu/ */
extern "C" {
struct vnode;
typedef struct vnode *vnode_t;
struct vfs_context;
typedef struct vfs_context *vfs_context_t;
struct ucred;
typedef struct ucred *kauth_cred_t;
struct proc;
typedef struct proc *proc_t;

vfs_context_t vfs_context_kernel(void);
kauth_cred_t  vfs_context_ucred(vfs_context_t ctx);
proc_t        vfs_context_proc(vfs_context_t ctx);
int  vnode_lookup(const char *path, int flags, vnode_t *vpp, vfs_context_t ctx);
int  vnode_put(vnode_t vp);
int  vnode_size(vnode_t vp, off_t *sizep, vfs_context_t ctx);
int  vn_rdwr(int rw, vnode_t vp, char *base, int len, off_t offset, int segflg,
             int ioflg, kauth_cred_t cred, int *aresid, proc_t p);

extern kmod_info_t g_kernel_kmod_info;
extern void _GLOBAL__sub_I_Tb375fcTouchHid_cpp(void) asm("__GLOBAL__sub_I_Tb375fcTouchHid.cpp");
extern IOService *tb375fc_touch_hid_allocate(void);
extern IOReturn   tb375fc_touch_hid_send_report(const nvt_hid_pointer_report_t *rep);
}

#ifndef UIO_READ
#define UIO_READ     0
#endif
#ifndef UIO_SYSSPACE
#define UIO_SYSSPACE 2
#endif

namespace {

struct MappedRegion {
    IOMemoryDescriptor *desc;
    IOMemoryMap        *map;
    uintptr_t           va;
};

static MappedRegion gTopckgen  = {};
static MappedRegion gPericfgAo = {};
static MappedRegion gGpio      = {};
static MappedRegion gIocfgTm   = {};
static MappedRegion gSpi2      = {};

static mt6897_spi_ctrl_t gSpi = {};
static uint32_t gTouchStage   = 0;
static uint32_t gTouchRot     = 0;
static uint32_t gTouchLog     = 1;

static bool mapMmioRegion(uint64_t phys, uint64_t size, MappedRegion *out) {
    out->desc = IOMemoryDescriptor::withPhysicalAddress(phys, size, kIODirectionInOut);
    if (!out->desc) return false;
    out->map = out->desc->map(kIOMapInhibitCache);
    if (!out->map) {
        out->desc->release();
        out->desc = nullptr;
        return false;
    }
    out->va = (uintptr_t)out->map->getVirtualAddress();
    return out->va != 0;
}

static uint32_t mmio_read32(void *, uintptr_t base, uint32_t offset) {
    volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(base + offset);
    uint32_t v = *p;
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}

static void mmio_write32(void *, uintptr_t base, uint32_t offset, uint32_t val) {
    __asm__ volatile("dsb st" ::: "memory");
    volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(base + offset);
    *p = val;
    __asm__ volatile("dsb st" ::: "memory");
}

static void mmio_udelay(void *, uint32_t us) {
    IODelay(us);
}

static int bus_spi_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len) {
    return mt6897_spi_xfer(static_cast<mt6897_spi_ctrl_t *>(ctx), tx, rx, len);
}

static void bus_msleep(void *, uint32_t ms) {
    IOSleep(ms);
}

static uint64_t now_uptime_ms(void) {
    uint64_t abs_t = 0, ns = 0;
    clock_get_uptime(&abs_t);
    absolutetime_to_nanoseconds(abs_t, &ns);
    return ns / 1000000ULL;
}

static void logSnapshot(const char *tag, const mt6897_hw_snapshot_t &s) {
    IOLog("Tb375fcTouch[%s]: topck_mux=0x%08x perao_cg0=0x%08x mode1=0x%08x mode7=0x%08x "
          "dir0=0x%08x dir1=0x%08x din0=0x%08x dout1=0x%08x ies0=0x%08x ies1=0x%08x\n",
          tag, s.topck_spi2_mux, s.perao_cg0_sta, s.gpio_mode1, s.gpio_mode7,
          s.gpio_dir0, s.gpio_dir1, s.gpio_din0, s.gpio_dout1, s.iocfg_ies0, s.iocfg_ies1);
    IOLog("Tb375fcTouch[%s]: spi cfg0=0x%08x cfg1=0x%08x cfg2=0x%08x cfg3=0x%08x "
          "cmd=0x%08x pad=0x%08x st1=0x%08x\n",
          tag, s.spi_cfg0, s.spi_cfg1, s.spi_cfg2, s.spi_cfg3,
          s.spi_cmd, s.spi_pad_sel, s.spi_status1);
}

static bool registerHidDevice(void) {
    OSReturn kr = OSKextLoadKextWithIdentifier("com.apple.iokit.IOHIDFamily");
    OSReturn kr_ed = OSKextLoadKextWithIdentifier("com.apple.iokit.IOHIDEventDriver");
    OSReturn kr_sb = OSKextLoadKextWithIdentifier("com.apple.iokit.IOHIDEventDriverSafeBoot");
    IOLog("Tb375fcTouch: kext load IOHIDFamily=0x%x IOHIDEventDriver=0x%x SafeBoot=0x%x\n",
          kr, kr_ed, kr_sb);
    if (kr != kOSReturnSuccess) return false;

    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {
        IOLog("Tb375fcTouch: OSMetaClass::preModLoad failed\n");
        return false;
    }
    _GLOBAL__sub_I_Tb375fcTouchHid_cpp();
    OSReturn post_kr = OSMetaClass::postModLoad(handle);
    IOLog("Tb375fcTouch: postModLoad(Tb375fcTouchHid)=0x%x\n", post_kr);
    if (post_kr != kOSReturnSuccess) return false;

    IOService *root = IOService::getPlatform();
    if (!root) return false;

    IOService *hid = tb375fc_touch_hid_allocate();
    if (!hid) return false;

    OSDictionary *props = OSDictionary::withCapacity(4);
    if (!props || !hid->init(props)) {
        if (props) props->release();
        hid->release();
        return false;
    }
    props->release();
    hid->setName("Tb375fcTouchHid");

    bool attached = hid->attach(root);
    bool started  = attached && hid->start(root);
    if (started) {
        hid->registerService();
        IOLog("Tb375fcTouch: Tb375fcTouchHid PUBLISHED (2944x1840 Absolute Pointer)\n");
        return true;
    }
    IOLog("Tb375fcTouch: Tb375fcTouchHid attach=%d start=%d FAILED\n", attached, started);
    if (attached) hid->detach(root);
    hid->release();
    return false;
}

static uint8_t *loadFirmwareFromVfs(size_t *len_out, const char **chosen_path) {
    static const char *const kPaths[] = {
        "/private/var/ngpu/novatek_ts_fw_tm.bin",
        "/private/var/ngpu/novatek_ts_fw_boe.bin",
        "/private/var/touch/novatek_ts_fw_tm.bin",
        "/private/var/touch/novatek_ts_fw_boe.bin",
        "/Library/GPUBundles/novatek_ts_fw_tm.bin",
        "/Library/GPUBundles/novatek_ts_fw_boe.bin",
    };
    vfs_context_t ctx = vfs_context_kernel();
    for (int wait_s = 0; wait_s < 90; ++wait_s) {
        for (size_t i = 0; i < sizeof(kPaths) / sizeof(kPaths[0]); ++i) {
            vnode_t vp = nullptr;
            if (vnode_lookup(kPaths[i], 0, &vp, ctx) == 0 && vp) {
                off_t fsz = 0;
                if (vnode_size(vp, &fsz, ctx) == 0 && fsz >= 4096 && fsz <= 512 * 1024) {
                    uint8_t *buf = static_cast<uint8_t *>(IOMallocData((size_t)fsz));
                    if (buf) {
                        int resid = 0;
                        int err = vn_rdwr(UIO_READ, vp, reinterpret_cast<char *>(buf), (int)fsz, 0,
                                          UIO_SYSSPACE, 0, vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
                        vnode_put(vp);
                        if (err == 0 && resid == 0) {
                            *len_out = (size_t)fsz;
                            *chosen_path = kPaths[i];
                            return buf;
                        }
                        IOFreeData(buf, (size_t)fsz);
                        continue;
                    }
                }
                vnode_put(vp);
            }
        }
        IOSleep(1000);
    }
    return nullptr;
}

static void touchWorkerBody(void) {
    IOLog("Tb375fcTouch: worker started stage=%u rot=%u manual_cs=%u clk26m=%u\n",
          gTouchStage, gTouchRot, (unsigned)gSpi.manual_cs, (unsigned)gSpi.use_26m_parent);

    /* Stage 1: Map MMIO pages and log non-invasive snapshot. */
    if (!mapMmioRegion(MT6897_TOPCKGEN_PHYS, MT6897_TOPCKGEN_SIZE, &gTopckgen) ||
        !mapMmioRegion(MT6897_PERICFG_AO_PHYS, MT6897_PERICFG_AO_SIZE, &gPericfgAo) ||
        !mapMmioRegion(MT6897_GPIO_PHYS, MT6897_GPIO_SIZE, &gGpio) ||
        !mapMmioRegion(MT6897_IOCFG_TM_PHYS, MT6897_IOCFG_TM_SIZE, &gIocfgTm) ||
        !mapMmioRegion(MT6897_SPI2_PHYS, MT6897_SPI2_SIZE, &gSpi2)) {
        IOLog("Tb375fcTouch: FATAL MMIO map failed!\n");
        return;
    }

    gSpi.mmio.ctx     = nullptr;
    gSpi.mmio.read32  = mmio_read32;
    gSpi.mmio.write32 = mmio_write32;
    gSpi.mmio.udelay  = mmio_udelay;
    gSpi.topckgen_base   = gTopckgen.va;
    gSpi.pericfg_ao_base = gPericfgAo.va;
    gSpi.gpio_base       = gGpio.va;
    gSpi.iocfg_tm_base   = gIocfgTm.va;
    gSpi.spi2_base       = gSpi2.va;
    gSpi.bus_hz          = 1000000u; /* Boot 52 proven Tianma NT36532 SPI2 config: clk26m=1 mcs=1 hz=1M tdly=0 */
    gSpi.tick_dly        = 0u;

    mt6897_hw_snapshot_t snap;
    mt6897_spi_snapshot(&gSpi, &snap);
    logSnapshot("stage1-pre", snap);
    if (gTouchStage < 2u) return;

    /* Stage 2: Enable clocks, configure pins, initialize SPI2 IPM v2 controller. */
    int clk_rc = mt6897_spi_enable_clocks(&gSpi);
    mt6897_spi_configure_pins(&gSpi);
    IOSleep(25); /* Allow NT36532 reset recovery after GPIO 60 (RESET) rises 0->1 */
    mt6897_spi_init_controller(&gSpi);
    mt6897_spi_snapshot(&gSpi, &snap);
    IOLog("Tb375fcTouch: stage2 clk_rc=%d src_hz=%u bus_hz=%u\n", clk_rc, gSpi.src_clk_hz, gSpi.bus_hz);
    logSnapshot("stage2-post", snap);
    if (clk_rc != 0 || gTouchStage < 3u) return;

    /* Stage 3: Identify NT36532 over live SPI2 (with 1-shot diagnostic sweep if default fails). */
    nvt_bus_t bus = { &gSpi, bus_spi_xfer, bus_msleep };
    nvt_chip_id_t cid = {};
    /* First issue a clean HW reset pulse on GPIO 60: low 10ms -> high 30ms */
    mt6897_tp_set_reset(&gSpi, 0);
    IOSleep(10);
    mt6897_tp_set_reset(&gSpi, 1);
    IOSleep(30);

    int id_rc = nvt_identify(&bus, &cid);
    IOLog("Tb375fcTouch: stage3 nvt_identify rc=%d matched=%u trim=[%02x %02x %02x %02x %02x %02x] "
          "addr=0x%06x swrst=0x%06x\n",
          id_rc, (unsigned)cid.matched_nt36532,
          cid.trim[0], cid.trim[1], cid.trim[2], cid.trim[3], cid.trim[4], cid.trim[5],
          cid.trim_addr, cid.swrst_addr);

    if (id_rc != 0 || !cid.matched_nt36532) {
        /* 1-shot diagnostic sweep across (clk26m, manual_cs, bus_hz, tick_dly) to lock onto NT36532 */
        static const struct { uint8_t clk26m; uint8_t mcs; uint32_t hz; uint8_t tdly; } kSweep[] = {
            { 1, 0, 1000000u, 0 },
            { 1, 1, 1000000u, 0 },
            { 0, 1, 1000000u, 3 },
            { 0, 0, 1000000u, 0 },
            { 0, 1, 9600000u, 3 },
            { 1, 1, 6500000u, 0 },
        };
        for (size_t s = 0; s < sizeof(kSweep) / sizeof(kSweep[0]); ++s) {
            gSpi.use_26m_parent = kSweep[s].clk26m;
            gSpi.manual_cs      = kSweep[s].mcs;
            gSpi.bus_hz         = kSweep[s].hz;
            gSpi.tick_dly       = kSweep[s].tdly;
            (void)mt6897_spi_enable_clocks(&gSpi);
            mt6897_spi_configure_pins(&gSpi);
            mt6897_spi_init_controller(&gSpi);
            mt6897_tp_set_reset(&gSpi, 0);
            IOSleep(10);
            mt6897_tp_set_reset(&gSpi, 1);
            IOSleep(25);
            nvt_chip_id_t scid = {};
            int src = nvt_identify(&bus, &scid);
            IOLog("Tb375fcTouch[sweep#%zu]: clk26m=%u mcs=%u hz=%u tdly=%u -> rc=%d match=%u trim=[%02x %02x %02x %02x %02x %02x]\n",
                  s, (unsigned)kSweep[s].clk26m, (unsigned)kSweep[s].mcs, kSweep[s].hz, (unsigned)kSweep[s].tdly,
                  src, (unsigned)scid.matched_nt36532,
                  scid.trim[0], scid.trim[1], scid.trim[2], scid.trim[3], scid.trim[4], scid.trim[5]);
            if (src == 0 && scid.matched_nt36532) {
                cid = scid;
                id_rc = 0;
                break;
            }
        }
    }
    if (id_rc != 0 || !cid.matched_nt36532 || gTouchStage < 4u) return;

    /* Stage 4: Load firmware from VFS (or check if already running), download SRAM FW, publish HID, poll! */
    size_t fw_len = 0;
    const char *fw_path = nullptr;
    uint8_t *fw_buf = loadFirmwareFromVfs(&fw_len, &fw_path);
    if (!fw_buf) {
        IOLog("Tb375fcTouch: stage4 firmware not found in /private/var/ngpu/novatek_ts_fw_{tm,boe}.bin\n");
        return;
    }
    IOLog("Tb375fcTouch: stage4 loaded %s (%zu B), downloading to NT36532 SRAM...\n", fw_path, fw_len);
    nvt_fw_info_t fw_info = {};
    int dl_rc = nvt_download_firmware(&bus, &cid, fw_buf, fw_len, &fw_info);
    IOLog("Tb375fcTouch: stage4 download rc=%d fw_ver=0x%02x PID=0x%04x %ux%u max=(%u,%u)\n",
          dl_rc, fw_info.fw_ver, fw_info.pid, fw_info.x_num, fw_info.y_num,
          fw_info.abs_x_max, fw_info.abs_y_max);
    if (dl_rc != 0) {
        uint8_t cflg = 0xFF, rst5[5] = {0}, ilm_c[4] = {0}, dlm_c[4] = {0}, sram8[8] = {0};
        (void)nvt_read_bytes(&bus, 0x001FB533u, &cflg, 1u);
        (void)nvt_read_bytes(&bus, 0x001FB520u, ilm_c, 4u);
        (void)nvt_read_bytes(&bus, 0x001FB524u, dlm_c, 4u);
        (void)nvt_read_bytes(&bus, NVT36532_EVENT_BUF_ADDR + NVT36532_RESET_STATE_OFF, rst5, 5u);
        (void)nvt_read_bytes(&bus, 0x00000000u, sram8, 8u);
        uint32_t ilm_hw = (uint32_t)ilm_c[0] | ((uint32_t)ilm_c[1] << 8) | ((uint32_t)ilm_c[2] << 16) | ((uint32_t)ilm_c[3] << 24);
        uint32_t dlm_hw = (uint32_t)dlm_c[0] | ((uint32_t)dlm_c[1] << 8) | ((uint32_t)dlm_c[2] << 16) | ((uint32_t)dlm_c[3] << 24);
        IOLog("Tb375fcTouch[diag]: crc_flg=0x%02x ilm_hw=0x%08x dlm_hw=0x%08x rst=[%02x %02x %02x %02x %02x]\n",
              cflg, ilm_hw, dlm_hw, rst5[0], rst5[1], rst5[2], rst5[3], rst5[4]);
        IOLog("Tb375fcTouch[diag]: sram0=[%02x %02x %02x %02x %02x %02x %02x %02x] fw200=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
              sram8[0], sram8[1], sram8[2], sram8[3], sram8[4], sram8[5], sram8[6], sram8[7],
              fw_buf[0x200], fw_buf[0x201], fw_buf[0x202], fw_buf[0x203],
              fw_buf[0x204], fw_buf[0x205], fw_buf[0x206], fw_buf[0x207]);
        uint8_t dlm8[8] = {0};
        (void)nvt_read_bytes(&bus, 0x00100000u, dlm8, 8u);
        IOLog("Tb375fcTouch[diag]: dlm0=[%02x %02x %02x %02x %02x %02x %02x %02x] fw22000=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
              dlm8[0], dlm8[1], dlm8[2], dlm8[3], dlm8[4], dlm8[5], dlm8[6], dlm8[7],
              fw_buf[0x22000], fw_buf[0x22001], fw_buf[0x22002], fw_buf[0x22003],
              fw_buf[0x22004], fw_buf[0x22005], fw_buf[0x22006], fw_buf[0x22007]);
        IOFreeData(fw_buf, fw_len);
        return;
    }

    bool hid_ok = registerHidDevice();
    nvt_pointer_state_t pstate;
    nvt_pointer_init(&pstate);
    nvt_rotation_t rot = static_cast<nvt_rotation_t>(gTouchRot & 3u);

    uint32_t log_counter = 0;
    int last_irq = 0;
    for (;;) {
        /* NT36532 irq-gpio flags = 0x2001 (RISING edge / active high). Poll at ~125 Hz (8 ms),
         * reading whenever IRQ asserts or while fingers are active. */
        int irq = mt6897_tp_read_irq(&gSpi);
        if (irq || pstate.mode != 0u || (irq != last_irq)) {
            nvt_frame_t frame;
            int frc = nvt_read_frame(&bus, rot, &frame);
            if (frc == 0) {
                if (frame.esd_reset_req) {
                    IOLog("Tb375fcTouch: ESD reset requested — re-downloading firmware\n");
                    (void)nvt_download_firmware(&bus, &cid, fw_buf, fw_len, &fw_info);
                } else if (frame.checksum_ok) {
                    nvt_hid_pointer_report_t reps[2];
                    int nrep = nvt_pointer_update(&pstate, &frame, now_uptime_ms(), reps);
                    if (gTouchLog && frame.contact_count > 0u && ((log_counter++ % 16u) == 0u)) {
                        IOLog("Tb375fcTouch: contacts=%u [0] raw=(%u,%u) disp=(%u,%u) btn=0x%x\n",
                              frame.contact_count,
                              frame.contacts[0].raw_x, frame.contacts[0].raw_y,
                              frame.contacts[0].disp_x, frame.contacts[0].disp_y,
                              nrep > 0 ? reps[0].buttons : 0u);
                    }
                    if (hid_ok) {
                        for (int r = 0; r < nrep; ++r) {
                            (void)tb375fc_touch_hid_send_report(&reps[r]);
                        }
                    }
                }
            }
        }
        last_irq = irq;
        IOSleep(8);
    }
}

static void touchWorkerThread(void *, wait_result_t) {
    touchWorkerBody();
    IOLog("Tb375fcTouch: worker exiting cleanly (stage=%u)\n", gTouchStage);
    (void)thread_terminate(current_thread());
}

}  // namespace

extern "C" void tb375fc_mali_probe(void);
extern "C" void tb375fc_disp_probe(void);
extern "C" void tb375fc_touch_probe(void);
extern "C" void tb375fc_touch_probe(void) {
    static bool started = false;
    if (started) return;
    started = true;

    tb375fc_mali_probe();
    tb375fc_disp_probe();

    uint32_t stage = 0;
    if (!PE_parse_boot_argn("touch", &stage, sizeof(stage)) || stage == 0u) {
        return; /* Strictly OFF unless boot-arg touch=1..4 is explicitly passed! */
    }
    gTouchStage = stage;
    (void)PE_parse_boot_argn("touch-rot", &gTouchRot, sizeof(gTouchRot));
    (void)PE_parse_boot_argn("touch-log", &gTouchLog, sizeof(gTouchLog));
    gSpi.manual_cs      = 1u;
    gSpi.use_26m_parent = 1u;
    uint32_t mcs = 1, c26 = 1;
    if (PE_parse_boot_argn("touch-manual-cs", &mcs, sizeof(mcs))) gSpi.manual_cs = (uint8_t)(mcs != 0u);
    if (PE_parse_boot_argn("touch-26m", &c26, sizeof(c26))) gSpi.use_26m_parent = (uint8_t)(c26 != 0u);

    thread_t th = THREAD_NULL;
    if (kernel_thread_start(touchWorkerThread, nullptr, &th) == KERN_SUCCESS && th != THREAD_NULL) {
        thread_deallocate(th);
    }
}
