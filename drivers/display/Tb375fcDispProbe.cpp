/* SPDX-License-Identifier: BSD-2-Clause-Patent
 * TB375FC / MT6897 P4 Read-Only Display & Refresh-Rate Probe (disp=1).
 *
 * Strictly READ-ONLY (ZERO writes to any display/DSI/mutex/MMSYS register).
 * Measures the live physical refresh rate of the Tianma NT36532 dual-DSI panel
 * left running by LK/UEFI before any 120 Hz mode/vblank changes.
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <stdint.h>

namespace {

struct MappedIo {
    IOMemoryDescriptor *desc = nullptr;
    IOMemoryMap        *map  = nullptr;
    uintptr_t           va   = 0;
};

static bool map_io(uint64_t pa, size_t len, MappedIo &out) {
    out.desc = IOMemoryDescriptor::withPhysicalAddress(pa, len, kIODirectionInOut);
    if (!out.desc) return false;
    out.map = out.desc->createMappingInTask(kernel_task, 0, kIOMapAnywhere | kIOMapInhibitCache);
    if (!out.map) {
        out.desc->release();
        out.desc = nullptr;
        return false;
    }
    out.va = (uintptr_t)out.map->getVirtualAddress();
    return out.va != 0;
}

static void unmap_io(MappedIo &m) {
    if (m.map)  { m.map->release();  m.map = nullptr; }
    if (m.desc) { m.desc->release(); m.desc = nullptr; }
    m.va = 0;
}

static inline uint32_t rd32(uintptr_t base, uint32_t off) {
    uint32_t v = *(volatile uint32_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}

static inline uint64_t read_cntpct(void) {
    uint64_t v;
    __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(v));
    return v;
}

static inline uint64_t read_cntfrq(void) {
    uint64_t v;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v ? v : 13000000ULL;
}

static void dump_dsi(const char *tag, uintptr_t va) {
    IOLog("Tb375fcDisp[%s]: START=0x%x STA=0x%x INTEN=0x%x INTSTA=0x%x CON=0x%x MODE=0x%x TXRX=0x%x PSCON=0x%x\n",
          tag, rd32(va, 0x00), rd32(va, 0x04), rd32(va, 0x08), rd32(va, 0x0c),
          rd32(va, 0x10), rd32(va, 0x14), rd32(va, 0x18), rd32(va, 0x1c));
    IOLog("Tb375fcDisp[%s]: V=(VSA=%u VBP=%u VFP=%u VACT=%u) H=(HSA=%u HBP=%u HFP=%u BLLP=%u)\n",
          tag, rd32(va, 0x20), rd32(va, 0x24), rd32(va, 0x28), rd32(va, 0x2c),
          rd32(va, 0x50), rd32(va, 0x54), rd32(va, 0x58), rd32(va, 0x5c));
    IOLog("Tb375fcDisp[%s]: PHY_TIM=(0x%08x 0x%08x 0x%08x 0x%08x) DBG0..3=(0x%08x 0x%08x 0x%08x 0x%08x)\n",
          tag, rd32(va, 0x110), rd32(va, 0x114), rd32(va, 0x118), rd32(va, 0x11c),
          rd32(va, 0x160), rd32(va, 0x164), rd32(va, 0x168), rd32(va, 0x16c));
    IOLog("Tb375fcDisp[%s]: DBG4..8=(0x%08x 0x%08x 0x%08x 0x%08x 0x%08x)\n",
          tag, rd32(va, 0x170), rd32(va, 0x174), rd32(va, 0x178), rd32(va, 0x17c), rd32(va, 0x180));
}

static void measure_refresh(uintptr_t dsi0_va) {
    uint64_t t_start_ns = 0, now_ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &t_start_ns);
    const uint64_t duration_ns = 180000000ULL; /* 180 ms */

    uint32_t prev_d2 = rd32(dsi0_va, 0x168) & 0x1fffu;
    uint32_t min_d2 = prev_d2, max_d2 = prev_d2;
    uint64_t wrap_ns[48];
    unsigned wrap_cnt = 0;
    unsigned samples = 0;

    for (;;) {
        IODelay(25);
        absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
        if (now_ns - t_start_ns >= duration_ns) break;
        ++samples;
        uint32_t d2 = rd32(dsi0_va, 0x168) & 0x1fffu;
        if (d2 < min_d2) min_d2 = d2;
        if (d2 > max_d2) max_d2 = d2;
        /* Live line counter wraps from near VACT (1840) back near 0 on every physical frame */
        if (prev_d2 > 1200u && d2 < 400u && wrap_cnt < 48u) {
            wrap_ns[wrap_cnt++] = now_ns;
        }
        prev_d2 = d2;
    }

    uint64_t wrap_us = 0, wrap_hz10 = 0;
    if (wrap_cnt >= 2u) {
        uint64_t total_ns = wrap_ns[wrap_cnt - 1u] - wrap_ns[0];
        wrap_us = total_ns / (1000ULL * (uint64_t)(wrap_cnt - 1u));
        if (wrap_us > 0) wrap_hz10 = 10000000ULL / wrap_us;
    }

    IOLog("Tb375fcDisp[RATE]: elapsed=%llu ms samples=%u d2_line=[%u..%u] wraps=%u period=%llu us (%llu.%llu Hz)\n",
          (unsigned long long)((now_ns - t_start_ns) / 1000000ULL), samples,
          min_d2, max_d2, wrap_cnt, (unsigned long long)wrap_us,
          (unsigned long long)(wrap_hz10 / 10ULL), (unsigned long long)(wrap_hz10 % 10ULL));
}

} // namespace

extern "C" void tb375fc_disp_probe(void);
extern "C" void tb375fc_disp_probe(void) {
    uint32_t stage = 0;
    if (!PE_parse_boot_argn("disp", &stage, sizeof(stage)) || stage == 0u) {
        return;
    }
    IOLog("Tb375fcDisp: read-only probe start (disp=%u)\n", stage);

    MappedIo dsi0, dsi1, mutex0, mmsys0;
    if (map_io(0x1400d000ULL, 0x1000, dsi0)) {
        dump_dsi("DSI0", dsi0.va);
        measure_refresh(dsi0.va);
        unmap_io(dsi0);
    } else {
        IOLog("Tb375fcDisp: failed to map DSI0 @ 0x1400d000\n");
    }

    if (map_io(0x1420d000ULL, 0x1000, dsi1)) {
        dump_dsi("DSI1", dsi1.va);
        unmap_io(dsi1);
    }

    if (map_io(0x14001000ULL, 0x1000, mutex0)) {
        IOLog("Tb375fcDisp[MUTEX0]: INTEN=0x%08x INTSTA=0x%08x M0_EN=0x%08x M0_SOF=0x%08x M0_MOD0=0x%08x M0_MOD1=0x%08x\n",
              rd32(mutex0.va, 0x00), rd32(mutex0.va, 0x04), rd32(mutex0.va, 0x20),
              rd32(mutex0.va, 0x2c), rd32(mutex0.va, 0x30), rd32(mutex0.va, 0x34));
        unmap_io(mutex0);
    }

    if (map_io(0x14000000ULL, 0x1000, mmsys0)) {
        IOLog("Tb375fcDisp[MMSYS0]: CG0=0x%08x CG1=0x%08x CG2=0x%08x\n",
              rd32(mmsys0.va, 0x100), rd32(mmsys0.va, 0x110), rd32(mmsys0.va, 0x1a0));
        unmap_io(mmsys0);
    }
    IOLog("Tb375fcDisp: read-only probe done\n");
}
