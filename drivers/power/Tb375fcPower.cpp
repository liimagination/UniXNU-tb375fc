/*
 * Tb375fcPower.cpp — MT6897 / TB375FC Power, Charger, Backlight & IOPMPowerSource Battery Driver for XNU
 *
 * Strictly isolated in local-artifacts/macos27-firstboot/ram-expansion/kernel/.
 * Hardware addresses verified from TB375FC live-fdt.dtb and MT6897 Linux 6.1 BSP:
 *   - disp_pwm0@1100e000  : mediatek,mt6897-disp-pwm (backlight duty cycle)
 *   - i2c5@11f01000       : mediatek,mt6897-i2c (fifo-use-polling) -> mt6375@34
 *   - i2c@11d02000        : mediatek,mt6897-i2c -> mm8013a_gauge@55
 *
 * Publishes a live IOPMPowerSource (Tb375fcBattery) IOKit service backed by the real
 * MM8013 fuel gauge (10,200 mAh) + MT6375 charger so macOS powerd, pmset, Menu Bar,
 * Control Center, and System Settings -> Battery display accurate live battery & charging state.
 */

#include <stdint.h>
#include <stdbool.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <IOKit/pwr_mgt/IOPM.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSSymbol.h>

extern "C" {
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern void delay(int usec);
extern void kprintf(const char *fmt, ...);
extern void clock_get_calendar_microtime(clock_sec_t *secs, clock_usec_t *microsecs);
}

/* ---- MMIO helpers ---- */
static inline uint32_t mmio_r32(vm_offset_t base, uint32_t off) {
    uint32_t v = *(volatile uint32_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}
static inline void mmio_w32(vm_offset_t base, uint32_t off, uint32_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint32_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}
static inline uint16_t mmio_r16(vm_offset_t base, uint32_t off) {
    uint16_t v = *(volatile uint16_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}
static inline void mmio_w16(vm_offset_t base, uint32_t off, uint16_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint16_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}
static inline uint8_t mmio_r8(vm_offset_t base, uint32_t off) {
    uint8_t v = *(volatile uint8_t *)(base + off);
    __asm__ volatile("dsb ld" ::: "memory");
    return v;
}
static inline void mmio_w8(vm_offset_t base, uint32_t off, uint8_t v) {
    __asm__ volatile("dsb st" ::: "memory");
    *(volatile uint8_t *)(base + off) = v;
    __asm__ volatile("dsb st" ::: "memory");
}

/* ---- Hardware base addresses (from TB375FC live-fdt.dtb) ---- */
#define PHYS_DISP_PWM0      0x1100e000ULL
#define PHYS_I2C5_MT6375    0x11f01000ULL
#define PHYS_I2C_GAUGE      0x11d02000ULL
#define PHYS_FB_LAYER0      0xfc16f000ULL
#define FB_LAYER0_BYTES     0x014aa000ULL

/* ---- MT6897 I2C v2 + slave_addr_ver=1 register offsets ---- */
#define I2C_DATA_PORT         0x00
#define I2C_INTR_MASK         0x08
#define I2C_INTR_STAT         0x0C
#define I2C_CONTROL           0x10
#define I2C_TRANSFER_LEN      0x14
#define I2C_TRANSAC_LEN       0x18
#define I2C_START             0x24
#define I2C_FIFO_ADDR_CLR     0x38
#define I2C_TRANSFER_LEN_AUX  0x44
#define I2C_SLAVE_ADDR1       0x94

#define I2C_INTR_COMP         (1U << 0)
#define I2C_INTR_ACKERR       (1U << 1)
#define I2C_INTR_HS_NACKERR   (1U << 2)
#define I2C_INTR_ARB_LOST     (1U << 3)
#define I2C_INTR_ALL          0x01FFU

#define I2C_CTRL_RS           (1U << 1)
#define I2C_CTRL_CLK_EXT_EN   (1U << 3)
#define I2C_CTRL_DIR_CHANGE   (1U << 4)
#define I2C_CTRL_ACKERR_DET   (1U << 5)
#define I2C_CTRL_LEN_CHANGE   (1U << 6)

/* ---- MT6375 PMU (Bank 0x01 -> 7-bit I2C slave 0x34) ---- */
#define MT6375_SLAVE_PMU      0x34
#define MT6375_REG_DEV_INFO   0x00
#define MT6375_REG_CHG_TOP1   0x20
#define MT6375_REG_CHG_AICR   0x22
#define MT6375_REG_CHG_MIVR   0x23
#define MT6375_REG_CHG_VCHG   0x25
#define MT6375_REG_CHG_ICHG   0x26
#define MT6375_REG_CHG_WDT    0x2A
#define MT6375_REG_CHG_STAT   0x34
#define MT6375_REG_CHG_STAT0  0xE0
#define MT6375_REG_CHG_STAT1  0xE1

#define MT6375_VID_MASK       0xF0
#define MT6375_VID_EXPECTED   0x70

/* ---- MM8013A Fuel Gauge (7-bit I2C slave 0x55, 10,200 mAh pack) ---- */
#define MM8013_SLAVE_ADDR     0x55
#define MM8013_REG_TEMP       0x06  /* 0.1 K, 16-bit LE */
#define MM8013_REG_VOLT       0x08  /* mV, 16-bit LE */
#define MM8013_REG_CURR       0x14  /* mA, signed 16-bit LE */
#define MM8013_REG_CYCLE      0x2A  /* cycle count, 16-bit LE */
#define MM8013_REG_SOC        0x2C  /* %, 16-bit LE */
#define TB375FC_DESIGN_MAH    10200U

/* ---- IOPMPowerSource Battery Service (AppleSmartBattery) for macOS powerd / pmset / Menu Bar / System Settings ---- */
class AppleSmartBattery : public IOPMPowerSource {
    OSDeclareDefaultStructors(AppleSmartBattery)

private:
    const OSSymbol *symFullyCharged;
    const OSSymbol *symDesignCapacity;
    const OSSymbol *symNominalCapacity;
    const OSSymbol *symRawCurCap;
    const OSSymbol *symRawMaxCap;
    const OSSymbol *symRawVoltage;
    const OSSymbol *symRawExtConn;
    const OSSymbol *symInstantAmperage;
    const OSSymbol *symInstantTimeToEmpty;
    const OSSymbol *symTemperature;
    const OSSymbol *symAvgTimeToEmpty;
    const OSSymbol *symAvgTimeToFull;
    const OSSymbol *symAdapterDetails;
    const OSSymbol *symRawAdapterDetails;
    const OSSymbol *symBestAdapterIndex;
    const OSSymbol *symAdapterInfo;
    const OSSymbol *symDeviceName;
    const OSSymbol *symSerialStr;
    const OSSymbol *symMaxErr;
    const OSSymbol *symBuiltIn;
    const OSSymbol *symPermFailure;
    const OSSymbol *symUpdateTime;
    const OSSymbol *symBootPathUpdated;
    const OSSymbol *symFullPathUpdated;
    const OSSymbol *symUserVisPathUpdated;
    const OSSymbol *symDesignCycleCount70;
    const OSSymbol *symDesignCycleCount9C;
    const OSSymbol *symInvalidWakeSecs;
    const OSSymbol *symPostChargeWait;
    const OSSymbol *symPostDischargeWait;
    const OSSymbol *symOperationStatus;
    const OSSymbol *symLegacyBatteryInfo;
    const OSSymbol *symBatteryData;
    uint32_t bootPathTs;

public:
    virtual bool init(void) override;
    virtual void free(void) override;
    void publishSample(uint32_t soc, uint32_t vbat_mv, int32_t ibat_ma,
                       uint32_t temp_dk, uint32_t cycles,
                       bool ext_connected, bool is_charging, uint32_t aicr_ma);
};

#define super IOPMPowerSource
OSDefineMetaClassAndStructors(AppleSmartBattery, IOPMPowerSource)

bool AppleSmartBattery::init(void) {
    if (!super::init()) return false;

    symFullyCharged       = OSSymbol::withCString("FullyCharged");
    symDesignCapacity     = OSSymbol::withCString("DesignCapacity");
    symNominalCapacity    = OSSymbol::withCString("NominalChargeCapacity");
    symRawCurCap          = OSSymbol::withCString("AppleRawCurrentCapacity");
    symRawMaxCap          = OSSymbol::withCString("AppleRawMaxCapacity");
    symRawVoltage         = OSSymbol::withCString("AppleRawBatteryVoltage");
    symRawExtConn         = OSSymbol::withCString("AppleRawExternalConnected");
    symInstantAmperage    = OSSymbol::withCString("InstantAmperage");
    symInstantTimeToEmpty = OSSymbol::withCString("InstantTimeToEmpty");
    symTemperature        = OSSymbol::withCString("Temperature");
    symAvgTimeToEmpty     = OSSymbol::withCString("AvgTimeToEmpty");
    symAvgTimeToFull      = OSSymbol::withCString("AvgTimeToFull");
    symAdapterDetails     = OSSymbol::withCString("AdapterDetails");
    symRawAdapterDetails  = OSSymbol::withCString("AppleRawAdapterDetails");
    symBestAdapterIndex   = OSSymbol::withCString("BestAdapterIndex");
    symAdapterInfo        = OSSymbol::withCString("AdapterInfo");
    symDeviceName         = OSSymbol::withCString("DeviceName");
    symSerialStr          = OSSymbol::withCString("BatterySerialNumber");
    symMaxErr             = OSSymbol::withCString("MaxErr");
    symBuiltIn            = OSSymbol::withCString("BuiltIn");
    symPermFailure        = OSSymbol::withCString("PermanentFailureStatus");
    symUpdateTime         = OSSymbol::withCString("UpdateTime");
    symBootPathUpdated    = OSSymbol::withCString("BootPathUpdated");
    symFullPathUpdated    = OSSymbol::withCString("FullPathUpdated");
    symUserVisPathUpdated = OSSymbol::withCString("UserVisiblePathUpdated");
    symDesignCycleCount70 = OSSymbol::withCString("DesignCycleCount70");
    symDesignCycleCount9C = OSSymbol::withCString("DesignCycleCount9C");
    symInvalidWakeSecs    = OSSymbol::withCString("BatteryInvalidWakeSeconds");
    symPostChargeWait     = OSSymbol::withCString("PostChargeWaitSeconds");
    symPostDischargeWait  = OSSymbol::withCString("PostDischargeWaitSeconds");
    symOperationStatus    = OSSymbol::withCString("OperationStatus");
    symLegacyBatteryInfo  = OSSymbol::withCString("LegacyBatteryInfo");
    symBatteryData        = OSSymbol::withCString("BatteryData");
    bootPathTs            = 0U;

    OSSymbol *mfg = const_cast<OSSymbol *>(OSSymbol::withCString("Lenovo"));
    OSSymbol *mdl = const_cast<OSSymbol *>(OSSymbol::withCString("TB375FC"));
    OSSymbol *ser = const_cast<OSSymbol *>(OSSymbol::withCString("TB375FC-BATT"));
    if (mfg) { setManufacturer(mfg); mfg->release(); }
    if (mdl) { setModel(mdl); mdl->release(); }
    if (ser) { setSerial(ser); ser->release(); }

    OSString *devName = OSString::withCString("TB375FC");
    OSString *serStr  = OSString::withCString("TB375FC-BATT");
    if (devName) { setPSProperty(symDeviceName, devName); devName->release(); }
    if (serStr)  { setPSProperty(symSerialStr, serStr); serStr->release(); }
    setPSProperty(symBuiltIn, kOSBooleanTrue);

    auto setStaticU32 = [&](const OSSymbol *sym, uint32_t v) {
        if (!sym) return;
        OSNumber *n = OSNumber::withNumber((unsigned long long)v, 32);
        if (n) { setPSProperty(sym, n); n->release(); }
    };
    setStaticU32(symDesignCapacity, TB375FC_DESIGN_MAH);
    setStaticU32(symNominalCapacity, TB375FC_DESIGN_MAH);
    setStaticU32(symRawMaxCap, TB375FC_DESIGN_MAH);
    setStaticU32(symMaxErr, 1U);
    setStaticU32(symPermFailure, 0U);
    setStaticU32(symDesignCycleCount70, 0U);
    setStaticU32(symDesignCycleCount9C, 1000U);
    setStaticU32(symInvalidWakeSecs, 30U);
    setStaticU32(symPostChargeWait, 120U);
    setStaticU32(symPostDischargeWait, 120U);
    setStaticU32(symOperationStatus, 0U);
    setStaticU32(symBestAdapterIndex, 0U);
    setStaticU32(symAdapterInfo, 0U);

    setBatteryInstalled(true);
    setMaxCapacity(100);
    setLocation(0);
    return true;
}

void AppleSmartBattery::free(void) {
    OSSafeReleaseNULL(symFullyCharged);
    OSSafeReleaseNULL(symDesignCapacity);
    OSSafeReleaseNULL(symNominalCapacity);
    OSSafeReleaseNULL(symRawCurCap);
    OSSafeReleaseNULL(symRawMaxCap);
    OSSafeReleaseNULL(symRawVoltage);
    OSSafeReleaseNULL(symRawExtConn);
    OSSafeReleaseNULL(symInstantAmperage);
    OSSafeReleaseNULL(symInstantTimeToEmpty);
    OSSafeReleaseNULL(symTemperature);
    OSSafeReleaseNULL(symAvgTimeToEmpty);
    OSSafeReleaseNULL(symAvgTimeToFull);
    OSSafeReleaseNULL(symAdapterDetails);
    OSSafeReleaseNULL(symRawAdapterDetails);
    OSSafeReleaseNULL(symBestAdapterIndex);
    OSSafeReleaseNULL(symAdapterInfo);
    OSSafeReleaseNULL(symDeviceName);
    OSSafeReleaseNULL(symSerialStr);
    OSSafeReleaseNULL(symMaxErr);
    OSSafeReleaseNULL(symBuiltIn);
    OSSafeReleaseNULL(symPermFailure);
    OSSafeReleaseNULL(symUpdateTime);
    OSSafeReleaseNULL(symBootPathUpdated);
    OSSafeReleaseNULL(symFullPathUpdated);
    OSSafeReleaseNULL(symUserVisPathUpdated);
    OSSafeReleaseNULL(symDesignCycleCount70);
    OSSafeReleaseNULL(symDesignCycleCount9C);
    OSSafeReleaseNULL(symInvalidWakeSecs);
    OSSafeReleaseNULL(symPostChargeWait);
    OSSafeReleaseNULL(symPostDischargeWait);
    OSSafeReleaseNULL(symOperationStatus);
    OSSafeReleaseNULL(symLegacyBatteryInfo);
    OSSafeReleaseNULL(symBatteryData);
    super::free();
}

void AppleSmartBattery::publishSample(uint32_t soc, uint32_t vbat_mv, int32_t ibat_ma,
                                      uint32_t temp_dk, uint32_t cycles,
                                      bool ext_connected, bool is_charging, uint32_t aicr_ma) {
    if (soc > 100U) soc = 100U;
    bool fully_charged = ext_connected && (soc >= 100U);
    if (fully_charged) is_charging = false;

    uint32_t raw_cur_mah = (soc * TB375FC_DESIGN_MAH) / 100U;
    int32_t time_rem = 65535;
    uint32_t tte = 65535U;
    uint32_t ttf = 65535U;
    if (fully_charged) {
        time_rem = 0;
        ttf = 0U;
    } else if (is_charging && ibat_ma > 50) {
        uint32_t rem_mah = (TB375FC_DESIGN_MAH > raw_cur_mah) ? (TB375FC_DESIGN_MAH - raw_cur_mah) : 0U;
        uint32_t mins = (rem_mah * 60U) / (uint32_t)ibat_ma;
        if (mins < 1U) mins = 1U;
        if (mins > 600U) mins = 600U;
        time_rem = (int32_t)mins;
        ttf = mins;
    } else if (!ext_connected && ibat_ma < -50) {
        uint32_t mins = (raw_cur_mah * 60U) / (uint32_t)(-ibat_ma);
        if (mins < 1U) mins = 1U;
        if (mins > 1440U) mins = 1440U;
        time_rem = (int32_t)mins;
        tte = mins;
    }

    setBatteryInstalled(true);
    setExternalConnected(ext_connected);
    setExternalChargeCapable(ext_connected);
    setIsCharging(is_charging);
    setAtWarnLevel(!ext_connected && soc <= 15U);
    setAtCriticalLevel(!ext_connected && soc <= 5U);
    setCurrentCapacity(soc);
    setMaxCapacity(100U);
    setVoltage(vbat_mv);
    setAmperage(ibat_ma);
    setTimeRemaining(time_rem);
    setCycleCount(cycles);

    setPSProperty(symFullyCharged, fully_charged ? kOSBooleanTrue : kOSBooleanFalse);
    setPSProperty(symRawExtConn, ext_connected ? kOSBooleanTrue : kOSBooleanFalse);

    auto setU32 = [&](const OSSymbol *sym, uint32_t v) {
        if (!sym) return;
        OSNumber *n = OSNumber::withNumber((unsigned long long)v, 32);
        if (n) { setPSProperty(sym, n); n->release(); }
    };
    auto setS32 = [&](const OSSymbol *sym, int32_t v) {
        if (!sym) return;
        OSNumber *n = OSNumber::withNumber((unsigned long long)(int64_t)v, 32);
        if (n) { setPSProperty(sym, n); n->release(); }
    };

    clock_sec_t secs = 0;
    clock_usec_t usecs = 0;
    clock_get_calendar_microtime(&secs, &usecs);
    uint32_t ts32 = (secs >= 1700000000ULL && secs < 0x7FFFFFFFULL) ? (uint32_t)secs : 1794400000U;
    if (bootPathTs == 0U || (bootPathTs == 1794400000U && ts32 != 1794400000U)) {
        bootPathTs = ts32;
    }

    setU32(symRawCurCap, raw_cur_mah);
    setU32(symRawVoltage, vbat_mv);
    setS32(symInstantAmperage, ibat_ma);
    setU32(symInstantTimeToEmpty, tte);
    setU32(symTemperature, temp_dk);
    setU32(symAvgTimeToEmpty, tte);
    setU32(symAvgTimeToFull, ttf);
    setU32(symUpdateTime, ts32);
    setU32(symBootPathUpdated, bootPathTs);
    setU32(symFullPathUpdated, ts32);
    setU32(symUserVisPathUpdated, ts32);

    /* LegacyBatteryInfo: consumed by SPPowerReporter and legacy IOPowerSources readers */
    OSDictionary *legacy = OSDictionary::withCapacity(6);
    if (legacy) {
        uint32_t flags = 0x04U | (ext_connected ? 0x01U : 0U) | (is_charging ? 0x02U : 0U);
        OSNumber *lAmp = OSNumber::withNumber((unsigned long long)(int64_t)ibat_ma, 32);
        OSNumber *lFlg = OSNumber::withNumber((unsigned long long)flags, 32);
        OSNumber *lCap = OSNumber::withNumber((unsigned long long)TB375FC_DESIGN_MAH, 32);
        OSNumber *lCur = OSNumber::withNumber((unsigned long long)raw_cur_mah, 32);
        OSNumber *lVlt = OSNumber::withNumber((unsigned long long)vbat_mv, 32);
        OSNumber *lCyc = OSNumber::withNumber((unsigned long long)cycles, 32);
        if (lAmp) { legacy->setObject("Amperage", lAmp); lAmp->release(); }
        if (lFlg) { legacy->setObject("Flags", lFlg); lFlg->release(); }
        if (lCap) { legacy->setObject("Capacity", lCap); lCap->release(); }
        if (lCur) { legacy->setObject("Current", lCur); lCur->release(); }
        if (lVlt) { legacy->setObject("Voltage", lVlt); lVlt->release(); }
        if (lCyc) { legacy->setObject("Cycle Count", lCyc); lCyc->release(); }
        setPSProperty(symLegacyBatteryInfo, legacy);
        legacy->release();
    }

    /* BatteryData: consumed by powerd BatteryHealth / SPPowerReporter */
    OSDictionary *bdata = OSDictionary::withCapacity(7);
    if (bdata) {
        OSNumber *dSoc = OSNumber::withNumber((unsigned long long)soc, 32);
        OSNumber *dDes = OSNumber::withNumber((unsigned long long)TB375FC_DESIGN_MAH, 32);
        OSNumber *dCyc = OSNumber::withNumber((unsigned long long)cycles, 32);
        OSNumber *dVlt = OSNumber::withNumber((unsigned long long)vbat_mv, 32);
        OSString *dSer = OSString::withCString("TB375FC-BATT");
        OSDictionary *life = OSDictionary::withCapacity(1);
        if (dSoc) { bdata->setObject("StateOfCharge", dSoc); dSoc->release(); }
        if (dDes) { bdata->setObject("DesignCapacity", dDes); dDes->release(); }
        if (dCyc) { bdata->setObject("CycleCount", dCyc); dCyc->release(); }
        if (dVlt) { bdata->setObject("Voltage", dVlt); dVlt->release(); }
        if (dSer) { bdata->setObject("Serial", dSer); dSer->release(); }
        if (life) {
            OSNumber *lUpd = OSNumber::withNumber((unsigned long long)ts32, 32);
            if (lUpd) { life->setObject("UpdateTime", lUpd); lUpd->release(); }
            bdata->setObject("LifetimeData", life);
            life->release();
        }
        setPSProperty(symBatteryData, bdata);
        bdata->release();
    }

    OSDictionary *adapter = OSDictionary::withCapacity(9);
    if (adapter) {
        uint32_t watts = ext_connected ? ((aicr_ma * 5U + 500U) / 1000U) : 0U;
        if (ext_connected && watts < 5U) watts = 5U;
        OSNumber *nW   = OSNumber::withNumber((unsigned long long)watts, 32);
        OSNumber *nC   = OSNumber::withNumber((unsigned long long)(ext_connected ? aicr_ma : 0U), 32);
        OSNumber *nV   = OSNumber::withNumber((unsigned long long)(ext_connected ? 5000U : 0U), 32);
        OSNumber *nF   = OSNumber::withNumber(ext_connected ? 0x00baULL : 0ULL, 32);
        OSNumber *nId  = OSNumber::withNumber(ext_connected ? 256ULL : 0ULL, 32);
        OSNumber *nRev = OSNumber::withNumber(0ULL, 32);
        OSNumber *nSn  = OSNumber::withNumber(ext_connected ? 1ULL : 0ULL, 32);
        OSString *sD   = OSString::withCString(ext_connected ? "usb host" : "none");
        if (nW)   { adapter->setObject("Watts", nW); nW->release(); }
        if (nC)   { adapter->setObject("Current", nC); nC->release(); }
        if (nV)   { adapter->setObject("Voltage", nV); nV->release(); }
        if (nF)   { adapter->setObject("FamilyCode", nF); nF->release(); }
        if (nId)  { adapter->setObject("AdapterID", nId); nId->release(); }
        if (nRev) { adapter->setObject("AdapterRevision", nRev); nRev->release(); }
        if (nSn)  { adapter->setObject("SerialNumber", nSn); nSn->release(); }
        if (sD)   { adapter->setObject("Description", sD); sD->release(); }
        adapter->setObject("IsWireless", kOSBooleanFalse);
        setPSProperty(symAdapterDetails, adapter);

        OSArray *rawAdapters = OSArray::withCapacity(1);
        if (rawAdapters) {
            if (ext_connected) rawAdapters->setObject(adapter);
            setPSProperty(symRawAdapterDetails, rawAdapters);
            rawAdapters->release();
        }
        adapter->release();
    }

    updateStatus();
}

/* ---- Hardware state ---- */
static vm_offset_t g_pwm0_base  = 0;
static vm_offset_t g_i2c5_base  = 0;
static vm_offset_t g_i2cg_base  = 0;
static uint32_t    g_target_aicr_ma = 1500;
static uint32_t    g_target_bl_pct  = 35;
static bool        g_mt6375_ok  = false;
static AppleSmartBattery *g_battery = nullptr;

static int mtk_i2c_wait_done(vm_offset_t base, uint16_t *out_stat) {
    for (int i = 0; i < 2000; i++) {
        uint16_t st = mmio_r16(base, I2C_INTR_STAT);
        if (st & (I2C_INTR_COMP | I2C_INTR_ACKERR | I2C_INTR_HS_NACKERR | I2C_INTR_ARB_LOST)) {
            mmio_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
            if (out_stat) *out_stat = st;
            if (st & (I2C_INTR_ACKERR | I2C_INTR_HS_NACKERR | I2C_INTR_ARB_LOST)) {
                return -1;
            }
            return 0;
        }
        delay(10);
    }
    if (out_stat) *out_stat = mmio_r16(base, I2C_INTR_STAT);
    mmio_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
    return -2;
}

static int mtk_i2c_read_reg(vm_offset_t base, uint8_t slave7, uint8_t reg, uint8_t *buf, uint32_t len) {
    if (!base || !buf || len == 0 || len > 8) return -3;
    mmio_w16(base, I2C_INTR_MASK, 0);
    mmio_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
    mmio_w16(base, I2C_FIFO_ADDR_CLR, 0x0001);

    uint16_t ctrl = I2C_CTRL_ACKERR_DET | I2C_CTRL_CLK_EXT_EN | I2C_CTRL_DIR_CHANGE | I2C_CTRL_RS;
    if (len != 1) ctrl |= I2C_CTRL_LEN_CHANGE;
    mmio_w16(base, I2C_CONTROL, ctrl);

    mmio_w16(base, I2C_SLAVE_ADDR1, (uint16_t)(slave7 << 1));
    mmio_w16(base, I2C_TRANSFER_LEN, 1);
    mmio_w16(base, I2C_TRANSFER_LEN_AUX, (uint16_t)len);
    mmio_w16(base, I2C_TRANSAC_LEN, 2);

    mmio_w8(base, I2C_DATA_PORT, reg);
    mmio_w16(base, I2C_START, 0x0001);

    uint16_t st = 0;
    int rc = mtk_i2c_wait_done(base, &st);
    if (rc != 0) return rc;

    for (uint32_t i = 0; i < len; i++) {
        buf[i] = mmio_r8(base, I2C_DATA_PORT);
    }
    return 0;
}

static int mtk_i2c_write_reg8(vm_offset_t base, uint8_t slave7, uint8_t reg, uint8_t val) {
    if (!base) return -3;
    mmio_w16(base, I2C_INTR_MASK, 0);
    mmio_w16(base, I2C_INTR_STAT, I2C_INTR_ALL);
    mmio_w16(base, I2C_FIFO_ADDR_CLR, 0x0001);

    mmio_w16(base, I2C_CONTROL, I2C_CTRL_ACKERR_DET | I2C_CTRL_CLK_EXT_EN);
    mmio_w16(base, I2C_SLAVE_ADDR1, (uint16_t)(slave7 << 1));
    mmio_w16(base, I2C_TRANSFER_LEN, 2);
    mmio_w16(base, I2C_TRANSAC_LEN, 1);

    mmio_w8(base, I2C_DATA_PORT, reg);
    mmio_w8(base, I2C_DATA_PORT, val);
    mmio_w16(base, I2C_START, 0x0001);

    uint16_t st = 0;
    return mtk_i2c_wait_done(base, &st);
}

static uint8_t mt6375_aicr_ma_to_sel(uint32_t ma) {
    if (ma < 100) ma = 100;
    if (ma > 3225) ma = 3225;
    return (uint8_t)(((ma - 100) / 25) + 2);
}

static uint32_t mt6375_aicr_sel_to_ma(uint8_t sel) {
    sel &= 0x7F;
    if (sel <= 2) return 100;
    return 100U + (uint32_t)(sel - 2) * 25U;
}

static void tb375fc_tune_backlight(void) {
    g_pwm0_base = ml_io_map(PHYS_DISP_PWM0, 0x1000);
    if (!g_pwm0_base) {
        kprintf("[tb375fc-pwr] pwm0: ml_io_map failed\n");
        return;
    }
    uint32_t en  = mmio_r32(g_pwm0_base, 0x00);
    uint32_t r08 = mmio_r32(g_pwm0_base, 0x08);
    uint32_t r10 = mmio_r32(g_pwm0_base, 0x10);
    uint32_t r14 = mmio_r32(g_pwm0_base, 0x14);
    uint32_t r18 = mmio_r32(g_pwm0_base, 0x18);
    uint32_t r1c = mmio_r32(g_pwm0_base, 0x1C);
    kprintf("[tb375fc-pwr] pwm0: en=0x%x cmt=0x%x r10=0x%x r14=0x%x r18=0x%x r1c=0x%x\n",
            en, r08, r10, r14, r18, r1c);

    if ((en & 1U) == 0) return;

    uint32_t con1_off = 0;
    uint32_t con1_val = 0;
    uint32_t p14 = r14 & 0xFFFU, h14 = (r14 >> 16) & 0x1FFFU;
    uint32_t p1c = r1c & 0xFFFU, h1c = (r1c >> 16) & 0x1FFFU;
    if (p14 >= 63 && h14 > 0 && h14 <= p14 + 1) {
        con1_off = 0x14;
        con1_val = r14;
    } else if (p1c >= 63 && h1c > 0 && h1c <= p1c + 1) {
        con1_off = 0x1C;
        con1_val = r1c;
    }
    if (con1_off == 0) {
        kprintf("[tb375fc-pwr] pwm0: CON1 signature not matched, leaving backlight unchanged\n");
        return;
    }
    uint32_t period = con1_val & 0xFFFU;
    uint32_t cur_hi = (con1_val >> 16) & 0x1FFFU;
    uint32_t new_hi = (period * g_target_bl_pct) / 100U;
    if (new_hi < period / 10U) new_hi = period / 10U;
    if (new_hi >= cur_hi) {
        kprintf("[tb375fc-pwr] pwm0: already dim (%u/%u)\n", cur_hi, period);
        return;
    }
    uint32_t new_val = (con1_val & ~0x1FFF0000U) | (new_hi << 16);
    mmio_w32(g_pwm0_base, con1_off, new_val);
    if (con1_off == 0x14) {
        mmio_w32(g_pwm0_base, 0x08, 1U);
    }
    kprintf("[tb375fc-pwr] pwm0: dimmed off=0x%x duty %u->%u / %u (%u%%)\n",
            con1_off, cur_hi, new_hi, period, g_target_bl_pct);
}

static void tb375fc_tune_mt6375(bool initial) {
    if (!g_i2c5_base) return;
    uint8_t dev = 0;
    int rc = mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_DEV_INFO, &dev, 1);
    if (rc != 0 || (dev & MT6375_VID_MASK) != MT6375_VID_EXPECTED) {
        if (initial) {
            kprintf("[tb375fc-pwr] mt6375 probe failed rc=%d dev=0x%02x (skipping writes)\n", rc, dev);
        }
        return;
    }
    g_mt6375_ok = true;

    uint8_t top1 = 0, aicr = 0, mivr = 0, ichg = 0, wdt = 0, st = 0, st0 = 0, st1 = 0;
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_TOP1, &top1, 1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_AICR, &aicr, 1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_MIVR, &mivr, 1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_ICHG, &ichg, 1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_WDT,  &wdt,  1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_STAT, &st,   1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_STAT0,&st0,  1);
    mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_STAT1,&st1,  1);

    if (initial) {
        kprintf("[tb375fc-pwr] mt6375 BEFORE: dev=0x%02x top1=0x%02x aicr=0x%02x(%umA) mivr=0x%02x ichg=0x%02x wdt=0x%02x st=0x%02x st0=0x%02x st1=0x%02x\n",
                dev, top1, aicr, mt6375_aicr_sel_to_ma(aicr), mivr, ichg, wdt, st, st0, st1);
    }

    uint8_t new_wdt = (uint8_t)((wdt & ~0x08U) | 0x04U);
    mtk_i2c_write_reg8(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_WDT, new_wdt);

    uint8_t new_mivr = (uint8_t)((mivr & 0x80U) | 0x06U);
    mtk_i2c_write_reg8(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_MIVR, new_mivr);

    uint8_t new_aicr = mt6375_aicr_ma_to_sel(g_target_aicr_ma) & 0x7FU;
    mtk_i2c_write_reg8(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_AICR, new_aicr);

    uint8_t new_ichg = (uint8_t)((ichg & 0xC0U) | 0x28U);
    mtk_i2c_write_reg8(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_ICHG, new_ichg);

    uint8_t new_top1 = (uint8_t)((top1 | 0x03U) & ~0x08U);
    mtk_i2c_write_reg8(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_TOP1, new_top1);

    if (initial) {
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_TOP1, &top1, 1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_AICR, &aicr, 1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_MIVR, &mivr, 1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_ICHG, &ichg, 1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_WDT,  &wdt,  1);
        kprintf("[tb375fc-pwr] mt6375 AFTER : top1=0x%02x aicr=0x%02x(%umA) mivr=0x%02x ichg=0x%02x wdt=0x%02x\n",
                top1, aicr, mt6375_aicr_sel_to_ma(aicr), mivr, ichg, wdt);
    }
}

static void tb375fc_ensure_battery_service(uint32_t soc, uint32_t vbat, int32_t ibat,
                                           uint32_t temp_dk, uint32_t cycles,
                                           bool ext, bool chg, uint32_t aicr_ma) {
    if (g_battery) {
        g_battery->publishSample(soc, vbat, ibat, temp_dk, cycles, ext, chg, aicr_ma);
        return;
    }
    IOService *plat = IOService::getPlatform();
    if (!plat) return;

    AppleSmartBattery *bat = new AppleSmartBattery;
    if (!bat) return;
    if (!bat->init()) {
        bat->release();
        return;
    }
    if (!bat->attach(plat)) {
        bat->release();
        return;
    }
    bat->publishSample(soc, vbat, ibat, temp_dk, cycles, ext, chg, aicr_ma);
    bat->registerService();
    g_battery = bat;
    kprintf("[tb375fc-pwr] AppleSmartBattery (IOPMPowerSource) REGISTERED: soc=%u%% vbat=%umV ibat=%dmA temp=%udK cyc=%u ext=%u chg=%u\n",
            soc, vbat, ibat, temp_dk, cycles, ext ? 1U : 0U, chg ? 1U : 0U);
}

static void tb375fc_poll_and_publish(bool do_log) {
    uint8_t aicr = 0, st = 0, st0 = 0, st1 = 0;
    if (g_mt6375_ok && g_i2c5_base) {
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_AICR,  &aicr, 1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_STAT,  &st,   1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_STAT0, &st0,  1);
        mtk_i2c_read_reg(g_i2c5_base, MT6375_SLAVE_PMU, MT6375_REG_CHG_STAT1, &st1,  1);
    }

    uint8_t vb[2] = {0}, ib[2] = {0}, sb[2] = {0}, tb[2] = {0}, cb[2] = {0};
    int rc_v = g_i2cg_base ? mtk_i2c_read_reg(g_i2cg_base, MM8013_SLAVE_ADDR, MM8013_REG_VOLT, vb, 2) : -1;
    int rc_i = g_i2cg_base ? mtk_i2c_read_reg(g_i2cg_base, MM8013_SLAVE_ADDR, MM8013_REG_CURR, ib, 2) : -1;
    int rc_s = g_i2cg_base ? mtk_i2c_read_reg(g_i2cg_base, MM8013_SLAVE_ADDR, MM8013_REG_SOC,  sb, 2) : -1;
    int rc_t = g_i2cg_base ? mtk_i2c_read_reg(g_i2cg_base, MM8013_SLAVE_ADDR, MM8013_REG_TEMP, tb, 2) : -1;
    int rc_c = g_i2cg_base ? mtk_i2c_read_reg(g_i2cg_base, MM8013_SLAVE_ADDR, MM8013_REG_CYCLE, cb, 2) : -1;

    uint32_t aicr_ma = mt6375_aicr_sel_to_ma(aicr);
    bool ext = ((st0 & 1U) != 0U);
    uint8_t chg_st = (uint8_t)(st & 0x0FU);

    if (rc_v == 0 && rc_i == 0 && rc_s == 0) {
        uint16_t vbat = (uint16_t)((uint32_t)vb[0] | ((uint32_t)vb[1] << 8));
        int16_t  ibat = (int16_t)((uint16_t)((uint32_t)ib[0] | ((uint32_t)ib[1] << 8)));
        uint16_t soc  = (uint16_t)((uint32_t)sb[0] | ((uint32_t)sb[1] << 8));
        uint32_t temp_dk = 3000U;
        if (rc_t == 0) {
            uint16_t raw_t = (uint16_t)((uint32_t)tb[0] | ((uint32_t)tb[1] << 8));
            if (raw_t >= 2000U && raw_t <= 3600U) temp_dk = raw_t;
        }
        uint32_t cycles = 1U;
        if (rc_c == 0) {
            uint16_t raw_c = (uint16_t)((uint32_t)cb[0] | ((uint32_t)cb[1] << 8));
            if (raw_c <= 5000U) cycles = raw_c;
        }
        bool is_charging = ext && ((chg_st == 4U) || (chg_st == 3U) || (ibat > 10)) && (soc < 100U);
        tb375fc_ensure_battery_service(soc, vbat, ibat, temp_dk, cycles, ext, is_charging, aicr_ma);
        if (do_log) {
            kprintf("[tb375fc-pwr] bat: soc=%u%% vbat=%umV ibat=%dmA temp=%udK cyc=%u | chg: st=0x%02x pwr=%u mivr=%u aicr=%umA\n",
                    (unsigned)soc, (unsigned)vbat, (int)ibat, (unsigned)temp_dk, (unsigned)cycles,
                    (unsigned)chg_st, ext ? 1U : 0U, (unsigned)((st1 >> 7) & 1U), aicr_ma);
        }
    } else if (do_log) {
        kprintf("[tb375fc-pwr] chg: st=0x%02x pwr=%u mivr=%u aicr=%umA (gauge rc=%d/%d/%d)\n",
                (unsigned)chg_st, ext ? 1U : 0U, (unsigned)((st1 >> 7) & 1U),
                aicr_ma, rc_v, rc_i, rc_s);
    }
}

static void tb375fc_wipe_fb_layer0(void) {
    vm_offset_t l0 = ml_io_map(PHYS_FB_LAYER0, FB_LAYER0_BYTES);
    if (!l0) {
        kprintf("[tb375fc-pwr] layer0: ml_io_map(0x%llx, 0x%llx) failed\n",
                (unsigned long long)PHYS_FB_LAYER0, (unsigned long long)FB_LAYER0_BYTES);
        return;
    }
    volatile uint64_t *p = (volatile uint64_t *)l0;
    uint64_t words = FB_LAYER0_BYTES / sizeof(uint64_t);
    uint64_t nonZero = 0;
    for (uint64_t i = 0; i < words; i++) {
        if (p[i] != 0) nonZero++;
        p[i] = 0ULL;
    }
    __asm__ volatile("dsb sy" ::: "memory");
    kprintf("[tb375fc-pwr] layer0: wiped 0x%llx..0x%llx (cleared %llu non-zero words of bootloader Lenovo/Android splash)\n",
            (unsigned long long)PHYS_FB_LAYER0,
            (unsigned long long)(PHYS_FB_LAYER0 + FB_LAYER0_BYTES),
            (unsigned long long)nonZero);
}

static void __attribute__((noreturn)) tb375fc_power_thread(void *param __unused, wait_result_t wr __unused) {
    delay_for_interval(1, NSEC_PER_SEC);
    tb375fc_wipe_fb_layer0();

    delay_for_interval(2, NSEC_PER_SEC);

    tb375fc_tune_backlight();

    g_i2c5_base = ml_io_map(PHYS_I2C5_MT6375, 0x1000);
    g_i2cg_base = ml_io_map(PHYS_I2C_GAUGE,   0x1000);
    tb375fc_tune_mt6375(true);

    /* Wait briefly for IOService::getPlatform() so IOPMPowerSource publishes before powerd starts */
    for (int i = 0; i < 40 && !IOService::getPlatform(); ++i) {
        delay_for_interval(250, NSEC_PER_MSEC);
    }
    tb375fc_poll_and_publish(true);

    uint32_t tick = 0;
    for (;;) {
        delay_for_interval(10, NSEC_PER_SEC);
        ++tick;
        if (g_mt6375_ok && (tick % 3U) == 0U) {
            tb375fc_tune_mt6375(false);
        }
        tb375fc_poll_and_publish((tick % 3U) == 0U);
    }
}

static void tb375fc_power_init(void) {
    uint32_t nopower = 0;
    if (PE_parse_boot_argn("nopower", &nopower, sizeof(nopower)) && nopower != 0) {
        kprintf("[tb375fc-pwr] disabled via nopower=1\n");
        return;
    }
    uint32_t arg = 0;
    if (PE_parse_boot_argn("aicr", &arg, sizeof(arg))) {
        if (arg < 500) arg = 500;
        if (arg > 2500) arg = 2500;
        g_target_aicr_ma = arg;
    }
    if (PE_parse_boot_argn("bl", &arg, sizeof(arg))) {
        if (arg < 15) arg = 15;
        if (arg > 100) arg = 100;
        g_target_bl_pct = arg;
    }

    thread_t th = THREAD_NULL;
    kern_return_t kr = kernel_thread_start(tb375fc_power_thread, NULL, &th);
    if (kr == KERN_SUCCESS && th != THREAD_NULL) {
        thread_deallocate(th);
        kprintf("[tb375fc-pwr] started (aicr=%umA bl=%u%% + IOPMPowerSource)\n",
                g_target_aicr_ma, g_target_bl_pct);
    } else {
        kprintf("[tb375fc-pwr] kernel_thread_start failed kr=0x%x\n", kr);
    }
}

STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, tb375fc_power_init);
