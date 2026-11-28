// SPDX-License-Identifier: MIT
// Virtual / Bridged IOEthernetController (en0) for TB375FC macOS 27.3 Full Desktop.
// Provides a real IONetworkingFamily IOEthernetInterface (en0) with MAC 02:37:5f:c0:00:01
// so IOTimeSyncClockManager / clocksyncd / WindowServer start cleanly at boot, and bridges
// live Ethernet II TX/RX frames directly to the native MT6897 ConnSys Wi-Fi 6 kernel driver.

#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOOutputQueue.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <sys/kpi_mbuf.h>
#include <string.h>

extern "C" {
extern kmod_info_t g_kernel_kmod_info;
void _GLOBAL__sub_I_Tb375fcDummyEth_cpp(void) __asm__("__GLOBAL__sub_I_Tb375fcDummyEth.cpp");

typedef int (*tb375fc_eth_tx_cb_t)(const uint8_t *frame, uint32_t len);
void tb375fc_eth_register_tx(tb375fc_eth_tx_cb_t cb);
int  tb375fc_eth_rx_deliver(const uint8_t *frame, uint32_t len);
void tb375fc_eth_set_link(bool up, uint64_t speed_bps);
void tb375fc_dummy_eth_probe(void);
}

class Tb375fcDummyEth : public IOEthernetController {
    OSDeclareDefaultStructors(Tb375fcDummyEth)

private:
    IOEthernetInterface *fNetIf = nullptr;
    IOWorkLoop *fWorkLoop = nullptr;
    OSDictionary *fMediumDict = nullptr;
    IOLock *fRxLock = nullptr;
    bool fEnabled = false;

public:
    bool init(OSDictionary *properties = nullptr) override;
    void free(void) override;
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;
    bool createWorkLoop(void) override;
    IOWorkLoop *getWorkLoop(void) const override;
    IOReturn enable(IONetworkInterface *netif) override;
    IOReturn disable(IONetworkInterface *netif) override;
    IOReturn getHardwareAddress(IOEthernetAddress *addr) override;
    UInt32 outputPacket(mbuf_t m, void *param) override;
    IOReturn setPromiscuousMode(bool active) override { return kIOReturnSuccess; }
    IOReturn setMulticastMode(bool active) override { return kIOReturnSuccess; }
    IOReturn setMulticastList(IOEthernetAddress *addrs, UInt32 count) override { return kIOReturnSuccess; }
    const OSString *newVendorString(void) const override { return OSString::withCString("MediaTek"); }
    const OSString *newModelString(void) const override { return OSString::withCString("MT6897 ConnSys Wi-Fi 6"); }

    int deliverRxFrame(const uint8_t *frame, uint32_t len);
    void updateLink(bool up, uint64_t speed_bps);
};

#define super IOEthernetController
OSDefineMetaClassAndStructors(Tb375fcDummyEth, IOEthernetController)

static Tb375fcDummyEth *g_eth_instance = nullptr;
static volatile tb375fc_eth_tx_cb_t g_eth_tx_cb = nullptr;

bool Tb375fcDummyEth::init(OSDictionary *properties) {
    if (!super::init(properties)) return false;
    fRxLock = IOLockAlloc();
    return fRxLock != nullptr;
}

void Tb375fcDummyEth::free(void) {
    if (g_eth_instance == this) g_eth_instance = nullptr;
    if (fRxLock) { IOLockFree(fRxLock); fRxLock = nullptr; }
    if (fMediumDict) { fMediumDict->release(); fMediumDict = nullptr; }
    if (fNetIf) { fNetIf->release(); fNetIf = nullptr; }
    if (fWorkLoop) { fWorkLoop->release(); fWorkLoop = nullptr; }
    super::free();
}

bool Tb375fcDummyEth::createWorkLoop(void) {
    fWorkLoop = IOWorkLoop::workLoop();
    return fWorkLoop != nullptr;
}

IOWorkLoop *Tb375fcDummyEth::getWorkLoop(void) const {
    return fWorkLoop;
}

bool Tb375fcDummyEth::start(IOService *provider) {
    if (!super::start(provider)) {
        IOLog("Tb375fcDummyEth: super::start failed\n");
        return false;
    }

    // Publish medium dictionary (1000baseT full-duplex) so link status is valid
    fMediumDict = OSDictionary::withCapacity(2);
    if (fMediumDict) {
        IONetworkMedium *med = IONetworkMedium::medium(
            kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex, 1000000000ULL);
        if (med) {
            IONetworkMedium::addMedium(fMediumDict, med);
            publishMediumDictionary(fMediumDict);
            setCurrentMedium(med);
            setSelectedMedium(med);
            med->release();
        }
    }

    if (!attachInterface((IONetworkInterface **)&fNetIf, true)) {
        IOLog("Tb375fcDummyEth: attachInterface failed\n");
        return false;
    }

    g_eth_instance = this;
    setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), 1000000000ULL);
    registerService();
    IOLog("Tb375fcDummyEth: en0 attached and registered (MAC 02:37:5f:c0:00:01, Wi-Fi bridge ready)\n");
    return true;
}

void Tb375fcDummyEth::stop(IOService *provider) {
    if (g_eth_instance == this) g_eth_instance = nullptr;
    if (fNetIf) {
        detachInterface(fNetIf);
    }
    super::stop(provider);
}

IOReturn Tb375fcDummyEth::enable(IONetworkInterface *netif) {
    fEnabled = true;
    setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), 1000000000ULL);
    return kIOReturnSuccess;
}

IOReturn Tb375fcDummyEth::disable(IONetworkInterface *netif) {
    fEnabled = false;
    setLinkStatus(kIONetworkLinkValid, nullptr, 0);
    return kIOReturnSuccess;
}

IOReturn Tb375fcDummyEth::getHardwareAddress(IOEthernetAddress *addr) {
    if (!addr) return kIOReturnBadArgument;
    addr->bytes[0] = 0x02;
    addr->bytes[1] = 0x37;
    addr->bytes[2] = 0x5f;
    addr->bytes[3] = 0xc0;
    addr->bytes[4] = 0x00;
    addr->bytes[5] = 0x01;
    return kIOReturnSuccess;
}

UInt32 Tb375fcDummyEth::outputPacket(mbuf_t m, void *param) {
    (void)param;
    if (!m) return kIOReturnOutputSuccess;
    tb375fc_eth_tx_cb_t cb = g_eth_tx_cb;
    if (cb) {
        size_t pkt_len = mbuf_pkthdr_len(m);
        if (pkt_len >= 14 && pkt_len <= 1600) {
            uint8_t buf[1600];
            if (mbuf_copydata(m, 0, pkt_len, buf) == 0) {
                (void)cb(buf, static_cast<uint32_t>(pkt_len));
            }
        }
    }
    freePacket(m);
    return kIOReturnOutputSuccess;
}

int Tb375fcDummyEth::deliverRxFrame(const uint8_t *frame, uint32_t len) {
    if (!fNetIf || !fEnabled || !frame || len < 14 || len > 1600) return -1;
    mbuf_t m = allocatePacket(len);
    if (!m) return -2;
    void *dst = mbuf_data(m);
    if (!dst) {
        freePacket(m);
        return -3;
    }
    memcpy(dst, frame, len);
    mbuf_setlen(m, len);
    mbuf_pkthdr_setlen(m, len);
    IOLockLock(fRxLock);
    fNetIf->inputPacket(m, len, 0, nullptr);
    IOLockUnlock(fRxLock);
    return 0;
}

void Tb375fcDummyEth::updateLink(bool up, uint64_t speed_bps) {
    if (up) {
        setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive,
                      getCurrentMedium(),
                      speed_bps ? speed_bps : 1000000000ULL);
    } else {
        setLinkStatus(kIONetworkLinkValid, nullptr, 0);
    }
}

extern "C" void tb375fc_eth_register_tx(tb375fc_eth_tx_cb_t cb) {
    g_eth_tx_cb = cb;
}

extern "C" int tb375fc_eth_rx_deliver(const uint8_t *frame, uint32_t len) {
    Tb375fcDummyEth *eth = g_eth_instance;
    if (!eth) return -1;
    return eth->deliverRxFrame(frame, len);
}

extern "C" void tb375fc_eth_set_link(bool up, uint64_t speed_bps) {
    Tb375fcDummyEth *eth = g_eth_instance;
    if (!eth) return;
    eth->updateLink(up, speed_bps);
}

extern "C" void tb375fc_dummy_eth_probe(void) {
    static bool probed = false;
    if (probed) return;
    probed = true;

    OSReturn kr_net = OSKextLoadKextWithIdentifier("com.apple.iokit.IONetworkingFamily");
    IOLog("Tb375fcDummyEth: kext load IONetworkingFamily=0x%x\n", kr_net);
    if (kr_net != kOSReturnSuccess) {
        IOLog("Tb375fcDummyEth: IONetworkingFamily load failed\n");
        return;
    }

    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {
        IOLog("Tb375fcDummyEth: preModLoad failed\n");
        return;
    }
    _GLOBAL__sub_I_Tb375fcDummyEth_cpp();
    OSReturn post_kr = OSMetaClass::postModLoad(handle);
    IOLog("Tb375fcDummyEth: postModLoad status=0x%x\n", post_kr);
    if (post_kr != kOSReturnSuccess) {
        return;
    }

    IOPlatformExpert *plat = IOService::getPlatform();
    if (!plat) {
        IOLog("Tb375fcDummyEth: no platform expert\n");
        return;
    }
    Tb375fcDummyEth *eth = new Tb375fcDummyEth;
    if (!eth) return;
    if (!eth->init(nullptr)) {
        eth->release();
        return;
    }
    eth->setName("Tb375fcDummyEth");
    if (!eth->attach(plat)) {
        eth->release();
        return;
    }
    if (!eth->start(plat)) {
        eth->detach(plat);
        eth->release();
        return;
    }
}
