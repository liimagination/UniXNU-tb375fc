// SPDX-License-Identifier: MIT
// Minimal virtual IOEthernetController (en0) for TB375FC macOS 27.3 Full Desktop bring-up.
// Provides a real IONetworkingFamily IOEthernetInterface (en0) with a fixed locally-administered
// MAC address (02:37:5f:c0:00:01) so IOTimeSyncClockManager matches an IONF interface,
// registers itself, and unblocks clocksyncd -> com.apple.timesync.expositor -> WindowServer.

#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOOutputQueue.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>

extern "C" {
extern kmod_info_t g_kernel_kmod_info;
void _GLOBAL__sub_I_Tb375fcDummyEth_cpp(void) __asm__("__GLOBAL__sub_I_Tb375fcDummyEth.cpp");
}

class Tb375fcDummyEth : public IOEthernetController {
    OSDeclareDefaultStructors(Tb375fcDummyEth)

private:
    IOEthernetInterface *fNetIf = nullptr;
    IOWorkLoop *fWorkLoop = nullptr;
    OSDictionary *fMediumDict = nullptr;

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
    const OSString *newVendorString(void) const override { return OSString::withCString("TB375FC"); }
    const OSString *newModelString(void) const override { return OSString::withCString("Virtual Ethernet"); }
};

#define super IOEthernetController
OSDefineMetaClassAndStructors(Tb375fcDummyEth, IOEthernetController)

extern "C" void tb375fc_dummy_eth_probe(void);

bool Tb375fcDummyEth::init(OSDictionary *properties) {
    if (!super::init(properties)) return false;
    return true;
}

void Tb375fcDummyEth::free(void) {
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

    setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), 1000000000ULL);
    registerService();
    IOLog("Tb375fcDummyEth: en0 attached and registered (MAC 02:37:5f:c0:00:01)\n");
    return true;
}

void Tb375fcDummyEth::stop(IOService *provider) {
    if (fNetIf) {
        detachInterface(fNetIf);
    }
    super::stop(provider);
}

IOReturn Tb375fcDummyEth::enable(IONetworkInterface *netif) {
    setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), 1000000000ULL);
    return kIOReturnSuccess;
}

IOReturn Tb375fcDummyEth::disable(IONetworkInterface *netif) {
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
    if (m) freePacket(m);
    return kIOReturnOutputSuccess;
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
