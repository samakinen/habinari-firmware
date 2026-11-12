// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// ---------------------------------------------------------------------------
// The transport, and the only thing about this device that differs by medium.
//
// Everything else in the adapter — every group object, every parameter
// callback, the publish policy, programming mode — is the same code on TP1 and
// on KNXnet/IP. This file and its sibling are the seam: each implements the
// medium hooks of knx_service_internal.hpp, and startProduct() brings its own
// transport up and hands back the identical started product handle, so nothing
// downstream knows which medium it is running on. Exactly one of the two is
// compiled (main/CMakeLists.txt).
// ---------------------------------------------------------------------------
//
// This is the TP1 side: the STKNX transceiver on the bit-banged bus driver.
// knx_medium_ip.cpp is the other.

#include "knx_service_internal.hpp"

#include "knx/physical/bitbang_driver_timer_isr_espidf.hpp"
#include "knx/physical/physical_factory.hpp"
#include "knx/util/log.hpp"

#include <memory>
#include <utility>

using namespace knx;
using namespace knx::product;

namespace habinari_knx::service {

namespace {

util::Result<std::unique_ptr<physical::Tp1MacPhysical>> createTp1Physical(platform::Esp32Platform &platform)
{
    static physical::BitBangDriverTimerIsrEspIdf bitbangDriver;
    physical::Tp1PlatformDependencies dependencies{};
    dependencies.bitbangDriver = &bitbangDriver;
    dependencies.bitbangTp1Driver = &bitbangDriver;
    dependencies.platform = &platform;

    auto physicalResult = physical::createTp1PhysicalForPlatform(dependencies);
    if (physicalResult.isOk() && physicalResult.value()) {
        // STKNX drives PIN_KNX_OK (GPIO3) while the bus supply is good. The
        // stack uses it to stop transmitting into a dead bus; here it is only
        // reported, so a field log shows bus outages and returns explicitly
        // rather than as a burst of unacknowledged transmissions.
        physicalResult.value()->setLinkStateCallback(
            [](const physical::LinkStatus &status, void *) {
                if (status.state == physical::LinkState::Down) {
                    KNX_LOGW(TAG, "KNX bus power lost — transmission suspended");
                } else {
                    KNX_LOGI(TAG, "KNX bus power %s", physical::linkStateToString(status.state));
                }
            },
            nullptr);
    }

    return physicalResult;
}

}  // namespace

void prepareMedium()
{
    // Nothing to do before the task exists: the bus needs no credentials.
}

util::Result<HabinariProductHandle> startProduct(platform::Esp32Platform &platform,
                                                 HabinariBindings &&bindings)
{
    auto physicalResult = createTp1Physical(platform);
    if (physicalResult.isError()) {
        KNX_LOGE(TAG, "TP1 physical init failed: %d", static_cast<int>(physicalResult.error()));
        return physicalResult.error();
    }

    return startCommissionedProduct(platform, kHabinariProduct, std::move(bindings),
                                    std::move(physicalResult.value()));
}

void mediumStarted(HabinariApp &)
{
    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");
}

void serviceMedium(HabinariApp &)
{
}

}  // namespace habinari_knx::service
