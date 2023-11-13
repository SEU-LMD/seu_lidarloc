/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: All driven base classes
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_DRIVER_H
#define VRTF_VCC_DRIVER_H
#include <memory>
#include <functional>
#include <map>
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/driver/event_handler.h"
#include "vrtf/vcc/driver/method_handler.h"
namespace vrtf {
namespace vcc {
namespace driver {
class Driver {
public:
    Driver() {}
    virtual ~Driver() {}

    virtual void OfferService(const std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>& protocolData) = 0;
    virtual void StopOfferService(
        const std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>& protocolData) = 0;
    virtual vrtf::core::ErrorCode StartFindService(
        const std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>& protocolData,
        std::function<void(const std::map<vrtf::vcc::api::types::HandleType, bool>&)> cb) = 0;
    virtual void StopFindService(const std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>& protocolData) = 0;
    virtual std::shared_ptr<vrtf::vcc::driver::EventHandler> CreateEvent(
        std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo) = 0;
    virtual std::shared_ptr<vrtf::vcc::driver::EventHandler> SubscribeEvent(
        std::shared_ptr<vrtf::vcc::api::types::EventInfo> eventInfo,
        vrtf::vcc::api::types::EventHandleReceiveHandler  handler) = 0;

    virtual std::shared_ptr<vrtf::vcc::driver::MethodHandler> CreateMethodServer(
        std::shared_ptr<vrtf::vcc::api::types::MethodInfo> protocolData) = 0;
    virtual std::shared_ptr<vrtf::vcc::driver::MethodHandler> CreateMethodClient(
        std::shared_ptr<vrtf::vcc::api::types::MethodInfo> protocolData) = 0;

    virtual void PrintEventInfo(const std::shared_ptr<vrtf::vcc::api::types::EventInfo>& info) = 0;
    virtual void PrintMethodInfo(const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& info) = 0;
    virtual void PrintServiceInfo(const std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>& info) = 0;
};
}
}
}
#endif
