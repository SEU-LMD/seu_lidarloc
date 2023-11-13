/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Index the error class include domain & codetype
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_INTERNAL_SERVICE_CALLBACK_HANDLE_H
#define VRTF_VCC_INTERNAL_SERVICE_CALLBACK_HANDLE_H
#include "vrtf/vcc/api/types.h"
#include "ara/hwcommon/log/log.h"
#include <map>
#include <memory>
#include <mutex>
namespace vrtf {
namespace vcc {
namespace internal {
// Singleton， Handle the ServiceStatusChangedCallback of proxy's vcc
class ServiceCallbackHandle {
public:
    ServiceCallbackHandle()
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    ~ServiceCallbackHandle() = default;
    static std::shared_ptr<ServiceCallbackHandle>& GetInstance();
    bool AddServiceStatusChangedCallback(const api::types::HandleType& handle, uint16_t& proxyVccId,
        const std::function<void(const bool&)>&);
    void EraseServiceStatusChangedCallback(const api::types::HandleType& handle, const uint16_t& proxyVccId);
    void TriggerServiceStatusChangedCallback(const api::types::HandleType& handle, const bool& isAvailable);
private:
    bool GetProxyVccId(const api::types::HandleType& handle, uint16_t& proxyVccId);
    // Save every proxy's ServiceStatusChanged callback.
    std::map<api::types::HandleType,
        std::map<std::uint16_t, std::function<void(const bool&)>>> serviceStatusChangedCallback_;
    std::map<api::types::HandleType, uint16_t> currentProxyVccId_;
    std::mutex callbackMutex_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}
#endif

