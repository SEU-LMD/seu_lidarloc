/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Index the error class include domain & codetype
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_INTERNAL_SERVICE_LIST_HANDLE_H
#define VRTF_VCC_INTERNAL_SERVICE_LIST_HANDLE_H
#include "vrtf/vcc/api/types.h"
#include "ara/hwcommon/log/log.h"
#include <set>
#include <vector>
#include <map>
#include <memory>
#include "vrtf/vcc/api/internal/driver_manager.h"
#include "vrtf/vcc/utils/thread_pool.h"
namespace vrtf {
namespace vcc {
namespace internal {
// Singleton， ServiceList is used to save available services、services handlers
class ServiceListHandle {
public:
    using ServiceDiscoveryInfo = std::map<api::types::HandleType, std::shared_ptr<api::types::ServiceDiscoveryInfo>>;
    ServiceListHandle();
    ~ServiceListHandle();
    static std::shared_ptr<ServiceListHandle>& GetInstance();
    vrtf::vcc::api::types::FindServiceHandle AddServiceAvailableHandler(
        const std::multimap<api::types::DriverType, std::shared_ptr<api::types::ServiceDiscoveryInfo>>& sd,
        const vrtf::vcc::api::types::FindServiceHandler<vrtf::vcc::api::types::HandleType>& cb);
    void EraseServiceAvailableHandler(const vrtf::vcc::api::types::FindServiceHandle& findHandle);
    vrtf::vcc::api::types::ServiceHandleContainer<api::types::HandleType> GetAvailableServices(
        const vrtf::vcc::api::types::FindServiceHandle& findHandle);
    bool AddAvailableService(const api::types::HandleType& handle);
    bool EraseAvailableService(const api::types::HandleType& handle);
    void PrintServices(const vrtf::vcc::api::types::ServiceHandleContainer<api::types::HandleType>& services);
    void TriggerCallback(const api::types::HandleType& handle);
    void TriggerStartFindService(const api::types::HandleType& handle,
        const std::function<vrtf::core::ErrorCode()> startFindServiceCallback);
    void EnqueueServiceStatusChangedTask(const std::map<api::types::HandleType, bool>& handleContainer);
    void ServiceAvailableCallback(const std::map<api::types::HandleType, bool>& handleContainer);

private:
    void EraseAvailableServiceForFindHandleListByHandle(const api::types::HandleType& handle);
    void AddAvailableServiceForFindHandleListByHandle(const api::types::HandleType& handle);
    void InsertAvailableServicesByHandle(std::set<vrtf::vcc::api::types::HandleType>& instances,
        const vrtf::vcc::api::types::HandleType& handle);

    struct FindServiceHandleInfo {
        std::map<api::types::HandleType, std::shared_ptr<api::types::ServiceDiscoveryInfo>> handleGroup_;
        vrtf::vcc::api::types::FindServiceHandler<vrtf::vcc::api::types::HandleType> cb_;
        // availableServicesByFindServiceHandle_ used to saves available services which are group into different
        // FindServiceHandle
        std::set<vrtf::vcc::api::types::HandleType> availableServicesByFindServiceHandle_;
    };

    struct HandleInfo {
        bool isStartFindServiceTriggered_;
        std::set<api::types::FindServiceHandle> findHandles_;
    };

    using ServiceList = std::map<vrtf::vcc::api::types::DriverType,
        std::map<vrtf::vcc::api::types::ServiceId, std::set<vrtf::vcc::api::types::HandleType>>>;
    using FindServiceHandleList = std::map<api::types::FindServiceHandle, FindServiceHandleInfo>;
    std::mutex mutex_;
    // availableServices_ used to saves all available services
    ServiceList availableServices_;
    // findServiceHandleList_ used to save all findServiceHandle and related service handles and callback info
    FindServiceHandleList findServiceHandleList_;
    /* findServiceHandleTypeContainer_ is used to save all handletype which is use to find service, when deconstruction,
    findServiceHandleTypeContainer_ is used to stop find service */
    std::map<api::types::HandleType, std::shared_ptr<api::types::ServiceDiscoveryInfo>> findServiceHandleTypeContainer_;
    // availableServices_ used to save all findServiceHandles which are group into different service handle
    std::map<vrtf::vcc::api::types::HandleType, HandleInfo> findHandleListByHandle_;
    std::shared_ptr<api::internal::DriverManager> drvManager_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    // record service discovery callback thread id
    std::thread::id callbackThreadId_;
    std::unique_ptr<utils::ThreadPool> pool_;
    bool stop_ = false;
};
}
}
}
#endif
