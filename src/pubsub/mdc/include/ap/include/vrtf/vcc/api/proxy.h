/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: proxy in vcc
 * Create: 2019-11-19
 */
#ifndef INC_ARA_GODEL_COMMON_VCC_PROXY_HPP_
#define INC_ARA_GODEL_COMMON_VCC_PROXY_HPP_

#include <string>
#include <map>
#include <set>
#include <mutex>
#include <memory>
#include <future>
#include <vector>
#include "vrtf/vcc/api/types.h"
#include "ara/core/future.h"
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/vcc.h"
#include "vrtf/vcc/utils/log.h"
#include "ara/com/e2e/E2EXf/E2EXf_CM.h"
namespace vrtf {
namespace vcc {
class Proxy {
public:
    using SMState = ara::com::e2e::SMState;
    using ServiceDiscoveryInfo = vrtf::vcc::api::types::ServiceDiscoveryInfo;
    using ServiceAvailableHandler = vrtf::vcc::api::types::ServiceAvailableHandler;
    using DriverType = vrtf::vcc::api::types::DriverType;
    Proxy(const vrtf::vcc::api::types::HandleType& handleType, const bool& mode = false);
    Proxy() = delete;
    ~Proxy();
    Proxy(Proxy && other) = default;
    Proxy& operator=(Proxy && other) = default;
    static vrtf::core::Result<vrtf::vcc::api::types::FindServiceHandle> StartFindService(
        const vrtf::vcc::api::types::FindServiceHandler<vrtf::vcc::api::types::HandleType>&,
        const std::multimap<vrtf::vcc::api::types::DriverType, std::shared_ptr<ServiceDiscoveryInfo>>&);
    static vrtf::vcc::api::types::ServiceHandleContainer<vrtf::vcc::api::types::HandleType> FindService(
        const std::multimap<vrtf::vcc::api::types::DriverType, std::shared_ptr<ServiceDiscoveryInfo>>&);

    static void StopFindService(const vrtf::vcc::api::types::FindServiceHandle&) noexcept;
    template<class SampleType>
    bool Subscribe(const vrtf::vcc::api::types::EntityId& id, size_t maxSampleCount,
                   std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo,
                   std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr)
    {
        if (!vcc_->InitializeSubscribe<SampleType>(id, maxSampleCount, eventInfo, pool)) {
            return false;
        }
        return vcc_->SubscribeEvent(maxSampleCount, eventInfo);
    }

    void Unsubscribe(const vrtf::vcc::api::types::EntityId& id) noexcept;
    vrtf::vcc::api::types::EventSubscriptionState GetSubscriptionState(const vrtf::vcc::api::types::EntityId& id) const;

    bool IsSubscribed(const vrtf::vcc::api::types::EntityId& id);
    void SetSubscriptionStateChangeHandler(const vrtf::vcc::api::types::SubscriptionStateChangeHandler& handler,
                                           const vrtf::vcc::api::types::EntityId& id);
    void UnsetSubscriptionStateChangeHandler(const vrtf::vcc::api::types::EntityId& id);
    void SetReceiveHandler(const vrtf::vcc::api::types::EventReceiveHandler& handler,
                           const vrtf::vcc::api::types::EntityId& id);
    void UnsetReceiveHandler(const vrtf::vcc::api::types::EntityId& id);
    size_t GetFreeSampleCount(const vrtf::vcc::api::types::EntityId& id) const;
    void SetMethodStateChangeHandler(const vrtf::vcc::api::types::MethodStateChangeHandler& handler,
                                     const vrtf::vcc::api::types::EntityId& id);
    void UnsetMethodStateChangeHandler(const vrtf::vcc::api::types::EntityId& id);

    void DestroyWaitingMethodPromise(const vrtf::core::ErrorCode& errorCode, const vrtf::vcc::api::types::EntityId& id);
    template<class SampleType>
    ara::core::Result<size_t> GetNewSamples(std::function<void(vrtf::vcc::api::types::SamplePtr<SampleType const>)> cb,
                                            const vrtf::vcc::api::types::EntityId& id,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return vcc_->GetNewSamples(cb, id, maxNumberOfSamples);
    }

    template<class SampleType>
    ara::core::Result<size_t> GetNewSamples(void(*cb)(vrtf::vcc::api::types::SamplePtr<SampleType const>),
                                            const vrtf::vcc::api::types::EntityId& id,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return vcc_->GetNewSamples(cb, id, maxNumberOfSamples);
    }

    SMState GetSMState(const vrtf::vcc::api::types::EntityId id) const noexcept;
    const ara::com::e2e::Result GetResult(const vrtf::vcc::api::types::EntityId id) const;
    const ara::com::e2e::Result GetMethodE2EResult(const vrtf::vcc::api::types::EntityId id) const;
    // method
    template<class Result, class... Args>
    vrtf::core::Future<Result> Request(vrtf::vcc::api::types::EntityId id, Args... args)
    {
        // Remove after vcc codes completed
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose() << "Proxy start to send method request[methodEntityId=" << id << "]";
        return vcc_->Request<Result>(id, args...);
    }
    /**
     * @brief Initialize Method
     * @details invoking vcc InitializeMethodProxy function
     *
     * @param id EntityId is the identification to different method/event/field
     * @param protocolData the method Info read from config file
     * @return Whether vcc InitializeMethodProxy function return value is successful
     *   @retval true return value is successful, method Initialize successful
     *   @retval false return value is fail, method Initialize fail
     * @note AUTOSAR AP R19-11 RS_CM_00211
     */
    template<class ResultType>
    bool InitializeMethod(const vrtf::vcc::api::types::EntityId& id,
                          std::shared_ptr<vrtf::vcc::api::types::MethodInfo> protocolData)
    {
        return vcc_->InitializeMethodProxy<ResultType>(id, protocolData);
    }
    void RegisterError(const vrtf::vcc::api::types::EntityId& id, const vrtf::core::ErrorCode& error);
    /**
     * @brief proxy Initialize Field
     * @details if field have set/get we will register as method
     *
     * @param protocolData the field Info read from config file
     * @return Whether Initialize field is successful
     *   @retval field initialize successful
     *   @retval field initialize fail
     * @note AUTOSAR AP R19-11 RS_CM_00216 RS_CM_00217 RS_CM_00218
     */
    template<class ResultType>
    bool InitializeField(std::shared_ptr<vrtf::vcc::api::types::FieldInfo> protocolData)
    {
        vcc_->RegisterFieldProxyInfoToMaintaind(protocolData);
        if (protocolData->IsHasGetter()) {
            protocolData->GetGetterMethodInfo()->SetMethodType(api::types::internal::MethodType::FIELD_GETTER);
            if (!InitializeMethod<ResultType>(protocolData->GetGetterMethodInfo()->GetEntityId(),
                protocolData->GetGetterMethodInfo())) {
                return false;
            }
        }
        if (protocolData->IsHasSetter()) {
            protocolData->GetSetterMethodInfo()->SetMethodType(api::types::internal::MethodType::FIELD_SETTER);
            if (!InitializeMethod<ResultType>(protocolData->GetSetterMethodInfo()->GetEntityId(),
                protocolData->GetSetterMethodInfo())) {
                return false;
            }
        }
        return true;
    }

private:
    static void ServiceAvailableCallback(const std::map<api::types::HandleType, bool>& handleContainer);
    std::shared_ptr<Vcc> vcc_;
    vrtf::vcc::api::types::HandleType handleType_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}

#endif /* INC_ARA_GODEL_COMMON_VCC_PROXY_HPP_ */
