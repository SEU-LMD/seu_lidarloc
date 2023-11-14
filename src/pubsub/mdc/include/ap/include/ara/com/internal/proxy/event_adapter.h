/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_PROXY_EVENT_ADAPTER_H_
#define ARA_COM_PROXY_EVENT_ADAPTER_H_
#include <set>
#include "ara/com/internal/proxy/proxy_adapter.h"
#include "vrtf/vcc/api/proxy.h"
#include "ara/com/types.h"
#include "ara/com/e2e/E2EXf/E2EXf_CM.h"
namespace ara {
namespace com {
namespace internal {
namespace proxy {
namespace event {
namespace impl {
class EventAdapterImpl {
public:
    using HandleType = vrtf::vcc::api::types::HandleType;
    using EntityId = vrtf::vcc::api::types::EntityId;
    using ServiceNameType = ara::com::internal::ServiceNameType;
    EventAdapterImpl(const std::shared_ptr<vrtf::vcc::Proxy>& proxy, const EntityId& entityId, const HandleType &handle,
                     const ServiceIdentifierType& serviceName)
        : proxy_(proxy),
          entityId_(entityId),
          handle_(handle),
          serviceName_(std::string(serviceName.toString().cbegin()))
    {
        logInstance_ = ara::godel::common::log::Log::GetLog("CM");
    }
    virtual ~EventAdapterImpl() {}

    bool IsSubscribed();
    void Unsubscribe();
    size_t GetFreeSampleCount() const;
    ara::com::SubscriptionState GetSubscriptionState() const;
    void SetSubscriptionStateChangeHandler(const ara::com::SubscriptionStateChangeHandler& handler);
    void UnsetSubscriptionStateChangeHandler();
    void SetReceiveHandler(const ara::com::EventReceiveHandler& handler);
    void UnsetReceiveHandler();
    ara::com::e2e::SMState GetSMState() const noexcept;
    const ara::com::e2e::Result GetResult() const;

protected:
    // Internal interface!!! Prohibit to use by Application!!!!
    template<class SampleType>
    void Subscribe(size_t maxSampleCount)
    {
        using namespace vrtf::vcc::api::types;
        if (maxSampleCount > MAX_EVENT_SUB_COUNT || maxSampleCount == 0) {
            ara::core::Abort("maxSampleCount max value ranges (0,1000]");
        }
        std::map<DriverType, std::shared_ptr<EventInfo>> protocolData;
        DoSubscribe(entityId_, protocolData, false);
        std::pair<DriverType, std::shared_ptr<EventInfo>> dataPair = *(protocolData.begin());
        dataPair.second->SetIsField(false);
        proxy_->Subscribe<SampleType>(entityId_, maxSampleCount, dataPair.second);
    }

    template<class SampleType>
    ara::core::Result<size_t> GetNewSamples(std::function<void(ara::com::SamplePtr<SampleType const>)> cb,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return proxy_->GetNewSamples(cb, entityId_, maxNumberOfSamples);
    }

    template<class SampleType>
    ara::core::Result<size_t> GetNewSamples(void(*cb)(ara::com::SamplePtr<SampleType const>),
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return proxy_->GetNewSamples(cb, entityId_, maxNumberOfSamples);
    }

    std::shared_ptr<vrtf::vcc::Proxy> proxy_;
    EntityId entityId_ = UNDEFINED_ENTITYID;
    HandleType handle_;
    ara::com::internal::ServiceNameType serviceName_ = ara::com::internal::UNDEFINED_SERVICE_NAME;
    void DoSubscribe(const vrtf::vcc::api::types::EntityId&,
    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::EventInfo>>&, bool);
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}

template<class SampleType>
class EventAdapter : public impl::EventAdapterImpl {
public:
    using value_type = typename std::decay<SampleType>::type;
    EventAdapter(std::shared_ptr<vrtf::vcc::Proxy>& proxy, EntityId entityId, const HandleType &handle,
                 const ServiceIdentifierType& serviceName)
        : EventAdapterImpl(proxy, entityId, handle, serviceName){}
    ~EventAdapter() = default;
    ara::core::Result<size_t> GetNewSamples(std::function<void(ara::com::SamplePtr<SampleType const>)> cb,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return EventAdapterImpl::GetNewSamples<SampleType>(cb, maxNumberOfSamples);
    }

    ara::core::Result<size_t> GetNewSamples(void(*cb)(ara::com::SamplePtr<SampleType const>),
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return EventAdapterImpl::GetNewSamples<SampleType>(cb, maxNumberOfSamples);
    }
    template<class T>
    ara::core::Result<size_t> GetNewSamples(void(T::*cb)(ara::com::SamplePtr<SampleType const>), T& obj,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return EventAdapterImpl::GetNewSamples<SampleType>(cb, obj, maxNumberOfSamples);
    }

    void Subscribe(size_t maxSampleCount)
    {
        EventAdapterImpl::Subscribe<SampleType>(maxSampleCount);
    }
};
}
}
}
}
}
#endif
