/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_PROXY_FIELD_ADAPTER_H_
#define ARA_COM_PROXY_FIELD_ADAPTER_H_
#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/com/internal/proxy/method_adapter.h"
#include "vrtf/vcc/api/proxy.h"
#include "ara/com/types.h"
#include "ara/hwcommon/log/log.h"
namespace ara {
namespace com {
namespace internal {
namespace proxy {
namespace field {
namespace impl {
class FieldAdapterImpl {
public:
    using HandleType = vrtf::vcc::api::types::HandleType;
    using EntityId = vrtf::vcc::api::types::EntityId;
    using ServiceNameType = ara::com::internal::ServiceNameType;
    FieldAdapterImpl(const std::shared_ptr<vrtf::vcc::Proxy>& proxy, EntityId entityId, const HandleType &handle,
                    const ServiceIdentifierType& serviceName)
        : proxy_(proxy),
          entityId_(entityId),
          handle_(handle),
          serviceName_(std::string(serviceName.toString().cbegin()))
    {
        logInstance_ = ara::godel::common::log::Log::GetLog("CM");
    }
    virtual ~FieldAdapterImpl() {}

    void Unsubscribe();
    bool IsSubscribed();
    ara::com::SubscriptionState GetSubscriptionState() const;
    void SetSubscriptionStateChangeHandler(const ara::com::SubscriptionStateChangeHandler& handler);
    void UnsetSubscriptionStateChangeHandler();
    void SetReceiveHandler(const ara::com::EventReceiveHandler& handler);
    void UnsetReceiveHandler();

    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetEntityId() const
    {
        return entityId_;
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    virtual void SetSetterEntityId(const EntityId& id)
    {
        setterEntityId_ = id;
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    virtual void SetGetterEntityId(const EntityId& id)
    {
        getterEntityId_ = id;
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetSetterEntityId() const
    {
        return setterEntityId_;
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetGetterEntityId() const
    {
        return getterEntityId_;
    }

protected:
    template<class SampleType>
    void Subscribe(size_t maxSampleCount)
    {
        using namespace vrtf::vcc::api::types;
        if (maxSampleCount > MAX_EVENT_SUB_COUNT || maxSampleCount == 0) {
            ara::core::Abort("maxSampleCount max value ranges (0,1000]");
        }
        std::map<DriverType, std::shared_ptr<EventInfo>> protocolData;
        DoSubscribe(entityId_, protocolData, true);
        std::pair<DriverType, std::shared_ptr<EventInfo>> dataPair = *(protocolData.begin());
        dataPair.second->SetIsField(true);
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

    void DoSubscribe(const vrtf::vcc::api::types::EntityId&,
    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::EventInfo>>&, bool);

private:
    std::shared_ptr<vrtf::vcc::Proxy> proxy_;
    EntityId entityId_ = UNDEFINED_ENTITYID;
    EntityId setterEntityId_ = UNDEFINED_ENTITYID;
    EntityId getterEntityId_ = UNDEFINED_ENTITYID;
    HandleType handle_;
    ara::com::internal::ServiceNameType serviceName_ = ara::com::internal::UNDEFINED_SERVICE_NAME;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
template<class SampleType>
class FieldAdapter : public impl::FieldAdapterImpl {
public:
    using value_type = typename std::decay<SampleType>::type;
    FieldAdapter(std::shared_ptr<vrtf::vcc::Proxy>& proxy, EntityId entityId, const HandleType &handle,
                 const ServiceIdentifierType& serviceName)
        : FieldAdapterImpl(proxy, entityId, handle, serviceName),
          Get(proxy, UNDEFINED_ENTITYID),
          Set(proxy, UNDEFINED_ENTITYID) {}
    ~FieldAdapter() = default;
    ara::core::Result<size_t> GetNewSamples(std::function<void(ara::com::SamplePtr<SampleType const>)> cb,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return FieldAdapterImpl::GetNewSamples<SampleType>(cb, maxNumberOfSamples);
    }

    ara::core::Result<size_t> GetNewSamples(void(*cb)(ara::com::SamplePtr<SampleType const>),
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return FieldAdapterImpl::GetNewSamples<SampleType>(cb, maxNumberOfSamples);
    }

    template<class T>
    ara::core::Result<size_t> GetNewSamples(void(T::*cb)(ara::com::SamplePtr<SampleType const>), T& obj,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max())
    {
        return FieldAdapterImpl::GetNewSamples<SampleType>(cb, obj, maxNumberOfSamples);
    }

    void Subscribe(size_t maxSampleCount)
    {
        FieldAdapterImpl::Subscribe<SampleType>(maxSampleCount);
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    virtual void SetSetterEntityId(const EntityId& id)
    {
        Set.SetEntityId(id);
        FieldAdapterImpl::SetSetterEntityId(id);
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    void SetGetterEntityId(const EntityId& id)
    {
        Get.SetEntityId(id);
        FieldAdapterImpl::SetGetterEntityId(id);
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetSetterEntityId()
    {
        return Set.GetEntityId();
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetGetterEntityId()
    {
        return Get.GetEntityId();
    }

    method::MethodAdapter<SampleType> Get;
    method::MethodAdapter<SampleType, SampleType> Set;
};
}
}
}
}
}

#endif
