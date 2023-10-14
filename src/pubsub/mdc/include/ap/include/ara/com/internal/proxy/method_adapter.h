/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_PROXY_METHOD_ADAPTER_H_
#define ARA_COM_PROXY_METHOD_ADAPTER_H_
#include <utility>
#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/core/future.h"
#include "ara/com/com_error_domain.h"
#include "ara/core/promise.h"
namespace ara {
namespace com {
namespace internal {
namespace proxy {
namespace method {
namespace impl {
class MethodAdapterImpl {
public:
    MethodAdapterImpl(const std::shared_ptr<vrtf::vcc::Proxy>& proxy, EntityId entityId)
        : proxy_(proxy), entityId_(entityId) {}
    virtual ~MethodAdapterImpl() {}
    // Internal interface!!! Prohibit to use by Application!!!!
    void Initialize(const std::shared_ptr<vrtf::vcc::Proxy>& proxy, EntityId entityId)
    {
        proxy_ = proxy;
        entityId_ = entityId;
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    void SetEntityId(EntityId entityId)
    {
        entityId_ = entityId;
    }
    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetEntityId() const
    {
        return entityId_;
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return proxy_->GetSMState(entityId_);
    }

protected:
    std::shared_ptr<vrtf::vcc::Proxy> proxy_;
    EntityId entityId_ = UNDEFINED_ENTITYID;
};
}
template<class Result, class... Params>
class MethodAdapter : public impl::MethodAdapterImpl {
    using EntityId= vrtf::vcc::api::types::EntityId;
public:
    MethodAdapter(std::shared_ptr<vrtf::vcc::Proxy>& proxy, EntityId entityId): MethodAdapterImpl(proxy, entityId) {}
    ~MethodAdapter() = default;
    ara::core::Future<Result> operator()(Params... args)
    {
        return proxy_->Request<Result, Params...>(entityId_, args...);
    }
};
}
}
}
}
}
#endif
