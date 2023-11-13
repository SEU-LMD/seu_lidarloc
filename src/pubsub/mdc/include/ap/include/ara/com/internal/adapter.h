/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_INTERNAL_ROLE_ADAPTER_H_
#define ARA_COM_INTERNAL_ROLE_ADAPTER_H_
#include <string>
#include "ara/com/types.h"
#include "ara/core/instance_specifier.h"
#include "ara/hwcommon/log/log.h"
namespace ara {
namespace com {
namespace runtime {
    /**
     * @brief Resolve instanceSpecifier to InstanceIds
     * @details user's input InstanceSpecifier to return the binding InstanceIdentifierContainer
     *
     * @param modelName the InstanceSpecifier
     * @param[in] protocolData the method Info read from config file
     * @return the Container may include multi-InstancdeIdentifiers
     */
    ara::com::InstanceIdentifierContainer ResolveInstanceIDs(const ara::core::InstanceSpecifier& modelName);
}
namespace internal {
class Adapter {
public:
    Adapter(const ServiceNameType& serviceName, const InstanceIdentifier& instanceId);
    Adapter(const ara::core::StringView& serviceName,  const InstanceIdentifier& instanceId)
        : serviceName_(std::string(serviceName.cbegin())),
          instanceId_(instanceId),
          instanceSpec_(ara::com::DEFAULTINSTANCE) {}
    Adapter(const ServiceNameType& service, const ara::core::InstanceSpecifier& instanceSpec);
    Adapter(const ServiceNameType& serviceName, const ara::com::InstanceIdentifierContainer& instanceIdContainer);
    Adapter(Adapter && other) = default;
    virtual Adapter& operator=(Adapter && other) = default;
    virtual ~Adapter() = default;

    ServiceNameType GetServiceName()
    {
        return serviceName_;
    }
    void SetServiceName(const ServiceNameType& serviceName)
    {
        serviceName_ = serviceName;
    }

    InstanceIdentifier GetInstanceId()
    {
        return instanceId_;
    }
    void SetInstanceId(const InstanceIdentifier& instanceId)
    {
        instanceId_ = instanceId;
    }

protected:
    ServiceNameType serviceName_ = UNDEFINED_SERVICE_NAME;
    InstanceIdentifier instanceId_;
    std::string instanceSpecString_;
    ara::core::InstanceSpecifier instanceSpec_;
    ara::com::InstanceIdentifierContainer instanceIdContainer_;
    bool isInstanceSpec_ = false;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}

#endif
