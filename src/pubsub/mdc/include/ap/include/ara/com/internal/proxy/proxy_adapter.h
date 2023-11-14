/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_PROXY_PROXY_ADAPTER_H
#define ARA_COM_PROXY_PROXY_ADAPTER_H

#include <mutex>
#include <functional>
#include "ara/com/types.h"
#include "ara/com/internal/adapter.h"
#include "vrtf/vcc/api/proxy.h"
#include "ara/hwcommon/log/log.h"
#include "ara/com/com_error_domain.h"
#include "vrtf/vcc/utils/log.h"
#include "ara/core/instance_specifier.h"

namespace ara {
namespace com {
namespace internal {
namespace proxy {
namespace field {
namespace impl {
class FieldAdapterImpl;
}
}

namespace method {
namespace impl {
class MethodAdapterImpl;
}
}

// Internal class!!! Prohibit to use by Application!!!!
class ProxyAdapter : public ara::com::internal::Adapter {
public:
    using HandleType = vrtf::vcc::api::types::HandleType;
    using EntityId = vrtf::vcc::api::types::EntityId;
    ProxyAdapter(const ServiceIdentifierType& serviceName, const HandleType &handle);
    ProxyAdapter(ProxyAdapter && other) = default;
    virtual ~ProxyAdapter();
    virtual ProxyAdapter& operator=(ProxyAdapter && other) = default;
    using ServiceCallback = std::function<void(std::vector<ara::com::internal::proxy::ProxyAdapter::HandleType>,
                                        FindServiceHandle)>;
    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ServiceIdentifierType& serviceName, const ara::com::InstanceIdentifier& instanceId);

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ServiceIdentifierType& serviceName, const ara::core::InstanceSpecifier& instanceSpec);

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ServiceIdentifierType& serviceName, const ara::com::InstanceIdentifier& instance);
    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ServiceIdentifierType& serviceName, const ara::core::InstanceSpecifier& instanceSpec);
    static void StopFindService(const ara::com::FindServiceHandle& handle);
    std::shared_ptr<vrtf::vcc::Proxy>& GetProxy()
    {
        return proxy_;
    }

    HandleType GetHandle() const
    {
        return handle_;
    }

    /**
     * @brief Initialize Method Config
     * @details Check Is generated json file true with xml configure about method config
     *
     * @param id EntityId is the identification to different method/event/field
     * @return Whether Init Method Config is successful
     *   @retval true Method Config is successful
     *   @retval false Method Config is fail
     * @note AUTOSAR AP R19-11 SWS_CM_00196
     */
    template<class ResultType>
    ara::core::Result<void> InitializeMethod(const EntityId id)
    {
        using namespace ara::godel::common;
        using namespace vrtf::vcc::api::types;
        std::map<DriverType, std::shared_ptr<MethodInfo>> protocolData;
        if (InitializeMethodInfo(id, protocolData) == true) {
            std::pair<DriverType, std::shared_ptr<MethodInfo>> dataPair = *(protocolData.begin());
            vrtf::vcc::utils::PrintMethodInfo(protocolData);
            proxy_->InitializeMethod<ResultType>(id, dataPair.second);
        } else {
            return ara::core::Result<void>(ara::com::ComErrc::kNetworkBindingFailure);
        }
        return ara::core::Result<void>();
    }

    void RegisterError(const method::impl::MethodAdapterImpl& method, const ara::core::ErrorCode& error);

    /**
     * @brief Initialize Field Config
     * @details Check Is generated json file true with xml configure about field config
     *
     * @param id EntityId is the identification to different method/event/field
     * @return Whether Init Field Config is successful
     *   @retval true Field Config is successful
     *   @retval false Field Config is fail
     * @note AUTOSAR AP R19-11 SWS_CM_10414
     */
    template<class ResultType>
    ara::core::Result<void> InitializeField(const field::impl::FieldAdapterImpl& field)
    {
        using namespace ara::godel::common;
        using namespace vrtf::vcc::api::types;
        std::map<DriverType, std::shared_ptr<FieldInfo>> protocolData;
        if (InitializeFieldInfo(field, protocolData) == true) {
            std::pair<DriverType, std::shared_ptr<FieldInfo>> dataPair = *(protocolData.begin());
            vrtf::vcc::utils::PrintFieldInfo(protocolData);
            if (proxy_->InitializeField<ResultType>(dataPair.second) == false) {
                return ara::core::Result<void>(ara::com::ComErrc::kNetworkBindingFailure);
            }
        } else {
            return ara::core::Result<void>(ara::com::ComErrc::kNetworkBindingFailure);
        }
        return ara::core::Result<void>();
    }
    std::shared_ptr<vrtf::vcc::Proxy> proxy_;
private:
    HandleType handle_;
    std::string proxyInfo_;
    static std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    /**
     * @brief Initialize method config, read Initialize data
     * @details due to entityId to read method param from config files
     *
     * @param[in] id the id represent protocolData
     * @param[out] protocolData store the data read from json file
     * @return Initialize method is successful
     *   @retval true method init is successful
     *   @retval false method init is failed
     */
bool InitializeMethodInfo(const vrtf::vcc::api::types::EntityId id,
    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::MethodInfo>>& protocolData);

    /**
     * @brief Initialize field config, read Initialize data
     * @details due to entityId to read field param from config files
     *
     * @param[in] field the field represent thies skeleton include fieldAdapter
     * @param[out] protocolData store the data read from json file
     * @return Initialize field is successful
     *   @retval true field init  is successful
     *   @retval false field init is failed
     */
bool InitializeFieldInfo(const field::impl::FieldAdapterImpl& field,
    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::FieldInfo>>& protocolData);
};
}
}
}
}

#endif
