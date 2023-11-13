/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_SKELETON_SKELETON_ADAPTER_H
#define ARA_COM_SKELETON_SKELETON_ADAPTER_H
#include <mutex>
#include <set>
#include <functional>
#include "ara/com/types.h"
#include "ara/com/internal/adapter.h"
#include "ara/com/e2e_error_domain.h"
#include "vrtf/vcc/api/skeleton.h"
#include "ara/hwcommon/log/log.h"
#include "ara/core/future.h"
#include "ara/core/promise.h"
#include "ara/core/instance_specifier.h"
#include "ara/core/result.h"
namespace ara {
namespace com {
namespace internal {
namespace skeleton {
namespace event {
namespace impl {
class EventAdapterImpl;
}
}

namespace field {
namespace impl {
class FieldAdapterImpl;
}
}
// Internal class!!! Prohibit to use by Application!!!!
class SkeletonAdapter : public ara::com::internal::Adapter {
public:
    SkeletonAdapter(const ServiceIdentifierType& serviceName,
                    const InstanceIdentifier& instanceId,
                    const MethodCallProcessingMode& methodMode);
    SkeletonAdapter(const ServiceIdentifierType& serviceName,
                    const ara::core::InstanceSpecifier& instanceSpec,
                    const MethodCallProcessingMode& methodMode);
    SkeletonAdapter(const ServiceIdentifierType& serviceName,
                    const ara::com::InstanceIdentifierContainer& instanceSpec,
                    const MethodCallProcessingMode& methodMode);
    SkeletonAdapter(SkeletonAdapter && other) = default;

    virtual SkeletonAdapter& operator=(SkeletonAdapter && other) = default;

    virtual ~SkeletonAdapter(void);

    // Service Discovery
    void OfferService(void);
    void StopOfferService(void);
    // Method
    ara::core::Future<bool> ProcessNextMethodCall(void);

    // Internal interface!!! Prohibit to use by Application!!!!
    const std::shared_ptr<vrtf::vcc::Skeleton>& GetSkeleton(void) const
    {
        return skeleton_;
    }
    /**
     * @brief Initialize Event Config
     * @details Check Is generated json file true with xml configure about Event config
     *          then create Event by driverType
     *
     * @param event eventImpl represent this event, include event data struct, user can use it to send data
     * @return Whether Init Event Init is successful
     * @note AUTOSAR AP R19-11 RS_CM_00201
     */
    // Internal interface!!! Prohibit to use by Application!!!!
    ara::core::Result<void> InitializeEvent(const event::impl::EventAdapterImpl& event);
    /**
     * @brief Initialize Method Config
     * @details Check Is generated json file true with xml configure about Method config
     *          then create Event by driverType
     *
     * @param id EntityId is the identification to different method/event/field
     * @return Whether Init Method Init is successful
     * @note AUTOSAR AP R19-11 RS_CM_00211
     */
    template<class Result>
    ara::core::Result<void> InitializeMethod(const vrtf::vcc::api::types::EntityId id)
    {
        std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::MethodInfo>> protocolData;
        const auto result = InitializeMethodInfo(id, protocolData);
        if (!result.HasValue()) {
            return result;
        }
        if (!skeleton_->InitializeMethod<Result>(protocolData)) {
            return ara::core::Result<void>(ara::com::ComErrc::kNetworkBindingFailure);
        }
        return ara::core::Result<void>();
    }
    /**
     * @brief Initialize Field Config
     * @details Check Is generated json file true with xml configure about Method config
     *          then create Event by driverType
     *
     * @param field eventImpl represent this event, include event and method, user use it to field communication
     * @return Whether Init Field Init is successful
     *   @retval true Field Init is successful
     *   @retval false Field Init is fail
     * @note AUTOSAR AP R19-11 RS_CM_00216 RS_CM_00217 RS_CM_00218
     */
    template<class SampleType>
    ara::core::Result<void> InitializeField(field::impl::FieldAdapterImpl& field)
    {
        std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::FieldInfo>> protocolData;
        const auto result = InitializeFieldInfo(field, protocolData);
        if (!result.HasValue()) {
            return result;
        }
        if (skeleton_->InitializeField<SampleType>(protocolData) == false) {
            return ara::core::Result<void>(ara::com::ComErrc::kNetworkBindingFailure);
        }
        return ara::core::Result<void>();
    }

    template<class T, class Result, class... Args>
    void RegisterMethod(Result(T::*callback)(Args...), T& c, const vrtf::vcc::api::types::EntityId id)
    {
        skeleton_->RegisterMethod(callback, c, id);
    }

    bool SetMethodThreadNumber(const std::uint16_t number, const std::uint16_t queueSize);
    std::uint16_t GetMethodThreadNumber(const std::uint16_t number) const;

    /**
     * @brief   Register E2E error handler which will be called if the E2E checking result is error
     *
     * @param[in] callback   The handler of handling E2E checking result is error
     * @param[in] instance   The class of callback belong to
     */
    template<class T>
    void RegisterE2EErrorHandler(void(T::*callback)(ara::com::e2e::E2EErrorCode,
        ara::com::e2e::DataID, ara::com::e2e::MessageCounter), T& instance)
    {
        skeleton_->RegisterE2EErrorHandler(callback, instance);
    }
private:
    MethodCallProcessingMode methodMode_ {MethodCallProcessingMode::kEvent};
    std::shared_ptr<vrtf::vcc::Skeleton> skeleton_;
    static const std::map<MethodCallProcessingMode, std::string> MethodCallMap;
    std::map<vrtf::vcc::api::types::DriverType,
        std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>> sdProtocolData;
    /**
     * @brief Check mode is valid
     * @details Check mode is valid, if invalid, use default value
     *
     * @return void
     */
    void VertifyModeValue();
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
    ara::core::Result<void> InitializeMethodInfo(const vrtf::vcc::api::types::EntityId id,
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
    ara::core::Result<void> InitializeFieldInfo(const field::impl::FieldAdapterImpl& field,
        std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::FieldInfo>>& protocolData);
};
}
}
}
}
#endif
