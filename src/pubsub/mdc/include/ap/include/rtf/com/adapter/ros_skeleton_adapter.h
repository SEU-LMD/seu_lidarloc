/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Adapter layer between Ros and Vcc Proxy
 * Create: 2020-04-22
 */
#ifndef RTF_COM_ADAPTER_ROS_SKELETON_ADAPTER_H_
#define RTF_COM_ADAPTER_ROS_SKELETON_ADAPTER_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <ara/core/promise.h>

#include "rtf/com/types/ros_types.h"
#include "rtf/com/utils/logger.h"
#include "vrtf/vcc/api/raw_buffer.h"
#include "vrtf/vcc/utils/rtf_spin_lock.h"
#include "rtf/com/types/method_result.h"
#include "vrtf/vcc/api/vcc_method_return_type.h"

namespace rtf {
namespace com {
namespace utils {
    class SomeipJsonHelper;
}
namespace adapter {
class RosSkeletonAdapter {
public:
    /**
     * @brief RosSkeletonAdapter constructor
     */
    RosSkeletonAdapter(void);

    /**
     * @brief RosSkeletonAdapter destructor
     */
    ~RosSkeletonAdapter(void);

    /**
     * @brief Initialize RosSkeletonAdapter
     * @param[in] uri           The entity uri that the adapter manage
     * @param[in] dataType      The data type of the entity
     * @param[in] type          The adapter type (EVENT/METHOD)
     * @param[in] threadPool    The thread poll that handles operations
     * @param[in] role          The role of the entity
     */
    bool Initialize(const std::string& uri, const MaintainConfig& maintainConfig,
                    AdapterType type, const std::shared_ptr<VccThreadPool>& threadPool, const Role& role) noexcept;

    /**
     * @brief Return whether the adapter is initialized
     * @return Is the adapter initialized
     */
    bool IsInitialized(void) const noexcept;

    /**
     * @brief Return whether the adapter is ready to send/receive data
     * @return Is the adapter ready
     */
    bool IsValid(void) const noexcept;

    /**
     * @brief Create and register a method
     *
     * @tparam Request the type of request message
     * @tparam Response  the type of reponse message
     * @param callback  the callback function
     * @return bool the result of create and register a method
     */
    template<class Request, class Response>
    bool RegisterMethod(std::function<bool(Request&, Response&)> callback) noexcept
    {
        using namespace rtf::com::utils;

        // Initialize Method
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Registering method '" << uri_ << "'...";
        if (!InitializeMethod<vrtf::core::Future<Response>>()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Failed to register method '" << uri_ << "'";
            return false;
        }
        std::function<vrtf::core::Future<Response>(Request)> methodHandler =
            [callback] (Request request) -> vrtf::core::Future<Response> {
                Response response;
                callback(request, response);
                vrtf::core::Promise<Response> promise;
                promise.set_value(response);
                return promise.get_future();
            };
        spinLock_.Lock();
        auto entityMapBack = entityMap_;
        spinLock_.Unlock();
        for (auto& entityMapIterator : entityMapBack) {
            const auto& protocol = entityMapIterator.first;
            const auto& entity   = entityMapIterator.second;
            const auto& skeleton = entity->skeleton;
            // Register method
            skeleton->RegisterMethod(entityId_, methodHandler, threadPool_);
            // Offer entity
            if (!isUsingDefaultConfig_) {
                OfferEntity(entity, protocol);
            }
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Method '" << uri_ << "' is registered";
        return true;
    }

    /**
     * @brief Create and register a method
     *
     * @tparam Request the type of request message
     * @tparam Response  the type of reponse message
     * @param callback  the callback function with E2E result
     * @return bool the result of create and register a method
     */
    template<class Request, class Response>
    bool RegisterMethod(std::function<bool(Request&, Response&, MethodServerResult&)> callback) noexcept
    {
        using namespace rtf::com::utils;

        // Initialize Method
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Registering method '" << uri_ << "'...";
        if (!InitializeMethod<vrtf::vcc::api::VccMethodReturnType<Response>>()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Failed to register method '" << uri_ << "'";
            return false;
        }
        std::function<vrtf::vcc::api::VccMethodReturnType<Response>(rtf::com::e2e::Result, Request)> methodHandler =
            [callback] (rtf::com::e2e::Result result,
                        Request request) -> vrtf::vcc::api::VccMethodReturnType<Response> {
                Response response;
                MethodServerResult serverResult;
                serverResult.SetE2EResult(result);
                callback(request, response, serverResult);
                vrtf::core::Promise<Response> promise;
                promise.set_value(response);
                return vrtf::vcc::api::VccMethodReturnType<Response>(std::move((promise.get_future().GetResult())),
                                                                               serverResult.IsUsingIncorrectE2EId());
            };
        spinLock_.Lock();
        auto entityMapBack = entityMap_;
        spinLock_.Unlock();
        for (auto& entityMapIterator : entityMapBack) {
            const auto& protocol = entityMapIterator.first;
            const auto& entity   = entityMapIterator.second;
            const auto& skeleton = entity->skeleton;
            // Register method
            skeleton->RegisterMethod(entityId_, methodHandler, threadPool_);
            // Offer entity
            if (!isUsingDefaultConfig_) {
                OfferEntity(entity, protocol);
            }
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Method '" << uri_ << "' is registered";
        return true;
    }

    /**
     * @brief Publish a message
     * @param[in] message    Message data
     * @return void
     */
    template <typename EventDataType>
    void Publish(const EventDataType& message, std::shared_ptr<internal::SampleInfoImpl>& info)
    {
        using namespace rtf::com::utils;
        if (!IsValid()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot publish message to '" << uri_ << "', service is not avaliable";
            return;
        }
        spinLock_.Lock();
        auto entityMapBack = entityMap_;
        spinLock_.Unlock();
        for (auto& entityMapIterator : entityMapBack) {
            const auto& entity          = entityMapIterator.second;
            const auto& isEntityOffered = entity->isOffered;
            if (isUsingDefaultConfig_ || isEntityOffered) {
                const auto& skeleton = entity->skeleton;
                skeleton->Send<EventDataType>(message, entityId_, info);
            }
        }
    }

    /**
     * @brief Stop responding the ros uri that adapter handles
     * @return void
     */
    void Shutdown(void) noexcept;

    /**
     * @brief Allocate Buffer for using RawMemory
     *
     * @param[in] size    The size of buffer will be allocated
     * @return RawMemory  The buffer was allocated
     */
    RawMemory AllocateRawMemory(std::size_t size) noexcept;

    /**
     * @brief  Free the allocated buffer last time
     *
     * @param[inout] buffer  the buffer will be free
     */
    void DeallocateRawMemory(RawMemory && buffer) noexcept;

    /**
     * @brief Publish a raw buffer
     *
     * @param[inout] buffer The buffer will be sent
     */
    void PubRawMemory(RawMemory && buffer, std::shared_ptr<internal::SampleInfoImpl>& info) noexcept;

private:
    struct Entity {
        std::shared_ptr<VccSkeleton>  skeleton;
        std::shared_ptr<EntityConfig> config;
        bool isOffered;
    };
    vrtf::vcc::utils::RtfSpinLock spinLock_;
    std::string     uri_;
    EntityId        entityId_;
    AdapterType     type_;
    MaintainConfig  maintainConfig_;
    std::shared_ptr<VccThreadPool> threadPool_;
    std::unordered_map<AdapterProtocol, std::shared_ptr<Entity>> entityMap_;

    bool isUsingDefaultConfig_;
    bool isInitialized_;
    bool isOffered_;
    std::atomic<bool> isShutDown_ {false};
    // maintain SomeipJsonHelper life cycle
    std::shared_ptr<rtf::com::utils::SomeipJsonHelper> someipJsonHelperPtr_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;

    /**
     * @brief Parse config at the begining
     * @param[in] role  The role of the entity
     */
    bool ParseConfig(const Role& role) noexcept;

    void SyncorizeEntityConfig(void) noexcept;

    /**
     * @brief Initialize enity for current config
     *
     * @return Entity initialization result
     */
    bool InitializeEntity(void) noexcept;

    /**
     * @brief Initialize event
     *
     * @return Event initialization result
     */
    bool InitializeEvent(void) noexcept;

    template <class VccResponse>
    bool InitializeMethod(void) noexcept
    {
        using namespace rtf::com::utils;

        bool result = false;
        spinLock_.Lock();
        auto entityMapBack = entityMap_;
        spinLock_.Unlock();
        for (auto& entityMapIterator : entityMapBack) {
            const auto& protocol = entityMapIterator.first;
            const auto& entity   = entityMapIterator.second;

            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] Initializing method '" << uri_ << "'...";
            const auto& methodInfo = std::static_pointer_cast<VccMethodInfo>((entity->config)->entityInfo);
            if (methodInfo == nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Cannot find method configuration for '" << uri_ << "'";
                result = false;
                break;
            }

            const auto& skeleton   = entity->skeleton;
            const auto& driverType = PROTOCOL_DRIVER_MAP.at(protocol);

            // Additional operation is needed for SOME/IP protocol
            if ((driverType == VccDriverType::SOMEIPTYPE) && (!AddServiceToSOMEIPD())) {
                result = false;
                break;
            }
            if (!(skeleton->InitializeMethod<VccResponse>({{ driverType, methodInfo }}))) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Failed to initialize method '" << uri_ << "'";
                result = false;
                break;
            }
            if (entity->config->trafficCrtlPolicy != nullptr) {
                if (skeleton->SetMethodTrafficCtrl(entity->config->trafficCrtlPolicy, methodInfo->GetEntityId())) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                    logger_->Info() << "[RTFCOM] method " << uri_ << " enable traffic control successful";
                } else {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                    logger_->Error() << "[RTFCOM] method " << uri_ << " enable traffic control failed";
                }
            }
            result = true;
        }
        return result;
    }
    /**
     * @brief Offer entity
     * @param[in] protocol The entity of the service
     * @param[in] protocol The protocol that service offered on
     */
    void OfferEntity(const std::shared_ptr<Entity>& entity,
                     const AdapterProtocol& protocol) noexcept;

    /**
     * @brief Stop offer entity
     */
    void StopOfferEntity(void) noexcept;

    /* Static map <key = serviceName, value = entityList{ uri, ... }>
    Since there would be multiple skeletons providing same service across RosSkeletonAdapters,
    we need this map to count literal how many VccSkeleton are providing the service.
    This mechanism ensures the service would be stopped on the right time:
    1. When a VccSkeleton offers a service, it should add its uri to the entity list.
    2. On the contrary, when a VccSkeleton shutsdown, it should NOT simply stop offering the
    service, instead, it should check and erase its uri from the entity list.
    3. When the entity list is empty, the service should be stopped. */
    void AddEntityToServiceMap(const std::string& serviceName) noexcept;
    void RemoveEntityFromServiceMap(const std::string& serviceName) noexcept;
    static std::mutex serviceMapMutex_;
    static std::unordered_map<std::string, std::unordered_set<std::string>> serviceMap_;
    static const std::unordered_map<AdapterProtocol, VccDriverType> PROTOCOL_DRIVER_MAP;
    /**
     * @brief Add an service info to someipd
     *
     * @return true  successfully add to someipd
     * @return false falied to add service info into someipd
     */
    bool AddServiceToSOMEIPD(void) noexcept;

    /**
     * @brief Delete service info from someipd if the last user of the service is called by shutdown
     */
    void DeleteEventServiceFromSOMEIPD(void) noexcept;
};
} // namspace adapter
} // namspace com
} // namspace rtf
#endif // RTF_COM_ADAPTER_ROS_SKELETON_ADAPTER_H_
