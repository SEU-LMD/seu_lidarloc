/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description: CM vcc lint adapter & driver
 * Create: 2019-07-24
 */
#ifndef INC_VCC_HPP_
#define INC_VCC_HPP_

#include <map>
#include <memory>
#include <set>
#include <utility>
#include "ara/core/future.h"
#include "vrtf/vcc/api/types.h"
#include "vrtf/vcc/api/internal/driver_manager.h"
#include "vrtf/vcc/event_skeleton.h"
#include "vrtf/vcc/event_proxy.h"
#include "vrtf/vcc/method_skeleton.h"
#include "vrtf/vcc/method_proxy.h"
#include "vrtf/vcc/utils/thread_pool.h"
#include "vrtf/vcc/utils/rw_lock.h"
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/serialize/method_param_deserializer.h"
#include "vrtf/vcc/internal/service_list_handle.h"
#include "vrtf/vcc/internal/service_callback_handle.h"
#include "vrtf/vcc/internal/traffic_crtl_policy.h"
#include "ara/com/com_error_domain.h"
#include "vrtf/vcc/internal/rtf_maintaind_client_base.h"
#include "vrtf/vcc/utils/safe_map.h"
#include "vrtf/vcc/api/vcc_method_return_type.h"
#include "ara/com/e2e_error_domain.h"
namespace vrtf {
namespace vcc {
class Vcc : public std::enable_shared_from_this<Vcc> {
public:
    using SharedPtrServiceDiscoveryInfo = std::shared_ptr<vrtf::vcc::api::types::ServiceDiscoveryInfo>;
    using TypesSubscriptionStateChangeHandler = vrtf::vcc::api::types::SubscriptionStateChangeHandler;
    using SMState = ara::com::e2e::SMState;
    using DriverType = vrtf::vcc::api::types::DriverType;
    using EventInfo = vrtf::vcc::api::types::EventInfo;
    using MethodInfo = vrtf::vcc::api::types::MethodInfo;
    using FieldInfo = vrtf::vcc::api::types::FieldInfo;
    using MethodInfoMap =
        std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::MethodInfo>>;

    Vcc(std::string const &serviceName, vrtf::vcc::api::types::MethodCallProcessingMode const methodMode);
    Vcc(std::string const &serviceName, vrtf::vcc::api::types::MethodCallProcessingMode const methodMode,
        const bool IsServiceDiscoverySkippedMode);
    Vcc(vrtf::vcc::api::types::HandleType const &handleType, const bool mode = false);
    ~Vcc();
    static vrtf::core::Result<vrtf::vcc::api::types::FindServiceHandle> StartFindService(
        vrtf::vcc::api::types::FindServiceHandler<vrtf::vcc::api::types::HandleType> const &callback,
        std::multimap<api::types::DriverType, std::shared_ptr<api::types::ServiceDiscoveryInfo>> const &srvMap);

    static vrtf::vcc::api::types::ServiceHandleContainer<vrtf::vcc::api::types::HandleType> FindService(
        const std::multimap<vrtf::vcc::api::types::DriverType, SharedPtrServiceDiscoveryInfo>&);

    static void StopFindService(const vrtf::vcc::api::types::FindServiceHandle&);

    static void ServiceAvailableCallback(std::map<api::types::HandleType, bool> const &handleContainer);
    void UnsubscribeEvent(vrtf::vcc::api::types::EntityId const id);
    bool SubscribeEvent(size_t maxSampleCount, std::shared_ptr<vrtf::vcc::api::types::EventInfo> const &eventInfo);
    bool IsSubscribed(vrtf::vcc::api::types::EntityId const id);
    void SetSubscriptionStateChangeHandler(vrtf::vcc::api::types::SubscriptionStateChangeHandler const &handler,
                                           vrtf::vcc::api::types::EntityId const id);
    void UnsetSubscriptionStateChangeHandler(vrtf::vcc::api::types::EntityId const id);
    void SetMethodStateChangeHandler(vrtf::vcc::api::types::MethodStateChangeHandler const &handler,
                                     vrtf::vcc::api::types::EntityId const id);
    void UnsetMethodStateChangeHandler(vrtf::vcc::api::types::EntityId const id);
    size_t GetFreeSampleCount(vrtf::vcc::api::types::EntityId const id);
    // uss in rawbuffer pub
    /**
     * @brief Allocate rawBuffer
     * @details Allocate rawBuffer
     *
     * @param size the size will be allocated
     * @param id EntityId
     * @return RawBuffer return one RawBuffer for pub
     *   @retval rawBuffer allocate buffer successful
     *   @retval nullptr allocate buffer failed
     */
    vrtf::vcc::api::types::RawBuffer AllocateRawBuffer(const size_t size, vrtf::vcc::api::types::EntityId const id);

    /**
     * @brief deallocate buffer when rawBuffer is unuseless
     * @details deallocate buffer when rawBuffer is unuseless
     *
     * @param rawBuffer the rawBuffer class store data pointer to be destroy
     * @param id EntityId
     */
    void DeAllocateRawBuffer(api::types::RawBuffer && rawBuffer, vrtf::vcc::api::types::EntityId const id);

    /**
     * @brief pub rawBuffer
     * @details pub rawBuffer
     *
     * @param rawBuffer the rawBuffer class store data pointer to be send
     * @param id EntityId
     */
    void PubRawBuffer(api::types::RawBuffer && rawBuffer, vrtf::vcc::api::types::EntityId const id,
                      std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info);

    vrtf::vcc::api::types::EventSubscriptionState GetSubscriptionState(
        vrtf::vcc::api::types::EntityId const id) const;

    void DestroyWaitingMethodPromise(vrtf::core::ErrorCode const errorCode, vrtf::vcc::api::types::EntityId const id);

    template<class SampleType>
    ara::core::Result<size_t> GetNewSamples(std::function<void(vrtf::vcc::api::types::SamplePtr<SampleType const>)> &cb,
                                            vrtf::vcc::api::types::EntityId const id,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max()) noexcept
    {
        size_t getSampleFail {0};
        std::shared_ptr<vrtf::vcc::EventProxy> eventProxyPtr;
        if (mapEventProxy_.Find(id, eventProxyPtr) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Event " << id << " was not initialized, can not get new samples";
            return ara::core::Result<size_t>(getSampleFail);
        }

        if (eventProxyPtr->GetDriver() == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Event " << id << " was not Subscribed, can not get new samples";
            return ara::core::Result<size_t>(getSampleFail);
        }
        EventProxy* eventProxy {eventProxyPtr.get()};
        EventProxyImpl<SampleType> *ptr {static_cast<EventProxyImpl<SampleType>* >(eventProxy)};
        return ptr->GetNewSamples(cb, maxNumberOfSamples);
    }

    template<class SampleType>
    ara::core::Result<size_t> GetNewSamples(void(*cb)(vrtf::vcc::api::types::SamplePtr<SampleType const>),
                                            vrtf::vcc::api::types::EntityId const id,
                                            size_t maxNumberOfSamples = std::numeric_limits<size_t>::max()) noexcept
    {
        size_t getSampleFail {0};
        std::shared_ptr<vrtf::vcc::EventProxy> eventProxyPtr;
        if (mapEventProxy_.Find(id, eventProxyPtr) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Event " << id << " was not initialized, can not get new samples";
            return ara::core::Result<size_t>(getSampleFail);
        }

        if (eventProxyPtr->GetDriver() == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Event " << id << " was not Subscribed, can not get new samples";
            return ara::core::Result<size_t>(getSampleFail);
        }
        EventProxy* eventProxy {eventProxyPtr.get()};
        EventProxyImpl<SampleType> *ptr {static_cast<EventProxyImpl<SampleType>* >(eventProxy)};
        return ptr->GetNewSamples([this, &cb](vrtf::vcc::api::types::SamplePtr<SampleType const> ptr) {
                cb(std::move(ptr));
                return;
            }, maxNumberOfSamples);
    }

    SMState GetSMState(vrtf::vcc::api::types::EntityId const id) const noexcept;
    ara::com::e2e::Result const GetResult(vrtf::vcc::api::types::EntityId const id) const;
    ara::com::e2e::Result const GetMethodE2EResult(vrtf::vcc::api::types::EntityId const id) const;

    template<class SampleType>
    bool InitializeSubscribe(vrtf::vcc::api::types::EntityId const id,
                             size_t maxSampleCount,
                             std::shared_ptr<vrtf::vcc::api::types::EventInfo> eventInfo,
                             std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        vrtf::vcc::utils::PlogInfo::InitPlog(vrtf::vcc::utils::CM_RECV);
        if (mapEventProxy_.Find(id) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "New Event subscribe prepare " << id;
            std::shared_ptr<EventProxy> eventProxy {
                std::static_pointer_cast<EventProxy>(
                std::make_shared<EventProxyImpl<SampleType>>(id, maxSampleCount, handle_.GetDriver()))};
            eventProxy->SetEventInfo(eventInfo);
            eventProxy->AllocateSamplePtr();
            RegisterProxyLatencyCallback(eventProxy, eventInfo);

            if (pool != nullptr) {
                eventProxy->SetRosThreadPool(pool);
            }
            mapEventProxy_.Insert(id, eventProxy);
            return true;
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Event subscription alreay initialized " << id;
            return false;
        }
    }

    void SetReceiveHandler(
        vrtf::vcc::api::types::EventReceiveHandler const &handler, vrtf::vcc::api::types::EntityId const id);
    void UnsetReceiveHandler(vrtf::vcc::api::types::EntityId const id);

    // method
    template<class Result, class... Args>
    vrtf::core::Future<Result> Request(vrtf::vcc::api::types::EntityId const id, Args... args) noexcept
    {
        // In case the ServiceStatusChanged(false) is triggered, after the isServiceAvailable_ checking and
        // before Reuqest(i.e. Add request promise in MethodProxy's proxy_), which will lead to a never returned
        // there should be a lock in this function and ServiceStatusChanged function.
        std::lock_guard<std::mutex> lock(serviceStatusMutex_);
        // Method request will be returned immediately with kServiceNotAvailable error, when server is offline
        if ((!isServiceAvailable_) && (!isServiceDiscoverySkippedMode_)) {
            vrtf::core::Promise<Result> promise;
            promise.SetError(vrtf::core::ErrorCode(vrtf::vcc::api::types::ComErrc::kServiceNotAvailable));
            return promise.get_future();
        }
        vrtf::core::Future<Result> ret;
        /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
        const auto iter = mapMethodProxy_.find(id);
        if (iter != mapMethodProxy_.end()) {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto mpImpl = std::static_pointer_cast<MethodProxyImpl<Result>>(iter->second);
            ret = mpImpl->Request(args...);
        }
        return ret;
    }

    bool SetEventTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy>& policy,
        vrtf::vcc::api::types::EntityId const id);
    bool SetMethodTrafficCtrl(const std::shared_ptr<rtf::TrafficCtrlPolicy>& policy,
        vrtf::vcc::api::types::EntityId const id);
    void OfferService(
        std::map<vrtf::vcc::api::types::DriverType, SharedPtrServiceDiscoveryInfo> protocolData);
    void StopOfferService(
        std::map<vrtf::vcc::api::types::DriverType, SharedPtrServiceDiscoveryInfo> protocolData);

    // for client
    /**
     * @brief Initialize Method Proxy
     * @details Create MethodProxy to distingush driver type and control method send/receive
     * @param[in] id EntityId is the identification to different method/event/field
     * @param[in] protocolData the method Info read from config file
     * @return Whether Create MethodProxy is successful
     *   @retval true Create MethodProxy is successful
     *   @retval false Create MethodProxy is fail
     * @note AUTOSAR AP R19-11 RS_CM_00211
     */
    template<class ResultType>
    bool InitializeMethodProxy(vrtf::vcc::api::types::EntityId const id,
                               const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& protocolData) noexcept
    {
        /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
        const auto iter = mapMethodProxy_.find(id);
        if (iter == mapMethodProxy_.end()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "New Method initialize prepare " << id;
            std::shared_ptr<MethodProxy> methodProxy {
                std::static_pointer_cast<MethodProxy>(std::make_shared<MethodProxyImpl<ResultType>>(id))};
            methodProxy->SetMethodInfo(protocolData);
            static_cast<void>(mapMethodProxy_.emplace(std::pair<vrtf::vcc::api::types::EntityId,
                                   std::shared_ptr<MethodProxy>>(id, methodProxy)));
            return InitializeMethodHandler(protocolData);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method " << id << " alreay initialized";
            return false;
        }
    }

    bool SetMethodThreadNumber(const std::uint16_t number, const std::uint16_t queueSize);

    // for server
    /**
     * @brief Initialize Method Skeleton
     * @details Create MethodSkeleton and use DoInitializeMethodSkel
     * @param[in] methodData the method Info read from config file
     * @return Whether Create MethodSkeleton & method driver layer is successful
     *   @retval true Create MethodSkeleton & method driver layer  is successful
     *   @retval false Create MethodProxy & method driver layer is fail
     */
    template<class Result>
    bool InitializeMethodSkel(std::map<vrtf::vcc::api::types::DriverType,
                              std::shared_ptr<vrtf::vcc::api::types::MethodInfo>> methodData) noexcept
    {
        using namespace vrtf::vcc::api::types;
        if ((methodData.size() == 0) || (methodData.begin()->second == nullptr)) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Invalid method data.";
            return false;
        }
        /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
        auto id = methodData.begin()->second->GetEntityId();
        auto iter = mapMethodSkel_.find(id);
        /* AXIVION enable style AutosarC++19_03-A8-5-2 */
        if (iter != mapMethodSkel_.end()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method " << id << " was registered. Ignore this time...";
        } else {
            std::shared_ptr<MethodSkeleton> ptr =
                std::static_pointer_cast<MethodSkeleton>((std::make_shared<MethodSkeletonImpl<Result>>)
                    (id, methodMode_));
            ptr->SetMethodInfo(methodData);
            static_cast<void>(mapMethodSkel_.emplace(std::pair<EntityId, std::shared_ptr<MethodSkeleton>>(id, ptr)));
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Method " << id << " register success!";
        }
        if (DoInitializeMethodSkel(methodData) == false) {
            return false;
        }
        return true;
    }

    // for server
    /**
     * @brief Initialize Event Skeleton
     * @details Create EventSkeleton to distingush driver type and control method send/receive
     * @param[in] eventData the event Info read from config file
     * @return Whether Create EventSkeleton & method driver layer is successful
     *   @retval true Create EventSkeleton & method driver layer  is successful
     *   @retval false Create EventSkeleton & method driver layer is fail
     * @note AUTOSAR AP R19-11 RS_CM_00201
     */
    bool InitializeEventSkel(std::map<vrtf::vcc::api::types::DriverType,
                             std::shared_ptr<vrtf::vcc::api::types::EventInfo>> eventData);
    // for server
    /**
     * @brief Initialize Field Skeleton
     * @details Create FieldSkeleton to distingush driver type and control method send/receive
     * @param[in] eventData the field Info read from config file
     * @param[in] hasNotifier the field is have notify function
     * @return Whether Create FieldSkeleton & method/event driver layer is successful
     *   @retval true Create FieldSkeleton & method/event driver layer is successful
     *   @retval false Create FieldSkeleton & method/event driver layer is fail
     * @note AUTOSAR AP R19-11 RS_CM_00216 RS_CM_00217 RS_CM_00218
     */
    template<class SampleType>
    bool InitializeFieldSkel(std::map<vrtf::vcc::api::types::DriverType,
        std::shared_ptr<vrtf::vcc::api::types::EventInfo>> eventData, bool hasNotifier = true) noexcept
    {
        using namespace vrtf::vcc::api::types;
        for (auto event : eventData) {
            std::shared_ptr<vrtf::vcc::driver::EventHandler> drv {nullptr};
            if (hasNotifier) {
                drv = drvManager_->CreateEvent(event.first, event.second);
                if (drv == nullptr) {
                    return false;
                }
            }
            EntityId id {event.second->GetEntityId()};
            std::shared_ptr<vrtf::vcc::EventSkeleton> fieldSkeleton;
            if (mapEventSkel_.Find(id, fieldSkeleton) == true) {
                if (hasNotifier) {
                    fieldSkeleton->AddDriver(event.first, drv);
                }
            } else {
                std::shared_ptr<FieldSkelton<SampleType>> es {
                    std::make_shared<FieldSkelton<SampleType>>(id, hasNotifier)};
                es->SetEventInfo(eventData);
                mapEventSkel_.Insert(id, es);
                fieldSkeleton = es;
                if (hasNotifier) {
                    es->AddDriver(event.first, drv);
                }
            }
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logInstance_->info() << "Create field[serviceId=" << event.second->GetServiceId()
                            << ", instanceId="  << event.second->GetInstanceId() << ", shortName="
                            << event.second->GetShortName() << ",  protocol=" << DriverTypeMap.at(event.first) << "]";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            fieldSkeleton->SetShortNameByDriver(event.first, event.second->GetShortName());
            fieldSkeleton->SetInstanceId(event.second->GetInstanceId());
        }
        return true;
    }

    std::shared_ptr<EventSkeleton> GetEventSkel(vrtf::vcc::api::types::EntityId const id);

    /**
     * @brief Send event/field By skeleton applications, skeleton should initialize field first.
     * @param[in] data  Data will be send.
     * @param[in] id    The entity id of event.
     */
    template<class SampleType>
    void Send(const SampleType& data, vrtf::vcc::api::types::EntityId const id,
              std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Event '"<< id << "' were not initialized";
            return;
        }

        if (!(eventSkeleton)->IsField()) {
            if ((isOffered_ == true) || (isServiceDiscoverySkippedMode_)) {
                eventSkeleton->Send<SampleType>(data, info);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance_->warn() << "Event '"<< id << "' can not be send, please offer service first";
            }
        } else {
            (std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton))->Send(data, isOffered_, info);
        }
    }

    /* Send Event By skeleton applications, skeleton should initialize event first. */
    /**
     * @brief Send event/field By skeleton applications, skeleton should initialize field first.
     * @param[in] data  one pointer item which point to Data will be send.
     * @param[in] id    The entity id of event.
     */
    template<class SampleType>
    void Send(std::unique_ptr<SampleType> data, vrtf::vcc::api::types::EntityId const id,
              std::shared_ptr<vrtf::vcc::api::types::internal::SampleInfoImpl>& info) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Event '"<< id << "' were not initialized";
            return;
        }
        if ((isOffered_ == true) || isServiceDiscoverySkippedMode_) {
            eventSkeleton->Send<SampleType>(std::move(data), info);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Event '"<< id << "' can not be send, please offer service first";
        }
    }

    // for sever
    /**
     * @brief Register method callback for field set
     * @param[in] callback The registered callback.
     * @param[in] c the class which the callback belong to.
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class T, class Result, class SampleType>
    void RegisterMethodForSet(Result(T::*callback)(SampleType, bool), T& c, vrtf::vcc::api::types::EntityId const id,
                        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        EncapsulateAndRegisterMethodForSet<T, Result, SampleType>(callback, c, id, pool);
    }

    // for sever
    /**
     * @brief Register method callback
     * @param[in] callback The registered callback.
     * @param[in] c the class which the callback belong to.
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class T, class Result, class... Args>
    void RegisterMethod(Result(T::*callback)(Args...), T& c, vrtf::vcc::api::types::EntityId const id,
                        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        EncapsulateAndRegisterMethod<T, Result, Args...>(callback, c, id, std::index_sequence_for<Args...>(), pool);
    }

    // for sever
    /**
     * @brief Register method callback
     * @param[in] callback The registered callback.
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result, class... Args>
    void RegisterMethod(vrtf::vcc::api::types::EntityId const id, Result(*callback)(const Args &...),
                        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        EncapsulateAndRegisterMethod<Result>(
            id, callback, std::index_sequence_for<Args...>(), pool);
    }

    // for sever
    /**
     * @brief Register method callback
     * @param[in] callback The registered callback.
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result, class... Args>
    void RegisterMethod(vrtf::vcc::api::types::EntityId const id, std::function<Result(Args...)> callback,
                        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        EncapsulateAndRegisterMethod<Result>(
            id, callback, std::index_sequence_for<Args...>(), pool);
    }

    // for sever
    /**
     * @brief Register method callback
     * @param[in] callback The registered callback with E2E result.
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result, class... Args>
    void RegisterMethod(vrtf::vcc::api::types::EntityId const id,
                        std::function<Result(ara::com::e2e::Result, Args...)> callback,
                        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        EncapsulateAndRegisterMethod<Result>(
            id, callback, std::index_sequence_for<Args...>(), pool);
    }

    // for sever
    /**
     * @brief Register method callback
     * @param[in] callback The registered callback.
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result>
    void RegisterMethod(vrtf::vcc::api::types::EntityId const id, std::function<Result()> callback,
                        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool = nullptr) noexcept
    {
        DoRegisterMethod<Result>(
            id, [callback](const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                MethodInfoMap& methodInfo)->Result {
            static_cast<void>(msg);
            static_cast<void>(methodInfo);
            return callback();
        }, pool);
    }

    void UnregisterAllMethod();

    template<class SampleType>
    void RegisterGetMethod(std::function<vrtf::core::Future<SampleType>()> getHandler,
                           vrtf::vcc::api::types::EntityId const id) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Field " << id <<" isn't initialized can't register get handler ";
        } else {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto fs = std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton);
            fs->RegisterGetMethod(getHandler);
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Register get handler for field " << id <<" successfully! ";
        }
    }

    template<class SampleType>
    void RegisterSetMethod(std::function<vrtf::core::Future<SampleType>(const SampleType& value)> setHandler,
                           vrtf::vcc::api::types::EntityId const id) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton) == false) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->error() << "Field " << id <<" isn't initialized can't register set handler ";
        } else {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto fs = std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton);
            fs->RegisterSetMethod(setHandler);
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->verbose() << "Register set handler for field " << id <<" successfully! ";
        }
    }
    /**
     * @brief Query sethandler of the field
     * @param[in] id-EntityId for the field
     * @param[out] handler- getHandler for the field registered by RegisterGetMethod().
     *
     */
    template<class SampleType>
    void QueryGetMethod(vrtf::vcc::api::types::EntityId const id,
        std::function<vrtf::core::Future<SampleType>()>& handler) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton)) {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto fs = std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton);
            handler = fs->QueryGetMethod();
        }
    }

    /**
     * @brief Query sethandler of the field
     * @param[in] id        EntityId for the field
     * @param[out] handler  setHandler for the field registered by RegisterSetMethod().
     *
     */
    template<class SampleType>
    void QuerySetMethod(vrtf::vcc::api::types::EntityId const id,
        std::function<vrtf::core::Future<SampleType>(const SampleType& value)>& handler) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton)) {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto fs = std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton);
            handler = fs->QuerySetMethod();
        }
    }

    /**
     * @brief Query field info of the field
     * @details Field info: hasGetter_, hasSetter_, hasNotifier_
     * @param[in] id EntityId for the field
     * @param[out] info Field info for the field parsed from config file.
     *
     */
    void QueryFieldInfo(vrtf::vcc::api::types::EntityId const id,
        std::shared_ptr<vrtf::vcc::api::types::FieldInfo>& info);

    /**
     * @brief check if the field initialized.
     * @return the result of check
     *   @retval true the field has initialized
     *   @retval false the field has not initialized
     */
    template<class SampleType>
    bool IsFieldInitialized(vrtf::vcc::api::types::EntityId const id) noexcept
    {
        bool res {false};
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton)) {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto fs = std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton);
            res = fs->HasInitData();
        }
        return res;
    }

    /**
     * @brief Set field to uninitialized state
     * @param[in] id method entity id.
     */
    template<class SampleType>
    void ResetInitState(vrtf::vcc::api::types::EntityId const id) noexcept
    {
        std::shared_ptr<vrtf::vcc::EventSkeleton> eventSkeleton;
        if (mapEventSkel_.Find(id, eventSkeleton)) {
            std::static_pointer_cast<FieldSkelton<SampleType>>(eventSkeleton)->SetFieldStatus(false);
        }
    }

    void RegisterError(vrtf::vcc::api::types::EntityId const id, vrtf::core::ErrorCode const error);

    bool ProcessNextMethodCall();
    void ProcessMethodRequest(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg);
    // use by maintaind
    void RegisterFieldProxyInfoToMaintaind(const std::shared_ptr<vrtf::vcc::api::types::FieldInfo>& protocolData);
    void SetFieldInfo(
        const std::map<vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::api::types::FieldInfo>>& fieldmap,
        vcc::api::types::EntityId const fieldid);

    void RegisterE2EErrorHandler(const std::function<void(vrtf::vcc::api::types::E2EErrorCode,
        vrtf::vcc::api::types::E2EDataId, vrtf::vcc::api::types::MessageCounter)>& callback)
    {
        e2eErrorCallback_ = callback;
    }
    void AddServiceStatusChangedCallback() noexcept;
private:
    /**
     * @brief Encapsulate the registed method in a format that the input parameter is MethodMsg type and then register
     *        the Encapsulation method for field set in server
     * @details the Encapsulation function deserializes the serialized parameters from MethodMsg and either the
     *          registered function result is returned, if successfully,
     *          or set flag false to field set callback.
     * @param[in] callback The registered callback(Field set function).
     * @param[in] c the class which the callback belong to.
     * @param[in] index_sequence Input parameters sequence
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class T, class Result, class SampleType>
    void EncapsulateAndRegisterMethodForSet(Result(T::*callback)(SampleType, bool), T& c,
        vrtf::vcc::api::types::EntityId const id, std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool) noexcept
    {
        DoRegisterMethod<Result>(
            id, [callback, &c, id, this](const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                MethodInfoMap& methodInfo)->Result {
            using namespace ara::godel::common;
            /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            const auto logInstance = log::Log::GetLog("CM");
            const auto info = methodInfo[msg->GetDriverType()];
            auto sizeTmp = msg->GetSize();
            /* AXIVION enable style AutosarC++19_03-A8-5-2 */
            std::size_t deserializeSize {0};
            using value_type = typename std::decay<SampleType>::type;
            if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SHM) {
                vrtf::serialize::dds::Deserializer<value_type> desr(msg->GetPayload(), sizeTmp,
                    info->GetRequestSerializeConfig());
                deserializeSize = desr.GetSize();
                if (deserializeSize > sizeTmp) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->debug() << "Serialize method parameter failed with entity" << id;
                    return (c.*callback)(typename std::decay<SampleType>::type(), true);
                } else {
                    return (c.*callback)(desr.GetValue(), false);
                }
            } else if (msg->GetSerializeType() == vrtf::serialize::SerializeType::SOMEIP) {
                vrtf::serialize::someip::Deserializer<value_type> desr(msg->GetPayload(), sizeTmp,
                    info->GetRequestSerializeConfig());
                deserializeSize = desr.GetSize();
                if (deserializeSize > sizeTmp) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->debug() << "Serialize method parameter failed with entity" << id;
                    return (c.*callback)(typename std::decay<SampleType>::type(), true);
                } else {
                    return (c.*callback)(desr.GetValue(), false);
                }
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->warn() << "Wrong serialize type, SHM deserialize type used!";
                vrtf::serialize::dds::Deserializer<value_type> desr(msg->GetPayload(), sizeTmp,
                    info->GetRequestSerializeConfig());
                deserializeSize = desr.GetSize();
                if (deserializeSize > sizeTmp) {
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                    logInstance->debug() << "Serialize method parameter failed with entity" << id;
                    return (c.*callback)(typename std::decay<SampleType>::type(), true);
                } else {
                    return (c.*callback)(desr.GetValue(), false);
                }
            }
        }, pool);
    }

    template<typename Result, typename std::enable_if<ara::core::internal::IsFuture<Result>::value>::type* = nullptr>
    Result ReturnMethodErrorCode() noexcept
    {
        typename Result::PromiseType promise;
        promise.SetError(vrtf::core::ErrorCode(vrtf::vcc::api::types::ComErrc::kNetworkBindingFailure));
        return promise.get_future();
    }

    template<typename Result, typename std::enable_if<!ara::core::internal::IsFuture<Result>::value>::type* = nullptr>
    Result ReturnMethodErrorCode() noexcept
    {
        return;
    }

    template<typename Result, typename std::enable_if<ara::core::internal::IsFuture<Result>::value>::type* = nullptr>
    Result ReturnE2EErrorCode(ara::com::e2e::ProfileCheckStatus profileCheckStatus)
    {
        typename Result::PromiseType promise;
        promise.SetError(vrtf::core::ErrorCode(mapE2EErrCode_.at(profileCheckStatus)));
        return promise.get_future();
    }

    template<typename Result, typename std::enable_if<!ara::core::internal::IsFuture<Result>::value>::type* = nullptr>
    Result ReturnE2EErrorCode(ara::com::e2e::ProfileCheckStatus profileCheckStatus)
    {
        static_cast<void>(profileCheckStatus);
        return;
    }

    /**
     * @brief Encapsulate the registed method in a format that the input parameter is MethodMsg type and then register
     *        the Encapsulation method in server
     * @details the Encapsulation function deserializes the serialized parameters from MethodMsg and either the
     *          registered function result is returned, if successfully,
     *          or the kNetworkBindingFailure error is returned.
     * @param[in] callback The registered callback.
     * @param[in] c the class which the callback belong to.
     * @param[in] index_sequence Input parameters sequence
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class T, class Result, class... Args, std::size_t... I>
    void EncapsulateAndRegisterMethod(Result(T::*callback)(Args...), T& c, vrtf::vcc::api::types::EntityId const id,
        std::index_sequence<I...>, std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool) noexcept
    {
        DoRegisterMethod<Result>(
            id, [this, callback, &c, id](const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                MethodInfoMap& methodInfo)->Result {
            using namespace ara::godel::common;
            /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            const auto logInstance = log::Log::GetLog("CM");
            const auto info = methodInfo[msg->GetDriverType()];
            /* AXIVION enable style AutosarC++19_03-A8-5-2 */
            if (info->GetE2EObject() != nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
                auto profileCheckStatus = msg->GetE2EResult().GetProfileCheckStatus();
                // ignore repeated error of receiving request in ara com
                if (profileCheckStatus == ara::com::e2e::ProfileCheckStatus::kRepeated) {
                    profileCheckStatus = ara::com::e2e::ProfileCheckStatus::kOk;
                }
                if (profileCheckStatus != ara::com::e2e::ProfileCheckStatus::kOk) {
                    // Step 1: call E2E Error Handler
                    this->CallE2EErrHandler(id, info, msg, this->mapE2EErrCode_);
                    // Step 2: set and return E2E ErrorCode
                    return this->ReturnE2EErrorCode<Result>(profileCheckStatus);
                }
            }
            vrtf::vcc::serialize::ParamsDeserializer<Args...> deserializer(
                *msg, info->GetRequestSerializeConfig());
            if (deserializer.IndexMethodParameter(typename std::decay<Args>::type()...)) {
                return (c.*callback)(deserializer.template GetValue<I>()...);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->debug() << "Serialize method parameter failed with entity" << id;
                return this->ReturnMethodErrorCode<Result>();
            }
        }, pool);
    }

    /**
     * @brief Encapsulate the registed method in a format that the input parameter is MethodMsg type and then register
     *        the Encapsulation method in server
     * @details the Encapsulation function deserializes the serialized parameter from MethodMsg and either the
     *          registered function result is returned, if successfully, or the kNetworkBindingFailure error is
     *          returned.
     * @param[in] callback The registered callback.
     * @param[in] index_sequence Input parameters sequence
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result, class... Args, std::size_t... I>
    void EncapsulateAndRegisterMethod(vrtf::vcc::api::types::EntityId const id, Result(*callback)(const Args &...),
        std::index_sequence<I...>, std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool) noexcept
    {
        DoRegisterMethod<Result>(
            id, [this, callback, id](const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                MethodInfoMap& methodInfo)->Result {
            using namespace ara::godel::common;
            /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            const auto logInstance = log::Log::GetLog("CM");
            const auto info = methodInfo[msg->GetDriverType()];
            /* AXIVION enable style AutosarC++19_03-A8-5-2 */
            if (info->GetE2EObject() != nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
                auto profileCheckStatus = msg->GetE2EResult().GetProfileCheckStatus();
                // ignore repeated error of receiving request in ara com
                if (profileCheckStatus == ara::com::e2e::ProfileCheckStatus::kRepeated) {
                    profileCheckStatus = ara::com::e2e::ProfileCheckStatus::kOk;
                }
                if (profileCheckStatus != ara::com::e2e::ProfileCheckStatus::kOk) {
                    // Step 1: call E2E Error Handler
                    this->CallE2EErrHandler(id, info, msg, this->mapE2EErrCode_);
                    // Step 2: set and return E2E ErrorCode
                    return this->ReturnE2EErrorCode<Result>(profileCheckStatus);
                }
            }
            vrtf::vcc::serialize::ParamsDeserializer<Args...> deserializer(
                *msg, info->GetRequestSerializeConfig());
            if (deserializer.IndexMethodParameter(typename std::decay<Args>::type()...)) {
                return (*callback)(deserializer.template GetValue<I>()...);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->debug() << "Serialize method parameter failed with entity" << id;
                return this->ReturnMethodErrorCode<Result>();
            }
        }, pool);
    }

    /**
     * @brief Encapsulate the registed method in a format that the input parameter is MethodMsg type and then register
     *        the Encapsulation method in server
     * @details the Encapsulation function deserializes the serialized parameter from MethodMsg and either the
     *          registered function result is returned, if successfully, or the kNetworkBindingFailure error is
     *          returned.
     * @param[in] callback The registered callback.
     * @param[in] index_sequence Input parameters sequence
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result, class... Args, std::size_t... I>
    void EncapsulateAndRegisterMethod(vrtf::vcc::api::types::EntityId const id,
        std::function<Result(Args...)> callback,
        std::index_sequence<I...>,
        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool) noexcept
    {
        DoRegisterMethod<Result>(
            id, [this, callback, id](const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                MethodInfoMap& methodInfo)->Result {
            using namespace ara::godel::common;
            /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            const auto logInstance = log::Log::GetLog("CM");
            const auto info = methodInfo[msg->GetDriverType()];
            /* AXIVION ensable style AutosarC++19_03-A8-5-2 */
            if (info->GetE2EObject() != nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
                auto profileCheckStatus = msg->GetE2EResult().GetProfileCheckStatus();
                // ignore repeated error of receiving request in ara com
                if (profileCheckStatus == ara::com::e2e::ProfileCheckStatus::kRepeated) {
                    profileCheckStatus = ara::com::e2e::ProfileCheckStatus::kOk;
                }
                if (profileCheckStatus != ara::com::e2e::ProfileCheckStatus::kOk) {
                    // Step 1: call E2E Error Handler
                    this->CallE2EErrHandler(id, info, msg, this->mapE2EErrCode_);
                    // Step 2: set and return E2E ErrorCode
                    return this->ReturnE2EErrorCode<Result>(profileCheckStatus);
                }
            }
            vrtf::vcc::serialize::ParamsDeserializer<Args...> deserializer(
                *msg, info->GetRequestSerializeConfig());
            if (deserializer.IndexMethodParameter(typename std::decay<Args>::type()...)) {
                return callback(deserializer.template GetValue<I>()...);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->debug() << "Serialize method parameter failed with entity" << id;
                return this->ReturnMethodErrorCode<Result>();
            }
        }, pool);
    }

    /**
     * @brief Encapsulate the registed method in a format that the input parameter is MethodMsg type and then register
     *        the Encapsulation method, used by server of rtf_com
     * @details the Encapsulation function deserializes the serialized parameter from MethodMsg and either the
     *          registered function result is returned, if successfully, or the kNetworkBindingFailure error is
     *          returned.
     * @param[in] callback The registered callback with E2E.
     * @param[in] index_sequence Input parameters sequence
     * @param[in] id method entity id.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result, class... Args, std::size_t... I>
    void EncapsulateAndRegisterMethod(vrtf::vcc::api::types::EntityId const id,
        std::function<Result(ara::com::e2e::Result, Args...)> callback,
        std::index_sequence<I...>,
        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool) noexcept
    {
        DoRegisterMethod<Result>(
            id, [this, callback, id](const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                MethodInfoMap& methodInfo)->Result {
            vrtf::vcc::serialize::ParamsDeserializer<Args...> deserializer(
                *msg, methodInfo[msg->GetDriverType()]->GetRequestSerializeConfig());
            if (deserializer.IndexMethodParameter(typename std::decay<Args>::type()...)) {
                return callback(msg->GetE2EResult(), deserializer.template GetValue<I>()...);
            } else {
                using namespace ara::godel::common;
                /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
                auto logInstance = log::Log::GetLog("CM");
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
                logInstance->debug() << "Serialize method parameter failed with entity" << id;
                return Result(vrtf::core::ErrorCode(vrtf::vcc::api::types::ComErrc::kNetworkBindingFailure));
            }
        }, pool);
    }

    void CallE2EErrHandler(vrtf::vcc::api::types::EntityId const id,
                        const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& info,
                        const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
                        std::map<ara::com::e2e::ProfileCheckStatus, vrtf::vcc::api::types::E2EErrorCode> mapE2EErrCode)
    {
        using namespace ara::godel::common;
        /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
        auto logInstance = log::Log::GetLog("CM");
        const auto e2eObject = info->GetE2EObject();
        const auto profileCheckStatus = msg->GetE2EResult().GetProfileCheckStatus();
        /* AXIVION enable style AutosarC++19_03-A8-5-2 */
        if (e2eErrorCallback_ != nullptr) {
            if (e2eObject->IsUsingDataID()) {
                ara::core::Vector<std::uint32_t> dataId;
                dataId.push_back(e2eObject->GetDataID());
                e2eErrorCallback_(mapE2EErrCode.at(profileCheckStatus), dataId, msg->GetE2ECounter());
            } else {
                /* AXIVION disable style AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
                const auto arr = e2eObject->GetDataIDList();
                const ara::core::Vector<std::uint32_t> dataId(arr.begin(), arr.end());
                e2eErrorCallback_(mapE2EErrCode.at(profileCheckStatus), dataId, msg->GetE2ECounter());
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance->debug() << "E2E error handler is nullptr for [entityId = " << id << "]";
        }
    }

    void RegisterMethodPool(vrtf::vcc::api::types::EntityId const id,
                            const std::shared_ptr<vrtf::vcc::utils::ThreadPool>& pool);
    /**
     * @brief Register encapsulated method callback into MethodSkeleton.
     *        And MethodSkeleton will be created in this function in server.
     * @param[in] id method entity id.
     * @param[in] callback The registered encapsulated callback.
     * @param[in] pool the thread pool which the method will work in, this param is used in RTFCOM interface.
     */
    template<class Result>
    void DoRegisterMethod(vrtf::vcc::api::types::EntityId const id,
        std::function<Result(const std::shared_ptr<vrtf::vcc::api::types::MethodMsg>& msg,
            MethodInfoMap& methodInfo)> callback,
        std::shared_ptr<vrtf::vcc::utils::ThreadPool> pool)
    {
        using namespace vrtf::vcc::api::types;
        /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
        auto iter = mapMethodSkel_.find(id);
        if (iter == mapMethodSkel_.end()) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->warn() << "Method " << id << " have not registered. Ignore this time register...";
        } else {
            /* AXIVION Next Line AutosarC++19_03-A8-5-2 : Function return type allow to use auto */
            auto implPtr = (std::static_pointer_cast<MethodSkeletonImpl<Result>>)(iter->second);
            implPtr->SetMethodSkelCallBack(callback);
            RegisterMethodPool(id, pool);
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
            logInstance_->debug() << "Method " << id << " register success!";
        }
    }

    /**
     * @brief Initialize Method Skeleton
     * @details distingush driver type and create method driver
     * @param[in] methodData the method Info read from config file
     * @return Whether create method driver is successful
     *   @retval create method driver is successful
     *   @retval create method driver is failed
     * @note AUTOSAR AP R19-11 RS_CM_00211
     */
    bool DoInitializeMethodSkel(std::map<vrtf::vcc::api::types::DriverType,
                                         std::shared_ptr<vrtf::vcc::api::types::MethodInfo>> methodData);

    bool InitializeMethodHandler(const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& protocolData);
    void UnRegisterMaintaindInfo(vrtf::vcc::api::types::EntityId const id);
    void ServiceStatusChanged(const bool isAvailable);
    static void HandleAvailableServices(const std::vector<vrtf::vcc::api::types::HandleType>& handleContainer);
    void SetReceiveHandlerAndStatusHandler(vrtf::vcc::api::types::EntityId const id,
        const std::shared_ptr<vrtf::vcc::EventProxy>& eventProxyPtr);
    void SendFieldInitData();
    void RegisterProxyLatencyCallback(const std::shared_ptr<EventProxy>& eventProxy,
                                      std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo) const;
    void RegisterSkeletonLatencyCallback(const std::shared_ptr<EventSkeleton>& eventSkeleton,
        std::map<api::types::DriverType, std::shared_ptr<api::types::EventInfo>>& eventData) const;

    static std::shared_ptr<internal::ServiceCallbackHandle> serviceCallback_;
    static std::shared_ptr<internal::ServiceListHandle> serviceList_;
    // common variables
    static const std::map<api::types::DriverType, std::string> DriverTypeMap;
    std::string serviceName_ {vrtf::vcc::api::types::UNDEFINED_SERVICE_NAME};
    uint16_t serviceId_ {vrtf::vcc::api::types::UNDEFINED_SERVICEID};
    bool isServiceDiscoverySkippedMode_ {false};
    std::shared_ptr<vrtf::vcc::api::internal::DriverManager> drvManager_ {nullptr};
    std::map<vrtf::vcc::api::types::EntityId, std::map<DriverType, std::shared_ptr<EventInfo>>> eventInfoList_;
    std::map<vrtf::vcc::api::types::EntityId, std::map<DriverType, std::shared_ptr<MethodInfo>>> methodInfoList_;
    std::map<vrtf::vcc::api::types::EntityId, std::map<DriverType, std::shared_ptr<FieldInfo>>> fieldInfoList_;
    // Save rtfMaintaindClient instance to prevent which is deconstructed before vcc
    std::shared_ptr<vrtf::vcc::RtfMaintaindClientBase> rtfMaintaindClient_ {nullptr};
    bool isSkeleton_;

    // skeleton variables
    bool isOffered_ {false};
    vrtf::vcc::api::types::MethodCallProcessingMode methodMode_ {api::types::MethodCallProcessingMode::kEvent};
    vrtf::vcc::utils::SafeMap<vrtf::vcc::api::types::EntityId, std::shared_ptr<vrtf::vcc::EventSkeleton>> mapEventSkel_;
    std::map<vrtf::vcc::api::types::EntityId, std::shared_ptr<vrtf::vcc::MethodSkeleton>> mapMethodSkel_;
    std::uint16_t methodThreadNumber_ {5}; // The default method thread number to processing method requests
    std::uint16_t methodThreadQueueSize_ {1024}; // The default method thread queue size to processing method requests
    std::shared_ptr<utils::ThreadPool> pool_;
    /* all methods share this queue in polling mode */
    vrtf::vcc::utils::RWLock msgQueueLock_;
    std::mutex processNextCallMutex_;
    const size_t MAX_MSG_QUEUE_SIZE {1024}; /* msg queue maxmum size is 1024 */
    std::queue<std::shared_ptr<vrtf::vcc::api::types::MethodMsg>> msgQueue_;
    // proxy variables
    api::types::HandleType handle_ {api::types::HandleType(api::types::UNDEFINED_SERVICEID,
        api::types::UNDEFINED_INSTANCEID, api::types::INVALIDTYPE)};
    bool isServiceAvailable_ {false};
    // In one process, there can be many proxys for the same serviceId-InstanceId-DriverType, every proxy vcc has its
    // own proxyVccId_
    std::uint16_t proxyVccId_ {0};
    vrtf::vcc::api::types::DriverType drvType_ {vrtf::vcc::api::types::INVALIDTYPE};
    utils::SafeMap<api::types::EntityId, api::types::EventReceiveHandler> mapEventHandler_;
    std::map<vrtf::vcc::api::types::EntityId,
        std::pair<api::types::EventSubscriptionState, TypesSubscriptionStateChangeHandler>> mapEventStatusHandler_;
    vrtf::vcc::utils::SafeMap<vrtf::vcc::api::types::EntityId, std::shared_ptr<vrtf::vcc::EventProxy>> mapEventProxy_;
    std::map<vrtf::vcc::api::types::EntityId, std::shared_ptr<vrtf::vcc::MethodProxy>> mapMethodProxy_;
    std::mutex serviceStatusMutex_;
    std::mutex eventStatusCallbackMutex_;
    std::mutex rawBufferCallbakcMutex_;
    static std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    std::once_flag methodNextCallFlag_;
    // use to maintain plog life cycle
    std::shared_ptr<rbs::plog::ProfileLogWriter> plogInstance_ {nullptr};
    // E2E ERROR HANDLER MAPPING
    using E2EErrorHandlerType =
        std::function<void(vrtf::vcc::api::types::E2EErrorCode, vrtf::vcc::api::types::E2EDataId,
                           vrtf::vcc::api::types::MessageCounter)>;
    E2EErrorHandlerType e2eErrorCallback_;
    const std::map<ara::com::e2e::ProfileCheckStatus, vrtf::vcc::api::types::E2EErrorCode> mapE2EErrCode_ {
        {ara::com::e2e::ProfileCheckStatus::kRepeated, vrtf::vcc::api::types::E2EErrorCode::repeated},
        {ara::com::e2e::ProfileCheckStatus::kWrongSequence, vrtf::vcc::api::types::E2EErrorCode::wrong_sequence_error},
        {ara::com::e2e::ProfileCheckStatus::kError, vrtf::vcc::api::types::E2EErrorCode::error},
        {ara::com::e2e::ProfileCheckStatus::kNotAvailable, vrtf::vcc::api::types::E2EErrorCode::not_available},
        {ara::com::e2e::ProfileCheckStatus::kNoNewData, vrtf::vcc::api::types::E2EErrorCode::no_new_data},
        {ara::com::e2e::ProfileCheckStatus::kCheckDisabled, vrtf::vcc::api::types::E2EErrorCode::check_disable}
    };
};
}
}
#endif
