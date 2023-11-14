/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Adapter layer between Ros and Vcc Proxy
 * Create: 2020-04-22
 */
#ifndef RTF_COM_ADAPTER_ROS_PROXY_ADAPTER_H_
#define RTF_COM_ADAPTER_ROS_PROXY_ADAPTER_H_

#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "ara/core/future.h"
#include "rtf/com/types/ros_types.h"
#include "rtf/com/utils/logger.h"
#include "rtf/com/entity/thread_group.h"
#include "vrtf/vcc/api/recv_buffer.h"
#include "rtf/com/types/method_result.h"
#include "vrtf/vcc/api/dynamic_error_domain.h"
#include "rtf/com/utils/type_name_helper.h"
namespace rtf {
namespace com {
namespace utils {
class SomeipJsonHelper;
namespace TemplateDeduction {
template<typename T> struct IsSharedPtr : std::false_type {};
template<typename T> struct IsSharedPtr<std::shared_ptr<T>> : std::true_type {};
}
}
namespace adapter {
class RosProxyAdapter : public std::enable_shared_from_this<RosProxyAdapter> {
public:
    /**
     * @brief RosProxyAdapter constructor
     */
    RosProxyAdapter(void);

    /**
     * @brief RosProxyAdapter destructor
     */
    ~RosProxyAdapter(void);

    /**
     * @brief Initialize RosProxyAdapter
     * @param[in] uri           The entity uri that the adapter manage
     * @param[in] dataType      The data type of the entity
     * @param[in] type          The adapter type (EVENT/METHOD)
     * @param[in] threadPool    The thread poll that handles operations
     * @param[in] role          The role of the entity
     */
    bool Initialize(std::string const &uri, MaintainConfig const &maintainConfig,
                    AdapterType type, std::shared_ptr<VccThreadPool> const &threadPool, Role const &role) noexcept;

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
     * @brief Stop responding the ros uri that adapter handles
     * @return void
     */
    void Shutdown(void) noexcept;

    /**
     * @brief Is a synchronous received event
     *
     * @retval true   it is an synchronous received event, the user should call GetEventData actively
     * @retval false  it is an asynchronous received event, the user shouldn't call GetEventData actively
     */
    bool IsSynchronousReceivedEvent(void) const noexcept;

    /**
     * @brief Subscribe an event and set its receive callback
     * @param[in] callback     event received callback
     * @param[in] queueSize    queue size
     * @return void
     */
    template<class EventDataType,
        typename std::enable_if<!utils::TemplateDeduction::IsSharedPtr<EventDataType>::value>::type* = nullptr>
    bool Subscribe(std::function<void(EventDataType)> callback, uint32_t queueSize,
                   ThreadGroup& threadGroup) noexcept
    {
        bool result {false};
        // If the adapter is using default configuration, there is no service discovery
        // procsss. We should ignore isServiceDiscovered_ flag
        std::unique_lock<std::mutex> lock(isServiceDiscoveredMutex_);
        if ((isUsingDefaultConfig_) || (isServiceDiscovered_)) {
            lock.unlock();
            result = SubscribeEvent(callback, queueSize, threadGroup);
        } else {
            // If the service is not ready, put the function call into a queue
            // then call it once the service is discovered
            AddPendingAction([this, callback, queueSize, threadGroup]() {
                this->SubscribeEvent<EventDataType>(callback, queueSize, threadGroup);
            });
            lock.unlock();
            result = true;
        }
        if (result == true && threadGroup.GetThreadMode() == ThreadMode::POLL && callback != nullptr) {
            std::weak_ptr<RosProxyAdapter> weakSelf = shared_from_this();
            threadGroup.AddSpinCallback([callback, weakSelf] (void) {
                std::shared_ptr<RosProxyAdapter> self = weakSelf.lock();
                if (self && !self->isShutdown_) {
                    self->EventRecvCallback(callback);
                }
            });
        }
        return result;
    }
    /**
    * @brief Subscribe an event and set its receive callback
    * @param[in] callback     event received callback, specialization of shared_ptr
    * @param[in] queueSize    queue size
    * @return void
    */
    template<class EventDataType,
        typename std::enable_if<utils::TemplateDeduction::IsSharedPtr<EventDataType>::value>::type* = nullptr>
    bool Subscribe(std::function<void(EventDataType)> callback, uint32_t queueSize,
                   ThreadGroup& threadGroup) noexcept
    {
        using DataType = typename EventDataType::element_type;
        std::function<void(DataType)> sharedPtrCallback = [callback](const DataType &value) {
            callback(std::make_shared<DataType>(value));
        };
        return Subscribe(sharedPtrCallback, queueSize, threadGroup);
    }
    /**
     * @brief Subscribe an event and set its receive callback include plog uid
     * @param[in] callback     event received callback and corresponding information
     * @param[in] queueSize    queue size
     * @return void
     */
    template<class EventDataType,
        typename std::enable_if<!utils::TemplateDeduction::IsSharedPtr<EventDataType>::value>::type* = nullptr>
    bool Subscribe(std::function<void(EventDataType, const SampleInfo&)> callback, uint32_t queueSize,
                   ThreadGroup& threadGroup) noexcept
    {
        bool result {false};
        if (callback == nullptr) {
            synchronousReceivedEvent_ = true;
        }
        // If the adapter is using default configuration, there is no service discovery
        // procsss. We should ignore isServiceDiscovered_ flag
        std::unique_lock<std::mutex> lock(isServiceDiscoveredMutex_);
        if ((isUsingDefaultConfig_) || (isServiceDiscovered_)) {
            lock.unlock();
            result = SubscribeEvent(callback, queueSize, threadGroup);
        } else {
            // If the service is not ready, put the function call into a queue
            // then call it once the service is discovered
            AddPendingAction([this, callback, queueSize, threadGroup]() {
                this->SubscribeEvent<EventDataType>(callback, queueSize, threadGroup);
            });
            lock.unlock();
            result = true;
        }
        if (result == true && threadGroup.GetThreadMode() == ThreadMode::POLL && callback != nullptr) {
            std::weak_ptr<RosProxyAdapter> weakSelf = shared_from_this();
            threadGroup.AddSpinCallback([weakSelf, callback] (void) {
                std::shared_ptr<RosProxyAdapter> self = weakSelf.lock();
                if (self && !self->isShutdown_) {
                    const auto callTimes = self->GetEventData(callback);
                    /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                    self->logger_->Verbose() << "[RTFCOM] Call event callback times is: " << callTimes;
                }
            });
        }
        return result;
    }
    /**
    * @brief Subscribe an event and set its receive callback include plog uid
    * @param[in] callback     event received callback and corresponding information, specialization of shared_ptr
    * @param[in] queueSize    queue size
    * @return void
    */
    template<class EventDataType,
        typename std::enable_if<utils::TemplateDeduction::IsSharedPtr<EventDataType>::value>::type* = nullptr>
    bool Subscribe(std::function<void(EventDataType, const SampleInfo&)> callback, uint32_t queueSize,
                   ThreadGroup& threadGroup) noexcept
    {
        using DataType = typename EventDataType::element_type;
        std::function<void(DataType, const SampleInfo&)> sharedPtrCallback =
        [callback](const DataType &value, const SampleInfo& sampleInfo) {
            callback(std::make_shared<DataType>(value), sampleInfo);
        };
        return Subscribe(sharedPtrCallback, queueSize, threadGroup);
    }
    /**
     * @brief  Initialize Method
     *
     * @return true   Initialize Method Successfully
     * @return false  Initialize Method failed
     */
    template <class MethodDataType>
    bool CreateMethodClient(void) noexcept
    {
        bool result {true};
        std::unique_lock<std::mutex> lock(isServiceDiscoveredMutex_);
        if (isUsingDefaultConfig_ || isServiceDiscovered_) {
            lock.unlock();
            result = InitializeMethod<MethodDataType>();
        } else {
            AddPendingAction(std::bind(&RosProxyAdapter::InitializeMethod<MethodDataType>, this));
            lock.unlock();
        }
        return result;
    }

    /**
     * @brief Make a remote procedure call
     * @param[in] methodData    whole method data structure
     * @param[in] timeout       rpc call timeout
     * @return rpc operation result
     */
    template<typename MethodDataType, typename Rep, typename Period>
    MethodClientResult Call(MethodDataType& methodData, const std::chrono::duration<Rep, Period>& timeout) noexcept
    {
        using namespace rtf::com::utils;
        MethodClientResult result;
        WaitForValid(timeout);
        std::shared_ptr<VccProxy> proxy {proxy_};
        if (isShutdown_ || proxy == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] The proxy may be shutted down or not created '" << uri_ << "'";
            return result;
        }
        if (!IsValid()) {
            result.SetErrorCode(ErrorCode::NOTAVAILABLE);
            PrintMethodRequestInfo(result, "");
            return result;
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Requesting data from '" << uri_ << "'";
        result = Request(methodData, timeout, proxy);
        result.SetE2EResult(proxy->GetMethodE2EResult(entityId_));
        return result;
    }
    template<typename MethodDataType, typename Rep, typename Period>
    MethodClientResult Request(MethodDataType& methodData, const std::chrono::duration<Rep, Period>& timeout,
             std::shared_ptr<VccProxy> const &proxy,
             typename std::enable_if<!vrtf::serialize::ros::IsRosMethodMsg<MethodDataType>::value>::type* = 0) noexcept
    {
        using namespace rtf::com::utils;
        MethodClientResult result;
        std::stringstream timeoutStr;
        timeoutStr << std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        auto future = proxy->Request<typename MethodDataType::Response, typename MethodDataType::Request>(
            entityId_, methodData.req);
        const auto& futureStatus = future.wait_for(timeout);
        if (futureStatus == ara::core::future_status::ready) {
            const auto& response = future.GetResult();
            if (response.HasValue()) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Debug() << "[RTFCOM] Data replied from '" << uri_ << "'";
                methodData.res = response.Value();
                result.SetErrorCode(ErrorCode::OK);
            } else {
                const auto error = response.Error();
                result = ParseErrorCode(error);
            }
        } else {
            result.SetErrorCode(ErrorCode::TIMEOUT);
        }
        PrintMethodRequestInfo(result, timeoutStr.str());
        return result;
    }
    template<typename MethodDataType, typename Rep, typename Period>
    MethodClientResult Request(MethodDataType& methodData, const std::chrono::duration<Rep, Period>& timeout,
             std::shared_ptr<VccProxy> const &proxy,
             typename std::enable_if<vrtf::serialize::ros::IsRosMethodMsg<MethodDataType>::value>::type* = 0) noexcept
    {
        using namespace rtf::com::utils;
        MethodClientResult result;
        std::stringstream timeoutStr;
        timeoutStr << std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        auto future = proxy->Request<typename MethodDataType::Response, typename MethodDataType::Request>(
            entityId_, methodData.request);
        const auto& futureStatus = future.wait_for(timeout);
        if (futureStatus == ara::core::future_status::ready) {
            const auto& response = future.GetResult();
            if (response.HasValue()) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Debug() << "[RTFCOM] Data replied from '" << uri_ << "'";
                methodData.response = response.Value();
                result.SetErrorCode(ErrorCode::OK);
            } else {
                const auto error = response.Error();
                result = ParseErrorCode(error);
            }
        } else {
            result.SetErrorCode(ErrorCode::TIMEOUT);
        }
        PrintMethodRequestInfo(result, timeoutStr.str());
        return result;
    }
    /**
     * @brief Get the Event Data through GetNewSample in vcc
     *
     * @tparam     EventDataType   The type of event data
     * @param[in]  callback        The user's callback to get the event data
     * @return std::size_t
     */
    template<class EventDataType,
        typename std::enable_if<!utils::TemplateDeduction::IsSharedPtr<EventDataType>::value>::type* = nullptr>
    size_t GetEventData(const std::function<void(EventDataType, const SampleInfo&)>& callback) noexcept
    {
        using namespace rtf::com::utils;
        std::size_t callTimes {0};
        std::shared_ptr<EntityConfig> entityConfig = entityConfig_;
        std::shared_ptr<vrtf::vcc::Proxy> proxy = proxy_;
        if (entityConfig == nullptr || proxy == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] The proxy may be shutted down or not created '" << uri_ << "'";
            return callTimes;
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Verbose() << "[RTFCOM] Data received from '" << uri_ << "'";
        using DataType = typename std::decay<EventDataType>::type;
        auto callTimesRet = proxy->GetNewSamples<DataType>(
            [this, callback](VccSamplePtr<DataType const> data) -> void {
                this->TriggerCallback(data, callback);
            },
            entityId_
        );
        if (callTimesRet.HasValue()) {
            callTimes = callTimesRet.Value();
        }
        return callTimes;
    }
    template<class EventDataType,
        typename std::enable_if<utils::TemplateDeduction::IsSharedPtr<EventDataType>::value>::type* = nullptr>
    size_t GetEventData(const std::function<void(EventDataType, const SampleInfo&)>& callback) noexcept
    {
        using DataType = typename EventDataType::element_type;
        std::function<void(DataType, const SampleInfo&)> sharedPtrCallback =
        [callback](const DataType &value, const SampleInfo& sampleInfo) {
            callback(std::make_shared<DataType>(value), sampleInfo);
        };
        return GetEventData(sharedPtrCallback);
    }
private:
    std::string     uri_;
    EntityId        entityId_ {0};
    AdapterType     type_ {AdapterType::UNKNOWN};
    AdapterProtocol protocol_ {AdapterProtocol::UNKNOWN};
    MaintainConfig  maintainConfig_;

    std::shared_ptr<EntityConfig>         entityConfig_ {nullptr};
    std::unique_ptr<VccFindServiceHandle> findServiceHandle_ {nullptr};
    std::shared_ptr<VccThreadPool>        threadPool_ {nullptr};
    std::shared_ptr<VccProxy>             proxy_ {nullptr};
    std::shared_ptr<rtf::com::utils::SomeipJsonHelper> someipJsonHelperPtr_;
    std::queue<std::function<void()>> pendingAction_;
    std::mutex                        pendingActionMutex_;
    std::mutex                        waitForValidMutex_;
    std::mutex                        proxyMutex_;
    std::mutex                        isServiceDiscoveredMutex_;
    std::condition_variable           validCondition_;

    bool isInitialized_ {false};
    bool isUsingDefaultConfig_ {false};
    bool isServiceDiscovered_ {false};
    bool isMethodOnline_ {false};
    bool synchronousReceivedEvent_ {false};
    std::atomic<bool> isShutdown_ {false};
    // Dfx helper info, count the number of online and offline services
    uint16_t serviceOnlineCount_ {0};
    uint16_t serviceOfflineCount_ {0};
    uint32_t sendRequestFailCount_ {0};
    bool sendRequestFail_ {false};
    std::shared_ptr<rtf::com::utils::Logger> logger_;
    /**
     * @brief Parse config at the begining
     * @param[in] role The role of entity
     * @return The result of parsing config
     */
    bool ParseConfig(const Role& role) noexcept;

    /**
     * @brief Syncorizing Maintaining config and entity config
     *
     */
    void SyncorizeEntityConfig(void) noexcept;

    /**
     * @brief Initialize enity for current config
     *
     * @return Entity initialization result
     */
    bool InitializeEntity(void) noexcept;

    bool StartFindService(void) noexcept;
    void StopFindService(void) noexcept;
    void OnServiceDiscovered(VccServiceHandleContainer const &handles, VccFindServiceHandle const &handle) noexcept;
    /**
     * @brief Update the method online status, only dds default param will call this function
     *
     * @return void
     */
    void OnMethodStateChanged(VccMethodState state) noexcept;

    void AddPendingAction(const std::function<void()>& action) noexcept;
    void ExecuteAllPendingActions(void) noexcept;

    void RegisterE2EConfigInfoToMaintaind() const;

    /**
     * @brief Wait for remote server becomes valid until timeout
     * @param[in] timeout    timeout
     * @return void
     */
    template<typename Rep, typename Period>
    void WaitForValid(const std::chrono::duration<Rep, Period>& timeout) noexcept
    {
        using namespace rtf::com::utils;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Waiting for '" << uri_ << "' becomes valid...";
        std::unique_lock<std::mutex> lock(waitForValidMutex_);
        validCondition_.wait_for(lock, timeout, [this] (void) -> bool {
            return ((IsValid()) || (isShutdown_));
        });
    }

    /**
     * @brief When data is available trigger user callback
     * @param[in] data user's data
     * @param[in] callback user's callback
     * @return void
     */
    template<class EventDataType>
    void TriggerCallback(VccSamplePtr<typename std::decay<EventDataType>::type const>& data,
                         const std::function<void(EventDataType)>& callback)
    {
        callback(*data);
    }

    /**
     * @brief When data is available trigger user callback(use in RecvMemory mode)
     * @param[in] data user's data
     * @param[in] callback user's callback
     * @return void
     */
    void TriggerCallback(VccSamplePtr<RecvMemory const>& data, const std::function<void(RecvMemory)>& callback)
    {
        RecvMemory recvMemory(data->Get(), data->GetSize(), data->GetReturnLoan());
        callback(std::move(recvMemory));
    }

    /**
     * @brief When data is available trigger user callback include corresponding information
     * @param[in] data user's data
     * @param[in] callback user's callback include corresponding information
     * @return void
     */
    template<class EventDataType>
    void TriggerCallback(VccSamplePtr<typename std::decay<EventDataType>::type const>& data,
                         const std::function<void(EventDataType, const SampleInfo&)>& callback)
    {
        SampleInfo info;
        info.SetSampleId(data.GetSampleId());
        info.SetE2EResult(data.GetE2EResult());
        callback(*data, info);
    }

    /**
     * @brief When data is available trigger user callback(use in RecvMemory mode) include corresponding information
     * @param[in] data user's data
     * @param[in] callback user's callback include corresponding information
     * @return void
     */
    void TriggerCallback(VccSamplePtr<RecvMemory const>& data,
                         const std::function<void(RecvMemory, const SampleInfo&)>& callback)
    {
        RecvMemory recvMemory(data->Get(), data->GetSize(), data->GetReturnLoan());
        SampleInfo info;
        info.SetSampleId(data.GetSampleId());
        info.SetE2EResult(data.GetE2EResult());
        callback(std::move(recvMemory), info);
    }
    /**
     * @brief Subscribe event
     * @param[in] callback     Event trigger callback
     * @param[in] queueSize    Queue size
     * @return Subscription result
     */
    template<class EventDataType>
    bool SubscribeEvent(const std::function<void(EventDataType)>& callback, uint32_t queueSize,
                        ThreadGroup const &threadGroup) noexcept
    {
        using namespace rtf::com::utils;
        bool result = false;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Subscribing '" << uri_ << "'...";
        auto eventInfo = std::static_pointer_cast<VccEventInfo>(entityConfig_->entityInfo);
        if (eventInfo == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot find event configuration for '" << uri_ << "'";
            return result;
        }
        if (eventInfo->GetE2EObject() != nullptr) {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logger_->Error() << "[RTFCOM] '" << uri_ <<
                               "' is using E2E protection but the using interface cannot obtain E2E results.";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return result;
        }
        std::shared_ptr<VccProxy> proxy = proxy_;
        if (proxy == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] The proxy may be shutted down or not created '" << uri_ << "'";
            return result;
        }
        if (threadGroup.GetThreadMode() == ThreadMode::EVENT) {
            std::weak_ptr<RosProxyAdapter> weakSelf = this->shared_from_this();
            proxy->SetReceiveHandler([weakSelf, callback] (void) {
                    std::shared_ptr<RosProxyAdapter> self = weakSelf.lock();
                    if (self) {
                        self->EventRecvCallback(callback);
                    }
                }, entityId_);
        }
        using DataType = typename std::decay<EventDataType>::type;
        result = proxy->Subscribe<DataType>(entityId_, queueSize, eventInfo, threadPool_);
        if (result) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] Subscribed '" << uri_ << "'";
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Failed to subscribe '" << uri_ << "'";
        }
        return result;
    }

    /**
     * @brief Register event receive callback
     * @param[in] callback     Event trigger callback include corresponding information
     * @param[in] proxy        The proxy of vcc level
     * @param[in] threadGroup  Thread group
     * @return Subscription result
     */
    template<class EventDataType>
    void RegisterEventCallback(std::function<void(EventDataType, const SampleInfo&)> const &callback,
                               std::shared_ptr<VccProxy> const &proxy,
                               ThreadGroup const &threadGroup)
    {
        using namespace rtf::com::utils;
        if (threadGroup.GetThreadMode() == ThreadMode::EVENT) {
            std::weak_ptr<RosProxyAdapter> weakSelf = this->shared_from_this();
            proxy->SetReceiveHandler([weakSelf, callback] (void) {
                    std::shared_ptr<RosProxyAdapter> self = weakSelf.lock();
                    if (self) {
                        const auto callTimes = self->GetEventData(callback);
                        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                        self->logger_->Verbose() << "[RTFCOM] Call event callback times is: " << callTimes;
                    }
                }, entityId_);
        }
    }

    /**
     * @brief Subscribe event
     * @param[in] callback     Event trigger callback include corresponding information
     * @param[in] queueSize    Queue size
     * @return Subscription result
     */
    template<class EventDataType>
    bool SubscribeEvent(
        std::function<void(EventDataType, const SampleInfo&)> const &callback, uint32_t queueSize,
        ThreadGroup const &threadGroup) noexcept
    {
        using namespace rtf::com::utils;

        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Subscribing '" << uri_ << "'...";
        auto eventInfo = std::static_pointer_cast<VccEventInfo>(entityConfig_->entityInfo);
        bool result = false;
        if (eventInfo != nullptr) {
            std::shared_ptr<VccProxy> proxy = proxy_;
            if (proxy == nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Debug() << "[RTFCOM] The proxy may be shutted down or not created '" << uri_ << "'";
                return false;
            }
            if (callback != nullptr) {
                RegisterEventCallback(callback, proxy, threadGroup);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] Subscribing '" << uri_ << "' using active trigger mode";
            }
            using DataType = typename std::decay<EventDataType>::type;
            result = proxy->Subscribe<DataType>(entityId_, queueSize, eventInfo, threadPool_);
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot find event configuration for '" << uri_ << "'";
        }
        if (result) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] Subscribed '" << uri_ << "'";
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Failed to subscribe '" << uri_ << "'";
        }
        return result;
    }

    /**
     * @brief Initialize method
     * @return Initialization result
     */
    template <typename MethodDataType>
    bool InitializeMethod(void) noexcept
    {
        using namespace rtf::com::utils;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Verbose() << "[RTFCOM] Initializing configured method '" << uri_ << "'...";

        bool result = false;
        std::shared_ptr<VccProxy> proxy = proxy_;
        if (proxy == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] The proxy may be shutted down or not created '" << uri_ << "'";
            return false;
        }
        std::weak_ptr<RosProxyAdapter> weakSelf = this->shared_from_this();
        std::function<void(VccMethodState)> methodStateChangeHandler = [weakSelf](VccMethodState state) {
            std::shared_ptr<RosProxyAdapter> self = weakSelf.lock();
            if (self) {
                self->OnMethodStateChanged(state);
            }
        };
        if (proxy->InitializeMethod<typename MethodDataType::Response>(
            entityId_, std::static_pointer_cast<VccMethodInfo>(entityConfig_->entityInfo))) {
            // Registering DynamicErrorCodes to receive CM user-defined error codes
            proxy->RegisterError(entityId_, vrtf::core::ErrorCode(vrtf::vcc::api::types::DynamicErrc::default_errc));
            // We use MethodStateChangeHandler only in default configuration, since
            // 1. It only works on DDS protocol
            // 2. On the other situation, we have service discovery to
            //    ensure the connectivity of the remote server
            if (isUsingDefaultConfig_) {
                proxy->SetMethodStateChangeHandler(methodStateChangeHandler, entityId_);
            }
            result = true;
        }
        return result;
    }

    bool AddClientToSOMEIPD() noexcept;
    /**
     * @brief Get the Event Data through GetNewSample in vcc
     *
     * @tparam     EventDataType   The type of event data
     * @param[in]  callback        The user's callback to get the event data
     * @return std::size_t
     */
    template<class EventDataType>
    void EventRecvCallback(const std::function<void(EventDataType)>& callback) noexcept
    {
        using namespace rtf::com::utils;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Verbose() << "[RTFCOM] Data received from '" << uri_ << "'";
        std::shared_ptr<EntityConfig> entityConfig = entityConfig_;
        std::shared_ptr<VccProxy> proxy = proxy_;
        if (proxy == nullptr || entityConfig == nullptr) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Debug() << "[RTFCOM] The proxy may be shutted down or not created '" << uri_ << "'";
            return;
        }
        using DataType = typename std::decay<EventDataType>::type;
        auto callTimesResult = proxy->GetNewSamples<DataType>(
                [this, callback](VccSamplePtr<DataType const> data) -> void {
                    this->TriggerCallback(data, callback);
                },
                entityId_
        );
        if (callTimesResult.HasValue()) {
            size_t callTimes = callTimesResult.Value();
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Verbose() << "[RTFCOM] Call event callback times is : " << callTimes;
        }
    }
    /**
     * @brief Parse the error code from the received method error response.
     * @param[in] errorCode     the received method error code
     * @return parse result
     */
    MethodClientResult ParseErrorCode(const vrtf::core::ErrorCode& errorCode) const noexcept;
    /**
     * @brief Print service status help info
     * @param[in] entityName The need to print entityName
     * @return void
     */
    void PrintServiceStatusHelperInfo(const std::string& entityName) const;
    void PrintMethodRequestInfo(MethodClientResult const &result, std::string const &timeoutStr);

    void UnsetProxyCallback() noexcept;
};
} // namspace adapter
} // namspace com
} // namspace rtf
#endif // RTF_COM_ADAPTER_ROS_PROXY_ADAPTER_H_
