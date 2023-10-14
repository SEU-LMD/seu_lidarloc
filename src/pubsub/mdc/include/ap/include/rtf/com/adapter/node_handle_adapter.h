/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This is a node can be used in sub or pub function
 * Create: 2020-04-22
 */
#ifndef RTF_COM_ADAPTER_NODE_HANDLE_ADAPTER_H_
#define RTF_COM_ADAPTER_NODE_HANDLE_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <functional>

#include "rtf/com/entity/publisher.h"
#include "rtf/com/entity/subscriber.h"
#include "rtf/com/entity/method_client.h"
#include "rtf/com/entity/method_server.h"
#include "rtf/com/utils/type_name_helper.h"
#include "rtf/com/utils/method_args_helper.h"
#include "rtf/com/utils/logger.h"
#include "rtf/com/utils/thread_pool_helper.h"
#include "rtf/com/adapter/internal/global_shutdown.h"
namespace rtf {
namespace com {
namespace adapter {
class NodeHandleAdapter {
public:
    /**
     * @brief MethodClient default constructor
     */
    NodeHandleAdapter(void);

    /**
     * @brief MethodClient destructor
     */
    ~NodeHandleAdapter(void);

    /**
     * @brief MethodClient copy constructor
     * @note deleted
     */
    NodeHandleAdapter(const NodeHandleAdapter& other) = delete;

    /**
     * @brief MethodClient move constructor
     */
    NodeHandleAdapter(NodeHandleAdapter && other);

    /**
     * @brief MethodClient copy assign operator
     * @note deleted
     */
    NodeHandleAdapter& operator=(const NodeHandleAdapter& other) = delete;

    /**
     * @brief MethodClient move assign operator
     */
    NodeHandleAdapter& operator=(NodeHandleAdapter && other);

    /**
     * @brief Initailize Node Handle Adapter
     * @param[in] type    The type of the node handle caller
     * @param[in] role    The role of the node handle caller
     * @return true   Initialization successfully
     * @return false  Initialization failed
     */
    bool Initialization(AdapterType type, Role role) noexcept;

    /**
     * @brief Advertise an event on the node
     * @param[in] uri    The ros uri of the event
     * @return The publisher of the event
     */
    template<class EventDataType>
    Publisher<EventDataType> Advertise(const std::string& uri) noexcept
    {
        using namespace rtf::com::utils;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Creating Publisher['" << uri << "']";
        std::shared_ptr<RosSkeletonAdapter> adapter {std::make_shared<RosSkeletonAdapter>()};
        if (AddNewSkeletonToList(uri, adapter) == true) {
            const MaintainConfig maintainConfig {
                GetTypeNameByRos<EventDataType>(), {}, {}};
            if (adapter->Initialize(uri, maintainConfig, AdapterType::EVENT, nullptr, Role::SERVER)) {
                isAdapterCreated_ = true;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] Publisher['" << uri << "'] created";
                globalShutDown_->AddRosSkeleton(uri, adapter);
            } else {
                EraseSkeleton(uri);
                adapter = nullptr;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Publisher['" << uri << "'] create failed";
            }
        } else {
            adapter = nullptr;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot create duplicate Publisher['" << uri << "']";
        }
        return Publisher<EventDataType>(adapter);
    }

    /**
     * @brief Send an event subscription request and create a subscriber
     * @param[in] uri          The ros uri of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The callback function for processing received data
     * @return The subscriber of the event
     */
    template<class EventDataType>
    Subscriber Subscribe(const std::string& uri, uint32_t queueSize, std::function<void(EventDataType)> callback,
                         ThreadGroup& threadGroup = defaultThreadGroup_) noexcept
    {
        using namespace rtf::com::utils;

        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Creating Subscriber['" << uri << "']";
        std::shared_ptr<RosProxyAdapter> adapter {std::make_shared<RosProxyAdapter>()};
        if (AddNewProxyToList(uri, adapter) == true) {
            std::shared_ptr<VccThreadPool> threadPool;
            if (&threadGroup != &defaultThreadGroup_) {
                threadPool = ThreadPoolHelper::QueryThreadPool(threadGroup);
            } else {
                threadPool = GetThreadPool(uri);
            }
            if (threadPool == nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Cannot find corresponding ThreadGroup of ['" << uri << "']";
            }
            const MaintainConfig maintainConfig {
                GetTypeNameByRos<EventDataType>(), {}, {}};
            if ((threadPool != nullptr &&
                adapter->Initialize(uri, maintainConfig, AdapterType::EVENT, threadPool, Role::CLIENT)) &&
                (adapter->Subscribe(callback, queueSize, threadGroup))) {
                isAdapterCreated_ = true;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] Subscriber['" << uri << "'] created";
                globalShutDown_->AddRosProxy(uri, adapter);
            } else {
                EraseProxy(uri);
                adapter = nullptr;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Subscriber['" << uri << "'] create failed";
            }
        } else {
            adapter = nullptr;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot create duplicate Subscriber['" << uri << "']";
        }
        return Subscriber(adapter);
    }

    /**
     * @brief Send an event subscription request and create a subscriber
     * @param[in] uri          The ros uri of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The callback function for processing received data and corresponding information
     * @return The subscriber of the event
     */
    template<class EventDataType>
    Subscriber Subscribe(const std::string& uri, uint32_t queueSize,
                         std::function<void(EventDataType, const SampleInfo&)> callback,
                         ThreadGroup& threadGroup = defaultThreadGroup_) noexcept
    {
        using namespace rtf::com::utils;

        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Creating Subscriber['" << uri << "']";
        std::shared_ptr<RosProxyAdapter> adapter {std::make_shared<RosProxyAdapter>()};
        if (AddNewProxyToList(uri, adapter) == true) {
            std::shared_ptr<VccThreadPool> threadPool;
            if (&threadGroup != &defaultThreadGroup_) {
                threadPool = ThreadPoolHelper::QueryThreadPool(threadGroup);
            } else {
                threadPool = GetThreadPool(uri);
            }
            if (threadPool == nullptr) {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Cannot find corresponding ThreadGroup of ['" << uri << "']";
            }
            const MaintainConfig maintainConfig {
                GetTypeNameByRos<EventDataType>(), {}, {}};
            if ((threadPool != nullptr &&
                adapter->Initialize(uri, maintainConfig, AdapterType::EVENT, threadPool, Role::CLIENT)) &&
                (adapter->Subscribe(callback, queueSize, threadGroup))) {
                isAdapterCreated_ = true;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] Subscriber['" << uri << "'] created";
                globalShutDown_->AddRosProxy(uri, adapter);
            } else {
                EraseProxy(uri);
                adapter = nullptr;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Subscriber['" << uri << "'] create failed";
            }
        } else {
            adapter = nullptr;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot create duplicate Subscriber['" << uri << "']";
        }
        return Subscriber(adapter);
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] uri         The ros uri of the method
     * @param[in] callback    The method callback function
     * @return The server of a specific method
     */
    template<class Request, class Response>
    MethodServer RegisterMethod(const std::string& uri,
                                const std::function<bool(Request&, Response&)>& callback) noexcept
    {
        using namespace rtf::com::utils;

        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Creating MethodServer['" << uri << "']";
        std::shared_ptr<RosSkeletonAdapter> adapter {std::make_shared<RosSkeletonAdapter>()};
        if (AddNewSkeletonToList(uri, adapter) == true) {
            ++methodNumber_;
            std::string const threadName {"cm_method_" + std::to_string(methodNumber_)};
            // 1 is the thread number, 1024 is the thread cache
            std::shared_ptr<VccThreadPool> threadPool = std::make_shared<VccThreadPool>(1, 1024, threadName);
            const MaintainConfig maintainConfig {
                "", GetTypeNameByRos<Request>(), GetTypeNameByRos<Response>()};
            if ((adapter->Initialize(uri, maintainConfig, AdapterType::METHOD, threadPool, Role::SERVER)) &&
                (adapter->RegisterMethod(callback))) {
                isAdapterCreated_ = true;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] MethodServer['" << uri << "'] created";
                globalShutDown_->AddRosSkeleton(uri, adapter);
            } else {
                EraseSkeleton(uri);
                adapter = nullptr;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] MethodServer['" << uri << "'] create failed";
            }
        } else {
            adapter = nullptr;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot create duplicate MethodServer['" << uri << "']";
        }
        return MethodServer(adapter);
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] uri         The ros uri of the method
     * @param[in] callback    The method callback function with E2E result
     * @return The server of a specific method
     */
    template<class Request, class Response>
    MethodServer RegisterMethod(const std::string& uri,
        const std::function<bool(Request&, Response&, MethodServerResult&)>& callback) noexcept
    {
        using namespace rtf::com::utils;

        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Creating MethodServer['" << uri << "']";
        std::shared_ptr<RosSkeletonAdapter> adapter {std::make_shared<RosSkeletonAdapter>()};
        if (AddNewSkeletonToList(uri, adapter) == true) {
            ++methodNumber_;
            std::string const threadName {"cm_method_" + std::to_string(methodNumber_)};
            // 1 is the thread number, 1024 is the thread cache
            std::shared_ptr<VccThreadPool> threadPool = std::make_shared<VccThreadPool>(1, 1024, threadName);
            const MaintainConfig maintainConfig {
                "", GetTypeNameByRos<Request>(), GetTypeNameByRos<Response>()};
            if ((adapter->Initialize(uri, maintainConfig, AdapterType::METHOD, threadPool, Role::SERVER)) &&
                (adapter->RegisterMethod(callback))) {
                isAdapterCreated_ = true;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] MethodServer['" << uri << "'] created";
                globalShutDown_->AddRosSkeleton(uri, adapter);
            } else {
                EraseSkeleton(uri);
                adapter = nullptr;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] MethodServer['" << uri << "'] create failed";
            }
        } else {
            adapter = nullptr;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot create duplicate MethodServer['" << uri << "']";
        }
        return MethodServer(adapter);
    }

    /**
     * @brief Create a client for a specific method
     * @param[in] uri    The ros uri of method
     * @return The client of the method with the methodName
     */
    template<typename MethodDataType>
    MethodClient CreateMethodClient(const std::string& uri) noexcept
    {
        using namespace rtf::com::utils;
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
        logger_->Debug() << "[RTFCOM] Creating MethodClient ['" << uri << "']";
        std::shared_ptr<RosProxyAdapter> adapter {std::make_shared<RosProxyAdapter>()};
        if (AddNewProxyToList(uri, adapter) == true) {
            const MaintainConfig maintainConfig {
                "", GetTypeNameByRos<typename MethodDataType::Request>(),
                                                GetTypeNameByRos<typename MethodDataType::Response>()
            };
            if ((adapter->Initialize(uri, maintainConfig, AdapterType::METHOD, nullptr, Role::CLIENT)) &&
                (adapter->CreateMethodClient<MethodDataType>())) {
                isAdapterCreated_ = true;
                globalShutDown_->AddRosProxy(uri, adapter);
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Info() << "[RTFCOM] MethodClient ['" << uri << "'] created";
            } else {
                EraseProxy(uri);
                adapter = nullptr;
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] MethodClient ['" << uri << "'] create failed";
            }
        } else {
            adapter = nullptr;
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Cannot create duplicate MethodClient ['" << uri << "']";
        }
        return MethodClient(adapter);
    }

    /**
     * @brief Set the thread number that handle all callbacks and its queue length
     * @param[in] threadNumber    The number of thread
     * @param[in] queueSize       The size of the queue
     * @return Operation result
     */
    bool SetMultiThread(uint16_t threadNumber, uint16_t queueSize) noexcept;

    /**
     * @brief Close all connections within the node
     */
    void Shutdown(void) noexcept;

    /**
     * @brief close all connections within the process
     *
     */
    void GlobalShutdown(void) noexcept;

    /**
     * @brief judge thread group is pass by user
     *
     */
    static ThreadGroup& GetDefaultThreadGroup();
private:
    bool AddNewSkeletonToList(const std::string& uri, const std::shared_ptr<RosSkeletonAdapter>& adapter);
    void EraseSkeleton(const std::string& uri);
    bool AddNewProxyToList(const std::string& uri, const std::shared_ptr<RosProxyAdapter>& adapter);
    void EraseProxy(const std::string& uri);
    std::shared_ptr<VccThreadPool> GetThreadPool(const std::string& uri);
    static const uint16_t DEFAULT_THREAD_NUMBER;
    static const uint16_t DEFAULT_QUEUE_SIZE;

    bool isAdapterCreated_;
    std::shared_ptr<ThreadGroup> threadGroup_;

    std::mutex skeletonMapMutex_;
    std::mutex proxyMapMutex_;
    std::mutex threadGroupMutex_;
    std::unordered_map<std::string, std::shared_ptr<adapter::RosSkeletonAdapter>> skeletonMap_;
    std::unordered_map<std::string, std::shared_ptr<adapter::RosProxyAdapter>> proxyMap_;
    std::shared_ptr<GlobalShutDown> globalShutDown_ = nullptr;
    std::shared_ptr<rtf::com::utils::Logger> logger_;
    static ThreadGroup defaultThreadGroup_;
    static std::uint16_t methodNumber_;
};
} // namespace adapter
} // namespace com
} // namespace rtf
#endif // RTF_COM_ADAPTER_NODE_HANDLE_ADAPTER_H_
